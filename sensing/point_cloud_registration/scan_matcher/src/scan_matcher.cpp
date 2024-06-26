#include <glog/logging.h>
#include <bnerf_utils/conversions.h>
#include <scan_matcher/scan_matcher.h>
#include <voxelizer/voxelizer_ndt.h>
#include <voxelizer/voxelizer_gicp.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>


namespace bnerf
{
    ScanMatcher::ScanMatcher()
        : nh_("~")
        , tf_finder_(nh_)
    {
        GET_REQUIRED(nh_, "max_iters", max_iters_);
        GET_OPTIONAL(nh_, "verbose", verbose_, false);
        GET_OPTIONAL(nh_, "epsilon", epsilon_, 1e-3);
        epsilon_ *= epsilon_;

        string solver;
        GET_REQUIRED(nh_, "solver", solver);
        use_gicp_ = solver == "gicp";
        LOG_ASSERT(use_gicp_ || solver == "ndt");
        
        double vox_win_span;
        GET_REQUIRED(nh_, "window_span", vox_win_span);
        vox_win_span_.fromSec(vox_win_span);

        bool visualize;
        GET_OPTIONAL(nh_, "visualize", visualize, false);
        if (visualize)
        {
            auto viz_pub = nh_.advertise<CloudXYZI>("combined_scan", 1);
            viz_pub_.reset(new ros::Publisher(move(viz_pub)));
        }

        edge_pub_ = nh_.advertise<bnerf_msgs::GraphBinaryEdge>("registration_binary_edge", 16);
        reg_pub_ = nh_.advertise<bnerf_msgs::ScanMatchingInfo>("scan_matching_info", 16);
        vox_pub_ = nh_.advertise<bnerf_msgs::VoxelizationInfo>("voxelization_info", 16);
        src_sub_ = nh_.subscribe("source_scan", 16, &ScanMatcher::SourceScanCallBack, this);
        tgt_sub_ = nh_.subscribe("target_scan", 16, &ScanMatcher::TargetScanCallBack, this);
    }


    void ScanMatcher::SourceScanCallBack(const sensor_msgs::PointCloud2::ConstPtr & source)
    {
        const auto t1 = ros::Time::now();
        const auto voxer = GetVoxelizer(source);
        if (!voxer)
            return;

        if (verbose_)
            LOG(WARNING) << "start optim. scan size: " << source->width;

        SE3d src_to_map = tf_finder_.InterpolateTransform(source->header.stamp);
        SE3d tgt_to_map = tf_finder_.InterpolateTransform(voxer->GetStamp());

        SE3d init_guess = tgt_to_map.inverse() * src_to_map;
        Optimizer optim(voxer, source);
        optim.SetEstimation(init_guess);

        bnerf_msgs::ScanMatchingInfo info_msg;
        while (ros::ok() && info_msg.num_iterations++ < uint(max_iters_))
        {
            optim.AccumulateHessian();

            if (verbose_)
                LOG(INFO) << setprecision(12)
                    << "iter: " << info_msg.num_iterations
                    << "\tnum_valids: " << optim.valid_ids_.size()
                    << "\tloss: " << optim.loss_;

            Vec6d inc = optim.H_.ldlt().solve(-optim.b_);
            LOG_ASSERT(inc.allFinite());
            if (inc.squaredNorm() < epsilon_)
                break;

            SE3d eps = SE3d::exp(inc);
            optim.SetEstimation(eps * optim.trans_);
        }

        const auto t2 = ros::Time::now();
        
        bnerf_msgs::GraphBinaryEdge edge_msg;
        optim.GetGraphBinaryEdge(edge_msg);
        edge_pub_.publish(edge_msg);
        
        if (viz_pub_)
        {
            CloudXYZI scan_msg;
            optim.GetCombinedScan(scan_msg);
            viz_pub_->publish(scan_msg);
        }
        
        optim.GetScanMatchingInfo(info_msg);
        info_msg.exec_time = t2 - t1;
        reg_pub_.publish(info_msg);
    }


    Voxelizer::ConstPtr ScanMatcher::GetVoxelizer(
        sensor_msgs::PointCloud2::ConstPtr source)
    {
        const auto & t2 = source->header.stamp;
        Voxelizer::ConstPtr best_voxer;
        double best_secs = numeric_limits<double>::max();

        lock_guard<mutex> lock(vox_mutex_);
        for (const auto & voxer: vox_win_)
        {
            const auto t1 = voxer->GetStamp();
            if (t1 == t2)
                continue;

            const double secs = abs((t2-t1).toSec());
            if (secs < best_secs)
            {
                best_secs = secs;
                best_voxer = voxer;
            }
        }

        return best_voxer;
    }


    void ScanMatcher::TargetScanCallBack(
        const sensor_msgs::PointCloud2::ConstPtr & target)
    {
        LOG_ASSERT(target);

        Voxelizer::Ptr voxer;
        if (use_gicp_)
            voxer.reset(new VoxelizerGICP(nh_));
        else
            voxer.reset(new VoxelizerNDT(nh_));

        auto msg = voxer->SetInputTarget(target);
        vox_pub_.publish(msg);

        auto t = voxer->GetStamp() - vox_win_span_;
        auto timeout = [&t](Voxelizer::ConstPtr vox) {return vox->GetStamp() < t; };

        lock_guard<mutex> lock(vox_mutex_);
        const auto iend = remove_if(vox_win_.begin(), vox_win_.end(), timeout);
        vox_win_.erase(iend, vox_win_.end());
        vox_win_.push_back(voxer);
    }
}