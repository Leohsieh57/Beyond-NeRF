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
    {
        GET_REQUIRED(nh_, "max_iters", max_iters_);
        GET_OPTIONAL(nh_, "verbose", verbose_, false);
        GET_OPTIONAL(nh_, "epsilon", epsilon_, 1e-3);
        epsilon_ *= epsilon_;

        string solver;
        GET_REQUIRED(nh_, "solver", solver);
        LOG_ASSERT(solver == "ndt" || solver == "gicp");
        use_gicp_ = solver == "gicp";

        double tgt_timeout;
        GET_REQUIRED(nh_, "target_timeout", tgt_timeout);
        tgt_timeout_.fromSec(tgt_timeout);

        bool visualize;
        GET_OPTIONAL(nh_, "visualize", visualize, false);
        if (visualize)
        {
            auto viz_pub = nh_.advertise<CloudXYZI>("combined_scan", 1);
            viz_pub_.reset(new ros::Publisher(move(viz_pub)));
        }

        edge_pub_= nh_.advertise<bnerf_msgs::GraphBinaryEdge>("regist_binary_edge", 16);
        src_sub_ = nh_.subscribe("source_scan", 16, &ScanMatcher::SourceScanCallBack, this);
        tgt_sub_ = nh_.subscribe("target_scan", 16, &ScanMatcher::TargetScanCallBack, this);
    }


    void ScanMatcher::SourceScanCallBack(const CloudXYZ::ConstPtr & source)
    {
        const auto voxer = GetVoxelizer(source);
        if (!voxer)
            return;

        const auto target = voxer->GetInputTarget();
        if (target->header.stamp == source->header.stamp)
            return;

        if (verbose_)
            LOG(WARNING) << "start optim. scan size: " << source->size();

        SE3d init_guess;
        Optimizer optim(voxer, source);

        optim.SetEstimation(init_guess);
        for (int i = 0; ros::ok() && i < max_iters_; i++)
        {
            optim.AccumulateHessian();

            if (verbose_)
                LOG(INFO) << setprecision(12)
                    << "iterarion: " << i
                    << "\tloss: " << optim.loss_
                    << "\tvalid points: " << optim.valid_ids_.size();

            Vec6d inc = optim.H_.ldlt().solve(-optim.b_);
            LOG_ASSERT(inc.allFinite());
            if (inc.squaredNorm() < epsilon_)
                break;

            SE3d eps = SE3d::exp(inc);
            optim.SetEstimation(eps * optim.trans_);
        }

        edge_pub_.publish(GetBinaryEdge(optim));
        if (viz_pub_)
            viz_pub_->publish(GetCombinedScan(optim));

    }


    CloudXYZI ScanMatcher::GetCombinedScan(
        const Optimizer & optim) const
    {
        LOG_ASSERT(viz_pub_);
        const auto target = optim.voxer_->GetInputTarget();
        Mat44f trans = optim.trans_.matrix().cast<float>();

        
        CloudXYZ source;
        pcl::transformPointCloud(*optim.source_, source, trans);
        const size_t num_source = source.size();

        CloudXYZI msg;
        pcl::copyPointCloud(source += *target, msg);
        for (size_t i = 0; i < msg.size(); i++)
            msg[i].intensity = i < num_source;

        msg.header = optim.source_->header;
        return msg;
    }


    bnerf_msgs::GraphBinaryEdge ScanMatcher::GetBinaryEdge(
        const Optimizer & optim) const
    {
        const auto target = optim.voxer_->GetInputTarget();
        const auto source = optim.source_;

        bnerf_msgs::GraphBinaryEdge msg;
        pcl_conversions::fromPCL(target->header, msg.header1);
        pcl_conversions::fromPCL(source->header, msg.header2);
        convert(optim.trans_, msg.transform);

        Eigen::SelfAdjointEigenSolver<Mat66d> solver;
        solver.compute(optim.H_);

        Vec6d evals = solver.eigenvalues();
        evals = evals.cwiseMax(1e-8);
        LOG_ASSERT((evals.array() > 0).all());
        evals.normalize();

        const Mat66d &evecs = solver.eigenvectors();
        const Mat66d cov = evecs.transpose() * evals.asDiagonal() * evecs;
        copy_n(cov.data(), 36, msg.covariance.data());

        return msg;
    }


    Voxelizer::ConstPtr ScanMatcher::GetVoxelizer(CloudXYZ::ConstPtr source)
    {
        ros::Time t2;
        pcl_conversions::fromPCL(source->header.stamp, t2);

        Voxelizer::ConstPtr best_voxer;
        double best_secs = numeric_limits<double>::max();

        lock_guard<mutex> lock(vox_mutex_);
        for (const auto & voxer: voxers_)
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


    void ScanMatcher::TargetScanCallBack(const CloudXYZ::ConstPtr & target)
    {
        LOG_ASSERT(target);

        Voxelizer::Ptr voxer;
        if (use_gicp_)
            voxer.reset(new VoxelizerGICP(nh_));
        else
            voxer.reset(new VoxelizerNDT(nh_));

        voxer->SetInputTarget(target);
        auto t = voxer->GetStamp() - tgt_timeout_;
        auto timeout = [&t](Voxelizer::ConstPtr vox) {return vox->GetStamp() < t; };

        lock_guard<mutex> lock(vox_mutex_);
        const auto iend = remove_if(voxers_.begin(), voxers_.end(), timeout);
        voxers_.erase(iend, voxers_.end());
        voxers_.push_back(voxer);
    }
}