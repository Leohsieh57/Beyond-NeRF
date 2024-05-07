#include <glog/logging.h>
#include <bnerf_utils/conversions.h>
#include <scan_matcher/scan_matcher.h>
#include <voxelizer/voxelizer_ndt.h>
#include <voxelizer/voxelizer_gicp.h>
#include <bnerf_msgs/GraphBinaryEdge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>


namespace bnerf
{
    ScanMatcher::ScanMatcher()
        : nh_("~")
    {
        GET_REQUIRED(nh_, "max_iters", max_iters_);
        GET_OPTIONAL(nh_, "epsilon", epsilon_, 1e-3);
        epsilon_ *= epsilon_;

        string solver;
        GET_REQUIRED(nh_, "solver", solver);

        use_gicp_ = solver == "gicp";
        if (!use_gicp_)
            LOG_ASSERT(solver == "ndt");

        edge_pub_ = nh_.advertise<bnerf_msgs::GraphBinaryEdge>("regist_binary_edge", 128);
        
        bool visualize;
        GET_OPTIONAL(nh_, "visualize", visualize, false);
        if (visualize)
        {
            src_pub_.reset(new ros::Publisher());
            tgt_pub_.reset(new ros::Publisher);
            *src_pub_ = nh_.advertise<CloudXYZ>("source_scan_matched", 128);
            *tgt_pub_ = nh_.advertise<CloudXYZ>("target_scan_matched", 128);
        }

        src_sub_ = nh_.subscribe("source_scan", 8, &ScanMatcher::SourceScanCallBack, this);
        tgt_sub_ = nh_.subscribe("target_scan", 8, &ScanMatcher::TargetScanCallBack, this);
    }


    void ScanMatcher::SourceScanCallBack(const CloudXYZ::ConstPtr & source)
    {
        const auto voxer = GetVoxelizer();
        if (!voxer)
            return;

        const auto target = voxer->GetInputTarget();
        if (target->header.stamp == source->header.stamp)
            return;

        SE3d init_guess;
        OptimData data(voxer, source);
        data.SetEstimation(init_guess);

        for (int i = 0; ros::ok() && i < max_iters_; i++)
        {
            data.best_.swap(data.temp_); //accept temp state
            data.AccumulateHessian(data.best_);

            Vec6d inc = data.H_.ldlt().solve(-data.b_);
            LOG_ASSERT(inc.allFinite());
            if (inc.squaredNorm() < epsilon_)
                break;

            SE3d eps = SE3d::exp(inc);
            data.SetEstimation(eps * data.best_->trans_);
        }

        PublishBinaryEdge(data);
        if (tgt_pub_)
            VisualizeAlignment(data);
    }


    void ScanMatcher::PublishBinaryEdge(const OptimData & data)
    {
        const auto target = data.voxer_->GetInputTarget();
        const auto source = data.source_;

        bnerf_msgs::GraphBinaryEdge msg;
        pcl_conversions::fromPCL(target->header.stamp, msg.stamp1);
        pcl_conversions::fromPCL(source->header.stamp, msg.stamp2);

        msg.type = bnerf_msgs::GraphBinaryEdge::LIDAR_TO_LIDAR;
        convert(data.best_->trans_, msg.transform);

        Eigen::SelfAdjointEigenSolver<Mat66d> solver;
        solver.compute(data.H_);

        Vec6d evals = solver.eigenvalues();
        evals = evals.cwiseMax(1e-3);
        LOG_ASSERT((evals.array() > 0).all());
        evals.normalize();

        const Mat66d &evecs = solver.eigenvectors();
        const Mat66d cov = evecs.transpose() * evals.asDiagonal() * evecs;
        copy_n(cov.data(), 36, msg.covariance.data());

        edge_pub_.publish(msg);
    }


    void ScanMatcher::VisualizeAlignment(const OptimData & data)
    {
        const auto target = data.voxer_->GetInputTarget();
        Mat44f trans = data.best_->trans_.matrix().cast<float>();

        CloudXYZ source;
        pcl::transformPointCloud(*data.source_, source, trans);
        source.header = target->header;
        src_pub_->publish(source);
        tgt_pub_->publish(*target);
    }


    Voxelizer::Ptr ScanMatcher::GetVoxelizer()
    {
        lock_guard<mutex> lock(vox_mutex_);
        return voxer_;
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
        lock_guard<mutex> lock(vox_mutex_);
        voxer_ = voxer;
    }
}