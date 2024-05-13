#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <floor_detector/floor_detector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <bnerf_msgs/FloorDetectionInfo.h>
#include <bnerf_msgs/FloorCoeffs.h>


namespace bnerf
{
    FloorDetector::FloorDetector(ros::NodeHandle & nh)
        : info_pub_ (nh.advertise<bnerf_msgs::FloorDetectionInfo>("info", 16))
        , floor_pub_(nh.advertise<bnerf_msgs::FloorCoeffs>("floor", 16))
    {
        bool visualize;
        GET_OPTIONAL(nh, "visualize", visualize, false);
        GET_REQUIRED(nh, "num_threads", threads_);
        GET_REQUIRED(nh, "dist_thres", dist_);
        GET_REQUIRED(nh, "probability", prob_);

        if (visualize)
        {
            auto pub = nh.advertise<CloudXYZI>("floor_segmentation", 128);
            scan_pub_.reset(new ros::Publisher(move(pub)));
        }

        scan_sub_ = nh.subscribe("input_scan", 32, &FloorDetector::ScanCallBack, this);
    }


    void FloorDetector::ScanCallBack(const CloudXYZ::ConstPtr & cloud) const
    {
        const auto t1 = ros::Time::now();
        pcl::SampleConsensusModelPlane<PointXYZ>::Ptr model;
        model.reset(new pcl::SampleConsensusModelPlane<PointXYZ>(cloud));

        pcl::RandomSampleConsensus<PointXYZ> ransac(model);
        ransac.setNumberOfThreads(threads_);
        ransac.setProbability(prob_);
        ransac.setDistanceThreshold(dist_);
        ransac.computeModel();
        
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        ransac.getInliers(inliers->indices);

        VecXf coeffs;
        ransac.getModelCoefficients(coeffs);
        const auto t2 = ros::Time::now();

        bnerf_msgs::FloorCoeffs floor_msg;
        pcl_conversions::fromPCL(cloud->header, floor_msg.header);

        for (int i = 0; i < 4; i++)
            floor_msg.coeffs[i] = coeffs[i];

        floor_pub_.publish(floor_msg);

        if (scan_pub_)
        {
            CloudXYZI seg_cloud;
            pcl::copyPointCloud(*cloud, seg_cloud);
            for (auto &pt : seg_cloud)
                pt.intensity = 1;

            for (const int & i : inliers->indices)
                seg_cloud[i].intensity = 0;
            
            seg_cloud.front().intensity = 2;
            seg_cloud.header = cloud->header;
            scan_pub_->publish(seg_cloud);
        }

        bnerf_msgs::FloorDetectionInfo info_msg;
        info_msg.header = floor_msg.header;
        info_msg.distance_threshold = ransac.getDistanceThreshold();
        info_msg.exec_time = t2 - t1;
        info_msg.num_scan_points = cloud->size();
        info_msg.probability = ransac.getProbability();
        info_msg.num_threads = ransac.getNumberOfThreads();
        info_pub_.publish(info_msg);
    }
}