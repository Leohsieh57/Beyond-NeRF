#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <floor_detector/floor_detector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <bnerf_msgs/FloorDetectionInfo.h>


namespace bnerf
{
    FloorDetector::FloorDetector(ros::NodeHandle & nh)
        : info_pub_(nh.advertise<bnerf_msgs::FloorDetectionInfo>("floor_detection_info", 16))
    {
        bool visualize;
        GET_OPTIONAL(nh, "visualize", visualize, false);
        GET_REQUIRED(nh, "num_threads", threads_);
        GET_REQUIRED(nh, "floor_dist_thres", dist_);
        GET_REQUIRED(nh, "floor_probability", prob_);

        if (visualize)
        {
            auto pub = nh.advertise<CloudXYZI>("floor_segmentation", 128);
            scan_pub_.reset(new ros::Publisher(move(pub)));
        }
    }


    Vec4d FloorDetector::ComputeFloorCoeffs(CloudXYZ::ConstPtr cloud) const
    {
        // RANSAC
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

        bnerf_msgs::FloorDetectionInfo msg;
        pcl_conversions::fromPCL(cloud->header, msg.header);
        msg.distance_threshold = ransac.getDistanceThreshold();
        msg.probability = ransac.getProbability();
        msg.exec_time = t2 - t1;
        msg.num_threads = ransac.getNumberOfThreads();
        msg.num_scan_points = cloud->size();
        info_pub_.publish(msg);

        return coeffs.cast<double>();
    }
}