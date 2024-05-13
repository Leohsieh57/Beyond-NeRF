#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <floor_detector/floor_detector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <bnerf_msgs/FloorDetectionInfo.h>
#include <bnerf_msgs/FloorDetection.h>


namespace bnerf
{
    FloorDetector::FloorDetector(ros::NodeHandle & nh)
        : info_pub_ (nh.advertise<bnerf_msgs::FloorDetectionInfo>("info", 16))
        , floor_pub_(nh.advertise<bnerf_msgs::FloorDetection>("floor_detection", 16))
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

        //scan_sub_ = nh.subscribe("input_scan", 32, &FloorDetector::ScanCallBack, this);
    }
}