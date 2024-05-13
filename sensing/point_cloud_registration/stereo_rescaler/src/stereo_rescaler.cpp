#include <glog/logging.h>
#include <bnerf_utils/conversions.h>
#include <stereo_rescaler/stereo_rescaler.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <bnerf_msgs/FloorDetectionInfo.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>


namespace bnerf
{
    StereoRescaler::StereoRescaler(ros::NodeHandle &nh)
        : src_cloud_sub_(nh, "dust3r_cloud", 16)
        , src_floor_sub_(nh, "dust3r_floor", 16)
        , sync_(src_cloud_sub_, src_floor_sub_, 128)
    {
        sync_.registerCallback(bind(&StereoRescaler::SyncedCallBack, this, _1, _2));
    }
    
    void StereoRescaler::SyncedCallBack(const CloudXYZ::ConstPtr &cloud, 
        const bnerf_msgs::FloorCoeffs::ConstPtr & floor)
    {
        LOG(INFO) << "cloud stamp: " << pcl_conversions::fromPCL(cloud->header.stamp);
        LOG(INFO) << "floor stamp: " << floor->header.stamp;
    }
}