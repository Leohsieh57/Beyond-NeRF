#include <glog/logging.h>
#include <bnerf_utils/conversions.h>
#include <dust3r_scale_finder/dust3r_scale_finder.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <bnerf_msgs/FloorDetectionInfo.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>


namespace bnerf
{
    DUSt3RScaleFinder::DUSt3RScaleFinder(ros::NodeHandle &nh)
        : src_cloud_sub_(nh, "dust3r_cloud", 16)
        , src_floor_sub_(nh, "dust3r_floor", 16)
        , sync_(src_cloud_sub_, src_floor_sub_, 128)
    {
        sync_.registerCallback(bind(&DUSt3RScaleFinder::SyncCallBack, this, _1, _2));
    }
    
    void DUSt3RScaleFinder::SyncCallBack(
        const CloudXYZ::ConstPtr &, const bnerf_msgs::FloorCoeffs::ConstPtr &)
    {

    }
}