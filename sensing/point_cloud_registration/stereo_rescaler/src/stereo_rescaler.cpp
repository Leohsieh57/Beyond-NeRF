#include <bnerf_utils/bnerf_utils.h>
#include <stereo_rescaler/stereo_rescaler.h>
#include <tf2_ros/transform_listener.h>


namespace bnerf
{
    StereoRescaler::StereoRescaler(ros::NodeHandle &nh)
        : src_cloud_sub_(nh, "dust3r_cloud", 16)
        , src_floor_sub_(nh, "dust3r_floor", 16)
        , sync_sub_(src_cloud_sub_, src_floor_sub_, 128)
        , scan_pub_(nh.advertise<CloudXYZ>("rescaled_cloud", 32))
    {
        
        string lidar_frame, stereo_frame;
        GET_REQUIRED(nh, "lidar_frame", lidar_frame);
        GET_REQUIRED(nh, "stereo_frame", stereo_frame);

        double win_span;
        GET_REQUIRED(nh, "window_span", win_span);
        win_span_.fromSec(win_span);

        tf2_ros::Buffer buf;
        tf2_ros::TransformListener lis(buf);

        auto msg = buf.lookupTransform(stereo_frame, lidar_frame, ros::Time(0), ros::Duration(20));
        to_stereo_ = convert<SE3d>(msg.transform).inverse().matrix();
        sync_sub_.registerCallback(bind(&StereoRescaler::SyncedCallBack, this, _1, _2));
        tgt_floor_sub_ = nh.subscribe("lidar_floor", 16, &StereoRescaler::CoeffsCallBack, this);
    }

    
    void StereoRescaler::SyncedCallBack(const CloudXYZ::ConstPtr &cloud, 
        const bnerf_msgs::FloorCoeffs::ConstPtr & floor)
    {
        // LOG(INFO) << "cloud stamp: " << pcl_conversions::fromPCL(cloud->header.stamp);
        // LOG(INFO) << "floor stamp: " << floor->header.stamp;
        const auto best_floor = GetFloorCoeffs(floor->header.stamp);
        if (!best_floor)
            return;

        Vec4d coeffs(best_floor->coeffs.data());
        coeffs = to_stereo_.transpose() * coeffs;
        const double scale = abs(coeffs[3] / floor->coeffs[3]);

        CloudXYZ rescaled(*cloud);
        auto data = rescaled.getMatrixXfMap();
        data.topRows<3>() *= scale;
        scan_pub_.publish(rescaled);
    }


    bnerf_msgs::FloorCoeffs::ConstPtr StereoRescaler::GetFloorCoeffs(const ros::Time & t1)
    {
        double best_secs = numeric_limits<double>::max();
        bnerf_msgs::FloorCoeffs::ConstPtr best_floor;

        lock_guard<mutex> lock(win_mutex_);
        for (const auto & floor: floor_win_)
        {
            const auto & t2 = floor->header.stamp;
            if (t1 == t2)
                continue;

            const double secs = abs((t2-t1).toSec());
            if (secs < best_secs)
            {
                best_secs = secs;
                best_floor = floor;
            }
        }

        return best_floor;
    }


    void StereoRescaler::CoeffsCallBack(const bnerf_msgs::FloorCoeffs::ConstPtr & msg)
    {
        const auto min_t = msg->header.stamp - win_span_;
        auto timeout = [&min_t](bnerf_msgs::FloorCoeffs::ConstPtr msg) 
        {
            return msg->header.stamp < min_t; 
        };
        
        lock_guard<mutex> lock(win_mutex_);
        auto iend = remove_if(floor_win_.begin(), floor_win_.end(), timeout);
        floor_win_.erase(iend, floor_win_.end());
        floor_win_.push_back(msg);
    }
}