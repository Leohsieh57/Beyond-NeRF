#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <graph_visualizer/graph_visualizer.h>
#include <glog/logging.h>
#include <bnerf_utils/bnerf_utils.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

namespace bnerf
{
    GraphVisualizer::GraphVisualizer(ros::NodeHandle & nh)
        : scan_pub_(nh.advertise<CloudXYZI>("combined_map", 128))
        , pose_pub_(nh.advertise<geometry_msgs::PoseArray>("odometry", 128))
        , path_pub_(nh.advertise<nav_msgs::Path>("trajectory", 128))
    {
        tf2_ros::Buffer buf;
        tf2_ros::TransformListener lis(buf);
        
        auto msg = buf.lookupTransform("velo_link", "imu_link", ros::Time(0), ros::Duration(20));
        velo_to_imu_ = convert<SE3d>(msg.transform);

        stat_sub_ = nh.subscribe("graph_status", 16, &GraphVisualizer::GraphStatusCallBack, this);
        scan_sub_ = nh.subscribe("input_scan", 16, &GraphVisualizer::ScanCallBack, this);
    }


    void GraphVisualizer::GraphStatusCallBack(
        const bnerf_msgs::GraphIterationStatus & msg)
    {
        LOG_ASSERT(!msg.graph_stamps.empty());
        LOG_ASSERT(msg.graph_stamps.size() == msg.graph_states.size());


        nav_msgs::Path traj_msg;
        traj_msg.header.frame_id = "map";
        traj_msg.header.stamp = ros::Time::now();

        geometry_msgs::PoseArray odom_msg;
        odom_msg.header = traj_msg.header;

        for (const auto &state : msg.graph_states)
        {
            auto &msg = traj_msg.poses.emplace_back();
            msg.header = traj_msg.header;

            convert(convert<SE3d>(state), msg.pose);
            odom_msg.poses.push_back(msg.pose);
            traj_msg.poses.emplace_back();
        }

        pose_pub_.publish(odom_msg);
        path_pub_.publish(traj_msg);

        auto min_stamp = *min_element(msg.graph_stamps.begin(), msg.graph_stamps.end());

        vector<sensor_msgs::PointCloud2::ConstPtr> clouds(msg.graph_stamps.size());
        {
            lock_guard<mutex> lock(scan_mutex_);
            auto mit = scan_msgs_.find(min_stamp);
            if (mit != scan_msgs_.end())
                scan_msgs_.erase(scan_msgs_.begin(), mit);

            for (size_t i = 0; i < clouds.size(); i++)
            {
                const auto &t = msg.graph_stamps.at(i);
                const auto mit = scan_msgs_.find(t);
                if (mit != scan_msgs_.end())
                    clouds[i] = mit->second;
            }
        }

        if (all_of(clouds.begin(), clouds.end(), [](auto s) {return !s; }))
            return;

        CloudXYZI combined;
        for (size_t i = 0; i < clouds.size(); i++)
        {
            if (!clouds[i])
                continue;

            auto trans = convert<SE3d>(msg.graph_states[i]);
            trans = trans * velo_to_imu_;
            CloudXYZI submap;
            pcl::fromROSMsg(*clouds[i], submap);
            pcl::transformPointCloud(submap, submap, trans.matrix().cast<float>());

            for (auto & pt : submap)
                pt.intensity = i;

            combined += submap;
        }

        if (combined.empty())
            return;

        combined.header.frame_id = "map";
        combined.header.stamp = pcl_conversions::toPCL(ros::Time::now());
        scan_pub_.publish(combined);
    }


    void GraphVisualizer::ScanCallBack(const sensor_msgs::PointCloud2::ConstPtr & msg)
    {
        const auto &t = msg->header.stamp;
        lock_guard<mutex> lock(scan_mutex_);
        scan_msgs_.emplace_hint(scan_msgs_.end(), t, msg);
        while (scan_msgs_.size() > 5000)
            scan_msgs_.erase(scan_msgs_.begin());
    }
}