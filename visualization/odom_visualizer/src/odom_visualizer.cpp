#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <odom_visualizer/odom_visualizer.h>
#include <glog/logging.h>
#include <bnerf_utils/bnerf_utils.h>
#include <pcl/common/transforms.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

namespace bnerf
{
    OdomVisualizer::OdomVisualizer(ros::NodeHandle &nh)
        : pose_pub_(nh.advertise<geometry_msgs::PoseArray>("odometry", 128))
        , path_pub_(nh.advertise<nav_msgs::Path>("trajectory", 128))
    {
        GET_REQUIRED(nh, "frame_id", frame_id_);
        odom_sub_ = nh.subscribe("input_odom", 16, &OdomVisualizer::OdomCallBack, this);

        double rate;
        GET_OPTIONAL(nh, "update_rate", rate, 30.0);

        const ros::Duration dt(1 / rate);
        timer_ = nh.createTimer(dt, &OdomVisualizer::PublishOdometry, this);
    }

    void OdomVisualizer::OdomCallBack(
        const geometry_msgs::TransformStamped &msg)
    {

        const auto odom = convert<SE3d>(msg.transform);
        LOG_ASSERT(frame_id_ == msg.header.frame_id);

        lock_guard<mutex> lock(odom_mutex_);
        odoms_.emplace_hint(odoms_.end(), msg.header.stamp, odom);

        while (odoms_.size() > 5000)
            odoms_.erase(odoms_.begin());
    }

    void OdomVisualizer::PublishOdometry(const ros::TimerEvent &)
    {
        const auto odoms = OdomVisualizer::GetOdometries();
        if (odoms.empty())
            return;

        vector<SE3d> trajs(1);
        for (const auto &odom : odoms)
            trajs.push_back(trajs.back() * odom);

        nav_msgs::Path traj_msg;
        traj_msg.header.frame_id = frame_id_;
        traj_msg.header.stamp = ros::Time::now();

        geometry_msgs::PoseArray odom_msg;
        odom_msg.header = traj_msg.header;

        for (const auto &traj : trajs)
        {
            auto &msg = traj_msg.poses.emplace_back();
            msg.header = traj_msg.header;

            convert(traj, msg.pose);
            odom_msg.poses.push_back(msg.pose);
            traj_msg.poses.emplace_back();
        }

        pose_pub_.publish(odom_msg);
        path_pub_.publish(traj_msg);
    }

    vector<SE3d> OdomVisualizer::GetOdometries()
    {
        vector<SE3d> odoms;

        lock_guard<mutex> lock(odom_mutex_);
        odoms.reserve(odoms_.size());

        for (const auto &[_, odom] : odoms_)
            odoms.push_back(odom);

        return odoms;
    }
}