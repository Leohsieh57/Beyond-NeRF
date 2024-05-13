#include <graph_edge_manager/graph_edge_manager.h>
#include <glog/logging.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <tf2_ros/transform_listener.h>


namespace bnerf
{
    GraphEdgeManager::GraphEdgeManager(ros::NodeHandle & nh)
        : edge_pub_(nh.advertise<bnerf_msgs::GraphEdgeCollection>("graph_edge_collection", 128))
    {
        GET_REQUIRED(nh, "map_frame", map_frame_);
        GET_REQUIRED(nh, "lidar_frame", reg_frame_);

        double win_span;
        GET_REQUIRED(nh, "window_span", win_span);
        win_span_.fromSec(win_span);

        tf2_ros::Buffer buf;
        tf2_ros::TransformListener lis(buf);

        auto msg = buf.lookupTransform(map_frame_, reg_frame_, ros::Time(0), ros::Duration(20));
        trans_ = convert<SE3d>(msg.transform);

        double rate;
        GET_REQUIRED(nh, "update_rate", rate);
        timer_ = nh.createTimer(ros::Duration(1 / rate), &GraphEdgeManager::PublishEdgeCollection, this);

        gps_sub_ = nh.subscribe("binary_edge", 128, &GraphEdgeManager::BinaryEdgeCallBack, this);
        reg_sub_ = nh.subscribe("unary_edge", 1024, &GraphEdgeManager::UnaryEdgeCallBack, this);
    }


    void GraphEdgeManager::PublishEdgeCollection(const ros::TimerEvent & e)
    {
        const auto t = e.current_real - win_span_;
        auto gps_timeout = [&t] (auto & msg) {return msg->stamp < t;};
        auto reg_timeout = [&t] (auto & msg) {return msg->start_stamp < t || msg->end_stamp < t;};

        bnerf_msgs::GraphEdgeCollection msg;
        {
            lock_guard<mutex> lock(gps_mutex_);
            auto iend = remove_if(gps_win_.begin(), gps_win_.end(), gps_timeout);
            gps_win_.erase(iend, gps_win_.end());
            if (gps_win_.empty())
                return;

            msg.unary_edges.reserve(gps_win_.size());
            for (const auto &e : gps_win_)
                msg.unary_edges.push_back(*e);
        }

        {
            lock_guard<mutex> lock(reg_mutex_);
            auto iend = remove_if(reg_win_.begin(), reg_win_.end(), reg_timeout);
            reg_win_.erase(iend, reg_win_.end());
            if (reg_win_.empty())
                return;

            msg.binary_edges.reserve(reg_win_.size());
            for (const auto &e : reg_win_)
                msg.binary_edges.push_back(*e);
        }

        edge_pub_.publish(msg);
    }


    void GraphEdgeManager::UnaryEdgeCallBack(
        const bnerf_msgs::GraphUnaryEdge::ConstPtr & msg)
    {
        lock_guard<mutex> lock(gps_mutex_);
        gps_win_.push_back(msg);
    }


    void GraphEdgeManager::BinaryEdgeCallBack(
        const bnerf_msgs::GraphBinaryEdge::ConstPtr &msg)
    {
        const SE3d ref_src = trans_;
        const SE3d ref_tgt = trans_;

        const SE3d tgt_src = convert<SE3d>(msg->mean);
        const SE3d trans = ref_tgt * tgt_src * ref_src.inverse();

        bnerf_msgs::GraphBinaryEdge::Ptr egde_msg;
        egde_msg.reset(new bnerf_msgs::GraphBinaryEdge);
        convert(trans, egde_msg->mean);
        egde_msg->start_stamp = msg->start_stamp;
        egde_msg->end_stamp = msg->end_stamp;

        Mat66d cov = Mat66d::Identity();
        copy_n(cov.data(), egde_msg->covariance.size(), egde_msg->covariance.data());

        lock_guard<mutex> lock(reg_mutex_);
        reg_win_.push_back(egde_msg);
    }
}