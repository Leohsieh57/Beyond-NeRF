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
        , utm_bias_set_(false)
    {
        GET_REQUIRED(nh, "reference_frame", ref_frame_);
        
        double cov_x, cov_y, cov_z;
        GET_REQUIRED(nh, "gps/cov_x", cov_x);
        GET_REQUIRED(nh, "gps/cov_y", cov_y);
        GET_REQUIRED(nh, "gps/cov_z", cov_z);

        gps_cov_.setZero();
        gps_cov_.diagonal() << cov_x, cov_y, cov_z;

        double win_span;
        GET_REQUIRED(nh, "window_span", win_span);
        win_span_.fromSec(win_span);

        vector<string> frames;
        GET_REQUIRED(nh, "source_frames", frames);

        tf2_ros::Buffer buf;
        tf2_ros::TransformListener lis(buf);
        for (const auto & frame : frames)
        {
            auto msg = buf.lookupTransform(ref_frame_, frame, ros::Time(0), ros::Duration(20));
            trans_[frame] = convert<SE3d>(msg.transform);
        }

        
        double rate;
        GET_REQUIRED(nh, "update_rate", rate);
        timer_ = nh.createTimer(ros::Duration(1 / rate), &GraphEdgeManager::EdgeCollectionCallBack, this);

        gps_sub_ = nh.subscribe("navsat_fix", 1024, &GraphEdgeManager::NavSatCallBack, this);
        reg_sub_ = nh.subscribe("scan_matching_factor", 32, &GraphEdgeManager::ScanMatchingCallBack, this);
    }


    void GraphEdgeManager::EdgeCollectionCallBack(const ros::TimerEvent & e)
    {
        const auto min_t = e.current_real - win_span_;
        auto timeout = [&min_t] (const auto &p) {return p.first < min_t; };

        bnerf_msgs::GraphEdgeCollection msg;
        {
            lock_guard<mutex> lock(gps_win_mutex_);
            auto iend = remove_if(gps_win_.begin(), gps_win_.end(), timeout);
            gps_win_.erase(iend, gps_win_.end());

            msg.unary_edges.reserve(gps_win_.size());
            for (const auto &[stamp, xyz]: gps_win_)
            {
                auto &e = msg.unary_edges.emplace_back();
                e.stamp = stamp;
                convert(xyz, e.mean);
            }
        }


        {
            lock_guard<mutex> lock(reg_win_mutex_);
            auto iend = remove_if(reg_win_.begin(), reg_win_.end(), timeout);
            reg_win_.erase(iend, reg_win_.end());

            msg.binary_edges.reserve(reg_win_.size());
            for (const auto &[_, e]: reg_win_)
                msg.binary_edges.push_back(e);
        }

        for (auto &e : msg.unary_edges)
            copy_n(gps_cov_.data(), e.covariance.size(), e.covariance.data());

        edge_pub_.publish(msg);
    }


    void GraphEdgeManager::NavSatCallBack(const sensor_msgs::NavSatFix & gps_msg)
    {
        LOG_ASSERT(gps_msg.header.frame_id == ref_frame_);

        geographic_msgs::GeoPoint utm_msg;
        utm_msg.latitude = gps_msg.latitude;
        utm_msg.longitude = gps_msg.longitude;
        utm_msg.altitude = gps_msg.altitude;
        
        geodesy::UTMPoint utm;
        geodesy::fromMsg(utm_msg, utm);

        Vec3d xyz;
        xyz << utm.easting, utm.northing, utm.altitude;
        {
            lock_guard<mutex> lock(utm_bias_mutex_);
            if (!utm_bias_set_)
            {
                utm_bias_set_ = true;
                utm_bias_ = xyz;
            }
        }

        xyz -= utm_bias_;

        lock_guard<mutex> lock(gps_win_mutex_);
        gps_win_.emplace_back(gps_msg.header.stamp, xyz);
    }


    void GraphEdgeManager::ScanMatchingCallBack(
        const bnerf_msgs::ScanMatchingFactor & msg)
    {
        LOG_ASSERT(trans_.count(msg.source_header.frame_id));
        LOG_ASSERT(trans_.count(msg.target_header.frame_id));

        const SE3d ref_src = trans_[msg.source_header.frame_id];
        const SE3d ref_tgt = trans_[msg.target_header.frame_id];

        const SE3d tgt_src = convert<SE3d>(msg.transform);
        const SE3d trans = ref_tgt * tgt_src * ref_src.inverse();

        bnerf_msgs::GraphBinaryEdge egde_msg;
        convert(trans, egde_msg.mean);
        egde_msg.source_stamp = msg.source_header.stamp;
        egde_msg.target_stamp = msg.target_header.stamp;

        Mat66d cov = Mat66d::Identity();
        copy_n(cov.data(), 36, egde_msg.covariance.data());

        const auto t = min(egde_msg.source_stamp, egde_msg.target_stamp);

        lock_guard<mutex> lock(reg_win_mutex_);
        reg_win_.emplace_back(t, egde_msg);
    }
}