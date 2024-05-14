#include <factor_graph_optimizer/factor_graph_optimizer.h>
#include <glog/logging.h>
#include <tf2_ros/transform_listener.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>


namespace bnerf
{
    FactorGraphOptimizer::FactorGraphOptimizer(ros::NodeHandle & nh)
        : stat_pub_(nh.advertise<bnerf_msgs::GraphIterationStatus>("graph_status", 10))
    {
        GET_REQUIRED(nh, "map_frame", map_frame_);
        GET_REQUIRED(nh, "lidar_frame", reg_frame_);

        double win_span;
        GET_REQUIRED(nh, "window_span", win_span);
        win_span_.fromSec(win_span);

        tf2_ros::Buffer buf;
        tf2_ros::TransformListener lis(buf);

        auto msg = buf.lookupTransform(map_frame_, reg_frame_, ros::Time(0), ros::Duration(20));
        map_to_reg_ = convert<SE3d>(msg.transform);
        map_from_reg_ = map_to_reg_.inverse();

        double rate;
        GET_REQUIRED(nh, "update_rate", rate);
        ros::Duration dt(1 / rate);
        timer_ = nh.createTimer(dt, &FactorGraphOptimizer::OptimizeGraph, this);

        gps_sub_ = nh.subscribe("binary_edge", 128, &FactorGraphOptimizer::BinaryEdgeCallBack, this);
        reg_sub_ = nh.subscribe("unary_edge", 1024, &FactorGraphOptimizer::UnaryEdgeCallBack, this);
        imu_sub_ = nh.subscribe("imu", 1024, &FactorGraphOptimizer::ImuCallBack, this);
    }


    size_t FactorGraphOptimizer::GetKey(const ros::Time & t, 
        vector<ros::Time> & key_to_stamp, map<ros::Time, size_t> & stamp_to_key)
    {
        LOG_ASSERT(key_to_stamp.size() == stamp_to_key.size());

        const auto key = key_to_stamp.size();
        const auto mit = stamp_to_key.emplace_hint(stamp_to_key.end(), t, key);

        if (key != stamp_to_key.size())
            key_to_stamp.push_back(t);

        return mit->second;
    }


    void FactorGraphOptimizer::PublishResults(const gtsam::Values & result, 
        const vector<ros::Time> & key_to_stamp)
    {
        bnerf_msgs::GraphIterationStatus status_msg;
        status_msg.graph_stamps.reserve(result.size());
        status_msg.graph_states.reserve(result.size());

        // Populate graph states for publishing
        for (const auto &key_value : result)
        {
            gtsam::Symbol key(key_value.key);
            gtsam::Pose3 pose = result.at<gtsam::Pose3>(key);

            geometry_msgs::Pose pose_msg;
            pose_msg.position.x = pose.x();
            pose_msg.position.y = pose.y();
            pose_msg.position.z = pose.z();
            auto quat = pose.rotation().toQuaternion();
            pose_msg.orientation.x = quat.x();
            pose_msg.orientation.y = quat.y();
            pose_msg.orientation.z = quat.z();
            pose_msg.orientation.w = quat.w();

            status_msg.graph_states.push_back(pose_msg);
            status_msg.graph_stamps.push_back(key_to_stamp[key]);
        }

        stat_pub_.publish(status_msg);

        geometry_msgs::TransformStamped msg;
        convert(convert<SE3d>(status_msg.graph_states.back()), msg.transform);
        msg.header.stamp = status_msg.graph_stamps.back();
        msg.header.frame_id = map_frame_;
        msg.child_frame_id = "map";

        caster_.sendTransform(msg);
    }


    void FactorGraphOptimizer::OptimizeGraph(const ros::TimerEvent &)
    {
        vector<bnerf_msgs::GraphUnaryEdge::ConstPtr> gps_msgs;
        vector<bnerf_msgs::GraphBinaryEdge::ConstPtr> reg_msgs;
        vector<sensor_msgs::Imu::ConstPtr> imu_msgs;

        {
            lock_guard<mutex> lock(gps_mutex_);
            gps_msgs.assign(gps_win_.begin(), gps_win_.end());
        }

        {
            lock_guard<mutex> lock(imu_mutex_);
            imu_msgs.assign(imu_win_.begin(), imu_win_.end());
        }

        {
            lock_guard<mutex> lock(reg_mutex_);
            reg_msgs.assign(reg_win_.begin(), reg_win_.end());
        }

        vector<ros::Time> key_to_stamp;
        map<ros::Time, size_t> stamp_to_key;

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values init_guess;

        auto noise = gtsam::noiseModel::Diagonal::Variances(Vec6d::Ones());
        for (const auto & reg : reg_msgs)
        {
            auto key1 = GetKey(reg->start_stamp, key_to_stamp, stamp_to_key);
            auto key2 = GetKey(reg->end_stamp, key_to_stamp, stamp_to_key);
            auto trans = bnerf::convert<bnerf::SE3d>(reg->mean);

            gtsam::BetweenFactor<gtsam::Pose3> factor(key1, key2, gtsam::Pose3(trans.matrix()), noise);
            graph.add(factor);
        }

        vector<Vec3d> key_to_vec;
        vector<Quatd> key_to_rot;
        for (const auto & t: key_to_stamp)
        {
            sensor_msgs::Imu::ConstPtr best_imu;
            double best_dt = numeric_limits<double>::max();
            for (const auto &msg : imu_msgs)
            {
                double dt = abs((t - msg->header.stamp).toSec());
                if (dt < best_dt)
                {
                    best_dt = dt;
                    best_imu = msg;
                }
            }

            bnerf_msgs::GraphUnaryEdge::ConstPtr best_gps;
            best_dt = numeric_limits<double>::max();
            for (const auto &msg : gps_msgs)
            {
                double dt = abs((t - msg->stamp).toSec());
                if (dt < best_dt)
                {
                    best_dt = dt;
                    best_gps = msg;
                }
            }

            if (!best_gps || !best_imu)
                return;

            convert(best_gps->mean, key_to_vec.emplace_back());
            convert(best_imu->orientation, key_to_rot.emplace_back());
        }

        
        gtsam::Values values;
        for (size_t key = 0; key < key_to_stamp.size(); key++)
        {
            const SE3d pose(key_to_rot[key], key_to_vec[key]);
            gtsam::Pose3 estim(pose.matrix());
            gtsam::PriorFactor<gtsam::Pose3> factor(key, estim, noise);
            graph.add(factor);
            init_guess.insert(key, estim);
        }

        if (graph.empty())
            return;

        gtsam::LevenbergMarquardtParams params;
        params.setVerbosityLM("SUMMARY");

        gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_guess, params);
        auto result = optimizer.optimize();
        PublishResults(result, key_to_stamp);
    }


    void FactorGraphOptimizer::UnaryEdgeCallBack(
        const bnerf_msgs::GraphUnaryEdge::ConstPtr & msg)
    {
        const auto min_t = msg->stamp - win_span_;

        lock_guard<mutex> lock(gps_mutex_);
        while (!gps_win_.empty() && gps_win_.front()->stamp < min_t)
            gps_win_.pop_front();

        gps_win_.push_back(msg);
    }


    void FactorGraphOptimizer::ImuCallBack(
        const sensor_msgs::Imu::ConstPtr & msg)
    {
        const auto min_t = msg->header.stamp - win_span_;

        lock_guard<mutex> lock(imu_mutex_);
        while (!imu_win_.empty() && imu_win_.front()->header.stamp < min_t)
            imu_win_.pop_front();

        imu_win_.push_back(msg);
    }

    void FactorGraphOptimizer::BinaryEdgeCallBack(
        const bnerf_msgs::GraphBinaryEdge::ConstPtr &msg)
    {
        SE3d odom = convert<SE3d>(msg->mean);
        odom = map_from_reg_ * odom * map_to_reg_;

        bnerf_msgs::GraphBinaryEdge::Ptr egde_msg;
        egde_msg.reset(new bnerf_msgs::GraphBinaryEdge);
        convert(odom, egde_msg->mean);
        egde_msg->start_stamp = msg->start_stamp;
        egde_msg->end_stamp = msg->end_stamp;

        Mat66d cov = Mat66d::Identity();
        copy_n(cov.data(), egde_msg->covariance.size(), egde_msg->covariance.data());

        const auto min_t = min(msg->start_stamp, msg->end_stamp) - win_span_;
        lock_guard<mutex> lock(reg_mutex_);
        while (!reg_win_.empty() && (reg_win_.front()->start_stamp < min_t || reg_win_.front()->end_stamp < min_t))
            reg_win_.pop_front();

        reg_win_.push_back(egde_msg);
    }
}