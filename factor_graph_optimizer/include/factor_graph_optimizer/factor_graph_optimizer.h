#ifndef __BNERF_FACTOR_GRAPH_OPTIMIZER_H__
#define __BNERF_FACTOR_GRAPH_OPTIMIZER_H__

#include <ros/ros.h>
#include <bnerf_utils/bnerf_utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <bnerf_msgs/GraphUnaryEdge.h>
#include <bnerf_msgs/GraphBinaryEdge.h>
#include <bnerf_msgs/GraphIterationStatus.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <gtsam/nonlinear/Values.h>

namespace bnerf
{
    class FactorGraphOptimizer
    {
    public:
        FactorGraphOptimizer(ros::NodeHandle &);

    private:
        void UnaryEdgeCallBack(const bnerf_msgs::GraphUnaryEdge::ConstPtr &);
        void BinaryEdgeCallBack(const bnerf_msgs::GraphBinaryEdge::ConstPtr &);
        void ImuCallBack(const sensor_msgs::Imu::ConstPtr &);
        void OptimizeGraph(const ros::TimerEvent &);
        void PublishResults(const gtsam::Values &, const vector<ros::Time> &);

        static size_t GetKey(const ros::Time &, vector<ros::Time> &, map<ros::Time, size_t> &);

        ros::Publisher stat_pub_;
        tf2_ros::TransformBroadcaster caster_;

        ros::Duration win_span_;
        ros::Subscriber gps_sub_;
        mutex gps_mutex_;
        deque<bnerf_msgs::GraphUnaryEdge::ConstPtr> gps_win_;

        ros::Subscriber reg_sub_;
        mutex reg_mutex_;
        deque<bnerf_msgs::GraphBinaryEdge::ConstPtr> reg_win_;

        ros::Subscriber imu_sub_;
        mutex imu_mutex_;
        deque<sensor_msgs::Imu::ConstPtr> imu_win_;

        string map_frame_; //the reference frame we run gtsam on, i,e., the gps/imu frame
        string reg_frame_; //the lidar frame
        SE3d map_to_reg_; //transformation to the reference frame
        SE3d map_from_reg_;
        ros::Timer timer_;
    };
}

#endif // __BNERF_ODOM_VISUALIZER_H__
