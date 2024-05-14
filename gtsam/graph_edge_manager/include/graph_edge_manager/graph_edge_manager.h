#ifndef __BNERF_GRAPH_EDGE_MANAGER_H__
#define __BNERF_GRAPH_EDGE_MANAGER_H__


#include <ros/ros.h>
#include <bnerf_utils/bnerf_utils.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>
#include <bnerf_msgs/GraphEdgeCollection.h>


namespace bnerf
{
    class GraphEdgeManager
    {
        public:
        GraphEdgeManager(ros::NodeHandle & );
        void GetAllEdges();
        
        private:
        void PublishEdgeCollection(const ros::TimerEvent &);
        void UnaryEdgeCallBack(const bnerf_msgs::GraphUnaryEdge::ConstPtr &);
        void BinaryEdgeCallBack(const bnerf_msgs::GraphBinaryEdge::ConstPtr &);
        
        ros::Publisher edge_pub_;
        ros::Timer timer_;
        ros::ServiceClient cli_;

        ros::Duration win_span_;
        ros::Subscriber gps_sub_;
        mutex gps_mutex_;
        vector<bnerf_msgs::GraphUnaryEdge::ConstPtr> gps_win_;

        ros::Subscriber reg_sub_;
        mutex reg_mutex_;
        vector<bnerf_msgs::GraphBinaryEdge::ConstPtr> reg_win_;

        string map_frame_; //the reference frame we run gtsam on, i,e., the gps/imu frame
        string reg_frame_; //the lidar frame
        SE3d trans_; //transformation to the reference frame
    };
}

#endif // __BNERF_GRAPH_EDGE_MANAGER_H__
