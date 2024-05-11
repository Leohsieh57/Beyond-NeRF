#ifndef __BNERF_GRAPH_EDGE_MANAGER_H__
#define __BNERF_GRAPH_EDGE_MANAGER_H__


#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <bnerf_msgs/ScanMatchingFactor.h>
#include <bnerf_utils/bnerf_utils.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <bnerf_msgs/GraphEdgeCollection.h>


namespace bnerf
{
    class GraphEdgeManager
    {
        public:
        GraphEdgeManager(ros::NodeHandle & );
        void GetAllEdges();
        
        private:
        void NavSatCallBack(const sensor_msgs::NavSatFix &);
        void ScanMatchingCallBack(const bnerf_msgs::ScanMatchingFactor &);
        
        ros::Publisher edge_pub_;

        bool utm_bias_set_;
        Vec3d utm_bias_;
        mutex utm_bias_mutex_;
        Mat33d cov_;
        ros::Duration win_span_;

        ros::Subscriber gps_sub_;
        mutex gps_win_mutex_;
        vector<pair<ros::Time, Vec3d>> gps_win_;

        ros::Subscriber reg_sub_;
        mutex reg_win_mutex_;
        vector<pair<ros::Time, bnerf_msgs::GraphBinaryEdge>> reg_win_;

        string ref_frame_; //the reference frame we run gtsam on, i,e., the gps/imu frame
        map<string, SE3d> trans_; //transformation to the reference frame
    };
}

#endif // __BNERF_GRAPH_EDGE_MANAGER_H__
