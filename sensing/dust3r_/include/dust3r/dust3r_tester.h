#ifndef __BNERF_DUST3R_TESTER_H__
#define __BNERF_DUST3R_TESTER_H__


#include <ros/ros.h>
#include <bnerf_msgs/PredictDUSt3R.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>


namespace bnerf
{
    class Dust3rTester
    {
        public:
        Dust3rTester(ros::NodeHandle &, const std::string &);
        void ImageCallBack(const sensor_msgs::Image &);
        
        private:
        ros::Publisher pub_;
        ros::Subscriber sub_;
        ros::ServiceClient cli_;
        bnerf_msgs::PredictDUSt3R::Request req_;
    };
}


#endif  // __BNERF_DUST3R_TESTER_H__
