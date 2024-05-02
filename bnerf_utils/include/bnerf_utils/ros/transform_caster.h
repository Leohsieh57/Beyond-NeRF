#ifndef __TF_CASTER_H_
#define __TF_CASTER_H_


#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <bnerf_utils/conversions.h>


namespace bnerf {
    class TransformCaster {
        public: 
        TransformCaster(const string &map_id = "map");

        void cast(const string &, const SE3d &, 
            const ros::Time &stamp = ros::Time::now());
            
        void cast(const string &, const geometry_msgs::Transform &, 
            const ros::Time &stamp = ros::Time::now());

        private:
        string map_id_;
        tf2_ros::TransformBroadcaster caster_;
    };
    
}

#endif