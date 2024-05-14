#ifndef __TRANSFORM_FINDER_H__
#define __TRANSFORM_FINDER_H__


#include <bnerf_utils/bnerf_utils.h>
#include <tf2_ros/transform_listener.h>

namespace bnerf
{
    class TransformFinder
    {
        public: 
        TransformFinder(ros::NodeHandle &);
        SE3d InterpolateTransform(const ros::Time &);

        //registrator stuffs
        private:
        string fix_frame_;
        string dyn_frame_;
        tf2_ros::Buffer tf_buf_;
        tf2_ros::TransformListener tf_lis_;
    };
}

#endif