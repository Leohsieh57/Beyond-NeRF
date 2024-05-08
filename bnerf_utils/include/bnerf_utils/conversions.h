#ifndef __UTILS_CONVERTER_H__
#define __UTILS_CONVERTER_H__


#include <Eigen/Core>
#include <bnerf_utils/typedef.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>


namespace bnerf {
    SE3d & convert(const Mat44f &, SE3d &);
    SE3d & convert(const geometry_msgs::Transform &, SE3d &);
    SE3d & convert(const geometry_msgs::Pose &, SE3d &);

    geometry_msgs::Quaternion & convert(const Quatd &, geometry_msgs::Quaternion &);
    Quatd & convert(const geometry_msgs::Quaternion &, Quatd &);
    
    geometry_msgs::Pose & convert(const geometry_msgs::Transform &, geometry_msgs::Pose &);
    geometry_msgs::Pose & convert(const SE3d &, geometry_msgs::Pose &);
    geometry_msgs::Transform & convert(const SE3d &, geometry_msgs::Transform &);
    geometry_msgs::Transform & convert(const Mat44f &, geometry_msgs::Transform &);

    Mat44f & convert(const geometry_msgs::Transform &, Mat44f &);
    Mat44f & convert(const SE3d &, Mat44f &);

    geometry_msgs::Point & convert(const Vec3d &, geometry_msgs::Point &);
    geometry_msgs::Point & convert(const PointXYZ &, geometry_msgs::Point &);
    geometry_msgs::Vector3 & convert(const Vec3d &, geometry_msgs::Vector3 &);

    Vec3d & convert(const geometry_msgs::Vector3 &, Vec3d&);
    Vec3d & convert(const geometry_msgs::Point &, Vec3d&);

    template<typename O, typename I> 
    O convert(const I &input) 
    {
        O output;
        return convert(input, output);
    }
}

#endif