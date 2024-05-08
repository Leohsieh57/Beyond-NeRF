#ifndef __UTILS_TYPEDEF_H__
#define __UTILS_TYPEDEF_H__


#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>


namespace bnerf {
    using boost::bind;
    using namespace boost::placeholders;
    using namespace std;

    using pcl::PointXYZ;
    using pcl::PointXYZI;
    typedef pcl::PointCloud<PointXYZ>  CloudXYZ;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    
    using Sophus::SE3d;
    using Sophus::SO3d;
    using Sophus::Sim3d;
    using visualization_msgs::Marker;
    using visualization_msgs::MarkerArray;
    
    typedef Eigen::VectorXd VecXd;
    typedef Eigen::Vector2d Vec2d;
    typedef Eigen::Vector3d Vec3d;
    typedef Eigen::Vector3f Vec3f;
    typedef Eigen::Vector2i Vec2i;
    typedef Eigen::Vector3i Vec3i;
    typedef Eigen::Vector4i Vec4i;
    typedef Eigen::Matrix3d Mat33d;
    typedef Eigen::Matrix4f Mat44f;
    typedef Eigen::Matrix3Xi Mat3Xi;
    typedef Eigen::Quaterniond Quatd;

    typedef Eigen::Matrix<double, 6, 1> Vec6d;
    typedef Eigen::Matrix<double, 7, 1> Vec7d;
    typedef Eigen::Matrix<double, 6, 6> Mat66d;
    typedef Eigen::Matrix<double, 6, 7> Mat67d;
    typedef Eigen::Matrix<double, 7, 7> Mat77d;
    typedef Eigen::Matrix<double, 7, 3> Mat73d;
    typedef Eigen::Matrix<double, 6, 3> Mat63d;
    typedef Eigen::Matrix<double, 6,-1> Mat6Xd;
    typedef Eigen::Matrix<double, 7,-1> Mat7Xd;
    typedef Eigen::Matrix<double, 3,-1> Mat3Xd;
    typedef Eigen::Matrix<double, 3, 4> Mat34d;
}

#endif