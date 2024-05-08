#include <tf/tf.h>
#include <bnerf_utils/conversions.h>
#include <glog/logging.h>


namespace bnerf {
    SE3d & convert(const geometry_msgs::Transform &msg, SE3d &trans) {
        convert(msg.translation, trans.translation());
        const auto &q = msg.rotation;
        trans.setQuaternion(Quatd(q.w, q.x, q.y, q.z));
        return trans;
    }


    SE3d & convert(const geometry_msgs::Pose &msg, SE3d &trans) {
        convert(msg.position, trans.translation());
        const auto &q = msg.orientation;
        trans.setQuaternion(Quatd(q.w, q.x, q.y, q.z));
        return trans;
    }


    geometry_msgs::Transform & convert(
        const SE3d &trans, geometry_msgs::Transform &msg) 
    {
        convert(trans.unit_quaternion(), msg.rotation);
        convert(trans.translation(), msg.translation);
        return msg;
    }


    geometry_msgs::Pose & convert(
        const SE3d &trans, geometry_msgs::Pose &msg)
    {
        convert(trans.unit_quaternion(), msg.orientation);
        convert(trans.translation(), msg.position);
        return msg;
    }


    geometry_msgs::Quaternion & convert(
        const Quatd &q, geometry_msgs::Quaternion &msg) 
    {
        tf::quaternionEigenToMsg(q, msg);
        return msg;
    }


    Quatd & convert(const geometry_msgs::Quaternion &msg, Quatd &q) {
        tf::quaternionMsgToEigen(msg, q);
        return q;
    }

    geometry_msgs::Point & convert(
        const Vec3d &pt, geometry_msgs::Point &msg) 
    {
        tf::pointEigenToMsg(pt, msg);
        return msg;
    }


    SE3d & convert(const Mat44f & input, SE3d & trans){
        const auto q = input.topLeftCorner<3, 3>();
        const auto t = input.topRightCorner<3, 1>();

        trans.setQuaternion(Quatd(q.cast<double>()));
        trans.translation() = t.cast<double>();

        return trans;
    }


    geometry_msgs::Vector3 & convert(
        const Vec3d &pt, geometry_msgs::Vector3 &msg) 
    {
        tf::vectorEigenToMsg(pt, msg);
        return msg;
    }


    Vec3d & convert(const geometry_msgs::Vector3 &msg,  Vec3d &pt) {
        tf::vectorMsgToEigen(msg, pt);
        return pt;
    }


    Vec3d & convert(const geometry_msgs::Point &msg,  Vec3d &pt) {
        tf::pointMsgToEigen(msg, pt);
        return pt;
    }


    geometry_msgs::Transform & convert(
        const Mat44f & trans, geometry_msgs::Transform & msg)
    {
        const auto q = trans.topLeftCorner<3, 3>();
        const auto t = trans.topRightCorner<3, 1>();
        convert(t.cast<double>(), msg.translation);
        convert(Quatd(q.cast<double>()), msg.rotation);
        return msg;
    }


    geometry_msgs::Point & convert(
        const PointXYZ &pt, geometry_msgs::Point &msg)
    {
        return convert(pt.getVector3fMap().cast<double>(), msg);
    }


    Mat44f & convert(const geometry_msgs::Transform & msg, Mat44f & trans){
        Eigen::Isometry3d output;
        tf::transformMsgToEigen(msg, output);
        return trans = output.matrix().cast<float>();
    }
    

    Mat44f & convert(const SE3d & trans, Mat44f & mat)
    {
        return mat = trans.matrix().cast<float>();
    }

    
    geometry_msgs::Pose & convert(
        const geometry_msgs::Transform &trans, 
        geometry_msgs::Pose &msg)
    {
        msg.orientation = trans.rotation;
        const auto &t = trans.translation;
        msg.position.x = t.x;
        msg.position.y = t.y;
        msg.position.z = t.z;
        return msg;
    }

}