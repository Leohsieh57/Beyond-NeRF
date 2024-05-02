#include <bnerf_utils/ros/transform_caster.h>


namespace bnerf {
    TransformCaster::TransformCaster(const string & map_id)
        : map_id_(map_id) {}


    void TransformCaster::cast(
        const string & scan_id, 
        const SE3d & trans, 
        const ros::Time &stamp)
    {
        const auto msg = convert<geometry_msgs::Transform>(trans);
        cast(scan_id, msg, stamp);
    }

    void TransformCaster::cast(
        const string &scan_id, 
        const geometry_msgs::Transform &trans, 
        const ros::Time &stamp) 
    {
        geometry_msgs::TransformStamped msg;
        msg.child_frame_id = scan_id;
        msg.header.frame_id = map_id_;
        msg.header.stamp = stamp;
        msg.transform = trans;
        caster_.sendTransform(msg);
    }
}

