#include <glog/logging.h>
#include <bnerf_utils/conversions.h>
#include <transform_finder/transform_finder.h>


namespace bnerf
{
    TransformFinder::TransformFinder(ros::NodeHandle & nh)
        : tf_buf_()
        , tf_lis_(tf_buf_)
    {
        GET_REQUIRED(nh, "fix_frame", fix_frame_);
        GET_REQUIRED(nh, "dynamic_frame", dyn_frame_);
    }


    SE3d TransformFinder::InterpolateTransform(const ros::Time & t)
    {
        const ros::Duration dt(0);
        geometry_msgs::TransformStamped dyn_to_fix_msg;
        try 
        {
            dyn_to_fix_msg = tf_buf_.lookupTransform(dyn_frame_, fix_frame_, t, dt);
        } 
        catch (tf2::TransformException & ex) 
        {
            LOG(ERROR) << ex.what();
            return SE3d();
        }

        return convert<SE3d>(dyn_to_fix_msg.transform);
    }
}