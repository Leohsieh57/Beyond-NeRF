#include <navsat_bias/navsat_bias.h>
#include <glog/logging.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <tf2_ros/transform_listener.h>


namespace bnerf
{
    NavSatBias::NavSatBias(ros::NodeHandle & nh)
        : edge_pub_(nh.advertise<bnerf_msgs::GraphUnaryEdge>("gps_unary_edge", 100))
    {
        string gps_frame;
        GET_REQUIRED(nh, "tgt_frame", tgt_frame_);
        GET_REQUIRED(nh, "gps_frame", gps_frame);

        double cov_x, cov_y, cov_z;
        GET_REQUIRED(nh, "cov_x", cov_x);
        GET_REQUIRED(nh, "cov_y", cov_y);
        GET_REQUIRED(nh, "cov_z", cov_z);

        cov_.setZero();
        cov_.diagonal() << cov_x, cov_y, cov_z;

        tf2_ros::Buffer buf;
        tf2_ros::TransformListener lis(buf);

        const auto msg = buf.lookupTransform(tgt_frame_, gps_frame, ros::Time(0), ros::Duration(5));
        const auto A = convert(msg.transform, gps_to_tgt_).rotationMatrix();
        cov_ = A * cov_ * A.transpose();

        gps_sub_ = nh.subscribe("navsat_fix", 100, &NavSatBias::NavSatCallBack, this);
    }


    void NavSatBias::NavSatCallBack(const sensor_msgs::NavSatFix & navsat_msg)
    {
        geographic_msgs::GeoPoint utm_msg;
        utm_msg.latitude = navsat_msg.latitude;
        utm_msg.longitude = navsat_msg.longitude;
        utm_msg.altitude = navsat_msg.altitude;
        
        geodesy::UTMPoint utm;
        geodesy::fromMsg(utm_msg, utm);

        Vec3d xyz(utm.easting, utm.northing, utm.altitude);
        {
            lock_guard<mutex> lock(utm_bias_mutex_);
            if (utm_bias_ == nullptr)
                utm_bias_.reset(new Vec3d(xyz));
        }

        xyz -= *utm_bias_;
        xyz = gps_to_tgt_ * xyz;

        bnerf_msgs::GraphUnaryEdge edge_msg;
        edge_msg.header.stamp = navsat_msg.header.stamp;
        edge_msg.header.frame_id = tgt_frame_;
        edge_msg.position.x = xyz.x();
        edge_msg.position.y = xyz.y();
        edge_msg.position.z = xyz.z();
        copy_n(cov_.data(), 9, edge_msg.covariance.data());

        edge_pub_.publish(edge_msg);
    }
}