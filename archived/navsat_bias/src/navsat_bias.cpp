#include <navsat_bias/navsat_bias.h>
#include <glog/logging.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <tf2_ros/transform_listener.h>


namespace bnerf
{
    NavSatBias::NavSatBias(ros::NodeHandle & nh)
        : utm_bias_set_(false)
        , edge_pub_(nh.advertise<bnerf_msgs::GraphUnaryEdge>("gps_unary_edge", 128))
    {
        double cov_x, cov_y, cov_z;
        GET_REQUIRED(nh, "cov_x", cov_x);
        GET_REQUIRED(nh, "cov_y", cov_y);
        GET_REQUIRED(nh, "cov_z", cov_z);

        gps_cov_.setZero();
        gps_cov_.diagonal() << cov_x, cov_y, cov_z;

        gps_sub_ = nh.subscribe("navsat_fix", 1024, &NavSatBias::NavSatCallBack, this);
    }


    void NavSatBias::NavSatCallBack(const sensor_msgs::NavSatFix & gps_msg)
    {
        geographic_msgs::GeoPoint utm_msg;
        utm_msg.latitude = gps_msg.latitude;
        utm_msg.longitude = gps_msg.longitude;
        utm_msg.altitude = gps_msg.altitude;
        
        geodesy::UTMPoint utm;
        geodesy::fromMsg(utm_msg, utm);

        Vec3d xyz;
        xyz << utm.easting, utm.northing, utm.altitude;
        {
            lock_guard<mutex> lock(utm_bias_mutex_);
            if (!utm_bias_set_)
            {
                utm_bias_set_ = true;
                utm_bias_ = xyz;
            }
        }

        xyz -= utm_bias_;
        bnerf_msgs::GraphUnaryEdge edge_msg;
        edge_msg.stamp = gps_msg.header.stamp;
        convert(xyz, edge_msg.mean);
        copy_n(gps_cov_.data(), edge_msg.covariance.size(), edge_msg.covariance.data());
        edge_pub_.publish(edge_msg);
    }
}