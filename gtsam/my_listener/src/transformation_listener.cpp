#include "ros/ros.h"
#include "my_listener/TimeTransform.h" // Include your custom message header
#include "gtsam/geometry/Pose3.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/BetweenFactor.h"
#include <unordered_map>
#include <string>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
// visualization stuff
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"



gtsam::NonlinearFactorGraph graph;
gtsam::Values initial;
std::unordered_map<std::string, int> time_to_key_map;
int current_key = 0;
static int updatesSinceLastOptimization = 0;
void transformCallback(const my_listener::TimeTransform::ConstPtr& msg) {
    std::string t1 = std::to_string(msg->t1.toSec());
    std::string t2 = std::to_string(msg->t2.toSec());

    // Convert ROS transform to GTSAM Pose3
    gtsam::Pose3 pose(gtsam::Rot3::Quaternion(msg->geo_trans.rotation.w,
                                              msg->geo_trans.rotation.x,
                                              msg->geo_trans.rotation.y,
                                              msg->geo_trans.rotation.z),
                      gtsam::Point3(msg->geo_trans.translation.x,
                                    msg->geo_trans.translation.y,
                                    msg->geo_trans.translation.z));

    int key_t1, key_t2;
    // Check if t1 and t2 have corresponding keys, if not -- add them
    if (time_to_key_map.find(t1) == time_to_key_map.end()) {
        key_t1 = current_key++;
        time_to_key_map[t1] = key_t1;
        initial.insert(gtsam::Symbol('x', key_t1), gtsam::Pose3());  // Default initialize
    } else {
        key_t1 = time_to_key_map[t1];
    }

    if (time_to_key_map.find(t2) == time_to_key_map.end()) {
        key_t2 = current_key++;
        time_to_key_map[t2] = key_t2;
        initial.insert(gtsam::Symbol('x', key_t2), pose);
    } else {
        key_t2 = time_to_key_map[t2];
    }

    // Create a between factor between t1 and t2
    auto model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(0.1));
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', key_t1), gtsam::Symbol('x', key_t2), pose, model));
    updatesSinceLastOptimization++;
    ROS_ERROR("Updates since last optimization first: %d", updatesSinceLastOptimization);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "transformation_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("transformations", 1000, transformCallback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("optimized_poses", 10);


    ros::Rate rate(10.0);
    while (ros::ok()) {
    	ros::spinOnce();
    	ROS_ERROR("Updates since last optimization: %d", updatesSinceLastOptimization);
	if (updatesSinceLastOptimization >= 5) {
		ROS_ERROR("Condition met, running optimizer...");
		gtsam::LevenbergMarquardtParams params;
	        params.setVerbosityLM("SUMMARY");
    		try {
       	    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
		    gtsam::Values result = optimizer.optimize();
		    ROS_ERROR("Optimizer has run. Final results below:");
		    result.print("Final results\n");
		    initial = result; // Update initial estimates with optimized values

        	// Convert and publish each pose in the result
        	for (const auto& key_value : result) {
                    gtsam::Symbol key(key_value.key);
                    gtsam::Pose3 pose = result.at<gtsam::Pose3>(key);

                    geometry_msgs::PoseStamped pose_msg;
                    pose_msg.header.frame_id = "world"; // or your desired frame
                    pose_msg.header.stamp = ros::Time::now();
                    pose_msg.pose.position.x = pose.x();
                    pose_msg.pose.position.y = pose.y();
                    pose_msg.pose.position.z = pose.z();
                    auto quat = pose.rotation().toQuaternion();
                    pose_msg.pose.orientation.x = quat.x();
                    pose_msg.pose.orientation.y = quat.y();
                    pose_msg.pose.orientation.z = quat.z();
                    pose_msg.pose.orientation.w = quat.w();
                pose_pub.publish(pose_msg);
                }

	    } catch (const std::exception& e) {
		ROS_ERROR("Exception during optimization: %s", e.what());
	    }
	    updatesSinceLastOptimization = 0;
	}


    	rate.sleep();
    }
    return 0;
}

