#include "ros/ros.h"
#include <bnerf_msgs/GraphBinaryEdge.h>
#include <bnerf_msgs/GraphUnaryEdge.h>
#include <bnerf_msgs/GraphIterationStatus.h> // edit as needed, the location needs to be changed
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
#include <chrono>

// listen for now you are gonna ignore my ugly code and lack of modularity. i'll refactor later <3 (i mixed tabs and spaces ok)
// timing variables
std::chrono::high_resolution_clock::time_point start_time, end_time;

gtsam::NonlinearFactorGraph graph;
gtsam::Values initial;
std::unordered_map<std::string, int> time_to_key_map;
int current_key = 0;
static int updatesSinceLastOptimization = 0;
static int iteration_count = 0;

// Publishers
ros::Publisher pose_pub;
ros::Publisher status_pub;
ros::Subscriber sub;
// Time variable for managing publication rate
ros::Time last_pub_time;

void transformCallback(const bnerf_msgs::GraphBinaryEdge::ConstPtr& msg) {
    std::string t1 = std::to_string(msg->header1.stamp.toSec());
    std::string t2 = std::to_string(msg->header2.stamp.toSec());

    // Convert ROS transform to GTSAM Pose3
    const auto &q = msg->transform.rotation;
    const auto &t = msg->transform.translation;
    gtsam::Pose3 pose(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
                      gtsam::Point3(t.x, t.y, t.z));

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
    //ROS_ERROR("Updates since last optimization first: %d", updatesSinceLastOptimization);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "transformation_listener");
    ros::NodeHandle nh;
    
    // subscribers and publishers
    sub = nh.subscribe("transformations", 1000, transformCallback);
    //pose_pub = nh.advertise<geometry_msgs::PoseStamped>("optimized_poses", 10);
    status_pub = nh.advertise<bnerf_msgs::GraphIterationStatus>("graph_status", 10);


    ros::Rate rate(10.0);
    while (ros::ok()) {
    	ros::spinOnce();
    	ROS_ERROR("Updates since last optimization: %d", updatesSinceLastOptimization);
    	start_time = std::chrono::high_resolution_clock::now(); // capture present time
	if (updatesSinceLastOptimization >= 5) {
	
	//if (ros::Time::now() - last_pub_time > ros::Duration(1.0)) { //placeholder condition to test
	    bnerf_msgs::GraphIterationStatus status_msg; // to be changed
	    ROS_ERROR("Condition met, running optimizer...");
	    
	    gtsam::LevenbergMarquardtParams params;
	    params.setVerbosityLM("SUMMARY");
	    //status_msg.iteration = iteration_count++; // we want to increment to publish
	    
	    status_msg.graph_stamps.reserve(100);  // reserve space if expecting many iterations (was getting an error)
	    status_msg.graph_states.reserve(100);

	    try {
                auto start_time = std::chrono::high_resolution_clock::now(); // start time capture

                gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
                gtsam::Values result = optimizer.optimize();

                auto end_time = std::chrono::high_resolution_clock::now();  // end time capture
                auto exec_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

                status_msg.exec_time = ros::Duration(exec_time.count() / 1000000.0); // convert to seconds
                status_msg.iteration = iteration_count++;

                // Populate graph states
                for (const auto& key_value : initial) {
                    gtsam::Symbol key(key_value.key);
                    gtsam::Pose3 pose = initial.at<gtsam::Pose3>(key);

                    geometry_msgs::Pose pose_msg;
                    pose_msg.position.x = pose.x();
                    pose_msg.position.y = pose.y();
                    pose_msg.position.z = pose.z();
                    auto quat = pose.rotation().toQuaternion();
                    pose_msg.orientation.x = quat.x();
                    pose_msg.orientation.y = quat.y();
                    pose_msg.orientation.z = quat.z();
                    pose_msg.orientation.w = quat.w();
                    status_msg.graph_states.push_back(pose_msg);
                }

                status_pub.publish(status_msg); // publish the status message
                ROS_ERROR("Optimizer has run. Final results below:");
                result.print("Final results\n");

                initial = result; // Uudate initial estimates with optimized values
                last_pub_time = ros::Time::now(); // update last publication time
                updatesSinceLastOptimization = 0;

            } catch (const std::exception& e) {
                ROS_ERROR("Exception during optimization: %s", e.what());
            }
        }

        rate.sleep();
    }
    return 0;
}

