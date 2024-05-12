#include "ros/ros.h"
#include <bnerf_msgs/GraphBinaryEdge.h>
#include <bnerf_msgs/GraphEdgeCollection.h>
#include <bnerf_msgs/GraphIterationStatus.h> // edit as needed, the location needs to be changed
#include "gtsam/geometry/Pose3.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/BetweenFactor.h"
#include <unordered_map>
#include <string>
#include <glog/logging.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include "bnerf_msgs/GraphEdgeCollection.h"
#include "bnerf_msgs/GraphBinaryEdge.h"
#include "bnerf_msgs/GraphUnaryEdge.h"
#include <cmath> // For std::abs
#include <limits> // For std::numeric_limits
#include <map>
#include <string>
// visualization stuff
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <chrono>

// listen for now you are gonna ignore my ugly code and lack of modularity. i'll refactor later <3 (i mixed tabs and spaces ok)
// timing variables
std::chrono::high_resolution_clock::time_point start_time, end_time;
double totalError = 0.0;
int count = 0;

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


double calculateError(const geometry_msgs::Transform& t1, const geometry_msgs::Vector3& t2) {
    double dx = t1.translation.x - t2.x;
    double dy = t1.translation.y - t2.y;
    double dz = t1.translation.z - t2.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

void EdgeCallBack(const bnerf_msgs::GraphEdgeCollection::ConstPtr& msg) {
    //LOG(INFO) << "received " << msg->binary_edges.size()
    //    << " binary edges and " << msg->unary_edges.size() << " unary edges";

    for (const auto& binary_edge : msg->binary_edges) {
        const bnerf_msgs::GraphUnaryEdge* closest_unary_edge = nullptr;
        double min_time_diff = std::numeric_limits<double>::infinity();

        for (const auto& unary_edge : msg->unary_edges) {
            double time_diff = std::abs(binary_edge.target_stamp.toSec() - unary_edge.stamp.toSec());
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_unary_edge = &unary_edge;
            }
        }

        if (closest_unary_edge) {
            //LOG(INFO) << "Closest unary edge found with time diff: " << min_time_diff;
            double error = calculateError(binary_edge.mean, closest_unary_edge->mean);
            totalError += error;
            count++;

            std::string binary_timestamp = std::to_string(binary_edge.target_stamp.toSec());
            std::string unary_timestamp = std::to_string(closest_unary_edge->stamp.toSec());

            int key_t1, key_t2;
            if (time_to_key_map.find(binary_timestamp) == time_to_key_map.end()) {
                key_t1 = current_key++;
                time_to_key_map[binary_timestamp] = key_t1;
                initial.insert(gtsam::Symbol('x', key_t1), gtsam::Pose3());
            } else {
                key_t1 = time_to_key_map[binary_timestamp];
            }

            if (time_to_key_map.find(unary_timestamp) == time_to_key_map.end()) {
                key_t2 = current_key++;
                time_to_key_map[unary_timestamp] = key_t2;
                initial.insert(gtsam::Symbol('x', key_t2), gtsam::Pose3());
            } else {
                key_t2 = time_to_key_map[unary_timestamp];
            }

            auto model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(0.1));
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', key_t1), gtsam::Symbol('x', key_t2), gtsam::Pose3()));
        }
    }
    if (count > 0) {
      double averageError = totalError / count;
      LOG(INFO) << "Average Error: " << averageError;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "transformation_listener");
    ros::NodeHandle nh;
    
    // subscribers and publishers
    sub = nh.subscribe("/graph_edge_manager_node/graph_edge_collection", 1000, EdgeCallBack);
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
                
                // Reset cumulative error calculation after successful optimization
                totalError = 0.0;
                count = 0;

            } catch (const std::exception& e) {
                ROS_ERROR("Exception during optimization: %s", e.what());
            }
        }

        rate.sleep();
    }
    return 0;
}

