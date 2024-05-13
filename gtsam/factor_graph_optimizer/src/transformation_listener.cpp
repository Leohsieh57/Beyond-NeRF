#include <ros/ros.h>
#include <bnerf_msgs/GraphBinaryEdge.h>
#include <bnerf_msgs/GraphEdgeCollection.h>
#include <bnerf_msgs/GraphUnaryEdge.h>
#include <bnerf_msgs/GraphIterationStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <unordered_map>
#include <string>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <chrono>
#include <glog/logging.h>

std::mutex initial_mutex;
gtsam::Values initial;
gtsam::NonlinearFactorGraph graph; // global graph
std::unordered_map<std::string, int> time_to_key_map;
int current_key = 0;
double totalError = 0.0;
int count = 0;
static int updatesSinceLastOptimization = 0;
static int iteration_count = 0;

ros::Publisher status_pub;
ros::Time last_pub_time;

double calculateError(const geometry_msgs::Transform& t1, const geometry_msgs::Vector3& t2) {
    double dx = t1.translation.x - t2.x;
    double dy = t1.translation.y - t2.y;
    double dz = t1.translation.z - t2.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

void EdgeCallBack(const bnerf_msgs::GraphEdgeCollection::ConstPtr& msg) {
    gtsam::NonlinearFactorGraph localGraph;
    gtsam::Values localInitial;

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
            double error = calculateError(binary_edge.mean, closest_unary_edge->mean);
            totalError += error;
            count++;

            std::string binary_timestamp = std::to_string(binary_edge.target_stamp.toSec());
            std::string unary_timestamp = std::to_string(closest_unary_edge->stamp.toSec());

            int key_t1, key_t2;
            if (time_to_key_map.find(binary_timestamp) == time_to_key_map.end()) {
                key_t1 = current_key++;
                time_to_key_map[binary_timestamp] = key_t1;
                localInitial.insert(gtsam::Symbol('x', key_t1), gtsam::Pose3());
            } else {
                key_t1 = time_to_key_map[binary_timestamp];
            }

            if (time_to_key_map.find(unary_timestamp) == time_to_key_map.end()) {
                key_t2 = current_key++;
                time_to_key_map[unary_timestamp] = key_t2;
                localInitial.insert(gtsam::Symbol('x', key_t2), gtsam::Pose3());
            } else {
                key_t2 = time_to_key_map[unary_timestamp];
            }

            auto model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(0.1));
            localGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', key_t1), gtsam::Symbol('x', key_t2), gtsam::Pose3(), model));
        }
    }
    if (!localGraph.empty()) {
        std::lock_guard<std::mutex> lock(initial_mutex);
        graph.push_back(localGraph);
        initial.insert(localInitial);
        updatesSinceLastOptimization++;
    }

    if (count > 0) {
        double averageError = totalError / count;
        LOG(INFO) << "Average Error: " << averageError;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "transformation_listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/graph_edge_manager_node/graph_edge_collection", 1000, EdgeCallBack);
    status_pub = nh.advertise<bnerf_msgs::GraphIterationStatus>("graph_status", 10);

    ros::Rate rate(10.0);
    while (ros::ok()) {
        ros::spinOnce();
        ROS_ERROR("Updates since last optimization: %d", updatesSinceLastOptimization);

        if (updatesSinceLastOptimization >= 5) {
            std::lock_guard<std::mutex> lock(initial_mutex);
            bnerf_msgs::GraphIterationStatus status_msg;
            ROS_ERROR("Condition met, running optimizer...");

            gtsam::LevenbergMarquardtParams params;
            params.setVerbosityLM("SUMMARY");
            status_msg.graph_stamps.reserve(100);  // Reserve space if expecting many iterations
            status_msg.graph_states.reserve(100);

            try {
                auto start_time = std::chrono::high_resolution_clock::now(); // Start time capture
                gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
                gtsam::Values result = optimizer.optimize();
                auto end_time = std::chrono::high_resolution_clock::now();  // End time capture
                auto exec_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

                status_msg.exec_time = ros::Duration(exec_time.count() / 1000000.0);
                status_msg.iteration = iteration_count++;

                // Populate graph states
                for (const auto& key_value : result) {
                    gtsam::Symbol key(key_value.key);
                    gtsam::Pose3 pose = result.at<gtsam::Pose3>(key);

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

                status_pub.publish(status_msg); // Publish the status message
                ROS_ERROR("Optimizer has run. Final results below:");
                result.print("Final results");

                initial = result; // Update initial estimates with optimized values
                last_pub_time = ros::Time::now(); // Update last publication time
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