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
#include <gtsam/slam/PriorFactor.h> // unary factor
#include <cstdlib>
#include <fstream>

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

double calculateError(const geometry_msgs::Transform &t1, const geometry_msgs::Vector3 &t2)
{
    double dx = t1.translation.x - t2.x;
    double dy = t1.translation.y - t2.y;
    double dz = t1.translation.z - t2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// No duplicates
int ensureKeyExists(const std::string &timestamp, gtsam::Values &values, const gtsam::Pose3 &defaultPose)
{
    auto it = time_to_key_map.find(timestamp);
    if (it == time_to_key_map.end())
    {
        int key = current_key++;
        time_to_key_map[timestamp] = key;
        values.insert(gtsam::Symbol('x', key), defaultPose);
        return key;
    }
    else
    {
        return it->second;
    }
}

void EdgeCallBack(const bnerf_msgs::GraphEdgeCollection::ConstPtr &msg)
{
    std::string homeDir = getenv("HOME");
    static std::ofstream outFile(homeDir + "/catkin_ws/src/Beyond-NeRF/gtsam/factor_graph_optimizer/pose_data.csv", std::ios::app);
    time_to_key_map.clear();
    current_key = 0;
    gtsam::NonlinearFactorGraph localGraph;
    gtsam::Values localInitial;
    auto poseNoiseModel = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

    if (!outFile.is_open())
    {
        outFile.open(homeDir + "/catkin_ws/src/Beyond-NeRF/gtsam/factor_graph_optimizer/pose_data.csv", std::ios::app);
    }

    // Process unary edges
    for (const auto &unary_edge : msg->unary_edges)
    {
        // coordinates of the position in 3D
        double x = unary_edge.mean.x;
        double y = unary_edge.mean.y;
        double z = unary_edge.mean.z;
        std::string timestamp = std::to_string(unary_edge.stamp.toSec());

        ROS_INFO("Generating new key for timestamp %s. Current key: %d", timestamp.c_str(), current_key);
        int key = ensureKeyExists(timestamp, localInitial, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, y, z)));
        ROS_INFO("Key for timestamp %s is %d", timestamp.c_str(), key);

        localGraph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', key), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, y, z)), poseNoiseModel));

        outFile << x << "," << y << "," << z << "\n";
    }

    // Process binary edges
    for (const auto &binary_edge : msg->binary_edges)
    {
        const bnerf_msgs::GraphUnaryEdge *closest_unary_edge = nullptr;
        double min_time_diff = std::numeric_limits<double>::infinity();

        for (const auto &unary_edge : msg->unary_edges)
        {
            double time_diff = std::abs(binary_edge.start_stamp.toSec() - unary_edge.stamp.toSec());
            if (time_diff < min_time_diff)
            {
                min_time_diff = time_diff;
                closest_unary_edge = &unary_edge;
            }
        }

        if (closest_unary_edge)
        {
            double error = calculateError(binary_edge.mean, closest_unary_edge->mean);
            totalError += error;
            count++;

            std::string binary_timestamp = std::to_string(binary_edge.start_stamp.toSec());
            std::string unary_timestamp = std::to_string(closest_unary_edge->stamp.toSec());

            int key_t1 = ensureKeyExists(binary_timestamp, localInitial, gtsam::Pose3());
            int key_t2 = ensureKeyExists(unary_timestamp, localInitial, gtsam::Pose3());

            localGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', key_t1), gtsam::Symbol('x', key_t2), gtsam::Pose3(), poseNoiseModel));
        }
    }

    if (!localGraph.empty())
    {
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosityLM("SUMMARY");
        std::string homeDir = getenv("HOME");
        params.setLogFile(homeDir + "/catkin_ws/src/Beyond-NeRF/gtsam/factor_graph_optimizer/optimizer_log.txt");

        gtsam::LevenbergMarquardtOptimizer optimizer(localGraph, localInitial, params);
        gtsam::Values result = optimizer.optimize();

        // Publish the results
        bnerf_msgs::GraphIterationStatus status_msg;
        status_msg.graph_stamps.reserve(100);
        status_msg.graph_states.reserve(100);

        // Populate graph states for publishing
        for (const auto &key_value : result)
        {
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

        status_pub.publish(status_msg);
        ROS_INFO("Optimizer has run. Final results published.");
    }

    if (count > 0)
    {
        double averageError = totalError / count;
        ROS_INFO("Average Error: %f", averageError);
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "transformation_listener");
    ros::NodeHandle nh;

    // Initialize subscriber to the graph edge collection topic
    ros::Subscriber sub = nh.subscribe("/graph_edge_manager_node/graph_edge_collection", 1000, EdgeCallBack);

    // Initialize publisher for the graph iteration status
    status_pub = nh.advertise<bnerf_msgs::GraphIterationStatus>("graph_status", 10);

    // Spin to continuously process incoming messages
    ros::spin();

    return 0;
}
