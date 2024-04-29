#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <unordered_map>
#include <string>

gtsam::NonlinearFactorGraph graph;
gtsam::Values initial;
std::unordered_map<std::string, int> time_to_key_map;
int current_key = 0;

void transformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    std::string t1 = std::to_string(msg->header.stamp.toSec());
    std::string t2 = std::to_string(msg->header.stamp.toSec() + 0.1);

    // Convert ROS transform to GTSAM Pose3
    gtsam::Pose3 pose(gtsam::Rot3::Quaternion(msg->transform.rotation.w,
                                              msg->transform.rotation.x,
                                              msg->transform.rotation.y,
                                              msg->transform.rotation.z),
                      gtsam::Point3(msg->transform.translation.x,
                                    msg->transform.translation.y,
                                    msg->transform.translation.z));

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

    // unary constraint
    auto model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(0.1));
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', key_t1), gtsam::Symbol('x', key_t2), pose, model));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "transformation_listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("registration_transform", 1000, transformCallback);

    ros::Rate rate(10.0);
    while (ros::ok()) {
        ros::spinOnce();
        if (graph.size() > 0 && graph.size() % 5 == 0) {  // trivial update period chosen
            gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
            gtsam::Values result = optimizer.optimize();
            result.print("Final results\n");
            initial = result; // Update initial estimates with optimized values
        }
        rate.sleep();
    }

    return 0;
}

