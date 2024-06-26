// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <glog/logging.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Transform.h>
#include <deque>
#include <Eigen/Dense>
#include <boost/array.hpp>
#include <algorithm>

// #include "bnerf_msgs/msg/imumsg.h"
#include "bnerf_msgs/IntegrateIMU.h"
#include <bnerf_utils/bnerf_utils.h>
#include <geometry_msgs/PoseArray.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

std::deque<sensor_msgs::Imu> globalImuDeque;
std::mutex globalImuDequeMutex;
std::unique_ptr<ros::Publisher> globalEdgePub;

Vector3 toVector3(const geometry_msgs::Vector3 &v)
{
    return Vector3(v.x, v.y, v.z);
}

// store IMU msgs in time order
void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // ROS_INFO("Received IMU message.");
    std::cout << "Received IMU message with timestamp: " << msg->header.stamp << std::endl;
    // wait until we are allowed to edit the IMU array
    std::lock_guard<std::mutex> lock(globalImuDequeMutex);

    // sort data by time
    auto insert_pos = globalImuDeque.begin();
    for (; insert_pos != globalImuDeque.end(); ++insert_pos)
    {
        if (msg->header.stamp < insert_pos->header.stamp)
        {
            break;
        }
    }

    // Insert the new message at the correct position
    globalImuDeque.insert(insert_pos, *msg);

    // limit que size to 5000
    if (globalImuDeque.size() > 5000)
    {
        globalImuDeque.pop_front();
    }
}

geometry_msgs::Transform integrate_subarray(std::vector<sensor_msgs::Imu> imu_msgs, ros::Time start_time, ros::Time end_time)
{
    // lock IMU array so it can't be changed until the function is done

    auto current_bias = imuBias::ConstantBias(); // init with zero bias
    double g = 9.81;
    auto w_coriolis = Vector3::Zero(); // zero vector

    // TODO: placeholder values, change later
    double accelerometer_sigma = 0.2;
    double gyroscope_sigma = 0.2;
    double integration_sigma = 0.2;

    // Set IMU preintegration parameters
    Matrix33 measured_acc_cov =
        I_3x3 * pow(accelerometer_sigma, 2);
    Matrix33 measured_omega_cov =
        I_3x3 * pow(gyroscope_sigma, 2);
    // error committed in integrating position from velocities
    Matrix33 integration_error_cov =
        I_3x3 * pow(integration_sigma, 2);

    auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(g);
    imu_params->accelerometerCovariance =
        measured_acc_cov; // acc white noise in continuous
    imu_params->integrationCovariance =
        integration_error_cov; // integration uncertainty continuous
    imu_params->gyroscopeCovariance =
        measured_omega_cov; // gyro white noise in continuous
    imu_params->omegaCoriolis = w_coriolis;

    std::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement =
        std::make_shared<PreintegratedImuMeasurements>(imu_params,
                                                       current_bias);

    // integration loop
    double dt;
    for (int i = 0; i < imu_msgs.size(); ++i)
    {
        if (i == 0)
        {
            dt = (imu_msgs[i].header.stamp - start_time).toSec();
        }
        else if (i == imu_msgs.size() - 1)
        {
            dt = (end_time - imu_msgs[i - 1].header.stamp).toSec();
        }
        else
        {
            dt = imu_msgs[i].header.stamp.toSec() - imu_msgs[i - 1].header.stamp.toSec();
        }
        if (dt == 0)
        {
            dt = std::numeric_limits<double>::epsilon(); // set to a very small value
        }
        current_summarized_measurement->integrateMeasurement(
            toVector3(imu_msgs[i].linear_acceleration),
            toVector3(imu_msgs[i].angular_velocity),
            dt);
    }

    // convert to transformation matrix
    Vector3 delta_position = current_summarized_measurement->deltaPij();
    geometry_msgs::Vector3 p;
    p.x = delta_position.x();
    p.y = delta_position.y();
    p.z = delta_position.z();

    Rot3 delta_rotation = current_summarized_measurement->deltaRij();
    Quaternion q(delta_rotation.matrix());

    geometry_msgs::Transform transform;
    transform.translation.x = p.x;
    transform.translation.y = p.y;
    transform.translation.z = p.z;
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.rotation.w = q.w();

    return transform;
}

bool integrate(bnerf_msgs::IntegrateIMU::Request &req, bnerf_msgs::IntegrateIMU::Response &res)
{
    std::vector<sensor_msgs::Imu> localImus;
    {
        std::lock_guard<std::mutex> lock(globalImuDequeMutex);
        localImus.assign(globalImuDeque.begin(), globalImuDeque.end());
    }

    if (localImus.size() < 20)
    {
        LOG(ERROR) << "imu size (" << localImus.size() << ") too low, skipping.. ";
        return false;
    }

    std::vector<ros::Time> stamps = req.stamps;
    if (stamps.empty())
    {
        auto t1 = localImus.front().header.stamp;
        auto t2 = localImus.back().header.stamp;

        int resolution = 10;
        auto dt = (t2 - t1) * double(1.0 / resolution);
        stamps = {t1};
        for (int i = 0; i < resolution; i++)
            stamps.push_back(stamps.back() + dt);
    }

    // ROS_INFO("STAMPS:");

    // for (const auto &stamp : stamps)
    // {
    //     std::cout << stamp << std::endl;
    // }

    // make our subarrays between each 2 consecutive timestamps
    for (size_t i = 0; i < stamps.size() - 1; ++i)
    {
        std::vector<sensor_msgs::Imu> subarray;

        // Find all the messages between the two timestamps
        for (const auto &imu_msg : localImus)
        {
            if (imu_msg.header.stamp >= stamps[i] && imu_msg.header.stamp < stamps[i + 1])
            {
                subarray.push_back(imu_msg);
            }
            else if (imu_msg.header.stamp >= stamps[i + 1])
            {
                break; // deque is sorted
            }
        }

        // LOG(ERROR) << "subarray size: " << subarray.size() << "";

        bnerf_msgs::GraphBinaryEdge edge;
        geometry_msgs::Transform transform;

        Eigen::Matrix<double, 6, 6> cov_input = Eigen::Matrix<double, 6, 6>::Identity();
        boost::array<double, 36> cov_output;
        copy_n(cov_input.data(), 36, cov_output.data());

        edge.start_stamp = stamps[i];
        edge.end_stamp = stamps[i + 1];
        edge.covariance = cov_output;

        bool is_valid = !subarray.empty();
        if (is_valid)
        {
            transform = integrate_subarray(subarray, stamps[i], stamps[i + 1]);
            edge.mean = transform;
            edge.type = bnerf_msgs::GraphBinaryEdge::IMU;
        }
        else
        {
            edge.type = bnerf_msgs::GraphBinaryEdge::INVALID;
        }
        res.edges.push_back(edge);
        if (globalEdgePub)
            globalEdgePub->publish(edge);
    }

    return true;
}


int main(int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, "imu_integration_node");
    ros::NodeHandle nh("~");

    bool visualize;
    GET_OPTIONAL(nh, "visualize", visualize, false);
    if (visualize)
    {
        auto pub = nh.advertise<bnerf_msgs::GraphBinaryEdge>("imu_edge", 16);
        globalEdgePub.reset(new ros::Publisher(move(pub)));
    }

    // TODO: Replace "/imu_topic" with the actual name of the topic
    ros::Subscriber sub = nh.subscribe("/kitti/oxts/imu", 1000, IMUCallback);

    ros::ServiceServer service = nh.advertiseService("integrate_imu", integrate);

    ros::spin();

    return 0;
}