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

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Transform.h>
#include <deque>

// #include "bnerf_msgs/msg/imumsg.h"
#include "bnerf_msgs/IntegrateIMU.h"

using namespace std;
using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

std::deque<sensor_msgs::Imu> globalImuDeque;
std::mutex globalImuDequeMutex;

Vector3 toVector3(const geometry_msgs::Vector3 &v)
{
    return Vector3(v.x, v.y, v.z);
}

// store IMU msgs in time order
void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
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
    std::lock_guard<std::mutex> lock(globalImuDequeMutex);

    auto current_bias = imuBias::ConstantBias(); // init with zero bias
    double g = 9.8;
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

    //integration loop
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

    //convert to transformation matrix
    Vector3 delta_velocity = current_summarized_measurement->deltaVij();
    geometry_msgs::Vector3 v;
    v.x = delta_velocity.x();
    v.y = delta_velocity.y();
    v.z = delta_velocity.z();

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
    std::vector<ros::Time> stamps = req.stamps;
    std::vector<geometry_msgs::Transform> all_transforms;
    std::vector<bool> validities;

    // make our subarrays between each 2 consecutive timestamps
    for (size_t i = 0; i < stamps.size() - 1; ++i)
    {
        std::vector<sensor_msgs::Imu> subarray;

        // Find all the messages between the two timestamps
        for (const auto &imu_msg : globalImuDeque)
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

        bool is_valid = !subarray.empty();
        validities.push_back(is_valid);

        geometry_msgs::Transform transform;
        if (is_valid)
        {
            transform = integrate_subarray(subarray, stamps[i], stamps[i + 1]);
        }
        all_transforms.push_back(transform);
    }

    res.transforms = all_transforms;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_listener_node");
    ros::NodeHandle nh;

    // TODO: Replace "/imu_topic" with the actual name of the topic
    ros::Subscriber sub = nh.subscribe("topic_name", 1000, IMUCallback);

    ros::ServiceServer service = nh.advertiseService("topic_name", integrate);

    ros::spin();

    return 0;
}