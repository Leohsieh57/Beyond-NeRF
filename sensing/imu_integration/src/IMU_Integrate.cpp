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
#include <deque>

#include <sensor_msgs/Imu.h>
#include <bnerf_msgs/IntegrateIMU.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

std::deque<sensor_msgs::Imu> globalImuDeque;

// store IMU msgs
void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // limit que size to 5000
    if (globalImuDeque.size() > 5000)
    {
        globalImuDeque.pop_front();
    }
    globalImuDeque.push_back(*msg);
}

bool integrate(bnerf_msgs::IntegrateIMU::Request &req, bnerf_msgs::IntegrateIMU::Response &res)
{
    std::vector<ros::Time> stamps = req.stamps;
    std::vector<sensor_msgs::Imu> imu_msgs;
    std::vector<geometry_msgs::Transform> all_transforms;

    for (const auto &stamp : stamps)
    {
        bool found = false;
        for (const auto &imu_msg : globalImuDeque)
        {
            if (fabs(imu_msg.header.stamp.toSec() - stamp.toSec()) < 0.001)
            {
                imu_msgs.push_back(imu_msg);
                found = true;
                break;
            }
        }
        if (!found)
        {
            res.transforms = all_transforms; // Return empty transform
            return false;
        }
    }

    auto current_bias = imuBias::ConstantBias(); // init with zero bias
    double g = 9.8;
    auto w_coriolis = Vector3::Zero(); // zero vector

    // TODO: dummy values, change later
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

    for (int i = 1; i < stamps.size(); ++i)
    {
        double dt = stamps[i].toSec() - stamps[i - 1].toSec();
        const auto & lin_acc = imu_msgs[i].linear_acceleration;
        const auto & ang_vel = imu_msgs[i].angular_velocity;
        current_summarized_measurement->integrateMeasurement(
            gtsam::Vector3(lin_acc.x, lin_acc.y, lin_acc.z),
            gtsam::Vector3(ang_vel.x, ang_vel.y, ang_vel.z), dt);

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
        all_transforms.push_back(transform);

        current_summarized_measurement->resetIntegration();
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