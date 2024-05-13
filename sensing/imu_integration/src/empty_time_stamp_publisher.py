#!/usr/bin/python3

import rospy
from bnerf_msgs.srv import IntegrateIMURequest, IntegrateIMUResponse, IntegrateIMU
from bnerf_msgs.msg import GraphBinaryEdge

def request_empty_time_stamps():
    rospy.wait_for_service('/integrate_imu')  # Wait for the service to be available
    try:
        print("Sending request at:", rospy.Time.now())
        integrate_imu = rospy.ServiceProxy('/integrate_imu', IntegrateIMU)
        request = IntegrateIMURequest()
        # request.stamps = [rospy.Time.now()]  # Set the stamps field to a list containing the current time
        response = integrate_imu(request)
        print("Received response at:", rospy.Time.now())
        print(response.edges)
        rospy.loginfo("Received edges")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('service_client_node')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        request_empty_time_stamps()
        rate.sleep()

    rospy.spin()