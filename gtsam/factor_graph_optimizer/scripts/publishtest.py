#!/usr/bin/env python3
import rospy
from my_listener.msg import GraphIterationStatus  # this needs to be changed to the correct message location, is just for testing rn

def callback(data):
    rospy.loginfo(f"Received iteration update for iteration: {data.iteration}")
    rospy.loginfo(f"Execution time: {data.exec_time.to_sec()} seconds")
    rospy.loginfo(f"Received {len(data.graph_states)} poses")
    for i, (stamp, state) in enumerate(zip(data.graph_stamps, data.graph_states)):
        rospy.loginfo(f"Pose {i}: Timestamp: {stamp.to_sec()}, Position: ({state.position.x}, {state.position.y}, {state.position.z}), Orientation: ({state.orientation.x}, {state.orientation.y}, {state.orientation.z}, {state.orientation.w})")


def listener():
    rospy.init_node('graph_status_test_subscriber', anonymous=True)
    rospy.Subscriber("graph_status", GraphIterationStatus, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
