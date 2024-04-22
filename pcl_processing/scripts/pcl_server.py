from std_srvs.srv import Trigger, TriggerResponse

def alignment_service_callback(req):
    # P
    return TriggerResponse(success=True, message="Point cloud aligned successfully")

if __name__ == "__main__":
    rospy.init_node("point_cloud_alignment_node")

    # subs to input topic pointcloud2
    rospy.Subscriber("/input_point_cloud", PointCloud2, point_cloud_callback)

    # Publisher for the aligned point cloud
    pub = rospy.Publisher("/aligned_point_cloud", PointCloud2, queue_size=10)

    # Create a service for triggering the alignment
    rospy.Service("/trigger_alignment", Trigger, alignment_service_callback)

    rospy.spin()
