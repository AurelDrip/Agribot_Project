#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import geonav_conversions as gc

class ProcessingDataNode:
    def __init__(self):
        rospy.init_node('processing_data')
        self.gps_sub = rospy.Subscriber('/gps/duro/fix', NavSatFix, self.gps_callback)
        self.true_pose_pub = rospy.Publisher('/true_pose', PointStamped, queue_size=1)

    def gps_callback(self, msg):
        # Extract latitude and longitude from the NavSatFix message
        latitude = msg.latitude
        longitude = msg.longitude

        # Replace the origin_lat and origin_lon with your desired origin coordinates
        origin_lat = 0.0
        origin_lon = 0.0

        # Convert latitude and longitude to UTM coordinates
        x, y = gc.ll2xy(latitude, longitude, origin_lat, origin_lon)

        # Publish the converted UTM coordinates on the /true_pose topic
        true_pose_msg = PointStamped()
        true_pose_msg.header = msg.header
        true_pose_msg.point.x = x
        true_pose_msg.point.y = y
        true_pose_msg.point.z = 0.0  # Assuming z-coordinate is 0 for simplicity

        self.true_pose_pub.publish(true_pose_msg)

if __name__ == "__main__":
    try:
        processing_node = ProcessingDataNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
