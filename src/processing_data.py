#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import geonav_conversions as gc

class ProcessingDataNode:
    def __init__(self):
        rospy.init_node('processing_data')

        #Subscribe to the gps data
        self.gps_sub = rospy.Subscriber('/gps/duro/fix', NavSatFix, self.gps_callback)

        #Publish data as point stamped
        self.true_pose_pub = rospy.Publisher('/true_pose', Odometry, queue_size=1)
        self.gpsdata = Odometry()  # GPS data
        self.gpsdata.header.frame_id = 'odom'
        # Flag to initialise first GPS data
        self.gps = True 



    def gps_callback(self, msg):
        if self.gps is True:
            print("data received")
            self.lat_origin = msg.latitude
            self.longi_origin = msg.longitude
            self.gps = False
        # Extract latitude and longitude from the NavSatFix message            
        lat = msg.latitude
        long = msg.longitude

        # Convert latitude and longitude to UTM coordinates
        self.xgps, self.ygps = gc.ll2xy(
            lat, long, self.lat_origin, self.longi_origin)

        # Publish the converted UTM coordinates on the /true_pose topic
        self.gpsdata.pose.pose.position.x = self.xgps
        self.gpsdata.pose.pose.position.y = self.ygps

        self.true_pose_pub.publish(self.gpsdata)

if __name__ == "__main__":
    try:
        processing_node = ProcessingDataNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
