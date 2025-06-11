#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

def fake_odom_publisher():
    # Initialize ROS node
    rospy.init_node('fake_odom_publisher', anonymous=True)
    
    # Create publisher
    pub = rospy.Publisher('/fake_odom', Odometry, queue_size=10)
    
    # Set publishing rate to 100Hz
    rate = rospy.Rate(100)
    
    # Create odometry message
    odom_msg = Odometry()
    odom_msg.header.frame_id = "map"
    odom_msg.child_frame_id = "base_link"
    
    # Set position to origin (0,0,0)
    odom_msg.pose.pose.position = Point(0.0, 0.0, 0.0)
    
    # Set orientation to x-axis (quaternion for rotation around z-axis by 0 degrees)
    # This creates a quaternion representing rotation along x-axis
    odom_msg.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    
    # Set covariance matrices (identity matrices for simplicity)
    odom_msg.pose.covariance = [0.1] + [0.0] * 35  # 6x6 matrix
    odom_msg.twist.covariance = [0.1] + [0.0] * 35  # 6x6 matrix
    
    while not rospy.is_shutdown():
        # Update timestamp
        odom_msg.header.stamp = rospy.Time.now()
        
        # Publish message
        pub.publish(odom_msg)
        
        # Sleep to maintain 100Hz rate
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_odom_publisher()
    except rospy.ROSInterruptException:
        pass 