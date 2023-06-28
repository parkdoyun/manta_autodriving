#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, tf
from nav_msgs.msg import Odometry

class Ego_listener():
    def __init__(self):
        rospy.Subscriber("odom", Odometry, self.odom_callback)

    def odom_callback(self,msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.orientation_x = msg.pose.pose.orientation.x
        self.orientation_y = msg.pose.pose.orientation.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w
        br = tf.TransformBroadcaster()
        br.sendTransform((self.x,self.y,0),(self.orientation_x,self.orientation_y,self.orientation_z,self.orientation_w),
                        rospy.Time.now(), "Ego", "map")
	
if __name__ == '__main__':
    rospy.init_node('tf', anonymous=True)
    Ego_listener()
    rospy.spin()
