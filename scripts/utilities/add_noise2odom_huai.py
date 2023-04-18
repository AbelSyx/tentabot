#! /usr/bin/env python
import rospy
import random
from math import *
import numpy as np
from nav_msgs.msg import Odometry
import tf

noisy_std = 0.1

new_odom_topic = ""
odom_topic = ""
 
def callback(odom_data):
    
    truth_odom = Odometry()
 
    truth_odom.header = odom_data.header
    
    truth_odom.pose.pose.position.x = odom_data.pose.pose.position.x
    truth_odom.pose.pose.position.y = odom_data.pose.pose.position.y
    truth_odom.pose.pose.position.z = odom_data.pose.pose.position.z
    truth_odom.pose.pose.orientation.x = odom_data.pose.pose.orientation.x 
    truth_odom.pose.pose.orientation.y = odom_data.pose.pose.orientation.y
    truth_odom.pose.pose.orientation.z = odom_data.pose.pose.orientation.z
    truth_odom.pose.pose.orientation.w = odom_data.pose.pose.orientation.w
    truth_odom.twist = odom_data.twist
    
    # Add noisy to x, y, z position
    odom_data.pose.pose.position.x = odom_data.pose.pose.position.x + random.gauss(0, noisy_std)
    odom_data.pose.pose.position.y = odom_data.pose.pose.position.y + random.gauss(0, noisy_std)
    odom_data.pose.pose.position.z = odom_data.pose.pose.position.z + random.gauss(0, noisy_std/10)
    
    # Add noisy to orientation
    odom_data.pose.pose.orientation.x = odom_data.pose.pose.orientation.x + random.gauss(0, noisy_std)
    odom_data.pose.pose.orientation.y = odom_data.pose.pose.orientation.y + random.gauss(0, noisy_std)
    odom_data.pose.pose.orientation.z = odom_data.pose.pose.orientation.z + random.gauss(0, noisy_std)
    odom_data.pose.pose.orientation.w = odom_data.pose.pose.orientation.w + random.gauss(0, noisy_std)
    
    
    pub.publish(truth_odom)
    
    '''
    # publish tf
    br = tf.TransformBroadcaster()
    #print("-------------------I'm coming!----------------------")
    br.sendTransform((pose[0] - odom_data.pose.pose.position.x, pose[1] - odom_data.pose.pose.position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, pose[2] - theta2),
                     odom_data.header.stamp,
                     odom_frame,
                     new_odom_frame)
    '''
    

 
if __name__ == '__main__':
    rospy.init_node('noisy_odometry', anonymous=True)
    
    if rospy.has_param("~noisy_std"):
        noisy_std = rospy.get_param("~noisy_std")
    else:
        noisy_std = 5
        rospy.logwarn("noisy_std is set to default")
        
    # get odom topic
    if rospy.has_param("~old_odom_topic"):
        odom_topic = rospy.get_param("~old_odom_topic")
    else:
        odom_topic = "/turtlebot3_0/odom"
    # set new odom topic
    if rospy.has_param("~new_odom_frame"):
        new_odom_topic = rospy.get_param("~new_odom_frame")
    else:
        new_odom_topic = "/turtlebot3_0/odom_truth"

    '''
    # get base frame
    if rospy.has_param("~odom_frame"):
        odom_frame = rospy.get_param("~odom_frame")
    else:
        odom_frame = "turtlebot3_0/odom"
    '''
 
    rospy.Subscriber(odom_topic, Odometry, callback)
    pub = rospy.Publisher(new_odom_topic, Odometry, queue_size=30)
    rospy.spin()