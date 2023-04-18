#! /usr/bin/env python
import rospy
import random
from math import *
import numpy as np
from nav_msgs.msg import Odometry
import tf

pos_noisy_std = 0.2
theta_noisy_std = 0.01

last_odom = None
new_odom_topic = ""
odom_topic = ""
 
def callback(odom_data):
    
    global last_odom
    noisy_odom = Odometry()
    
    '''
    q2 = [ odom_data.pose.pose.orientation.x,
           odom_data.pose.pose.orientation.y,
           odom_data.pose.pose.orientation.z,
           odom_data.pose.pose.orientation.w ]
    (r2, p2, theta2) = tf.transformations.euler_from_quaternion(q2)
    '''
    
    if last_odom == None:
        
        last_odom = odom_data
        noisy_odom = odom_data
 
    else:
        
        dx = odom_data.pose.pose.position.x - last_odom.pose.pose.position.x
        dy = odom_data.pose.pose.position.y - last_odom.pose.pose.position.y
        ds = sqrt(dx*dx + dy*dy)
        if ds > 6:
            last_odom = odom_data
            noisy_odom = odom_data
            
        else:
            dq_x = odom_data.pose.pose.orientation.x - last_odom.pose.pose.orientation.x
            dq_y = odom_data.pose.pose.orientation.y - last_odom.pose.pose.orientation.y
            dq_z = odom_data.pose.pose.orientation.z - last_odom.pose.pose.orientation.z
            dq_w = odom_data.pose.pose.orientation.w - last_odom.pose.pose.orientation.w
        
            '''
            q1 = [ last_odom.pose.pose.orientation.x,
                   last_odom.pose.pose.orientation.y,
                   last_odom.pose.pose.orientation.z,
                   last_odom.pose.pose.orientation.w ]
            (r1,p1, theta1) = tf.transformations.euler_from_quaternion(q1)
        
            q = [ odom_data.pose.pose.orientation.x,
                  odom_data.pose.pose.orientation.y,
                  odom_data.pose.pose.orientation.z,
                  odom_data.pose.pose.orientation.w ]
            (r,p, theta) = tf.transformations.euler_from_quaternion(q)
            '''
        
            noisy_odom.header = odom_data.header
            # Add noisy to x, y, z position
            noisy_odom.pose.pose.position.x += dx * (1 + pos_noisy_std)
            noisy_odom.pose.pose.position.y += dy * (1 + pos_noisy_std)
            noisy_odom.pose.pose.position.z = odom_data.pose.pose.position.z
    
            # Add noisy to orientation
            #dtheta = theta2 - theta1
            #theta += dtheta * (1 + theta_noisy_std)
        
            #noisy_q = tf.transformations.quaternion_from_euler(r2, p2, theta)
            noisy_odom.pose.pose.orientation.x = odom_data.pose.pose.orientation.x + random.gauss(0, theta_noisy_std)
            noisy_odom.pose.pose.orientation.y = odom_data.pose.pose.orientation.y + random.gauss(0, theta_noisy_std)
            noisy_odom.pose.pose.orientation.z = odom_data.pose.pose.orientation.z + random.gauss(0, theta_noisy_std)
            noisy_odom.pose.pose.orientation.w = odom_data.pose.pose.orientation.w + random.gauss(0, theta_noisy_std)
        
            '''
            noisy_odom.pose.pose.orientation.x += dq_x * (1 + theta_noisy_std)
            noisy_odom.pose.pose.orientation.y += dq_y * (1 + theta_noisy_std)
            noisy_odom.pose.pose.orientation.z += dq_z * (1 + theta_noisy_std)
            noisy_odom.pose.pose.orientation.w += dq_w * (1 + theta_noisy_std)
            '''
            noisy_odom.twist = odom_data.twist
    
    pub.publish(noisy_odom)
    

 
if __name__ == '__main__':
    rospy.init_node('noisy_odometry', anonymous=True)
    
    if rospy.has_param("~noisy_std"):
        pos_noisy_std = rospy.get_param("~noisy_std")
    else:
        pos_noisy_std = 0.2
        rospy.logwarn("noisy_std is set to default")
        
    if rospy.has_param("~noisy_std"):
        theta_noisy_std = rospy.get_param("~noisy_std")
    else:
        theta_noisy_std = 0.01
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
        new_odom_topic = "/turtlebot3_0/odom_noisy"

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