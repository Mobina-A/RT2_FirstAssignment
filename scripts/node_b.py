#!/usr/bin/env python3

"""
.. module:: node_b
   :platform: Unix
   :synopsis: Python module for the second assignment of Research Track I course
   
.. moduleauthor:: Mobina Alinaghian

A more detailed description of the node:

This node prints the number of target canceled and the number of target reached
	
"""

import rospy
#from assignment_2_2023.srv import last_target, last_targetResponse, GetLastTarget
from assignment_2_2023.msg import pos_vel
from nav_msgs.msg import Odometry

last_des_pos_x = 0
last_des_pos_y = 0
last_target = None  # Variable to store the last target coordinates

def callback(msg):
    """
    Callback function to update the last_target variable whenever a new target is set.
    
    *Args*: 
    *msg(pos_vel)*: Contains the last coordinates and velocity of the robot
    """
    global last_des_pos_x, last_des_pos_y, last_target
    last_des_pos_x = msg.pose.pose.position.x
    last_des_pos_y = msg.pose.pose.position.y
    last_target = (last_des_pos_x, last_des_pos_y)

def get_last_target(request):
    """
    Callback function for the service. It returns the coordinates of the last target sent by the user.
    *Args*: 
    *request(pos_vel)*: Contains the coordinates and velocity of the robot
    """
    global last_target
    response = pos_vel()

    if last_target is not None:
        response.x, response.y = last_target
        rospy.loginfo("Returning the last target coordinates: (%f, %f)", response.x, response.y)
    else:
        rospy.logwarn("No target has been set yet.")

    return response

def run():
    rospy.init_node("last_target_service_node")

    # Define a service to get the last target coordinates
    rospy.Service('get_last_target', GetLastTarget, get_last_target)

    # Define a subscriber to the Odometry message and call the callback function
    rospy.Subscriber("/odom", Odometry, callback)

    # Define a service to get the last desired target
    service = rospy.Service('last_target', last_target, callback)

    rospy.loginfo("Last Target Service Node has started.")

    # Spin to keep the node running
    rospy.spin()

if __name__ == "__main__":
    run()


