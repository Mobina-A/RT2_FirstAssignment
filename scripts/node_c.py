#!/usr/bin/env python

"""
.. module:: node_c
   :platform: Unix
   :synopsis: Python module for the second assignment of Research Track I course
   
.. moduleauthor:: Mobina Alinaghian

A more detailed description of the node:

This node prints the robot speed and the distance from the desired target

Subsribes to:
	/pos_vel
	
"""


import rospy
from assignment_2_2023.msg import pos_vel
from assignment_2_2023.srv import average, averageResponse
import math

# Global variables for robot position and velocities
global x, y, vel_x, vel_z
x = 0
y = 0
vel_x = []
vel_z = []


def service_callback(request):
    """
    Callback function for the 'avg_dis' service.
    Calculates the distance from the current robot position to the goal position,
    and computes the average velocities in the x and z directions.
    
    *Args*: 
    *request(pos_vel)*: Contains the coordinates and velocity of the robot
    """
    global x, y, vel_x, vel_z

    # Get the goal position from the parameter server
    goal_x = rospy.get_param('des_pos_x')
    goal_y = rospy.get_param('des_pos_y')

    # Calculate the Euclidean distance from the current position to the goal position
    distance = math.sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2))

    # Calculate average velocities in x and z directions
    avg_vel_x = sum(vel_x) / len(vel_x) if len(vel_x) > 0 else 0.0
    avg_vel_z = sum(vel_z) / len(vel_z) if len(vel_z) > 0 else 0.0

    # Return the response with distance and average velocities
    return averageResponse(distance, avg_vel_x, avg_vel_z)


def subscriber_callback(request):
    """
    Callback function for the '/pos_vel' topic subscriber.
    Updates the global variables with the current robot position and velocities.
    
    *Args*: 
    *request(pos_vel)*: Contains the coordinates and velocity of the robot
    """
    global x, y, vel_x, vel_z

    # Update robot position and velocities
    x = request.x
    y = request.y
    vel_x.append(request.vel_x)
    vel_z.append(request.vel_z)

    # Get the window size from the parameter server
    size_of_window = rospy.get_param("window_size")

    # Maintain a rolling window of velocities
    if len(vel_x) > size_of_window:
        vel_x.pop(0)  # Remove the first value (oldest)
        vel_z.pop(0)  # Remove the first value (oldest)


if __name__ == "__main__":
    rospy.init_node('node_c')

    # Subscribe to the '/pos_vel' topic
    rospy.Subscriber("/pos_vel", pos_vel, subscriber_callback)

    # Provide the 'avg_dis' service
    rospy.Service('avg_dis', average, service_callback)

    # Keep the node running
    rospy.spin()

