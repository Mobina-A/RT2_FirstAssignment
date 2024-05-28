#!/usr/bin/env python3
"""
.. module: node_a
   :platform: Unix
   :synopsis: Python module for the second assignment of Research Track I course 
   
.. moduleauthor:: Mobina Alinaghian


A more detailed description of the node:
This node implements an action client allowing the user to set a target (x, y) or to cancel it at any time.
Also, it publishes the robot position and velocity as a custom message by relying on the topic /odom.This use feedback of the action server to know when the target has been reached.

Subsribes to:
	/odom
	
Publishes to:
	/pos_vel
"""
import rospy
import os
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from std_srvs.srv import *
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import pos_vel


my_data = pos_vel()

def callback(data):
    """
    Callback function to publish position and velocity of the robot taken from */odom* topic *data(Odometry)*: Contains the odometry of the robot
    
    *Arg*:
    *data(Odometry)*: Contains the odometry of the robot
    """
    global pub
    # Create a custom message for having the location and velocities my_data = pos_vel()
    # Use the message to set the robot position and linear velocity
    my_data.x = data.pose.pose.position.x
    my_data.y = data.pose.pose.position.y
    my_data.vel_x = data.twist.twist.linear.x
    my_data.vel_z = data.twist.twist.linear.z
    # Publish the message
    pub.publish(my_data)


def set_target():
    """ 
    Function that collects user input for a target position (x, y) that
    the robot must reach inside the simulation environment and sends the target (goal) to the action server
    
    *Args*: None
    """
    # Get target position from terminal
    while True:
        x_pos = input("Enter the x value (-5 <= x <= 5): ")

        try:
            x_pos = float(x_pos)

            if -5 <= x_pos <= 5:
                break  # valid x value, exit the loop
            else:
                print("Invalid input. x value must be within the range [-5, 5]. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a numeric value for x.")

    while True:
        y_pos = input("Enter the y value (-5 <= y <= 5): ")

        try:
            y_pos = float(y_pos)

            if -5 <= y_pos <= 5:
                break  # valid y value, exit the loop
            else:
                print("Invalid input. y value must be within the range [-5, 5]. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a numeric value for y.")

    # Print the selected coordinates
    print("The position of the target is: (", x_pos, ", ", y_pos, ")")

    # Creates a goal message with the target coordinates
    goal = assignment_2_2023.msg.PlanningGoal()
    goal.target_pose.pose.position.x = x_pos
    goal.target_pose.pose.position.y = y_pos

    # Send the goal to the action server
    client.send_goal(goal)
    print("The target has been successfully sent to the server!!")

def UI():
    """
    This function is called at the beginning of the program. The user can choose to set a goal, cancel it, or exit the program by entering the correct number. In the other word, the user is able to communicate with the program through this part.
    
    *Args*: None
    """
    while True:
        print("Instruction for Robot Control\n")
        print("  Enter 1: Set Target Position\n")
        print("  Enter 2: Cancel Target\n")
        print("  Enter 3: Exit\n")

        # Ask the user to select a mission
        mission = input("Select 1, 2, or 3 and then press Enter: ")

        # Check the selected mission
        if mission == "1":
            set_target()
        elif mission == "2":
            client.cancel_goal()
            print("The target has been canceled successfully!")
        elif mission == "3":
            print("Exiting the program.")
            exit()
        else:
            print("Invalid choice! Choose one of the options,please!")
            continue  # Continue to the next iteration of the loop if the choice is invalid


if __name__ == '__main__':
    # Init Node
    rospy.init_node("node_a")
    # Define a global publisher in order to publish the pos_vel custom message
    global pub
    pub = rospy.Publisher("/pos_vel", pos_vel, queue_size=1)
    # Define a subscriber which listens to the Odometry message and calls the callback function
    rospy.Subscriber("/odom", Odometry, callback)
    # Create a new client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    # Wait for the server to be ready to receive the goal
    client.wait_for_server()
    # Call the UI function
    UI()

