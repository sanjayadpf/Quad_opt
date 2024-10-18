#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
import matplotlib.pyplot as plt
import numpy as np
import threading

# Global variables for storing data
initial_z_distance = None
joint01_z = None
joint11_z = None
distances = []  # List to store all z distances
times = []  # List to store all corresponding times

def link_states_callback(msg):
    global initial_z_distance, joint01_z, joint11_z

    try:
        # Replace with the actual names of joint01 and joint11
        joint01_index = msg.name.index("softactuator::sec01")
        joint11_index = msg.name.index("softactuator::sec11")
        
        current_z_position01 = msg.pose[joint01_index].position.z
        current_z_position11 = msg.pose[joint11_index].position.z

        if joint01_z is None or joint11_z is None:
            # Initialize joint positions
            joint01_z = current_z_position01
            joint11_z = current_z_position11

        # Calculate the initial distance
        if initial_z_distance is None:
            initial_z_distance = abs(joint01_z - joint11_z)
        
        # Calculate the current distance between joint01 and joint11
        current_distance = abs(current_z_position01 - current_z_position11)
        
        # Append new data
        distances.append(current_distance * 1000)  # Convert to mm
        times.append(rospy.get_time())
    except ValueError as e:
        rospy.logwarn("Link not found in link states: %s", e)

def plot_distance():
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    
    # Set up the initial plot with empty data
    line, = ax.plot([], [], 'r-')
    ax.set_ylim(265, initial_z_distance * 1000)  # Y-axis from 250 to max distance in mm
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (mm)')
    ax.set_title('Soft-limb actuation')

    # Define fixed increments for y axis
    y_ticks = np.arange(265, 290 + 10, 5)  # Y-axis ticks from 250 to max distance in mm with interval of 30

    ax.set_yticks(y_ticks)
    
    while not rospy.is_shutdown():
        if times:
            # Update the plot data
            line.set_xdata(times)
            line.set_ydata(distances)
            
            # Update plot limits
            ax.relim()
            ax.autoscale_view()  # Autoscale both x and y axes
            
            # Set fixed y ticks
            ax.set_yticks(y_ticks)
            
            plt.draw()
            plt.pause(0.1)  # Pause to update the plot

def main():
    rospy.init_node('plot_distance')

    rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_callback)

    plot_thread = threading.Thread(target=plot_distance)
    plot_thread.start()

    rospy.loginfo("Plotting distance between Joint01 and Joint11")
    rospy.spin()
    plot_thread.join()

if __name__ == '__main__':
    main()
