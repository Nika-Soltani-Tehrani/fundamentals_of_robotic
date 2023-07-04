#!/usr/bin/python3

import tf
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np


class VFHPlanner():

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('vfh', anonymous=False)

        # Define angular and linear velocities
        self.ang_v = 0.1
        self.ang_epsilon = 0.1
        self.lin_v = 0.1
        self.lin_epsilon = 0.05
        self.lin_lenght = 1.5

        # Define VFH parameters
        self.sector_size = 5
        self.a = 1
        self.b = 0.25
        self.threshold = 5
        self.s_max = 9

        # Create a ROS publisher for the velocity commands
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


    def set_position(self):
        # Get the robot's pose from the odometry data
        odom = rospy.wait_for_message("/odom", Odometry)
        orientation = odom.pose.pose.orientation
        position = odom.pose.pose.position
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        return position, yaw


    def path_planner(self, goal_x, goal_y):
        # Plan the path to reach the goal position
        sectors = self.create_Histogram()
        position, yaw = self.set_position()
        angle = math.atan2(goal_y - position.y, goal_x - position.x)

        if angle < 0:
            angle += 2 * math.pi
        dif = angle - yaw
        if dif < 0:
            dif += 2 * math.pi

        goal_idx = int(math.degrees(dif) / self.sector_size)
        goal_sector = goal_idx % self.number_of_sector

        thresholded = []
        for i in range(self.number_of_sector):
            if sectors[i] < self.threshold:
                thresholded.append(i)
        selected_sectors = thresholded

        if sectors[goal_sector] < self.threshold:
            best_sector = goal_sector
        else:
            best_sector = self.hist_valley(selected_sectors, goal_sector)

        if best_sector > 36:
            best_sector -= 72

        angle = math.radians(best_sector * 5)
        self.controller(angle)

    def controller(self, angle):
        # Control the robot's motion to reach the desired orientation and distance
        remaining = angle  # Initialize the remaining angle to be rotated
        prev_angle = self.set_position()[1]  # Get the initial angle of the robot's position

        rospy.sleep(1)  # Wait for 1 second

        sign = 1  # Variable to determine the sign of the angular velocity (clockwise or counterclockwise)
        if angle > math.pi:
            angle -= 2 * math.pi  # Normalize the angle to be between -pi and pi
        if angle < -math.pi:
            angle += 2 * math.pi
        if angle < 0:
            sign = -1  # Set the sign to -1 for counterclockwise rotation

        twist = Twist()  # Create a Twist object to control the robot's motion
        twist.angular.z = sign * self.ang_v  # Set the angular velocity based on the sign and robot's angular velocity
        self.cmd_vel.publish(twist)  # Publish the Twist message to control the robot's motion

        while abs(remaining) >= self.ang_epsilon:  # Continue rotating until the remaining angle is within the threshold
            current_angle = self.set_position()[1]  # Get the current angle of the robot's position
            delta = current_angle - prev_angle  # Calculate the change in angle since the last iteration

            if abs(delta) < 0.2:  # If the change in angle is small, update the remaining angle
                remaining -= delta
            prev_angle = current_angle

        twist.angular.z = 0  # Stop the angular motion
        twist.linear.x = 0  # Set the linear velocity to zero
        self.cmd_vel.publish(twist)  # Publish the Twist message to stop the robot's motion

        remaining = self.lin_lenght  # Set the remaining distance to be traveled
        prev_position = self.set_position()[0]  # Get the initial position of the robot

        rospy.sleep(1)  # Wait for 1 second

        twist = Twist()  # Create a new Twist object
        twist.linear.x = self.lin_v  # Set the linear velocity based on the robot's velocity
        self.cmd_vel.publish(twist)  # Publish the Twist message to control the robot's motion

        while remaining >= self.lin_epsilon:  # Continue moving until the remaining distance is within the threshold
            current_position = self.set_position()[0]  # Get the current position of the robot
            delta = np.linalg.norm([current_position.x - prev_position.x, current_position.y - prev_position.y])  # Calculate the change in position since the last iteration

            remaining -= delta  # Update the remaining distance
            remaining = abs(remaining)  # Make sure remaining distance is positive
            prev_position = current_position

        twist.linear.x = 0  # Stop the linear motion
        twist.angular.z = 0  # Stop the angular motion
        self.cmd_vel.publish(twist)  # Publish the Twist message to stop the robot's motion

        rospy.sleep(1)  # Wait for 1 second

        self.cmd_vel.publish(Twist())  # Publish an empty Twist message to ensure the robot is completely stopped


    def create_Histogram(self):
        # Create a histogram based on the laser scan data
        histogram = []  # List to store histogram values
        self.laser_scan = rospy.wait_for_message("/scan", LaserScan)  # Wait for laser scan message
        self.number_of_sector = int(len(self.laser_scan.ranges) / self.sector_size)  # Calculate number of sectors

        # Calculate the magnitude for each sector
        for i in range(self.number_of_sector):
            tmp_histogram = 0
            for j in range(i * self.sector_size, (i + 1) * self.sector_size):
                magnitude = self.a - self.b * min(6, self.laser_scan.ranges[j])  # Calculate magnitude based on laser range
                tmp_histogram += magnitude

            histogram.append(tmp_histogram)  # Add magnitude to histogram

        # Smooth the histogram values
        smoothed = []
        for t in range(self.number_of_sector):
            sum_h = 0
            for j in range(-2, 3):
                if j == 2 or j == -2:
                    jj = 1
                else:
                    jj = 2

                if i + j >= self.number_of_sector:
                    j = j * -1
                    t = self.number_of_sector - t - 1

                sum_h += histogram[t + j] * jj  # Calculate weighted sum of histogram values

            sum_h = sum_h / 5  # Average the sum over 5 values
            smoothed.append(sum_h)  # Add smoothed value to the list

        return smoothed  # Return the smoothed histogram


    def hist_valley(self, selected_sectors, goal_sector):
        # Find the closest valley to the goal sector
        my_min = 999  # Initialize a variable to hold the minimum distance
        my_index = 0  # Initialize a variable to hold the index of the closest valley

        valleys = []  # Create an empty list to store the valleys
        tmp = []  # Create an empty list to temporarily store sectors

        for i in range(len(selected_sectors)):
            j = i - 1

            if i == 0:
                tmp.append(selected_sectors[i])
                continue

            if selected_sectors[i] - selected_sectors[j] > 1:
                valleys.append(tmp)
                tmp = []

            tmp.append(selected_sectors[i])

        valleys.append(tmp)  # Append the remaining sectors to the valleys list
        tmp = []  # Reset the temporary list

        # Handle the special case where the first valley connects to the last valley
        if valleys[0][0] == 0 and valleys[-1][-1] == (self.number_of_sector - 1):
            tmp = valleys.pop(0)  # Remove the first valley from the list
            for i in tmp:
                valleys[-1].append(i)  # Append the sectors of the first valley to the last valley

        # Find the valley with the minimum distance to the goal sector
        for i in range(len(valleys)):
            for j in range(len(valleys[i])):
                distances = abs(valleys[i][j] - goal_sector)

                if distances > 36:
                    distances = 72 - distances

                if distances < my_min:
                    my_min = distances
                    my_index = i

        closest_valley = valleys[my_index]  # Get the sectors of the closest valley

        # Return the sector in the middle of the closest valley if it has a small size,
        # otherwise return a sector closer to the middle of the valley
        if len(closest_valley) <= self.s_max:
            return closest_valley[int(len(closest_valley) / 2)]
        else:
            return closest_valley[my_index + int(self.s_max / 2)]



    def run(self):
        target_points = [
            [4.5, -0.2],
            [4.3, 2.9],
            [3.6, 4],
            [2.3, 4.7],
            [2.5, 1.8],
            [1, 1.5],
            [0.8, 2.8],
            [1.2, 5.7]
        ]

        # Iterate over each target point
        for target_point in target_points:
            goal_x = target_point[0]
            goal_y = target_point[1]
            
            # Keep executing until ROS (Robot Operating System) is shutdown
            while not rospy.is_shutdown():
                vfh.path_planner(goal_x, goal_y)  # Invoke the path planner with the goal coordinates
                
                # Get the current position of the robot
                current_position = self.set_position()[0]
                
                # Print current position information
                print("after one iterate")
                print("current x", current_position.x)
                print("current y", current_position.y)
                print(abs(current_position.x - goal_x))
                print(abs(current_position.y - goal_y))
                
                # Check if the goal is achieved by comparing the current position with the goal position
                if (abs(current_position.x - goal_x) < 0.2) and (abs(current_position.y - goal_y) < 0.2):
                    print("----------------goal achieved!----------------------")
                    break  # Exit the loop if the goal is achieved


if __name__ == '__main__':
    vfh = VFHPlanner()
    vfh.run()
    