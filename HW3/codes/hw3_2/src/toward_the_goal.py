#!/usr/bin/python3

import tf
import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped


class Controller:
    
    def __init__(self):
        rospy.init_node("pid", anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        self.path = Path()
        self.odom_sub = rospy.wait_for_message("/odom", Odometry)
        self.path_publisher = rospy.Publisher("/path", Path, queue_size=10)

        self.errors = []
        
        self.pose_x = 0
        self.pose_y = 0
        self.orientation = 0
        self.goal_x = -3
        self.goal_y = -3
        self.goal_orientation = 0
        c_type = 'PID'
        self.set_controller_type(c_type)


    def set_controller_type(self, c_type):
        
        if c_type == "PD":
            self.angular_kp = 3
            self.angular_ki = 0.0
            self.angular_kd = 0

            # linear controller parameters
            self.linear_kp = 0.05
            self.linear_ki = 0.0
            self.linear_kd = 0

            self.dt = 0.005
            self.D = 0.0
            self.epsilon = 0.1
            rate = 1 / self.dt
            self.r = rospy.Rate(rate)
        
        if c_type == "P":
            self.angular_kp = 0.4
            self.angular_ki = 0.0
            self.angular_kd = 0.0

            # linear controller parameters
            self.linear_kp = 0.02
            self.linear_ki = 0.0
            self.linear_kd = 0.0

            self.dt = 0.005
            self.D = 0.0
            self.epsilon = 0.2
            rate = 1 / self.dt
            self.r = rospy.Rate(rate)
        
        if c_type == "PID":
            self.angular_kp = 2.6
            self.angular_ki = 0.001
            self.angular_kd = 0.7

            # linear controller parameters
            self.linear_kp = 0.1
            self.linear_ki = 0.001
            self.linear_kd = 0.1

            self.dt = 0.005
            self.D = 0.0
            self.epsilon = 0.2
            rate = 1 / self.dt
            self.r = rospy.Rate(rate)

    def update_odom_sub(self):
        self.odom_sub = rospy.wait_for_message("/odom", Odometry)

    def get_heading(self):
        msg = self.odom_sub
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)
        return yaw

    def get_position(self):
        msg = self.odom_sub
        position = msg.pose.pose.position

        x, y = position.x, position.y
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)
        return x, y

    def calculate_distance(self, x1, x2, y1, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def nearest_dot(self, mode=0):
        min_distance = 10000000
        min_x = 0
        min_y = 0

        if mode == 0:
            min_local_distance = self.calculate_distance(self.pose_x, self.goal_x, self.pose_y, self.goal_y)
            if min_local_distance < min_distance:
                min_distance = min_local_distance

        if mode == 1:
            diff_ang = np.arctan2((self.goal_y - self.pose_y), (self.goal_x - self.pose_x))
            diff_ang = self.calculate_difference_angle(diff_ang)
            if -math.pi / 2 < diff_ang < math.pi / 2:
                min_local_distance = self.calculate_distance(self.pose_x, self.goal_x, self.pose_y, self.goal_y)
                if min_distance > min_local_distance and min_local_distance > abs(self.epsilon):
                    min_distance = min_local_distance

        return min_distance
    
    def calculate_difference_angle(self, goal_angle):
        if self.orientation > 0:
            if (self.orientation - math.pi < goal_angle < self.orientation):
                direction = -1
            else:
                direction = 1
        else:
            if (self.orientation + math.pi > goal_angle > self.orientation):
                direction = 1
            else:
                direction = -1
        return direction * (math.pi - abs(abs(self.orientation - goal_angle) - math.pi))
        

    def calulate_angular_error(self):
        return self.calculate_difference_angle(self.goal_orientation)

    def calulate_linear_error(self):
        mode = 0
        if self.nearest_dot() < self.epsilon:
            mode = 1
        self.nearest_dot(mode=mode)
        self.goal_orientation = np.arctan2((self.goal_y - self.pose_y), (self.goal_x - self.pose_x))
        return self.nearest_dot(mode=mode)

    def run(self):
        twist = Twist()
        sum_angular_error = 0
        sum_linear_error = 0
        prev_angular_error = 0
        prev_linear_error = 0

        while not rospy.is_shutdown():
            self.update_odom_sub()
            self.orientation = self.get_heading()
            self.pose_x, self.pose_y = self.get_position()

            linear_error = self.calulate_linear_error()
            angular_error = self.calulate_angular_error()

            self.errors.append(linear_error)
            sum_angular_error += (angular_error * self.dt)
            sum_linear_error += (linear_error * self.dt)

            # calculate PID for linear speed
            P = self.linear_kp * linear_error
            I = self.linear_ki * sum_linear_error
            D = self.linear_kd * (linear_error - prev_linear_error)
            twist.linear.x = P + I + D

            # calculate PID for angular speed
            P = self.angular_kp * angular_error
            I = self.angular_ki * sum_angular_error
            D = self.angular_kd * (angular_error - prev_angular_error)
            twist.angular.z = P + I + D

            prev_angular_error = angular_error
            prev_linear_error = linear_error

            self.cmd_publisher.publish(twist)

            if self.nearest_dot() < self.epsilon:
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_publisher.publish(twist)
            rospy.sleep(self.dt)

    def on_shutdown(self):
        last_twist = Twist()
        self.cmd_publisher.publish(last_twist)
        rospy.sleep(20)


if __name__ == "__main__":
    controlled = Controller()
    controlled.run()
