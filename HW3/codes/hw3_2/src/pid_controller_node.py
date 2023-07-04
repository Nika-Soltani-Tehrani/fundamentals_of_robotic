#!/usr/bin/python3

import tf
import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped


class PIDController:
    
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
        self.goal_x = 0
        self.goal_y = 0
        self.goal_orientation = 0

        shape_name = 'rectangle'
        self.set_parameters(shape_name)
        self.set_path(shape_name)


    def set_parameters(self, shape_name):
        # getting specified parameters
        if shape_name == 'rectangle':
            # angular controller parameters
            self.angular_kp = 2.6
            self.angular_ki = 0.001
            self.angular_kd = 0.7

            # linear controller parameters
            self.linear_kp = 0.75
            self.linear_ki = 0.001
            self.linear_kd = 2.5

            self.dt = 0.005
            self.D = 0.0
            self.epsilon = 0.2
            rate = 1 / self.dt
            self.r = rospy.Rate(rate)
        
        if shape_name == 'star':
            # angular controller parameters
            self.angular_kp = 2.6
            self.angular_ki = 0.001
            self.angular_kd = 0.9

            # linear controller parameters
            self.linear_kp = 0.75
            self.linear_ki = 0.001
            self.linear_kd = 1.5

            self.dt = 0.005
            self.epsilon = 0.2
            self.D = 0.3
            rate = 1 / self.dt
            self.r = rospy.Rate(rate)

        if shape_name == "spiral_path":
            # angular controller parameters
            self.angular_kp = 1.75
            self.angular_ki = 0.001
            self.angular_kd = 0.3

            # linear controller parameters
            self.linear_kp = 0.5
            self.linear_ki = 0.001
            self.linear_kd = 1.5

            self.dt = 0.05
            self.epsilon = 0.2
            self.D = 0.0
            rate = 1 / self.dt
            self.r = rospy.Rate(rate)
        

        
    def set_path(self, shape_name):
        if shape_name == 'rectangle':
            X1 = np.linspace(-3, 3, 100)
            Y1 = np.array([2] * 100)

            Y2 = np.linspace(2, -2, 100)
            X2 = np.array([3] * 100)

            X3 = np.linspace(3, -3, 100)
            Y3 = np.array([-2] * 100)

            Y4 = np.linspace(-2, 2, 100)
            X4 = np.array([-3] * 100)

            self.X = np.concatenate([X1, X2, X3, X4]).tolist()
            self.Y = np.concatenate([Y1, Y2, Y3, Y4]).tolist()
            self.X_length = len(self.X)
            shape_tuple = (self.X, self.Y)
            self.shape = shape_tuple
        
        if shape_name == 'star':
            X1 = np.linspace(0, 3, 100)
            Y1 = -(7/3) * X1 + 12

            X2 = np.linspace(3, 10, 100)
            Y2 = np.array([5] * 100)

            X3 = np.linspace(10, 4, 100)
            Y3 = (5/6) * X3 - (10/3)

            X4 = np.linspace(4, 7, 100)
            Y4 = -(3) * X4 + 12

            X5 = np.linspace(7, 0, 100)
            Y5 = -(4/7) * X5 - 5

            X6 = np.linspace(0, -7, 100)
            Y6 = (4/7) * X6 - 5

            X7 = np.linspace(-7, -4, 100)
            Y7 = (3) * X7 + 12

            X8 = np.linspace(-4, -10, 100)
            Y8 = -(5/6) * X8 - (10/3)

            X9 = np.linspace(-10, -3, 100)
            Y9 = np.array([5] * 100)

            X10 = np.linspace(-3, 0, 100)
            Y10 = (7/3) * X10 + 12

            self.X = np.concatenate([X1, X2, X3, X4, X5, X6, X7, X8, X9, X10]).tolist()
            self.Y = np.concatenate([Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8, Y9, Y10]).tolist()
            self.X_length = len(self.X)
            shape_tuple = (self.X, self.Y)
            self.shape = shape_tuple

        if shape_name == "spiral_path":    
            a = 0.17
            k = math.tan(a)
            X , Y = [] , []

            for i in range(150):
                t = i / 20 * math.pi
                dx = a * math.exp(k * t) * math.cos(t)
                dy = a * math.exp(k * t) * math.sin(t)
                X.append(dx)
                Y.append(dy) 
            self.X = X
            self.Y = Y
            self.X_length = len(self.X)
            shape_tuple = (self.X, self.Y)
            self.shape = shape_tuple


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
            for x, y in zip(*self.shape):
                min_local_distance = self.calculate_distance(self.pose_x, x, self.pose_y, y)
                if min_local_distance < min_distance:
                    min_distance = min_local_distance
                    min_x = x
                    min_y = y

        if mode == 1:
            for x, y in zip(*self.shape):
                diff_ang = np.arctan2((y - self.pose_y), (x - self.pose_x))
                diff_ang = self.calculate_difference_angle(diff_ang)
                if -math.pi / 2 < diff_ang < math.pi / 2:
                    min_local_distance = self.calculate_distance(self.pose_x, x, self.pose_y, y)
                    if min_distance > min_local_distance and min_local_distance > abs(self.epsilon):
                        min_distance = min_local_distance
                        min_x = x
                        min_y = y

        self.goal_x = min_x
        self.goal_y = min_y
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
            rospy.sleep(self.dt)

    def on_shutdown(self):
        last_twist = Twist()
        self.cmd_publisher.publish(last_twist)
        rospy.sleep(20)


if __name__ == "__main__":
    controlled = PIDController()
    controlled.run()
