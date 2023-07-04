#!/usr/bin/env python3
import rospy
import math
from get_next_distance.srv import GetNextDestination, GetNextDestinationRequest
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


class Robot_controller():
    def __init__(self) -> None:
        rospy.init_node("control_node", anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=10)
        self.path = Path()
        self.rate = rospy.Rate(10)
        self.vel_msg = Twist()
        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.errors = []

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.current_x = self.position.x
        self.current_y = self.position.y
        self.orientation = msg.pose.pose.orientation
        self.quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(self.quaternion)
        self.current_yaw = yaw
        #self.path.header = msg.header
        self.pose = PoseStamped()
        self.path.poses.append(self.pose)
        self.path_publisher.publish(self.path)


    def get_next_destination(self):
        rospy.wait_for_service('/get_next_destination')
        try:
            get_next_destination_srv = rospy.ServiceProxy('/get_next_destination', GetNextDestination)
            
            req = GetNextDestinationRequest(self.current_x, self.current_y)
            resp = get_next_destination_srv(req)
            next_x = resp.next_x
            next_y = resp.next_y
            return next_x, next_y
        except rospy.ServiceException as e:
            print('Service call failed: %s' %e)


    def steering_angle(self, next_x, next_y):
        return math.atan2(next_y - self.current_y, next_x - self.current_x)

    def calculate_distance(self, next_x, next_y):
        return math.sqrt((next_x - self.current_x)**2 + (next_y - self.current_y)**2)
    
    def rotate_to_goal_angle(self, next_x, next_y):
        rotation_angle = self.steering_angle(next_x, next_y)
        self.vel_msg.angular.z = 1.5 * rotation_angle
        self.velocity_publisher.publish(self.vel_msg)
        
        while abs(rotation_angle - self.current_yaw) > 0.1:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0.3
            self.velocity_publisher.publish(self.vel_msg)
        
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def move_forward(self, goal_distance, next_x, next_y):
        current_distance = 0
        self.vel_msg.linear.x = 0.2
        self.velocity_publisher.publish(self.vel_msg)
        while abs(current_distance - goal_distance) > 0.01:
            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            current_distance = self.calculate_distance(next_x, next_y)
        rospy.loginfo(f"goal ditance {goal_distance}, current_distance {current_distance}")
        self.vel_msg.linear.x = 0
        self.velocity_publisher.publish(self.vel_msg)


    def move(self):
        #self.linear_speed = rospy.get_param("/control_node/linear_speed")
        #self.angular_speed = rospy.get_param("/get_next_destination/angular_speed")
        #self.epsilon = rospy.get_param("/get_next_destination/epsilon")
        running_index = 0
        rospy.loginfo(f"running index {running_index}")
        while not rospy.is_shutdown() and running_index < 5:
            next_x, next_y = self.get_next_destination()
            goal_distance = self.calculate_distance(next_x, next_y)
            while goal_distance > 0.5: 
                self.rotate_to_goal_angle(next_x, next_y)
                self.move_forward(goal_distance, next_x, next_y)

            running_index += 1
            rospy.loginfo(f"running index {running_index}")
            self.rate.sleep()
        

if __name__=='__main__':
    rc = Robot_controller()
    rc.move()