#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class FollowTheWall():

    def __init__(self):
        rospy.init_node('follow_the_wall', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.k_i = 0.0
        self.k_p = 1.2
        self.k_d = 18

        self.dt = 0.005
        self.v = 0.4
        self.D = 1

        rate = 1 / self.dt
        self.r = rospy.Rate(rate)

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.errs = []

    
    def distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        rng = laser_data.ranges[0:180]
        d = min(rng)
        return d
    
    def follow_wall(self):
        d = self.distance_from_wall()
        sum_i_theta = 0
        prev_theta_error = 0
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            err = d - self.D
            self.errs.append(err)
            sum_i_theta += err * self.dt

            P = self.k_p * err
            I = self.k_i * sum_i_theta
            D = self.k_d * (err - prev_theta_error)

            move_cmd.angular.z = P + I + D
            prev_theta_error = err
            move_cmd.linear.x = self.v
            d = self.distance_from_wall()
            self.r.sleep()

    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        plt.plot(list(range(len(self.errs))), self.errs, label='errs')
        plt.axhline(y=0, color='R')
        plt.draw()
        plt.legend(loc='upper left', frameon=False)
        plt.savefig(f"err_{self.k_p}_{self.k_d}_{self.k_i}.png")
        plt.show()

        rospy.sleep(1)

if __name__ == '__main__':
    try:
        follow_the_wall = FollowTheWall()
        follow_the_wall.follow_wall()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Terminated.")