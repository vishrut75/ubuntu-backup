#! /usr/bin/env python3

from numpy.lib.arraysetops import setdiff1d
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, tan, cos

x=0.0
y=0.0
theta = 0.0

speed = Twist()

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("controller_dsm")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

r = rospy.Rate(5)

mode = input('ter/goal (y/n)')

if(mode == 'y'):
    while(mode == 'y'):
        speed.linear.x = float(input('linear'))
        speed.angular.z = float(input('angular'))
        pub.publish(speed)
        r.sleep()
        mode = input('ter/goal (y/n)')
        print(x,y,theta)

else:
    goal_x = float(input('x'))
    goal_y = float(input('y'))

    while not rospy.is_shutdown():
        inc_x = goal_x-x
        inc_y = goal_y-y
        goal_theta = atan2(inc_x,inc_y)
        
        print(x,y,theta)

        if(abs(theta-goal_theta)>0.1):
            speed.angular.z=0.3
            speed.linear.x=0.0
        else:
            speed.angular.z=0.0
            speed.linear.x=0.5
        pub.publish(speed)
        r.sleep()

        if (inc_x**2 + inc_y**2)< 0.2  :
            break

