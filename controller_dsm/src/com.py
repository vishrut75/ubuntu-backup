#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, tan, cos, sin, atan
from matplotlib import pyplot as plt

x=0
y=0
theta=0
pie = 3.14159265359
v_max = 0.4
v_min = 0.2
w_max = 0.8
goal_x = 0
goal_y = 0
goal_theta = 0
cur_x = 0.0
cur_y = 0.0
cur_theta = 0.0
u1=0
u21=0
u22=0
speed = Twist()

def sgm(val):
    if val==0:
        return 0
    if val>0:
        return 1
    return -1

def u_to_v(u1,u2):
    global speed
    global cur_theta
    global v_max
    global w_max

    v = u1/cos(cur_theta)
    w = u2*cos(cur_theta)*cos(cur_theta)
    # if (abs(cur_x)<0.2 and abs(cur_y)<0.2):
    #     print(cur_x,cur_y)
    #     v=0
    #     if abs(sin(cur_theta))>0.2:
    #         w=0.5
    #     else :
    #         w=0
    # elif(cos(cur_theta)<0.2 and u2!=0):
    #     w=0.2*sgm(u2)
    if(abs(v)>v_max):
        v = v_max*sgm(v)
    if(abs(w)>w_max):
        w = w_max*sgm(w)
    speed.linear.x = v
    speed.angular.z = w

def cvt_coordinate():
    global goal_x
    global goal_y
    global goal_theta
    global x
    global y
    global theta
    r_x = x - goal_x
    r_y = y - goal_y
    r_th = theta - goal_theta
    l = sqrt(r_x**2+r_y**2)
    ang = atan2(r_y,r_x) - goal_theta
    r_x = l*cos(ang)
    r_y = l*sin(ang)
    return r_x, r_y, r_th  

def cvt_angle(ang):
    ang = ang%(2*pie)
    if(ang>pie):
        return ang-2*pie
    return ang

def newOdom(msg):
    global trig
    trig = True
    global x
    global y
    global theta
    global pie
    global cur_x
    global cur_y
    global cur_theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta = cvt_angle(theta)
    cur_x,cur_y,cur_theta = cvt_coordinate()
    cur_theta = cvt_angle(cur_theta)


def real_vel(msg):
    global goal_x
    global goal_y
    global goal_theta
    global u1
    global u21
    global u22
    goal_x = msg.linear.x
    goal_y = msg.linear.y
    goal_theta = msg.linear.z
    u1 = msg.angular.x
    u21 = msg.angular.y
    u22 = msg.angular.z

rospy.init_node("communicate_velocity")
sub = rospy.Subscriber("/state_vel", Twist, real_vel)
sub1 = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

r = rospy.Rate(16)

while not rospy.is_shutdown():
    u_to_v(u1,u21)
    pub.publish(speed)
    r.sleep()