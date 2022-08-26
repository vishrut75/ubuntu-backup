#! /usr/bin/env python3
from math import sqrt, atan2, cos, sin
x=0
y=0
theta=0
goal_x=0
goal_y=0
goal_theta=0
pie = 3.14159265359

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
    ang = atan2(r_x,r_y) - goal_theta
    r_x = l*cos(ang)
    r_y = l*sin(ang)
    return r_x, r_y, r_th  


def cvt_angle(ang):
    ang = ang%(2*pie)
    if(ang>pie):
        return ang-2*pie
    return ang

while True:

    goal_x = float(input('x'))
    goal_y = float(input('y'))
    goal_theta = cvt_angle(float(input('theta')))
    
    x = float(input('x'))
    y = float(input('y'))
    theta = cvt_angle(float(input('theta')))

    a,b,c = cvt_coordinate()
    print(a,b,c)
    
