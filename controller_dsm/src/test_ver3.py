#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, tan, cos, sin, atan
from matplotlib import pyplot as plt

pie = 3.14159265359
v_max = 0.4
v_min = 0.2
w_max = 0.8
x=0.0
y=0.0
theta = 0.0
goal_x = 0.0
goal_y = 0.0
goal_theta = 0.0
cur_x = 0.0
cur_y = 0.0
cur_theta = 0.0
x1_0 = 0
x2_0 = 0
x3_0 = 0
trig = False
k=0
speed = Twist()
x_ar = []
y_ar = []
z_ar = []
v_ar = []
w_ar = []
x_ex = []
y_ex = []
z_ex = []

def sgm(val):
    if val==0:
        return 0
    if val>0:
        return 1
    return -1
    
def cvt_angle(ang):
    ang = ang%(2*pie)
    if(ang>pie):
        return ang-2*pie
    return ang

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

def newOdom(msg):
    global trig
    trig = True
    global x
    global y
    global theta
    global pie

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta = cvt_angle(theta)

def reaching_law(k,val):
    global x1_0
    global x2_0
    global x3_0
    t=0.4
    cos_min = abs(cos(atan(x2_0)))
    u1_max = v_max*t*cos_min
    u2_max = w_max*t
    k1 = abs(x1_0)//u1_max + 1
    if(k==1):
        print(k1,'k1')
    k2 = abs(x2_0)//u2_max + 1
    if(k==1):
        print(k2,'k2')
    k3 = 4*abs(x3_0)//u1_max
    temp = k3 - 2*abs(x2_0)
    temp = temp//(2*abs(x2_0) + u2_max)
    k3 = k3//u2_max
    if(k3<0):
        k3=0
    k3 = k3+1
    if(k==1):
        print(k3,'k3') 
    k2 = max(temp+1,k2)
    # if(k3>100):
    #     k3=100
    # if(k2>100):
    #     k2 = 100
    # k2 = max(k3+1,k2)
    # k1 = max({k1,k2,k3})

    if(val==1):
        return (1 - min(k/k1,1))*x1_0
    elif(val==2):
        # return tan((1 - min(k/k2,1))*atan(x2_0))
        return (1 - min(k/k2,1))*x2_0
    elif(val==3):
        return (0*(1 - min(k/k3,1)))*x3_0
    else:
        return k<=k1
    
def reaching_law1(k,val):
    global x1_0
    global x2_0
    global x3_0
    t=0.4
    cos_min = abs(cos(atan(x2_0)))
    u1_max = v_max*t*cos_min
    u2_max = w_max*t
    temp_chk = 4*abs(x2_0)-u2_max
    k1 = abs(x1_0)//u1_max + 1
    if(k==1):
        print(k1,'k1')
    if(temp_chk>0):
        k3 = 2*abs(x3_0)//min(1,abs(x2_0))
        k3 = k3//u1_max
        k3=k3+1
        k2 = 2*abs(x2_0)//u2_max
        if(k==1):
            print(k2)
        k2 = max(k2,4*k3*abs(x2_0)//temp_chk)
        k2=max(k2+1,k3+1)
    else :
        k3 = 4*abs(x3_0)/u1_max
        k3 = k3//abs(u2_max)
        k3 = k3+1
        k2 = 2*abs(x2_0)//u2_max
        k2 = max(k3+1,k2+1)
    if(k==1):
        print(k3,'k3')
    if(k==1):
        print(k2,'k2')
    # k2 = max(k3+1,k2)
    k1 = max(k1,k2+1)

    if(val==1):
        return (1 - min(k/k1,1))*x1_0
    elif(val==2):
        # return tan((1 - min(k/k2,1))*atan(x2_0))
        return (1 - min(k/k2,1))*x2_0
    elif(val==3 and temp_chk>0):
        return ((1 - min(k/k3,1))**2)*x3_0
    elif(val==3):
        return ((1 - min(k/k3,1)))*x3_0
    else:
        return k<=k1

def u_to_v(u1,u2):
    global speed
    global cur_theta
    global v_max
    global w_max

    speed.angular.x = u1
    speed.angular.y = u2
    v = u1/cos(cur_theta)
    w = u2*cos(cur_theta)*cos(cur_theta)
    if(abs(v)>v_max):
        v = v_max*sgm(v)
    if(abs(w)>w_max):
        w = w_max*sgm(w)
    v_ar.append(v)
    w_ar.append(w)


def control_calc(i1,i2,i3,s1,s2,s3,tau,d10,d20):
    global cur_theta
    A1 = s1-i1
    A2 = s2-i2
    A3 = s3-i3
    U1 = A1 - d10
    val = 4*A3/U1
    u2m = w_max/(cos(cur_theta)**2)
    if A2>=0:
        if val>(4*i2 + A2 + u2m*tau):
            U1 = 4*A3/(4*i2 + A2 + u2m*tau)
        elif val<(4*i2 + 3*A2 - u2m*tau):
            U1 = 4*A3/(4*i2 + 3*A2 - u2m*tau)
    else:
        if val>(3*A2 + 4*i2 + u2m*tau):
            U1 = 4*A3/(3*A2 + 4*i2 + u2m*tau)
        elif val<(A2 + 4*i2 - u2m*tau):
            U1 = 4*A3/(A2 + 4*i2 - u2m*tau)
    fac_U = U1
    U21 = 4*(A3/fac_U) - 4*i2 - A2 - (4*d20/fac_U)
    U22 = 3*A2 -4*(A3/fac_U) + 4*i2 + (4*d20/fac_U)
    return U1/tau, U21/tau, U22/tau

rospy.init_node("controller_dsm")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/state_vel", Twist, queue_size=1)

r = rospy.Rate(5)


goal_x = float(input('x'))
goal_y = float(input('y'))
goal_theta = cvt_angle(float(input('theta')))
speed.linear.x=goal_x
speed.linear.y=goal_y
speed.linear.z=goal_theta
while not trig :
    print("waiting")
    r.sleep()

cur_x,cur_y,cur_theta = cvt_coordinate()
cur_theta = cvt_angle(cur_theta)
while True:
    if abs(cur_x) < 0.2 and abs(cur_y)<=0.2 and abs(sin(cur_theta))>0.2:
        u_to_v(0,0.5)
    elif(tan(cur_theta)>=5.0):
        u_to_v(0,0.2)
    elif abs(cur_x) < 0.2 and abs(cur_y) > 0.2:
        u_to_v(0.2*sgm(-1*cur_y)*sgm(sin(cur_theta)))
    else :
        break

    pub.publish(speed)
    r.sleep()
    cur_x,cur_y,cur_theta = cvt_coordinate()
    cur_theta = cvt_angle(cur_theta)

x_ar.append(cur_x)
y_ar.append(cur_y)
z_ar.append(cur_theta)
x1_0 = cur_x
x2_0 = tan(cur_theta)
x3_0 = cur_y

d1l = 0.0
d2l = 0.0
d1u = 0.0
d2u = 0.0
d10 = 0.0
d20 = 0.0
dd1 = 0.0
dd2 = 0.0
tau = 0.4

while not rospy.is_shutdown():

    cur_x,cur_y,cur_theta = cvt_coordinate()
    cur_theta = cvt_angle(cur_theta)
    sd1 = reaching_law1(k+1,1)
    sd2 = reaching_law1(k+1,2)
    sd3 = reaching_law1(k+1,3)
    while True:
        x_ar.append(cur_x)
        y_ar.append(cur_y)
        z_ar.append(cur_theta)
        # if abs(cur_x)<0.2 and (abs(cur_y)>0.2 or abs(sin(cur_theta))>0.2) :
        #     u_to_v(0.2*sgm(cur_x-sd1),0)
        # elif(tan(cur_theta)>=5.0):
        #     u_to_v(0,0.2)
        # elif abs(cur_x) < 0.2 and abs(cur_y) > 0.2:
        #     u_to_v(0.2*sgm(-1*cur_y)*sgm(sin(cur_theta)))
        # else :
        break
        # pub.publish(speed)
        # r.sleep()
        # cur_x,cur_y,cur_theta = cvt_coordinate()
        # cur_theta = cvt_angle(cur_theta)
    if abs(cur_x)<0.2 and abs(cur_y)< 0.2 and abs(sin(cur_theta))<0.2:
        break
    # if k>150:
    #     print('break')
    #     break
    x1 = cur_x
    x2 = tan(cur_theta)
    x3 = cur_y
    d1 = -1*(reaching_law1(k,1)-x1)
    d2 = -1*(reaching_law1(k,3)-x3)
    d1l = min(d1l,d1)
    d2l = min(d2l,d2)
    d1u = max(d1u,d1)
    d2u = max(d2u,d2)
    d10 = 0.5*(d1l+d1u)
    d20 = 0.5*(d2l+d2u)
    dd1 = 0.5*(d1u-d1l)
    dd2 = 0.5*(d2u-d2l)
    u1, u21, u22 = control_calc(x1,x2,x3,sd1,sd2,sd3,tau,0.0,0.0)
    k = k+1
    u_to_v(u1,u21)
    pub.publish(speed)
    r.sleep()

    u_to_v(u1,u22)
    pub.publish(speed)
    r.sleep()

u_to_v(0,0)
pub.publish(speed)
figure , ax = plt.subplots(1,2)
print(k)
ax[0].plot(x_ar,'r',label='x')
ax[0].plot(y_ar,'g',label='y')
ax[0].plot(z_ar,'b',label='theta')
ax[0].legend()
ax[1].plot(x_ar,y_ar)
ax[1].set_xlabel('x')
ax[1].set_ylabel('y')
plt.show()

figure1 , ax1 = plt.subplots(2,sharex=True)

ax1[0].plot(v_ar,'r')
ax1[0].set_ylabel('linear velocity')
ax1[1].plot(w_ar,'g')
ax1[1].set_ylabel('angular velocity')
plt.show()


