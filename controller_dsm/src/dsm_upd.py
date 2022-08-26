#! /usr/bin/env python3
import time
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, tan, cos, sin, atan
from matplotlib import pyplot as plt

pie = 3.14159265359
pi_2 = 1.57079632679
uv_tf = 1.3
uw_tf=1.0
v_max = 0.5
v_min = 0.1
w_max = 0.8
u1_max = v_max/uv_tf
u1_min = v_min/uv_tf
u2_max = w_max/uw_tf
x=0.0
y=0.0
theta = 0.0
x_ar = []
y_ar = []
z_ar = []
v_ar = []
w_ar = []
x_ex = []
y_ex = []
z_ex = []
goal_x = 0.0
goal_y = 0.0
goal_theta = 0.0
x1_0 = 0
x2_0 = 0
x3_0 = 0
trig = False

speed = Twist()

def cos1(angle):
    val = cos(angle)
    if abs(val)<0.0454:
        return 0.0454*val/abs(val)
    return val

def tan1(angle):
    val = tan(angle)
    if abs(val)>22.0:
        return 22.0*val/abs(val)
    return val

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
    global pi_2

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta = cvt_angle(theta)

def reaching_law(k,val):
    global x1_0
    global x2_0
    global x3_0
    global goal_x
    global goal_y
    global goal_theta

    k1 = (abs(x1_0-goal_x)//u1_max) + 1
    k2 = (abs(x2_0-goal_theta)//u2_max) + 1
    k3 = max(k1,10)

    if(val==1):
        return (1 - min(k/k1,1))*(x1_0-goal_x) + goal_x
    elif(val==2):
        return tan1((1 - min(k/k2,1))*(x2_0-goal_theta) + goal_theta)
    else:
        return (1 - min(k/k3,1))*(x3_0-goal_y) + goal_y

def u_to_v(u1,u2):
    u1 = u1*uv_tf
    u2 = u2*uw_tf
    delta = 0.2
    global speed
    global theta
    global v_max
    global w_max
    speed.linear.x = max(min(v_max,(u1/cos1(theta))),-1*v_max)
    val = u2*cos1(theta)*cos1(theta)
    val = max(min(val,w_max),-1*w_max)
    speed.angular.z = val
    v_ar.append(speed.linear.x)
    w_ar.append(speed.angular.z)

def control_calc(i1,i2,i3,s1,s2,s3,tau,d10,d20):
    A1 = s1-i1
    A2 = s2-i2
    val = (atan(s2)-atan(i2))%pie
    A3 = s3-i3
    # if val<0.1 or val>3.13:
    #     if abs(A1)<0.1:
    #         if abs(A3)<0.1:
    #             return 0,0,0
    #         U11 = A3*tan(theta)
    #     else:
    #         U11 = A1
    #     U12 = U11
    #     U2=0
    # elif abs(i2) >= 22.0:
    #     U2 = A2
    #     U12 = ((4*A3)/U2) + 3*A1
    #     U11 = 2*A1 - U12
    #     return U11/tau, U12/tau, U2/tau
    # else: 
    U2 = A2
    U12 = (4*(A3-i2*A1)/U2) - A1
    U11 = 2*A1 - U12
    return U11/tau, U12/tau, U2/tau
    
def control1_calc(i1,i2,i3,s1,s2,s3,tau,d10,d20):
    A1 = s1-i1
    A2 = s2-i2
    A3 = s3-i3
    #print(A1,A2,A3)
    alpha = atan(i2)
    if abs(i2)>=22.0:
        print('no')
        U1 = A3/tan1(theta)
        U21 = -A2
        U22 = 2*A2
        return U1/tau,U21/tau,U22/tau

    if abs(A1)<abs(u1_min*cos1(alpha)):
        print('yes')
        if abs(d20)<0.2:
            d20 = tau*0.2/(cos1(alpha)**2)#*abs(d20)/d20
        U1 = A3/tan1(theta)
        U21 = A2
        U22 = A2
        return U1/tau, U21/tau, U22/tau
    
    U1 = A1
    fac_U = min(abs(U1),abs(tau*u1_max/cos1(alpha)))*(U1/abs(U1))
    U21 = 4*(A3/fac_U) - 4*i2 - A2 +d20
    U22 = 3*A2 -4*(A3/fac_U) + 4*i2 +d20
    return U1/tau, U21/tau, U22/tau


rospy.init_node("controller_dsm")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

r = rospy.Rate(5)

while not trig :
    print("waiting")
    r.sleep()

print(x,y,theta)
x_ar.append(x)
y_ar.append(y)
z_ar.append(theta)
x_ex.append(x)
y_ex.append(y)
z_ex.append(tan1(theta))

goal_x = float(input('x'))
goal_y = float(input('y'))
goal_theta = cvt_angle(float(input('theta')))

x1_0 = x
x2_0 = theta
x3_0 = y
d1l = 0.0
d2l = 0.0
d1u = 0.0
d2u = 0.0
d10 = 0.0
d20 = 0.0
dd1 = 0.0
dd2 = 0.0
tau = 0.4
k=0

# u_to_v(1.0,0.0)
# pub.publish(speed)
# time.sleep(1)
# u_to_v(0,0)
# pub.publish(speed)
# print(sqrt((x-x1_0)**2 + (y-x3_0)**2),(x2_0-theta)%pie)

while not rospy.is_shutdown():
    x1 = x
    x2 = tan1(theta)
    x3 = y
    sd1 = reaching_law(k+1,1)
    sd2 = reaching_law(k+1,2)
    sd3 = reaching_law(k+1,3)
    d1 = -1*(reaching_law(k,1)-x1)
    d2 = -1*(reaching_law(k,3)-x3)
    d1l = min(d1l,d1)
    d2l = min(d2l,d2)
    d1u = max(d1u,d1)
    d2u = max(d2u,d2)
    d10 = 0.5*(d1l+d1u)
    d20 = 0.5*(d2l+d2u)
    dd1 = 0.5*(d1u-d1l)
    dd2 = 0.5*(d2u-d2l)
    x_ar.append(x)
    y_ar.append(y)
    z_ar.append(theta)
    u1, u21, u22 = control1_calc(x1,x2,x3,sd1,sd2,sd3,tau,d10,d20)
    x_ex.append(sd1)
    y_ex.append(sd3)
    z_ex.append(sd2)
    
    u_to_v(u1,u21)
    pub.publish(speed)
    r.sleep()

    u_to_v(u1,u22)
    pub.publish(speed)
    r.sleep()
    k = k+1

    if abs(x1-goal_x)<0.15 and abs(x3-goal_y)< 0.15 and sin(abs(theta-goal_theta)%pie)<0.2:
        break

    if(k>100):
        print("break")
        break
u_to_v(0,0)
pub.publish(speed)
figure , ax = plt.subplots(1,3)
print(k)
ax[0].plot(x_ar,'r')
ax[0].plot(y_ar,'g')
ax[0].plot(z_ar,'b')

ax[1].plot(x_ex,'r')
ax[1].plot(y_ex,'g')
ax[1].plot(z_ex,'b')

ax[2].plot(v_ar,'r')
ax[2].plot(w_ar,'g')
plt.show()
