#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, tan, cos, sin, atan
from matplotlib import pyplot as plt

pie = 3.14159265359
pi_2 = 1.57079632679
uv_tf = 1.0
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
    theta = cvt_angle(theta)

def reaching_law(k,val):
    global x1_0
    global x2_0
    global x3_0
    global goal_x
    global goal_y
    global goal_theta
    u1_avg = u1_max/1.414
    u2_avg = u2_max*2.0
    t=0.4
    k1 = (abs(x1_0-goal_x)//u1_avg) + 1
    k2 = min((abs(8*x2_0)//abs(t*t*t*u1_avg*u2_avg)) + 1, k1)
    spr = t*t*u2_avg-x2_0
    spr = spr*t*u1_avg
    k3 = (abs(8*x3_0)//abs(spr))+1
    k1 = max({k1,k2,k3})
    if(k==1):
        print(k1,k2,k3)
    if(val==1):
        return (1 - min(k/k1,1))*(x1_0-goal_x) + goal_x
    elif(val==2):
        return (1 - min(k/k2,1))*(x2_0-tan1(goal_theta)) + tan1(goal_theta)
    elif(val==3):
        return (1 - min(k/k3,1))*(x3_0-goal_y) + goal_y
    else:
        return k<=k1

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
    A3 = s3-i3
    #print(A1,A2,A3)
    alpha = atan(i2)
    
    if abs(A1)<abs(u1_min/cos1(alpha)):
        print('yes')
        U1 = A3/tan1(theta) - d20
        U21 = A2
        U22 = A2
        return U1/tau, U21/tau, U22/tau

    U1 = A1 - d10
    fac_U = U1
    U21 = 4*(A3/fac_U) - 4*i2 - A2 - (4*d20/fac_U)
    U22 = 3*A2 -4*(A3/fac_U) + 4*i2 + (4*d20/fac_U)
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
x2_0 = tan1(theta)
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
    u1, u21, u22 = control_calc(x1,x2,x3,sd1,sd2,sd3,tau,0.1,0.1)
    if reaching_law(k+1,4):
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

    if(k>200):
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
