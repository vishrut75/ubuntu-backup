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
v_max = 0.4
v_min = 0.1
w_max = 0.8
u1_max = v_max/uv_tf
u1_min = v_min/uv_tf
u2_max = w_max/uw_tf
x=0.0
y=0.0
theta = 0.0
cur_x = 0.0
cur_y = 0.0
cur_theta = 0.0
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
k=0

speed = Twist()

def sgm(val):
    if val==0:
        return 0
    if val>0:
        return 1
    return -1

def cos1(angle):
    val = cos(angle)
    if abs(val)<0.0454:
        return 0.0454*sgm(val)
    if abs(val)>0.998:
        return 0.998*val/abs(val)
    return val

def tan1(angle):
    val = tan(angle)
    if abs(val)>22.0:
        return 22.0*val/abs(val)
    if abs(val)<0.0454:
        return 0.0454*sgm(val)
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
    t=0.4
    u1_avg = u1_max*t/1.414
    u2_avg = u2_max*t*2
    k1 = (abs(x1_0-goal_x)//u1_avg) + 1
    k2 = (abs(x2_0-tan1(goal_theta))//u2_avg) + 1
    k3 = abs(x3_0 - goal_y)//u1_avg
    k3 = k3//(u2_avg*0.5 - max(x2_0,tan1(goal_theta)))
    k3=k3+1
    k3 = max(min(k3,40),1)
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
    global cur_theta
    global v_max
    global w_max
    speed.linear.x = max(min(v_max,(u1/cos1(cur_theta))),-1*v_max)
    val = u2*cos1(cur_theta)*cos1(cur_theta)
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
    w = u2_max/(cos1(alpha)*cos1(alpha))
    
    U1 = A1 - d10
    fac_U = U1
    if abs(U1)<abs(tau*v_min*cos1(alpha)):
        if i2>=22.0:
            print('no')
            U1 = A3/i2
            fac_U = U1
        else:
            print('yes')
            U1 = A3/i2
        # U1_ = A3
        # U1 = U1_/tan(alpha)
        # fac_U = U1_
        # U21 = 4*(A1/fac_U) - 4*i2 - A2 - (4*d20/fac_U)
        # U22 = 3*A2 -4*(A1/fac_U) + 4*i2 + (4*d20/fac_U)
        # val = max(abs(U22),abs(U21))
        # if(val<abs(w*tau)):
        #     return U1/tau, U21/tau, U22/tau
        # val = val/w
        # U21 = U21/val
        # U22 = U22/val
        # return U1/tau, U21, U22
            #U1 = tau*(v_min)*abs(cos1(alpha))*sgm(U1)
    #else:
        #print('no')
    U21 = 4*(A3/fac_U) - 4*i2 - A2 - (4*d20/fac_U)
    U22 = 3*A2 -4*(A3/fac_U) + 4*i2 + (4*d20/fac_U)
    val = max(abs(U22),abs(U21))
    if(val<w*tau or sgm(U21)*sgm(U22)>=0):
        return U1/tau, U21/tau, U22/tau
    val = val/w
    U21 = U21/val
    U22 = U22/val

    return U1/tau, U21, U22


rospy.init_node("controller_dsm")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

r = rospy.Rate(5)

while not trig :
    print("waiting")
    r.sleep()

print(x,y,theta)

goal_x = float(input('x'))
goal_y = float(input('y'))
goal_theta = cvt_angle(float(input('theta')))


x_ar.append(x)
y_ar.append(y)
z_ar.append(theta)
x_ex.append(x)
y_ex.append(y)
z_ex.append(tan1(theta))

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

while not rospy.is_shutdown():
    if abs(x-goal_x)<0.15 and abs(y-goal_y)< 0.15 and abs(sin(theta-goal_theta))<0.2:
        break
    cur_x = x
    cur_y = y
    cur_theta = theta
    x1 = cur_x
    x2 = tan1(cur_theta)
    x3 = cur_y
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
    x_ar.append(cur_x)
    y_ar.append(cur_y)
    z_ar.append(cvt_angle(cur_theta))
    u1, u21, u22 = control_calc(x1,x2,x3,sd1,sd2,sd3,tau,0.1,0.1)
    print(u1,u21,u22)
    if reaching_law(k+1,4):
        x_ex.append(sd1)
        y_ex.append(sd3)
        z_ex.append(sd2)
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
ax[0].plot(x_ar,'r')
ax[0].plot(y_ar,'g')
ax[0].plot(z_ar,'b')

ax[1].plot(x_ex,'r')
ax[1].plot(y_ex,'g')
ax[1].plot(z_ex,'b')
plt.show()

figure1 , ax1 = plt.subplots(2,sharex=True)

ax1[0].plot(v_ar,'r')
ax1[1].plot(w_ar,'g')
plt.show()
