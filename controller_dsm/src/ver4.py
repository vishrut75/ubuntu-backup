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
cur_phi = 0.0
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
# u2_ar = []
# u2_b = []

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
    k3 = abs(x3_0)//u1_max
    k3 = k3//abs(u2_max*0.5 - abs(x2_0))
    k3=k3+1
    if(k==1):
        print(k3,'k3')
    if(k3>100):
        k3=100
    if(k2>100):
        k2 = 100
    k2 = max(k2,k3)
    k1 = max(k1,k2)

    if(val==1):
        return (1 - min(k/k1,1))*x1_0
    elif(val==2):
        # return tan((1 - min(k/k2,1))*atan(x2_0))
        return (1 - min(k/k2,1))*x2_0
    elif(val==3):
        return (1 - min(k/k3,1))*x3_0
    else:
        return k<=k1
    
def u_to_v(u1,u2):
    global speed
    global cur_theta
    global v_max
    global w_max
    global cur_phi

    v = u1/cos(cur_theta-cur_phi)
    w1 = u2*cos(cur_theta-cur_phi)*cos(cur_theta-cur_phi)
    w = w1 + u1*sin(cur_theta-cur_phi)
    if(abs(v)>v_max):
        v = v_max*sgm(v)
    if(abs(w)>w_max):
        w = w_max*sgm(w)
    speed.linear.x = v
    speed.angular.z = w
    v_ar.append(v)
    w_ar.append(w)

def control_calc(i1,i2,i3,s1,s2,s3,tau,d10,d20):
    A1 = s1-i1
    A2 = s2-i2
    A3 = s3-i3
    #print(A1,A2,A3,i1,i2,i3)
    # w = abs(w_max/(cos(cur_theta)*cos(cur_theta)))
    U1 = A1 - d10
    # if(abs(U1)<0.1):
    #      print(U1)
    #     U1 = sgm(U1)*(-0.2)
    fac_U = U1
    U21 = 4*(A3/fac_U) - 4*i2 - A2 - (4*d20/fac_U)
    U22 = 3*A2 -4*(A3/fac_U) + 4*i2 + (4*d20/fac_U)
    # u2_ar.append(U21/tau)
    # u2_ar.append(U22/tau)
    # val = max(abs(U22),abs(U21))
    # if(val<w*tau or sgm(U21)*sgm(U22)>=0):
        # u2_b.append(U21/tau)
        # u2_b.append(U22/tau)
    return U1/tau, U21/tau, U22/tau
    # val = val/w
    # U21 = U21/val
    # U22 = U22/val
    # # u2_b.append(U21)
    # # u2_b.append(U22)
    # return U1/tau, U21, U22

rospy.init_node("controller_dsm")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
# pub1 = rospy.Publisher("/expec", Odometry, queue_size=1)

r = rospy.Rate(5)


goal_x = float(input('x'))
goal_y = float(input('y'))
goal_theta = cvt_angle(float(input('theta')))
6
while not trig :
    print("waiting")
    r.sleep()

cur_x,cur_y,cur_theta = cvt_coordinate()
cur_phi = atan2(cur_y,cur_x)
cur_theta = cvt_angle(cur_theta)
x1_0 = sqrt(cur_x**2+cur_y**2)
x2_0 = tan(cur_theta-cur_phi)
x3_0 = cur_phi
while True:
    if abs(x1_0) < 0.3 and abs(sin(cur_theta))>0.2:
        speed.linear.x = 0.0
        speed.angular.z = 0.5
    elif abs(x2_0)>=5.0 :
        speed.linear.x = 0
        speed.angular.z = -0.5*sgm(cur_x)*sgm(cur_y)
    else:
        break
    pub.publish(speed)
    r.sleep()
    cur_x,cur_y,cur_theta = cvt_coordinate()
    cur_theta = cvt_angle(cur_theta)
    x1_0 = sqrt(cur_x**2+cur_y**2)
    x2_0 = tan(cur_theta-cur_phi)
    x3_0 = cur_phi
    
print(cur_x,cur_y,cur_theta)

x_ar.append(cur_x)
y_ar.append(cur_y)
z_ar.append(cur_theta)
x_ex.append(x1_0)
y_ex.append(x2_0)
z_ex.append(x3_0)

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
    cur_phi = atan2(cur_y,cur_x)
    x1 = sqrt(cur_x**2+cur_y**2)
    x2 = tan(cur_theta-cur_phi)
    x3 = cur_phi
    #print(x1,sin(cur_theta))
    while True:
        x_ar.append(cur_x)
        y_ar.append(cur_y)
        z_ar.append(cur_theta)
        x_ex.append(x1)
        y_ex.append(cur_theta)
        z_ex.append(cur_phi)
        if abs(x1) < 0.3 and abs(sin(cur_theta))>0.2:
            speed.linear.x = 0.0
            speed.angular.z = 0.5
            v_ar.append(0)
            w_ar.append(0.5)
        elif abs(x2)>=5.0 :
            speed.linear.x = 0
            speed.angular.z = -0.5*sgm(cur_x)*sgm(cur_y)
            v_ar.append(0)
            w_ar.append(-0.5*sgm(cur_x)*sgm(cur_y))
        else:
            break
        pub.publish(speed)
        r.sleep()
        cur_x,cur_y,cur_theta = cvt_coordinate()
        cur_theta = cvt_angle(cur_theta)
        x1 = sqrt(cur_x**2+cur_y**2)
        x2 = tan(cur_theta-cur_phi)
        x3 = cur_phi
    if abs(x1)<0.3 and abs(sin(cur_theta))<0.2:
        break
    if k>150:
        print('break')
        break
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
    u1, u21, u22 = control_calc(x1,x2,x3,sd1,sd2,sd3,tau,0.0,0.0)
    # print(d10,d20)
    #print(cur_x,cur_y,cur_theta,cos1(cur_theta))
    #print(u21,u22)
    # if reaching_law(k+1,4):
    #     x_ex.append(sd1)
    #     y_ex.append(sd3)
    #     z_ex.append(sd2)
    # pub1.publish(curOdom(sd1,sd3,atan(sd2)))
    k = k+1
    u_to_v(u1,u21)
    pub.publish(speed)
    r.sleep()
    

    cur_x,cur_y,cur_theta = cvt_coordinate()
    cur_theta = cvt_angle(cur_theta)
    u_to_v(u1,u22)
    pub.publish(speed)
    r.sleep()

u_to_v(0,0)
pub.publish(speed)
# figure3 , ax3 = plt.subplots(1)
# ax3.plot(u2_ar,'r')
# ax3.plot(u2_b,'b')
figure , ax = plt.subplots(1,2)
print(k)
ax[0].plot(x_ar,'r',label='x')
ax[0].plot(y_ar,'g',label='y')
ax[0].plot(z_ar,'b',label='theta')
ax[0].legend()
ax[1].plot(x_ex,'r',label='r')
# ax[1].plot(y_ex,'g',label='x2')
# ax[1].plot(z_ex,'b',label='x3')
ax[1].legend()
figure1 , ax1 = plt.subplots(2,sharex=True)

ax1[0].plot(v_ar,'r')
ax1[0].set_ylabel('linear velocity')
ax1[1].plot(w_ar,'g')
ax1[1].set_ylabel('angular velocity')
plt.show()



