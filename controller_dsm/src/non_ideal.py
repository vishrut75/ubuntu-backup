#! /usr/bin/env python3
from math import atan2, sqrt, tan, cos, sin, atan
from matplotlib import pyplot as plt

pie = 3.14159265359
def cvt_angle(ang):
    ang = ang%(2*pie)
    if(ang>pie):
        return ang-2*pie
    return ang


def sgm(val):
    if val==0:
        return 0
    if val>0:
        return 1
    return -1

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

def reaching_law(k,val):
    global x1_0
    global x2_0
    global x3_0
    global k1
    global k2
    global k3
    if(val==1):
        return (1 - min(k/k1,1))*x1_0
    elif(val==2):
        return (1 - min(k/k2,1))*x2_0
    elif(val==3):
        return (1 - min(k/k3,1))*x3_0

def u_to_v(u1,u2):
    global tau
    global x
    global y
    global theta
    v = u1/cos(cur_theta)
    w = u2*cos(cur_theta)*cos(cur_theta)
    v_ar.append(v)
    w_ar.append(w)
    x = x + v*cos(theta)*tau/2
    y = y + v*sin(theta)*tau/2
    theta = theta + w*tau/2
    theta = cvt_angle(theta)

def u_to_v1(u1,u2):
    global tau
    global x
    global y
    global theta
    v = u1/cos(cur_theta)
    w = u2*cos(cur_theta)*cos(cur_theta)
    if(abs(v)>0.4):
        v = 0.4*sgm(v)
    if(abs(w)>0.8):
        w = 0.8*sgm(w)
    v_ar1.append(v)
    w_ar1.append(w)
    x = x + v*cos(theta)*tau/2
    y = y + v*sin(theta)*tau/2
    theta = theta + w*tau/2
    theta = cvt_angle(theta)

def control_calc(i1,i2,i3,s1,s2,s3):
    global tau
    A1 = s1-i1
    A2 = s2-i2
    A3 = s3-i3
    w = abs(0.8/(cos(cur_theta)*cos(cur_theta)))
    U1 = A1
    U21 = 4*(A3/U1) - 4*i2 - A2
    U22 = 3*A2 - 4*(A3/U1) + 4*i2
    val = max(abs(U22),abs(U21))
    return U1/tau, U21/tau, U22/tau


k=0
tau = 0.4

x_ar = []
y_ar = []
z_ar = []
v_ar = []
w_ar = []

x = float(input('x'))
y = float(input('y'))
theta = cvt_angle(float(input('theta')))

goal_x = float(input('goal x'))
goal_y = float(input('goal y'))
goal_theta = cvt_angle(float(input('goal theta')))

k1 = float(input('k1'))
k2 = float(input('k2'))
k3 = cvt_angle(float(input('k3')))

cur_x,cur_y,cur_theta = cvt_coordinate()
cur_theta = cvt_angle(cur_theta)

print(cur_x,cur_y,cur_theta)

while True:
    if abs(tan(cur_theta)) <= 0.454 or abs(tan(cur_theta))>=22.0 :
        theta = theta + 0.5*tau/2
        theta = cvt_angle(theta)
    elif abs(cur_x) < 0.2 :
        x = x + (0.2*tau*cos(cur_theta)/2)
        y = y + (0.2*tau*sin(cur_theta)/2)
    else:
        break
    cur_x,cur_y,cur_theta = cvt_coordinate()
    cur_theta = cvt_angle(cur_theta)


x_ar.append(cur_x)
y_ar.append(cur_y)
z_ar.append(cur_theta)
x1_0 = cur_x
x2_0 = tan(cur_theta)
x3_0 = cur_y

while True:
    cur_x,cur_y,cur_theta = cvt_coordinate()
    cur_theta = cvt_angle(cur_theta)
    if abs(cur_x)<0.15 and abs(cur_y)< 0.15 and abs(sin(cur_theta))<0.2:
        break
    if k>100:
        print('break')
        break

    while True:
        if abs(tan(cur_theta)) <= 0.454 or abs(tan(cur_theta))>=22.0 :
            theta = theta + 0.5*tau/2
            theta = cvt_angle(theta)
        elif abs(cur_x) < 0.2 and abs(cur_y) > 0.2:
            x = x + (0.2*tau*cos(cur_theta)/2)
            y = y + (0.2*tau*sin(cur_theta)/2)
        else:
            break
        cur_x,cur_y,cur_theta = cvt_coordinate()
        cur_theta = cvt_angle(cur_theta)
    x1 = cur_x
    x2 = tan(cur_theta)
    x3 = cur_y
    sd1 = reaching_law(k+1,1)
    sd2 = reaching_law(k+1,2)
    sd3 = reaching_law(k+1,3)
    x_ar.append(cur_x)
    y_ar.append(cur_y)
    z_ar.append(cur_theta)
    u1, u21, u22 = control_calc(x1,x2,x3,sd1,sd2,sd3)
    print(u1,u21,u22)
    print(cur_x,cur_y,cur_theta)
    u_to_v(u1,u21)
    cur_x,cur_y,cur_theta = cvt_coordinate()
    cur_theta = cvt_angle(cur_theta)

    u_to_v(u1,u22)

    k=k+1
    n = input('n')


x_ar.append(cur_x)
y_ar.append(cur_y)
z_ar.append(cur_theta)
k=0
tau = 0.4

x_ar1 = []
y_ar1 = []
z_ar1 = []
x_ar2 = []
y_ar2 = []
z_ar2 = []
v_ar1 = []
w_ar1 = []

x = float(input('x'))
y = float(input('y'))
theta = cvt_angle(float(input('theta')))

cur_x,cur_y,cur_theta = cvt_coordinate()
cur_theta = cvt_angle(cur_theta)

while True:
    if abs(tan(cur_theta)) <= 0.454 or abs(tan(cur_theta))>=22.0 :
        theta = theta + 0.5*tau/2
        theta = cvt_angle(theta)
    elif abs(cur_x) < 0.2 :
        x = x + (0.2*tau*cos(cur_theta)/2)
        y = y + (0.2*tau*sin(cur_theta)/2)
    else:
        break
    cur_x,cur_y,cur_theta = cvt_coordinate()
    cur_theta = cvt_angle(cur_theta)


x_ar1.append(cur_x)
y_ar1.append(cur_y)
z_ar1.append(cur_theta)
x_ar2.append(x)
y_ar2.append(y)
z_ar2.append(theta)
x1_0 = cur_x
x2_0 = tan(cur_theta)
x3_0 = cur_y

while True:
    cur_x,cur_y,cur_theta = cvt_coordinate()
    cur_theta = cvt_angle(cur_theta)
    if abs(cur_x)<0.15 and abs(cur_y)< 0.15 and abs(sin(cur_theta))<0.2:
        break
    if k>100:
        print('break')
        break

    while True:
        if abs(tan(cur_theta)) <= 0.454 or abs(tan(cur_theta))>=22.0 :
            theta = theta + 0.5*tau/2
            theta = cvt_angle(theta)
        elif abs(cur_x) < 0.2 and abs(cur_y) > 0.2:
            x = x + (0.2*tau*cos(cur_theta)/2)
            y = y + (0.2*tau*sin(cur_theta)/2)
        else:
            break
        cur_x,cur_y,cur_theta = cvt_coordinate()
        cur_theta = cvt_angle(cur_theta)
    x1 = cur_x
    x2 = tan(cur_theta)
    x3 = cur_y
    sd1 = reaching_law(k+1,1)
    sd2 = reaching_law(k+1,2)
    sd3 = reaching_law(k+1,3)
    x_ar1.append(cur_x)
    y_ar1.append(cur_y)
    z_ar1.append(cur_theta)
    x_ar2.append(x)
    y_ar2.append(y)
    z_ar2.append(theta)
    u1, u21, u22 = control_calc(x1,x2,x3,sd1,sd2,sd3)
    
    u_to_v1(u1,u21)

    u_to_v1(u1,u22)

    k=k+1


x_ar1.append(cur_x)
y_ar1.append(cur_y)
z_ar1.append(cur_theta)
x_ar2.append(x)
y_ar2.append(y)
z_ar2.append(theta)

figure , ax = plt.subplots(1,3)
print(k)
ax[0].plot(x_ar,'r')
ax[0].plot(y_ar,'g')
ax[0].plot(z_ar,'b')
ax[1].plot(x_ar1,'r')
ax[1].plot(y_ar1,'g')
ax[1].plot(z_ar1,'b')
ax[2].plot(x_ar2,'r')
ax[2].plot(y_ar2,'g')
ax[2].plot(z_ar2,'b')


figure1 , ax1 = plt.subplots(2,2,sharex=True)

ax1[0,0].plot(v_ar,'r')
ax1[1,0].plot(w_ar,'g')
ax1[0,1].plot(v_ar1,'r')
ax1[1,1].plot(w_ar1,'g')
plt.show()

