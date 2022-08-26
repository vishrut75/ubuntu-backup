#! /usr/bin/env python3
from math import atan2, sqrt, tan, cos, sin, atan
from matplotlib import pyplot as plt

pie = 3.14159265359
v_max = 0.4
v_min = 0.2
w_max = 0.8
goal_x = 0.0
goal_y = 0.0
goal_theta = 0.0
cur_x = 0.0
cur_y = 0.0
cur_theta = 0.0
x1_0 = 0
x2_0 = 0
x3_0 = 0
sd1 = 0.0
sd2 = 0.0
sd3 = 0.0
trig = False
k=0
x_ar = []
y_ar = []
z_ar = []
v_ar = []
w_ar = []
x_ex = []
y_ex = []
z_ex = []
x=0
y=0
theta=0
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

def reaching_law(k,val):
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
        k2 = max(k2,4*k3*abs(x2_0)/temp_chk)
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
    
def u_to_v(u1,u21,u22):
    global speed
    global cur_theta
    global v_max
    global w_max
    global cur_x
    global cur_y
    global cur_theta
    tau = 0.4
    v = u1/cos(cur_theta)
    w = u21*cos(cur_theta)*cos(cur_theta)
    w1 = u22*cos(cur_theta)*cos(cur_theta)
    if(abs(w1)>w_max):
        w1 = w_max*sgm(w1)
    if(abs(v)>v_max):
        v = v_max*sgm(v)
    if(abs(w)>w_max):
        w = w_max*sgm(w)
    cur_x = cur_x + v*cos(cur_theta)*tau/2
    cur_y = cur_y + v*sin(cur_theta)*tau/2
    cur_theta = cur_theta + w*tau/2
    cur_theta = cvt_angle(cur_theta)
    cur_x = cur_x + v*cos(cur_theta)*tau/2
    cur_y = cur_y + v*sin(cur_theta)*tau/2
    cur_theta = cur_theta + w1*tau/2
    cur_theta = cvt_angle(cur_theta)
    v_ar.append(v)
    v_ar.append(v)
    w_ar.append(w)
    w_ar.append(w1)

def u_to_v1(u1,u21,u22):
    global speed
    global cur_theta
    global v_max
    global w_max
    global cur_x
    global cur_y
    global cur_theta
    tau = 0.4
    v = u1/cos(cur_theta)
    w = u21*cos(cur_theta)*cos(cur_theta)
    w1 = u22*cos(cur_theta)*cos(cur_theta)
    v_ar.append(v)
    v_ar.append(v)
    w_ar.append(w)
    w_ar.append(w1)
    cur_x = cur_x + v*cos(cur_theta)*tau/2
    cur_y = cur_y + v*sin(cur_theta)*tau/2
    cur_theta = cur_theta + w*tau/2
    cur_theta = cvt_angle(cur_theta)
    cur_x = cur_x + v*cos(cur_theta)*tau/2
    cur_y = cur_y + v*sin(cur_theta)*tau/2
    cur_theta = cur_theta + w1*tau/2
    cur_theta = cvt_angle(cur_theta)

def control_calc(i1,i2,i3,s1,s2,s3,tau,d10,d20):
    global cur_theta
    global k
    A1 = s1-i1
    A2 = s2-i2
    A3 = s3-i3
    U1 = A1 - d10
    val = 4*A3/U1
    u2m = w_max/(cos(cur_theta)**2)
    fac_U = U1
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
    # if(abs(U1)==0.0):
    #     print('k',k,i3,i2,s2,A2)
    #     return 0, A2/(2*tau),A2/(2*tau)
    if(fac_U == U1):
        print('k',k)
    fac_U = U1
    U21 = 4*(A3/fac_U) - 4*i2 - A2 - (4*d20/fac_U)
    U22 = 3*A2 -4*(A3/fac_U) + 4*i2 + (4*d20/fac_U)
    return U1/tau, U21/tau, U22/tau


x = float(input('Cx'))
y = float(input('Cy'))
theta = cvt_angle(float(input('Ctheta')))
goal_x = float(input('x'))
goal_y = float(input('y'))
goal_theta = cvt_angle(float(input('theta')))

cur_x,cur_y,cur_theta = cvt_coordinate()
cur_theta = cvt_angle(cur_theta)

goal_x=0
goal_y=0
goal_theta=0

x_ar.append(cur_x)
y_ar.append(cur_y)
z_ar.append(cur_theta)
x1_0 = cur_x
x2_0 = tan(cur_theta)
x3_0 = cur_y
x_ex.append(x1_0)
y_ex.append(x3_0)
z_ex.append(x2_0)
print('xyt',x1_0,x3_0,cur_theta)

while True:
    sd1 = reaching_law(k+1,1)
    sd2 = reaching_law(k+1,2)
    sd3 = reaching_law(k+1,3)
    # print('sd',sd1,sd2,sd3)
    while True:
        x_ar.append(cur_x)
        y_ar.append(cur_y)
        z_ar.append(cur_theta)
        break
    if abs(cur_x)<0.2 and abs(cur_y)< 0.2 and abs(sin(cur_theta))<0.2:
        break
    x1 = cur_x
    x2 = tan(cur_theta)
    x3 = cur_y
    u1, u21, u22 = control_calc(x1,x2,x3,sd1,sd2,sd3,0.4,0.0,0.0)
    k = k+1
    u_to_v(u1,u21,u22)

u_to_v(0,0,0)
# figure3 , ax3 = plt.subplots(1)
# ax3.plot(u2_ar,'r')
# ax3.plot(u2_b,'b')
figure , ax = plt.subplots(1)
print('k',k)
ax.plot(x_ar,'r',label='x')
ax.plot(y_ar,'g',label='y')
ax.plot(z_ar,'b',label='theta')
ax.legend()
ax.set_title('States')
figure2 , ax2 = plt.subplots(1)
ax2.plot(x_ar,y_ar)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_title('Trajectory')

figure1 , ax1 = plt.subplots(2,sharex=True)

ax1[0].plot(v_ar,'r')
ax1[0].set_ylabel('linear velocity')
ax1[1].plot(w_ar,'g')
ax1[1].set_ylabel('angular velocity')
plt.show()



