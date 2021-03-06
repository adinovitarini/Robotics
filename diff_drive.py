# -*- coding: utf-8 -*-
"""Diff_drive.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1iuDIjgNJcGd3hzGeCh0QPPFIS5lG2A-1
"""

import numpy as np 
import matplotlib.pyplot as plt
def diff_drive(x,y,theta,vr,vl,t,l):
  if vr == vl :
    #straight
    x_new = x + vl*t*np.cos(theta)
    y_new = y + vl*t*np.sin(theta)
    theta_new = theta
  else :
    #circular motion 
    R = float(l*((vr+vl)/(vr-vl))*0.5)
    w = float((vr-vl)/l)
    ICC_x = float(x - R*np.sin(theta))
    ICC_y = float(y + R*np.cos(theta))
    a = float(w*t)
    x_new = (x-ICC_x)*np.cos(a)-(y+ICC_y)*np.sin(a)+ICC_x
    y_new = (x-ICC_x)*np.sin(a)-(y+ICC_y)*np.cos(a)+ICC_y
    theta_new = theta + w
  return x_new, y_new, theta_new
t = float(input('Driving Time : '))
print('Initial Pose robot : ')
x = float(input('x = '))
y = float(input('y = '))
theta = float(input('theta = '))
print('Jarak antara roda kanan dan kiri ')
l = float(input('l = '))
print('Kecepatan roda kanan dan kiri ')
vr = float(input('vr= '))
vl = float(input('vl= '))
[x_new,y_new,theta_new] = diff_drive(x,y,theta,vr,vl,t,l)
print ('New pose in x direction :  ',x_new,'New pose in y direction :  ',y_new)
print('New orientation :  ',theta_new)