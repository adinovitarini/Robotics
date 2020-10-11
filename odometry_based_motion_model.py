# -*- coding: utf-8 -*-
"""Odometry_based_motion_model.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1Sksf3Z3VntvNB1Ms9-bKLYD0O5RBLQEQ
"""

pip install scipy

#import sample_normal_distribution as snd
import scipy.stats
import numpy as np
import math
import matplotlib.pyplot as plt
""" Exercise 2 a) Implement odometry-based motion model """
def sample_normal_twelve(mu, sigma):
  """ Sample from a normal distribution using 12 uniform samples.
  See lecture on probabilistic motion models slide 19 for details.
  """
  # Formula returns sample from normal distribution with mean = 0
  x = 0.5 * np.sum(np.random.uniform(-sigma, sigma, 12))
  return mu + x
def sample_normal(mu, sigma):
  return sample_normal_twelve(mu,sigma)
  #return snd.sample_normal_rejection(mu,sigma)
  #return snd.sample_normal_boxmuller(mu,sigma)
  #return np.random.normal(mu,sigma)
def sample_odometry_motion_model(x, u, a):
  """ Sample odometry motion model.
  Arguments:
  x -- pose of the robot before moving [x, y, theta]
  u -- odometry reading obtained from the robot [rot1, rot2, trans]
  a -- noise parameters of the motion model [a1, a2, a3, a4]
  See lecture on probabilistic motion models slide 27 for details.
  """
  delta_hat_r1 = u[0] + sample_normal(0, a[0]*abs(u[0]) + a[1]*abs(u[2]))
  delta_hat_r2 = u[1] + sample_normal(0, a[0]*abs(u[1]) + a[1]*abs(u[2]))
  delta_hat_t = u[2] + sample_normal(0, a[2]*abs(u[2]) + a[3]*(abs(u[0])+abs(u[1])))
  x_prime = x[0] + delta_hat_t * math.cos(x[2] + delta_hat_r1)
  y_prime = x[1] + delta_hat_t * math.sin(x[2] + delta_hat_r1)
  theta_prime = x[2] + delta_hat_r1 + delta_hat_r2
  return np.array([x_prime, y_prime, theta_prime])
""" Exercise 2 c) Evaluate motion model """
def main():
  x = [2, 4, 0]
  u = [np.pi/2, 0, 1]
  a = [0.1, 0.1, 0.01, 0.01]
  num_samples = 5000
  x_prime = np.zeros([num_samples, 3])
  for i in range(0, num_samples):
    x_prime[i,:] = sample_odometry_motion_model(x,u,a)
  plt.plot(x[0], x[1], "bo")
  plt.plot(x_prime[:,0], x_prime[:,1], "r,")
  plt.xlim([1, 3])
  plt.axes().set_aspect('equal')
  plt.xlabel("x-position [m]")
  plt.ylabel("y-position [m]")
  plt.savefig("odometry_samples.pdf")
  plt.show()
if __name__ == "__main__":
  main()