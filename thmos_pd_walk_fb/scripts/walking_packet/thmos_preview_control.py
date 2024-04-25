#!/usr/bin/python
# -*- coding: UTF-8 -*-

# Copyright (c). All Rights Reserved.
# -----------------------------------------------------
# File Name:        thmos_preview_control
# Creator:          Jinyin Zhou
# Version:          0.2
# Created:          unknow
# Description:      one step preview controller for robot
# History:
#   <author>      <version>       <time>          <description>
#   Jinyin Zhou     0.2           2023/11/30        create
# -----------------------------------------------------

import numpy as np
import control
import control.matlab
import time
class thmos_preview_control():
  def __init__(self, dt, period, z, Q = 1.0e+8, H = 1.0):
    self.dt = dt
    self.period = period # optimize period
    G = 9.8
    A = np.matrix([
      [0.0, 1.0, 0.0],
      [0.0, 0.0, 1.0],
      [0.0, 0.0, 0.0]])
    B = np.matrix([[0.0], [0.0], [1.0]])
    C = np.matrix([[1.0, 0.0, -z/G]])
    D = 0
    sys = control.matlab.ss(A, B, C, D)
    
    # continuous to discrete
    sys_d = control.c2d(sys, dt)
    self.A_d, self.B_d, self.C_d, D_d = control.matlab.ssdata(sys_d)

    # modify car-table model
    Zero = np.matrix([[0.0], [0.0], [0.0]])
    Phai = np.block([[1.0, -self.C_d * self.A_d], [Zero, self.A_d]]) # A~
    G = np.block([[-self.C_d*self.B_d], [self.B_d]]) # b~
    GR = np.block([[1.0], [Zero]])
    
    # MPC optimize
    Qm = np.zeros((4,4))
    Qm[0][0] = Q
    P = control.dare(Phai, G, Qm, H)[0] # Qm -> Q; H -> R
    self.F = -np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*P*Phai # Kx
    xi = (np.eye(4)-G*np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*P)*Phai
    self.f = []
    self.xp, self.yp = np.matrix([[0.0],[0.0],[0.0]]), np.matrix([[0.0],[0.0],[0.0]])
    self.ux, self.uy = 0.0, 0.0
    for i in range(0,round(period/dt)):
      self.f += [-np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*np.linalg.matrix_power(xi.transpose(),i-1)*P*GR] # fi
    
  def CoM_motion(self, now_frame, current_x, current_y, zmp_plan, pre_reset = False):
    """
    output CoM motion
    """
    # set now state
    x, y = current_x.copy(), current_y.copy()

    # if reset
    if pre_reset == True:
      self.ux, self.uy = 0.0, 0.0

    # caculate Kp => now state feed back 
    px, py = self.C_d * x, self.C_d * y
    ex, ey = zmp_plan[0][0] - px, zmp_plan[0][1] - py
    X, Y = np.block([[ex], [x - self.xp]]), np.block([[ey], [y - self.yp]])
    self.xp, self.yp = x.copy(), y.copy()
    dux, duy = self.F * X, self.F * Y

    # caculate Kd => new target feed forward
    dux += self.f[round(self.period /self.dt) - now_frame] * (zmp_plan[1][0]-zmp_plan[0][0])
    duy += self.f[round(self.period /self.dt) - now_frame] * (zmp_plan[1][1]-zmp_plan[0][1])
          
    self.ux, self.uy = self.ux + dux, self.uy + duy

    # caculate new state
    x, y = self.A_d * x + self.B_d * self.ux, self.A_d * y + self.B_d * self.uy
    
    return x, y
  
if __name__ == '__main__':
  pass
