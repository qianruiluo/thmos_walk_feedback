#!/usr/bin/python
# -*- coding: UTF-8 -*-

# Copyright (c). All Rights Reserved.
# -----------------------------------------------------
# File Name:        thmos_kinematics.py
# Creator:          JinYin Zhou
# Version:          0.1
# Created:          2023/2/20
# Description:      leg ik packet
# Function List:        class LegIK:
# History:
#   <author>      <version>       <time>          <description>
#   Jinyin Zhou     0.1           2023/2/20       create
# -----------------------------------------------------

import math
import numpy as np


class LegIK:
    """inverse kinematics for one leg"""
    def __init__(self, LeftorRight, Legs_len, motor_offset, motor_way):
        """
        initialize class
        Args:
            LeftorRight :id-left leg or right leg string-'left' 'right' 
            Legs_len : a list of leg len [3]
            motor_offset : a list of offset angle [6]
            motor_way : a list of spin direction of motor [6,+1,-1]
        """
        self.LorR = LeftorRight
        self.legs_len = Legs_len
        self.offset = motor_offset
        self.way = motor_way
        self.theta = [0] * 6

    def RtoRPY(self, R):
        '''rotate matrix to RPY angles'''
        R = np.array(R)
        err = float(0.001)
        oy = math.atan2(-R[2,0], math.sqrt((R[0,0])**2 + (R[1,0])**2))

        if oy >= math.pi/2-err and oy <= math.pi/2+err:
            oy = math.pi/2
            oz = 0.0
            ox = math.atan2(R[0,1], R[0,2])
        elif oy >= -(math.pi/2)-err and oy <= -(math.pi/2)+err:
            oy = -math.pi/2
            oz = 0.0
            ox = math.atan2(-R[0,1], -R[0,2])
        else:
            oz = math.atan2((R[1,0])/(math.cos(oy)), (R[0,0])/(math.cos(oy)))
            ox = math.atan2((R[2,1])/(math.cos(oy)), (R[2,2])/(math.cos(oy)))

        return [ox, oy, oz]

    def RPYtoR(self,rpy):
        '''RPY angles to rotate matrix'''
        a = rpy[0]
        b = rpy[1]
        c = rpy[2]

        sinA = np.sin(a)
        cosA = np.cos(a)
        sinB = np.sin(b)
        cosB = np.cos(b)
        sinC = np.sin(c)
        cosC = np.cos(c)

        R = [[cosB*cosC,  cosC*sinA*sinB - cosA*sinC,  sinA*sinC + cosA*cosC*sinB],
             [cosB*sinC,  cosA*cosC + sinA*sinB*sinC, cosA*sinB*sinC - cosC*sinA],
             [-sinB, cosB*sinA,  cosA*cosB]]
        return R

    def axistoR(self, axis_joint, theta):
        '''torch to transform [numpy.array]'''
        SO3 = np.mat([[0,      -axis_joint[2], axis_joint[1]],
                      [ axis_joint[2], 0,     -axis_joint[0]],
                      [-axis_joint[1], axis_joint[0], 0     ]])
        R = np.mat(np.identity(3)) + np.sin(theta) * SO3 + (1 - np.cos(theta)) * SO3 * SO3
        return R

    def StdLegIK(self, end_point, end_rpy):
        '''inverse kinematics with standard axis'''

        #get foot target transformation
        Rt = np.array(self.RPYtoR(end_rpy))
        dt = np.array(end_point)

        #get inverse to change coordinates(from lab to foot)
        Rti = Rt.T
        d = np.dot(Rti, -dt)

        #caculate self.theta 3 4 5
        la = np.linalg.norm(d - np.array([0,0,self.legs_len[2]]))
        
        # 3
        if(abs(self.legs_len[0] - self.legs_len[1]) > la):
            self.theta[3] = - np.pi 
            theta_a = np.pi
        elif(self.legs_len[0] + self.legs_len[1] > la):
            self.theta[3] = np.arccos( (self.legs_len[0] ** 2  + self.legs_len[1] ** 2 - la ** 2)  / (2 * self.legs_len[0] * self.legs_len[1]) )- np.pi
            theta_a  = np.arccos( (self.legs_len[1] ** 2  + la ** 2 - self.legs_len[0] ** 2) / (2 * self.legs_len[1] * la) )
        else:
            self.theta[3] = 0
            theta_a = 0
        
        # 4
        self.theta[4] = theta_a + np.arcsin(d[0] / la)
        
        # 5
        self.theta[5] = np.arctan(- d[1] / (d[2] - self.legs_len[2]))

        #caculate self.theta 0 1 2
        R5 = np.mat(self.axistoR([1,0,0],self.theta[5]))
        R43 = np.mat(self.axistoR([0,1,0],self.theta[4] + self.theta[3]))
        Rfoot = R5 * R43
        Rlap = Rfoot.T * np.mat(Rti)
        self.theta[0:3] = self.RtoRPY(Rlap)
        
        #set theta
        for index in range(6):
            self.theta[index] = self.theta[index] * self.way[index] + self.offset[index]
        
        return self.theta

class thmos_legik:
    """inverse kinematics for two leg"""
    def __init__(self, way_left,  way_right, leg_rod_length):
        """
        initialize class
        """
        self.leg_left = LegIK('Left',leg_rod_length, [0,0,0,0,0,0], way_left)
        self.leg_right = LegIK('Right',leg_rod_length, [0,0,0,0,0,0],way_right)

    def LegIKMove(self, LeftorRight, end_pos, body_rpy = [0,0,0]):
        """
        move left and right leg
        """
        xyz = end_pos[0:3]
        rpy = end_pos[3:6]
        
        if (LeftorRight == 'Left' or LeftorRight == 'left'):
            return self.leg_left.StdLegIK(xyz, rpy)
        else:
            return self.leg_right.StdLegIK(xyz, rpy)
    
if __name__ == '__main__':
    leg_1 = thmos_legik()
    print(leg_1.LegIKMove("left",[0, 0, -0.32, 0.0, 0.0, 0.0]))



