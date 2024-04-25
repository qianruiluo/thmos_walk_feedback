#!/usr/bin/env python3

import sys
sys.path.append(sys.path[0] + '/walking_packet')
#print(sys.path)
import rospy
from geometry_msgs.msg import Twist
# webots version: from bitbots_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from thmos_walk_engine import walking
import numpy as np
class zmp_walker:
    def __init__(self, base_ns = ''):
        # control box ----
        sys.path.append(sys.path[0] + '/param.txt')
        param_path=sys.path[-1]		
        param=np.genfromtxt(fname=param_path,dtype=float,delimiter=",",comments="#",max_rows=38,invalid_raise=False)
        self.Params = {              
            'foot_width' : param[0],
            'ex_foot_width' : param[1],
            'foot_height' :param[2],
            'com_height' : param[3],
            'com_x_offset' : param[4],
            'com_y_offset' :param[5],
            'trunk_height' : param[6],
            'walking_period' : param[7],
            'both_foot_support_time' : param[8],
            'dt' : param[9],
            'max_vx' : param[10],
            'max_vy': param[11],
            'max_vth' : param[12],
            'k_x_offset':param[13],#ex_com_x_offset k
            'k_y_offset':param[14],#ex_com_y_offset k
            'trunk_pitch':param[15],
            'way_left' : [1,-1,-1,-1,-1,-1],
            'way_right' : [1,1,-1,1,1,-1],
            'leg_rod_length' : [0.156,0.12,0.045]
            }
        
        self.walk_gen = walking(** self.Params)
        
        self.joint_goal_msg = JointState()
        self.joint_goal_msg.name = ["R_leg_1", "R_leg_2", "R_leg_3", "R_leg_4", "R_leg_5", "R_leg_6",
                                           "L_leg_1", "L_leg_2", "L_leg_3", "L_leg_4", "L_leg_5", "L_leg_6"]  
                                           
        self.joint_angles_raw = [0.0] * 12

        self.next_walk_goal = [0.0, 0.0, 0.0] 
        self.old_walk_goal = [0.0, 0.0, 0.0]
        self.speed_change_flag = 1

        self.step_pic_remain = 0
        self.rate = rospy.Rate(100)  #100    

        self.walk_goal_subscriber = rospy.Subscriber(base_ns + '/cmd_vel', Twist, self.walk_goal_callback, queue_size=1, tcp_nodelay=True)
        self.joint_goal_publisher = rospy.Publisher(base_ns + '/walking_motor_goals', JointState, queue_size=1)

    def walk_goal_callback(self, walk_goal_msg):
        self.next_walk_goal = [walk_goal_msg.linear.x, walk_goal_msg.linear.y, walk_goal_msg.angular.z]
        self.walk_gen.com_y_offset=-self.Params['k_y_offset']*self.next_walk_goal[1]+self.Params['com_y_offset']
        self.walk_gen.com_x_offset=-self.Params['k_x_offset']*self.next_walk_goal[0]+self.Params['com_x_offset']
        
    def pub_joint_goal(self):
        self.joint_goal_msg.position = self.joint_angles_raw
        self.joint_goal_publisher.publish(self.joint_goal_msg)  
    
    def walk(self):
        if self.step_pic_remain == 0:
            self.walk_gen.setGoalVel(self.next_walk_goal)
        self.joint_angles_raw, self.step_pic_remain = self.walk_gen.getNextPos()
        self.rate.sleep()
        self.pub_joint_goal()
        
    
if __name__ == "__main__":
    rospy.init_node('thmos_zmp_walk', anonymous=True) 
    walk = zmp_walker()
          
    while not rospy.is_shutdown():
      walk.walk()
