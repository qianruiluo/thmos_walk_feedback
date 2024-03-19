#!/usr/bin/python
# -*- coding: UTF-8 -*-

# Copyright (c). All Rights Reserved.
# -----------------------------------------------------
# File Name:        thmos_step_planner
# Creator:          JinYin Zhou
# Version:          0.1
# Created:          2023/11/30
# Description:      step planner with state machine
# History:
#   <author>      <version>       <time>          <description>
#   Jinyin Zhou     0.1           2023/11/30       create
# -----------------------------------------------------

""" WALKING STATE
# start walking: double to single, zmp in middle to foot 
# BOOT
# keep walking: single support, zmp in foot
# WALK
# ready to stop: single to double, zmp in foot to middle
# STOP
# stay rested: double support, zmp in middle
# REST
"""

""" SUP_LEG STATE
#LEFT
#RIGHT
"""

class thmos_step_planner():
  
  def __init__(self, max_stride_x, max_stride_y, max_stride_th, width):
    """
    Parameters;
     goal_v - [goal_vx, goal_vy, goal_vth]
     now_step - [time, (com_step)x, y, theta, support_leg, (left_foot_step)x, y, (right_foot_step)x, y]
    """
    # lenth between legs
    self.width = width
    
    # max speed of step 
    self.max_stride_x = max_stride_x
    self.max_stride_y = max_stride_y
    self.max_stride_th = max_stride_th

  def step_plan(self, goal_v, now_pos_list, old_support_leg, walking_state):
    """
    Parameters;
     goal_v - [goal_vx, goal_vy, goal_vth]
     now_pos_list - [x, y, theta, (left)x, y, (right)x, y]
     old_support_leg - support leg of robot
     walking_state - walking state of robot
    Returns:
     new_pos_list - [x, y, theta, (left)x, y, (right)x, y]
     new_zmp_list - [px,py]
    """
    
    # limit speed
    goal_vx =  max(-self.max_stride_x,  min(goal_v[0], self.max_stride_x) )
    goal_vy =  max(-self.max_stride_y,  min(goal_v[1], self.max_stride_y) )
    goal_th =  max(-self.max_stride_th, min(goal_v[2], self.max_stride_th))
    
    # caculate com pos
    if  (walking_state == 'BOOT'):        
      goal_vx = 0
      goal_vy = 0
    elif(walking_state == 'WALK'):
      pass
    elif(walking_state == 'STOP'):
      pass
    elif(walking_state == 'REST'):
      pass
    else:
      print("walking state error!")
      
    new_x = now_pos_list[0] + goal_vx
    new_y  = now_pos_list[1] + goal_vy
    new_th = now_pos_list[2] + goal_th

    # if((old_support_leg == 'RIGHT' and goal_vy < 0) or (old_support_leg == 'LEFT' and goal_vy > 0)):
      # new_y  = now_pos_list[1]  + goal_vy
    # else:
      # new_y  = now_pos_list[1]
      
    # caculate foot pos
    new_l_x = now_pos_list[3]
    new_l_y = now_pos_list[4]
    new_r_x = now_pos_list[5]
    new_r_y = now_pos_list[6]
    
    if old_support_leg == 'RIGHT':
      new_r_x = new_x
      new_r_y = new_y - self.width * 0.5
    else:
      new_l_x = new_x
      new_l_y = new_y + self.width * 0.5

    new_pos_list = [new_x,new_y,new_th,new_l_x,new_l_y,new_r_x,new_r_y]
      
    # caculate zmp_step
    if old_support_leg == 'RIGHT':
      single_pxy = [new_r_x,new_r_y]
    else:
      single_pxy = [new_l_x,new_l_y]
    
    mid_pxy = [0.5 * (new_l_x + new_r_x) ,0.5 * (new_l_y + new_r_y)]
    
    if  (walking_state == 'BOOT'):
      new_zmp_list = single_pxy
    elif(walking_state == 'WALK'):
      new_zmp_list = single_pxy
    elif(walking_state == 'STOP'):
      new_zmp_list = single_pxy
    elif(walking_state == 'REST'):
      new_zmp_list = mid_pxy
    else:
      print("walking state error!")
    
    return new_pos_list, new_zmp_list

if __name__ == '__main__':
  pass

