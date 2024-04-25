from imu_data import *
import walking_packet.thmos_walk_engine
import numpy as np


class feed_walk():
    def __init__(self, walk_engine: walking_packet.thmos_walk_engine.walking, robotID): 
        '''
        [theta, x, y] -> [yaw, pitch, roll]
        :param walk_engine: the engine that conducts the walking pattern
        :param offset_limit: limits for feedback offset(in meters)
        '''
        self.walk_engine = walk_engine

        self.rotation_coef = 1.13137

        # feedback_coef: [yaw, pitch, roll]
        self.feedback_coef_endpos = np.array([0.003, 0.003, 0.003])
        self.offset_limit_endpos = np.array([0.05, 0.03, 0.03])
        self.feed_endpos_on = False
        
        
        self.imu = imu_Subscriber(0.5)
        self.average_orientation_1step = np.array([0.0, 0.0, 0.0]) # [yaw, pitch, roll] average in last 1 frame
        self.average_orientation_10step = np.array([0.0, 0.0, 0.0]) # [yaw, pitch, roll] average in last 10 frames
        self.orientation_nowframe = np.array([0.0, 0.0, 0.0])
        
        # initialize feeedback params
        self.endpos_offset = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]) # left/right step offset due to feedback 
        self.endpos_offset_d = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]) # left/right step offset increments in one frame

        self.body_offset = np.array([0.0, 0.0, 0.0]) # [com_x_offset, com_y_offset, trunk_pitch_offset]
        self.body_offset_p = np.array([0.0, 0.0, 0.0])
        self.body_offset_i = np.array([0.0, 0.0, 0.0])
        self.body_offset_d = np.array([0.0, 0.0, 0.0])
        # modify self.body_offset_coef_pid to make adjustments
        self.body_offset_coef_pid = np.array([[0.0, 0.0, 0.1],       # [comx, comy, pitch] : kp
                                              [0.0, 0.00, 0.005],   # [comx, comy, pitch] : ki
                                              [0.0, 0.09, 0.0]])           # [comx, comy, pitch] : kd
        self.offset_limit_body = np.array([0, 0.03, 0.2]) # [comx, comy, pitch]
        self.feed_body_on = True
        

        

    def calc_offset(self):
        '''
        Calculates step point offset for both feet. Should be called for each step frame. 

        :returns: offset arrays in [left_theta, left_x, left_y], [right_theta, right_x, right_y], [com_x_offset, com_y_offset, trunk_pitch_offset]
        '''
        self.ideal_goal = np.array([self.imu.yaw, 0, 0])
        # self.orientation_nowframe += np.array([self.imu.yaw, self.imu.pitch, self.imu.roll])
        ideal_coef = 0.3 # the ratio of control goal determined by ideal
        self.control_goal = ideal_coef * self.ideal_goal + (1-ideal_coef) * self.average_orientation_10step

        now_step = 0 # now_step: 0=LEFT, 1=RIGHT
        if self.walk_engine.old_support_leg == 'RIGHT':
            now_step = 1
        now_support = 1-now_step
        if self.walk_engine.now_frame == self.walk_engine.start_up_frame:
            self.endpos_offset[now_step] = np.array([0.0, 0.0, 0.0])
            self.endpos_offset_d[now_support] = -self.endpos_offset[now_support] / self.walk_engine.period_frames
        
        # calculate feedback offset with imu data and model prediction
        pitch_beta = self.rotation_coef * math.sqrt(self.walk_engine.trunk_height / 9.8)
        pitch_preview = 0.5 * (self.imu.pitch + self.walk_engine.trunk_pitch) * (1.0 + math.cosh(pitch_beta *  self.walk_engine.frames_left * self.walk_engine.dt)) + self.imu.wy * pitch_beta * math.sinh(pitch_beta *  self.walk_engine.frames_left * self.walk_engine.dt)
        
        roll_beta = self.rotation_coef * math.sqrt(self.walk_engine.trunk_height / 9.8)
        roll_preview = 0.5 * (self.imu.roll + math.asin(0.5 * self.walk_engine.foot_width / self.walk_engine.trunk_height)) * (1.0 + math.cosh(roll_beta *  self.walk_engine.frames_left * self.walk_engine.dt)) + self.imu.wx * pitch_beta * math.sinh(roll_beta * self.walk_engine.frames_left * self.walk_engine.dt) - math.asin(0.5 * self.walk_engine.foot_width / self.walk_engine.trunk_height)
        
        if now_step == 0:
            roll_preview = 0.5 * (self.imu.roll - math.asin(0.5 * self.walk_engine.foot_width / self.walk_engine.trunk_height)) * math.cosh(roll_beta *  self.walk_engine.frames_left * self.walk_engine.dt) + self.imu.wx * pitch_beta * math.sinh(roll_beta * self.walk_engine.frames_left * self.walk_engine.dt) + math.asin(0.5 * self.walk_engine.foot_width / self.walk_engine.trunk_height)
        
        self.endpos_offset_d[now_step] = np.array([self.feedback_coef_endpos[0] * (self.control_goal[0] - self.imu.yaw), self.feedback_coef_endpos[1] * (self.control_goal[1] - pitch_preview), self.feedback_coef_endpos[2] * (self.control_goal[2] - roll_preview)])
        self.endpos_offset = self.endpos_offset + self.endpos_offset_d 


        # body offset
        self.body_offset_i = self.body_offset_i + self.body_offset_coef_pid[1] * np.array([0, (self.control_goal[2] - self.imu.roll), -(self.control_goal[1] - self.imu.pitch)])
        self.body_offset_p = self.body_offset_coef_pid[0] * np.array([0, (self.control_goal[2] - self.imu.roll), -(self.control_goal[1] - self.imu.pitch)])
        self.body_offset_d = self.body_offset_coef_pid[2] * np.array([0, (self.imu.wx), -(self.imu.wy)])

        self.body_offset = self.body_offset_p + self.body_offset_i + self.body_offset_d

        if self.feed_endpos_on == False:
            self.endpos_offset = self.endpos_offset * 0.0
        if self.feed_body_on == False:
            self.body_offset = self.body_offset * 0.0

        # limit offsets
        np.clip(self.endpos_offset[0], -self.offset_limit_endpos, self.offset_limit_endpos, out=self.endpos_offset[0])
        np.clip(self.endpos_offset[1], -self.offset_limit_endpos, self.offset_limit_endpos, out=self.endpos_offset[1])

        np.clip(self.body_offset_i, -self.offset_limit_body, self.offset_limit_body, out=self.body_offset_i)
        np.clip(self.body_offset, -self.offset_limit_body, self.offset_limit_body, out=self.body_offset)

        return self.endpos_offset[0], self.endpos_offset[1], self.body_offset









