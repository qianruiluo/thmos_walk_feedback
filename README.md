# thmos_walk_feedback
walking control packet for humanoid robot with MPC and motion feedback

## thmos_pd_walk_fb_pybullet is for pybullet simulation.

- major feedback algorithm is in thmos_pd_walk/walking_packet/thmos_walk_engine walking.getNextPos()
- adjust thmos_pd_walk/walking_packet/thmos_walk_engine walking.__init__() self.feedback_coef to change the feedback intensity
- run walking_pybullet_sample.py to start simulation

## thmos_pd_walk_fb is for actual robot.