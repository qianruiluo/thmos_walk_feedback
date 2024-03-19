# thmos_walk_feedback
walking control packet for humanoid robot with MPC and motion feedback


- 核心反馈算法在thmos_pd_walk/walking_packet/thmos_walk_engine的walking.getNextPos()中
- 要调整反馈系数（反馈强度），请调整thmos\_pd\_walk/walking\_packet/thmos\_walk\_engine的walking\.\_\_init\_\_()中的self.feedback_coef初始化参数。
- 运行walking_pybullet_sample.py以启动仿真
