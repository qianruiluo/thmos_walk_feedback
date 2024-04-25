import numpy as np

body_offset_i = np.array([0.0, 0.0, 0.0])
body_offset_d = np.array([0.0, 0.0, 0.0])
body_offset_coef_pid = np.matrix([[0.0, 0.1, 0.1],       # [comx, comy, pitch] : kp
                                              [0.0, 0.001, 0.001],   # [comx, comy, pitch] : ki
                                              [0.0, 0.0, 0.0]])           # [comx, comy, pitch] : kd

body_offset_i = body_offset_i + np.array(body_offset_coef_pid[1]) * np.array([1.0, 2.0, 3.0])
body_offset_p = np.array(body_offset_coef_pid[0]) * np.array([1.0, 2.0, 3.0])

