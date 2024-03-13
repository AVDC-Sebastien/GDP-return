import numpy as np
alpha = 1
R_imu_camera = np.matrix([
                                [1, 0, 0],
                                [0, np.cos(alpha), np.sin(alpha)],
                                [0, -np.sin(alpha), np.cos(alpha)]
    ])
inv(R_imu_camera)