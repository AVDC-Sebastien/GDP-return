import threading
import time
import numpy as np
from Camera_top_thread import camera_top_task
from Camera_below_thread import camera_below_task
from imu_thread import imu_task
from Lidar_thread import lidar_task

class sensor_fusion():

    def __init__(self):
        self.stop_main_sensor_thread = False
        self.stop_measurement_thread = False

        # region parameters
        #----Paremeters-------
        #----IMU-------
        self.Q_imu = []
        self.R_imu = []

        #-----lidar-----
        self.Q_lidar = []
        self.R_lidar = []

        #--camera----
        self.Q_camera_top = []
        self.R_camera_top = []

        self.Q_camera_below = []
        self.R_camera_below = []

        self.initial_state_uav = []
        self.P_uav = []
        self.dt = 0.01
        self.state_uav = self.initial_state_uav

        self.initial_markers = []
        self.P_marker = []
        self.state_markers = self.initial_markers
        self.Q_markers =[]

        self.dt_acceptable = 0.01

        self.imu_measurement = 0
        self.lidar_measurement = 0
        self.camera_top_measurement = 0
        self.camera_below_measurement = 0

        self.dt_imu_lidar_camera = 0.1
        self.dt_imu_lidar = 0.1
        self.dt_imu_camera =0.1
        self.dt_lidar_camera = 0.1
        self.dt_imu = 0.1
        self.dt_lidar = 0.1
        self.dt_camera = 0.1

        #tous les offsets sont definis la

        self.offset_imu_camera_top = [0, 0.05, 0.5]
        self.offset_imu_camera_below = [0, 0.05, 0.5]
        self.offset_angle_top = 90
        self.offset_angle_below = 180
        self.offset_lidar = []

        self.target_angle_offset = (0,0,0)

        self.calibration_faite = 0

        # Structure de données partagée

        self.get_imu_meas = 0
        self.get_camera_top_meas = 0
        self.get_camera_below_meas = 0
        self.get_lidar_meas =0

        # endregion

        
    def state_function(state_uav, dt):
        
        #matrice rotation bodyframe->absolute
        position = np.transpose(state_uav[-1][0])
        velocity = np.transpose(state_uav[-1][1])
        euler = np.transpose(np.radians(state_uav[-1][2]))
        euler_deg = np.transpose(state_uav[-1][2])
        angular_rate = np.transpose(state_uav[-1][3])
        linear_acceleration = np.transpose(state_uav[-1][4])
        ax = linear_acceleration[0]
        ay = linear_acceleration[1]
        az = linear_acceleration[2]
        w1 = np.degrees(angular_rate[0])
        w2 = np.degrees(angular_rate[1])
        w3 = np.degrees(angular_rate[2])
        

        R_imu_abs = np.matrix([
                                    [np.cos(euler[1])*np.cos(euler[0]), np.cos(euler[1])*np.sin(euler[0]), -np.sin(euler[1])],
                                    [np.sin(euler[2])*np.sin(euler[1])*np.cos(euler[0])-np.cos(euler[2])*np.sin(euler[0]),  np.sin(euler[2])*np.sin(euler[1])*np.sin(euler[0])+np.cos(euler[2])*np.cos(euler[0]),  np.sin(euler[2])*np.cos(euler[1])],
                                    [np.cos(euler[2])*np.sin(euler[1])*np.cos(euler[0])+np.sin(euler[2])*np.sin(euler[0]),  np.cos(euler[2])*np.sin(euler[1])*np.sin(euler[0])-np.sin(euler[2])*np.cos(euler[0]),  np.cos(euler[2])*np.cos(euler[1])]
            ])
        
        #matrice pour passer de anglar rate a euler rate
        B_theta = np.matrix([
                            [0, np.sin(euler[2]), np.cos(euler[2])],
                            [0, np.cos(euler[1])*np.cos(euler[2]), -np.cos(euler[1])*np.sin(euler[2])],
                            [np.cos(euler[1]), np.sin(euler[1])*np.sin(euler[2]), np.sin(euler[1])*np.cos(euler[2])]
                            ])
        euler_dot = (1/(np.cos(euler[1])))*B_theta @ angular_rate
        position = position + velocity*dt

        velocity = velocity + (R_imu_abs@linear_acceleration)*dt

        estimated_euler = euler_deg + euler_dot*dt
    
        estimated_state_uav = np.array([[position[0], position[1], position[2],velocity[0], velocity[1], velocity[2], estimated_euler[0], estimated_euler[1], estimated_euler[2], angular_rate[0], angular_rate[1], angular_rate[2],linear_acceleration[0], linear_acceleration[1], linear_acceleration[2]]])

        F47 = (-np.cos(euler[1])*np.sin(euler[0])*ax + np.cos(euler[1])*np.cos(euler[0])*ay)*dt
        F48 = (-np.sin(euler[1])*np.cos(euler[0])*ax - np.sin(euler[1])*np.sin(euler[0])*ay - np.cos(euler[1])*az)*dt
        F413 = np.cos(euler[1])*np.cos(euler[0])*dt
        F414 = np.cos(euler[1])*np.sin(euler[0])*dt
        F415 = -np.sin(euler[1])*dt
        F57 = ((-np.sin(euler[2])*np.sin(euler[1])*np.sin(euler[0])-np.cos(euler[2])*np.cos(euler[0]))*ax + (np.sin(euler[2])*np.sin(euler[1])*np.cos(euler[0]) - np.cos(euler[2])*np.sin(euler[0]))*ay)*dt
        F58 = ((np.sin(euler[2])*np.cos(euler[1])*np.cos(euler[0]))*ax + np.sin(euler[2])*np.cos(euler[1])*np.sin(euler[0])*ay - np.sin(euler[2])*np.sin(euler[1])*az)*dt
        F59 = ((np.cos(euler[2])*np.sin(euler[1])*np.cos(euler[0]) + np.sin(euler[2])*np.sin(euler[0]))*ax + (np.cos(euler[2])*np.sin(euler[1])*np.sin(euler[0]) - np.cos(euler[0])*np.sin(euler[2]))*ay + np.cos(euler[2])*np.cos(euler[1])*az)*dt
        F513 = (np.sin(euler[2])*np.sin(euler[1])*np.cos(euler[0])-np.cos(euler[2])*np.sin(euler[0]))*dt
        F514 = (np.sin(euler[2])*np.sin(euler[1])*np.sin(euler[0]) + np.cos(euler[2])*np.cos(euler[0]))*dt
        F515 = np.sin(euler[2])*np.cos(euler[1])*dt
        F67 = ((-np.cos(euler[2])*np.sin(euler[1])*np.sin(euler[0]) + np.sin(euler[2])*np.cos(euler[0]))*ax + (np.cos(euler[2])*np.sin(euler[1])*np.cos(euler[0]) + np.sin(euler[2])*np.sin(euler[0]))*ay)*dt
        F68 = (np.cos(euler[2])*np.cos(euler[1])*np.cos(euler[0])*ax + np.cos(euler[2])*np.cos(euler[1])*np.sin(euler[0])*ay - np.cos(euler[2])*np.sin(euler[1])*az)*dt
        F69 = ((-np.sin(euler[2])*np.sin(euler[1])*np.cos(euler[0]) + np.cos(euler[2])*np.sin(euler[0]))*ax + (-np.sin(euler[2])*np.sin(euler[1])*np.sin(euler[0]) - np.cos(euler[2])*np.cos(euler[0]))*ay -np.sin(euler[2])*np.cos(euler[1])*az)*dt
        F613 = (np.cos(euler[2])*np.sin(euler[1])*np.cos(euler[0]) + np.sin(euler[2])*np.sin(euler[0]))*dt
        F614 = (np.cos(euler[2])*np.sin(euler[1])*np.sin(euler[0]) - np.sin(euler[2])*np.cos(euler[0]))*dt
        F615 = np.cos(euler[2])*np.cos(euler[1])*dt
        F78 = np.sin(euler[1])*(np.sin(euler[2])*w2 + np.cos(euler[2])*w3)*dt/(np.cos(euler[1])**2)
        F79 = (np.cos(euler[2])*w2 - np.sin(euler[2])*w3)*np.cos(euler[1])*dt/(np.cos(euler[1])**2)
        F711 = np.sin(euler[2])*dt/np.cos(euler[1])
        F712 = np.cos(euler[2])*dt/np.cos(euler[1])
        F88 = 1 + ((-np.sin(euler[1])*np.cos(euler[1])*w2 + np.sin(euler[1])*np.sin(euler[2])*w3)*np.cos(euler[1]) + np.sin(euler[1])*(np.cos(euler[1])*np.cos(euler[2])*w2 - np.cos(euler[1])*np.sin(euler[2])*w3))*dt/(np.cos(euler[1])**2)
        F89 = (-np.sin(euler[2])*np.cos(euler[1])*w2 - np.cos(euler[1])*np.cos(euler[2])*w3)*np.cos(euler[1])*dt/(np.cos(euler[1])**2)
        F811 = np.cos(euler[2])*dt
        F812 = -np.sin(euler[2])*dt
        F98 = ((-np.sin(euler[1])*w1 + np.cos(euler[1])*np.sin(euler[2])*w2 + np.cos(euler[1])*np.cos(euler[2])*w3)*np.cos(euler[1]) + np.sin(euler[1])*(np.cos(euler[1])*w1 + np.sin(euler[1])*np.sin(euler[2])*w2 + np.sin(euler[1])*np.cos(euler[2])*w3))*dt/(np.cos(euler[1])**2)
        F99 = 1 + ((np.sin(euler[1])*np.cos(euler[2])*w2 - np.sin(euler[1])*np.sin(euler[2])*w3)*np.cos(euler[1]))*dt/(np.cos(euler[1])**2)
        F911 = np.sin(euler[1])*np.sin(euler[2])*dt/np.cos(euler[1])
        F912 = np.sin(euler[1])*np.cos(euler[2])*dt/np.cos(euler[1])

        F = np.matrix([
                        [1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0, F47, F48, 0, 0, 0, 0, F413, F414, F415],
                        [0, 0, 0, 0, 1, 0, F57, F58, F59, 0, 0, 0, F513, F514, F515],
                        [0, 0, 0, 0, 0, 1, F67, F68, F69, 0, 0, 0, F613, F614, F615],
                        [0, 0, 0, 0, 0, 0, 1, F78, F79, 0, F711, F712, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, F88, F89, 0, F811, F812, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, F98, F99, dt, F911, F912, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        ])

        return estimated_state_uav, F
    def EKF_filter(z, h, H, predicted_state_uav, F, Q, R, P):

        Pp= F @ P @ F.transpose() + Q
        S = H @ Pp @ H.transpose() + R
        K = Pp @ H.transpose()/S
        residual = z.transpose() - h.transpose()
        #update
        update_state = predicted_state_uav.transpose() + K*residual
        P = Pp - K @ S @ K.transpose()

        return update_state, P
    def imu_measurement_matrix(estimated_state_uav):


        h = np.array([
                        [estimated_state_uav[6], estimated_state_uav[7], estimated_state_uav[8], estimated_state_uav[9], estimated_state_uav[10], estimated_state_uav[11], estimated_state_uav[12], estimated_state_uav[13], estimated_state_uav[14] ]
                        ])
        H = np.array([
                        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
                    ])

        return h, H
    def lidar_measurement_matrix(estimated_state_uav):
        x_z = estimated_state_uav[2]
        pitch = np.radians(estimated_state_uav[7])
        roll = np.radians(estimated_state_uav[8])

        h = (x_z/(np.cos(pitch)*np.cos(roll)))

        H = np.array([
                        [0, 0, 1/(np.cos(pitch)*np.cos(roll)), 0, 0, 0,
                        0, np.cos(roll)*np.sin(pitch)/((np.cos(pitch)*np.cos(roll))**2), np.cos(pitch)*np.sin(roll)/((np.cos(pitch)*np.cos(roll))**2),
                        0, 0, 0, 0, 0, 0]
                    ])

        return h, H

    def camera_markers_measurement(state_uav, state_markers, offset, offset_angle) :
        yaw = state_uav[6]
        pitch = state_uav[7]
        roll = state_uav[8]
        euler_rad = np.array([[yaw, pitch, roll]])
        euler = np.radians(euler_rad)
        R_imu_abs = np.matrix([
                                    [np.cos(euler[1])*np.cos(euler[0]), np.cos(euler[1])*np.sin(euler[0]), -np.sin(euler[1])],
                                    [np.sin(euler[2])*np.sin(euler[1])*np.cos(euler[0])-np.cos(euler[2])*np.sin(euler[0]),  np.sin(euler[2])*np.sin(euler[1])*np.sin(euler[0])+np.cos(euler[2])*np.cos(euler[0]),  np.sin(euler[2])*np.cos(euler[1])],
                                    [np.cos(euler[2])*np.sin(euler[1])*np.cos(euler[0])+np.sin(euler[2])*np.sin(euler[0]),  np.cos(euler[2])*np.sin(euler[1])*np.sin(euler[0])-np.sin(euler[2])*np.cos(euler[0]),  np.cos(euler[2])*np.cos(euler[1])]
            ])
        
        # camera_angle = np.radians(-90)
        alpha = np.radians(offset_angle)
        # R_imu_camera = np.matrix([
        #                             [np.cos(camera_angle),0,-np.sin(camera_angle)],
        #                             [0, 1, 0],
        #                             [np.sin(camera_angle), 0, np.cos(camera_angle)]
        # ])
        R_imu_camera = np.matrix([
                                    [1, 0, 0],
                                    [0, np.cos(alpha), np.sin(alpha)],
                                    [0, -np.sin(alpha), np.cos(alpha)]
        ])
        R_camera_aruco_mauvais_sens = R_imu_abs@R_imu_camera.transpose()
        R_camera_aruco = np.linalg.inv(R_camera_aruco_mauvais_sens)

        #euler_cam = np.zeros((3,1))

        # euler_cam[0] = np.degrees(-np.arctan2(R_camera_aruco[2,1],R_camera_aruco[2,2]))
        # euler_cam[1] = np.degrees(np.arcsin(R_camera_aruco[2,0]))
        # euler_cam[2] = np.degrees(-np.arctan2(R_camera_aruco[1,0],R_camera_aruco[0,0]))

        euler_cam_x = np.degrees(np.arctan2(R_camera_aruco[0,1],R_camera_aruco[0,0]))
        euler_cam_y = -np.degrees(np.arcsin(R_camera_aruco[0,2]))
        euler_cam_z = np.degrees(np.arctan2(R_camera_aruco[1,2],R_camera_aruco[2,2]))

        #offset entre camera et imu
        offset_x = offset[0]
        offset_y = offset[1]
        offset_z = offset[2]

        meas_x = state_uav[0]+offset_x-state_markers[0]
        meas_y = state_uav[1]+offset_y-state_markers[1]
        meas_z = state_uav[2]+offset_z-state_markers[2]

        estimation = np.array([
            [meas_x, meas_y, meas_z, euler_cam_x, euler_cam_y, euler_cam_z]
        ])
    
        h_camera = estimation.transpose()
        H_camera = np.array([
                                [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                [0, 0, 0, 0, 0, 0, -(((np.real(np.cos(yaw)*np.cos(roll)) + np.imag(np.cos(pitch)*np.sin(yaw)) + np.real(np.sin(yaw)*np.sin(pitch)*np.sin(roll)))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw))) + ((np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw)))*(np.imag(np.cos(yaw)*np.cos(roll)) + np.imag(np.sin(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(pitch)*np.sin(yaw))))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))^2)*(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))^2)/((np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))^2 + (np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw)))^2), (((np.real(np.cos(yaw)*np.cos(pitch)*np.sin(roll)) - np.imag(np.cos(yaw)*np.sin(pitch)))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw))) + ((np.imag(np.cos(yaw)*np.cos(pitch)*np.sin(roll)) + np.real(np.cos(yaw)*np.sin(pitch)))*(np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw))))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))^2)*(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))^2)/((np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))^2 + (np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw)))^2), (((np.real(np.cos(yaw)*np.cos(roll)*np.sin(pitch)) + np.real(np.sin(yaw)*np.sin(roll)))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw))) + ((np.imag(np.cos(yaw)*np.cos(roll)*np.sin(pitch)) + np.imag(np.sin(yaw)*np.sin(roll)))*(np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw))))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))^2)*(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))^2)/((np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))^2 + (np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw)))^2), 0, 0, 0, 0, 0, 0],
                                [ 0, 0, 0, 0, 0, 0, -(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))/np.sqrt(1 - (np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))^2), (np.cos(yaw)*np.cos(roll)*np.sin(pitch))/np.sqrt(1 - (np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))^2), -(np.cos(roll)*np.sin(yaw) - np.cos(yaw)*np.cos(pitch)*np.sin(roll))/np.sqrt(1 - (np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))^2), 0, 0, 0, 0, 0, 0],
                                [ 0, 0, 0, 0, 0, 0, (((np.real(np.cos(alpha)*(np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))) + np.imag(np.sin(alpha)*(np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll))) - ((np.imag(np.cos(alpha)*(np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))) - np.real(np.sin(alpha)*(np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))))*(np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll))))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2)*(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2)/((np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2 + (np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2), -(((np.real(np.cos(alpha)*np.cos(roll)*np.sin(yaw)*np.sin(pitch)) + np.imag(np.sin(alpha)*np.cos(roll)*np.sin(yaw)*np.sin(pitch)) + np.imag(np.cos(alpha)*np.cos(roll)*np.sin(pitch)) - np.real(np.sin(alpha)*np.cos(roll)*np.sin(pitch)))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll))) + ((np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))*(np.real(np.sin(alpha)*np.cos(roll)*np.sin(yaw)*np.sin(pitch)) - np.imag(np.cos(alpha)*np.cos(roll)*np.sin(yaw)*np.sin(pitch)) + np.real(np.cos(alpha)*np.cos(roll)*np.sin(pitch)) + np.imag(np.sin(alpha)*np.cos(roll)*np.sin(pitch))))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2)*(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2)/((np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2 + (np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2), -(((np.real(np.cos(alpha)*(np.cos(yaw)*np.cos(roll) + np.cos(pitch)*np.sin(yaw)*np.sin(roll))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.cos(roll) + np.cos(pitch)*np.sin(yaw)*np.sin(roll))) + np.imag(np.cos(alpha)*np.cos(pitch)*np.sin(roll)) - np.real(np.sin(alpha)*np.cos(pitch)*np.sin(roll)))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll))) + ((np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))*(np.real(np.sin(alpha)*(np.cos(yaw)*np.cos(roll) + np.cos(pitch)*np.sin(yaw)*np.sin(roll))) - np.imag(np.cos(alpha)*(np.cos(yaw)*np.cos(roll) + np.cos(pitch)*np.sin(yaw)*np.sin(roll))) + np.real(np.cos(alpha)*np.cos(pitch)*np.sin(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.sin(roll))))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2)*(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2)/((np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2 + (np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))^2), 0, 0, 0, 0, 0, 0]
                                ])
        H_markers = np.array([
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, -1],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ])

        return h_camera,H_camera, H_markers

    def slam(self,id_max, max_increment, camera_top_dict, uav_state, F_uav, P_uav, Q_uav, state_markers, P_markers, F_markers, Q_markers, R, offset, offset_angle):

        for value in id_max:
            mesure = camera_top_dict[value][max_increment][3][0]
            z = np.array([
                                            [mesure[0], mesure[1], mesure[2], mesure[3], mesure[4], mesure[5]]
                                        ])
            #------------EKF SLAM----------------------------------------------------------
            #loop for each landmarks detected
            #markers estimation
            Ppredicted_markers = F_markers*P_markers[value]*F_markers.transpose() + Q_markers
            h_markers, H_camera, H_markers = self.camera_markers_measurement(state_markers,uav_state, offset, offset_angle)
            S_markers = H_markers*Ppredicted_markers*H_markers.transpose() + R
            K_markers = Ppredicted_markers*H_markers.transpose()/S_markers
            #residual, verifier comment on le fait
            residual_markers = z-h_markers  #verifier a bien faire la transpose si necessaire

            state_markers[value]= state_markers[value] + K_markers*residual_markers
            P_markers[value] = Ppredicted_markers - K_markers*H_markers*Ppredicted_markers.transpose()

            #uav estimation
            Ppredicted_uav = F_uav*P_uav*F_uav.transpose() + Q_uav 
            h_camera, H_camera, H_markers = self.camera_markers_measurement(uav_state, state_markers,offset, offset_angle)
            S_uav = H_camera*Ppredicted_uav*H_camera.transpose() + R
            K_uav = Ppredicted_uav*H_camera.transpose/S_uav
            #faire residual uav
            residual_uav = z-h_camera

            uav_state = uav_state + K_uav*residual_uav
            P_uav = Ppredicted_uav - K_uav*H_camera*Ppredicted_uav.transpose()

            #end of the loop------------------------------------------------------
        return uav_state, P_uav, state_markers, P_markers


# ajouter def measurment pour camera ET pour markers

    def Start_measurement(self):
        self.imu_thread = threading.Thread(target=imu_task, args=[self.get_imu_meas,self.target_angle_offset,self.stop_measurement_thread])
        self.camera_top_thread = threading.Thread(target=camera_top_task, args=[self.get_camera_top_meas,self.stop_measurement_thread])
        self.camera_below_thread = threading.Thread(target=camera_below_task, args=[self.get_camera_top_meas,self.stop_measurement_thread])
        self.lidar_thread = threading.Thread(target=lidar_task, args=[self.get_lidar_meas,self.stop_measurement_thread])
        self.imu_thread.start()
        self.camera_top_thread.start()
        self.camera_below_thread.start()
        self.lidar_thread.start()

    def Stop_measurement(self):
        self.stop_measurement_thread = True
        self.stop_main_sensor_thread = True
        self.imu_thread.join()
        self.camera_top_thread.join()
        self.camera_below_thread.join()
        self.lidar_thread.join()

# Tâche Principale
    def main_task(self):
        while not self.stop_main_sensor_thread:

            t1 = time.time()
            camera_top_dict = self.get_camera_top_meas
            camera_below_dict = self.get_camera_below_meas
            imu_measurement = self.get_imu_meas
            lidar_measurement = self.get_lidar_meas

            if imu_measurement !=0:  
                if t1-self.dt_acceptable <= imu_measurement[0] <= t1+self.dt_acceptable: 
                    imu_measurement = np.array([
                                                    [imu_measurement[1][0], imu_measurement[1][1], imu_measurement[1][2], imu_measurement[2][0], imu_measurement[2][1], imu_measurement[2][2], imu_measurement[3][0], imu_measurement[3][1], imu_measurement[3][2]]
                                                    ])
                else:
                    imu_measurement = 0

            if lidar_measurement !=0:  
                if t1-self.dt_acceptable <= lidar_measurement[1] <= t1+self.dt_acceptable: 
                    lidar_measurement = lidar_measurement[0]
                else:
                    lidar_measurement = 0

            max_increment = max((increment for inner_dict in camera_top_dict.values() for increment in inner_dict), default=None)

            # Trouver tous les IDs ayant la valeur maximale d'incrémentation
            id_max = [id for id, inner_dict in camera_top_dict.items() if max_increment in inner_dict]
            
            if id_max is not None:   
                t = camera_top_dict[id_max[0]][max_increment][2]

                if t1-self.dt_acceptable <= t <= t1+self.dt_acceptable:
                    camera_top_measurement = camera_top_dict
                else:
                    camera_top_measurement = 0  
            else:
                camera_top_measurement = 0


            max_increment = max((increment for inner_dict in camera_below_measurement.values() for increment in inner_dict), default=None)

            # Trouver tous les IDs ayant la valeur maximale d'incrémentation
            id_max = [id for id, inner_dict in camera_below_dict.items() if max_increment in inner_dict]
            
            if id_max is not None:   
                t = camera_below_dict[id_max[0]][max_increment][2]

                if t1-self.dt_acceptable <= t <= t1+self.dt_acceptable:
                    camera_below_measurement = camera_below_dict
                else:
                    camera_below_measurement = 0  
            else:
                camera_below_measurement = 0

            #on defini les dt pour chaque cas
            if imu_measurement != 0 and lidar_measurement != 0 and camera_below_measurement != 0 :
                dt = dt_imu_lidar_camera
                situation = 1
            elif imu_measurement != 0 and lidar_measurement != 0 and camera_below_measurement == 0:
                dt = dt_imu_lidar 
                situation = 2
            elif imu_measurement != 0 and lidar_measurement == 0 and camera_below_measurement != 0:  
                dt = dt_imu_camera
                situation = 3
            elif imu_measurement == 0 and lidar_measurement != 0 and camera_below_measurement != 0:
                dt = dt_lidar_camera
                situation = 4
            elif imu_measurement != 0 and lidar_measurement == 0 and camera_below_measurement == 0:
                dt = dt_imu
                situation = 5
            elif imu_measurement == 0 and lidar_measurement != 0 and camera_below_measurement == 0:
                dt= dt_lidar
                situation = 6
            elif imu_measurement == 0 and lidar_measurement == 0 and camera_below_measurement != 0:
                dt= dt_camera
                situation = 7
            


            uav_state, F, F_markers = self.state_function(self.state_uav, dt)

            if imu_measurement != 0:

                h_imu, H_imu = self.imu_measurement_matrix(uav_state)
                uav_state, P_uav = self.EKF_filter(imu_measurement,h_imu, H_imu, uav_state, F, self.Q_imu, self.R_imu, P_uav)

            if lidar_measurement != 0:

                h_lidar, H_lidar = self.lidar_measurement_matrix(uav_state)
                uav_state, P_uav = self.EKF_filter(lidar_measurement,h_lidar, H_lidar, uav_state, F, self.Q_lidar, self.R_lidar, P_uav)  

            if camera_top_measurement != 0:  

                uav_state, P_uav, state_markers, P_markers = self.slam(id_max, max_increment, camera_top_measurement, uav_state, F, P_uav, self.Q_camera_top, state_markers, P_markers, F_markers, self.Q_markers, self.R_camera_top, self.offset_imu_camera_top, self.offset_angle_top )

            if camera_below_measurement != 0:  

                uav_state, P_uav, state_markers, P_markers = self.slam(id_max, max_increment, camera_top_measurement, uav_state, F, P_uav, self.Q_camera_below, state_markers, P_markers, F_markers, self.Q_markers, self.R_camera_below, self.offset_imu_camera_below, self.offset_angle_below )  

            # Mettre à jour le temps pour la prochaine itération
            true_uav_position = uav_state

            imu_measurement = 0
            lidar_measurement = 0
            camera_top_measurement = 0
            camera_below_measurement = 0

            t2 = time.time()
            dt = t2-t1
            if situation==1:
                dt_imu_lidar_camera = dt
            elif situation ==2:
                dt_imu_lidar =dt
            elif situation ==3:
                dt_imu_camera = dt
            elif situation ==4:
                dt_lidar_camera = dt
            elif situation ==5:
                dt_imu = dt
            elif situation ==6:
                dt_lidar = dt
            elif situation ==7:
                dt_camera = dt
            

        

# Exécution des threads en arrière-plan









