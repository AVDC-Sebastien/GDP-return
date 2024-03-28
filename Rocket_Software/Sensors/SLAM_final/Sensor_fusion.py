import threading
import time
import numpy as np
import time
import cv2 # Import the OpenCV library
import math # Math library
import numpy as np # Import Numpy library
import logging
import board
import adafruit_bno055
import RPi.GPIO as GPIO
from scipy.spatial.transform import Rotation as R
from numpy.linalg import inv
from time import sleep
import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from picamera2 import Picamera2
from Measurement_save import save_tuning_data


class sensor_fusion():

    def __init__(self,target_angle_offset):
        
        self.save_data = save_tuning_data()
        self.saving_data = True

        self.stop_main_sensor_thread = False
        self.stop_measurement_thread = False

        # region parameters
        #----Paremeters-------
        #----IMU-------
        Q1 = 1
        QV = 1
        Q2= 2
        self.Q = np.matrix([[Q1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                                [0,Q1,0,0,0,0,0,0,0,0,0,0,0,0,0],
                                [0,0,Q1,0,0,0,0,0,0,0,0,0,0,0,0],
                                [0,0,0,QV,0,0,0,0,0,0,0,0,0,0,0],
                                [0,0,0,0,QV,0,0,0,0,0,0,0,0,0,0],
                                [0,0,0,0,0,QV,0,0,0,0,0,0,0,0,0],
                                [0,0,0,0,0,0,Q1,0,0,0,0,0,0,0,0],
                                [0,0,0,0,0,0,0,Q1,0,0,0,0,0,0,0],
                                [0,0,0,0,0,0,0,0,Q1,0,0,0,0,0,0],
                                [0,0,0,0,0,0,0,0,0,Q2,0,0,0,0,0],
                                [0,0,0,0,0,0,0,0,0,0,Q2,0,0,0,0],
                                [0,0,0,0,0,0,0,0,0,0,0,Q2,0,0,0],
                                [0,0,0,0,0,0,0,0,0,0,0,0,Q2,0,0],
                                [0,0,0,0,0,0,0,0,0,0,0,0,0,Q2,0],
                                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,Q2]]) 

        self.R_imu = np.matrix([[0.05,0,0,0,0,0,0,0,0],
                        [0,0.05,0,0,0,0,0,0,0],
                        [0,0,0.05,0,0,0,0,0,0],
                        [0,0,0,0.2,0,0,0,0,0],
                        [0,0,0,0,0.2,0,0,0,0],
                        [0,0,0,0,0,0.2,0,0,0],
                        [0,0,0,0,0,0,0.2,0,0],
                        [0,0,0,0,0,0,0,0.2,0],
                        [0,0,0,0,0,0,0,0,0.2]])

        #-----lidar-----
        # self.Q_lidar = np.matrix([[0.3,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        #                         [0,0.3,0,0,0,0,0,0,0,0,0,0,0,0,0],
        #                         [0,0,0.3,0,0,0,0,0,0,0,0,0,0,0,0],
        #                         [0,0,0,0.3,0,0,0,0,0,0,0,0,0,0,0],
        #                         [0,0,0,0,0.3,0,0,0,0,0,0,0,0,0,0],
        #                         [0,0,0,0,0,0.3,0,0,0,0,0,0,0,0,0],
        #                         [0,0,0,0,0,0,0.3,0,0,0,0,0,0,0,0],
        #                         [0,0,0,0,0,0,0,0.3,0,0,0,0,0,0,0],
        #                         [0,0,0,0,0,0,0,0,0.3,0,0,0,0,0,0],
        #                         [0,0,0,0,0,0,0,0,0,2,0,0,0,0,0],
        #                         [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0],
        #                         [0,0,0,0,0,0,0,0,0,0,0,2,0,0,0],
        #                         [0,0,0,0,0,0,0,0,0,0,0,0,2,0,0],
        #                         [0,0,0,0,0,0,0,0,0,0,0,0,0,2,0],
        #                         [0,0,0,0,0,0,0,0,0,0,0,0,0,0,2]]) 
        
        self.R_lidar = 0.01

        #--camera----
        # self.Q_camera_top = np.matrix([[0.3,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        #                             [0,0.3,0,0,0,0,0,0,0,0,0,0,0,0,0],
        #                             [0,0,0.3,0,0,0,0,0,0,0,0,0,0,0,0],
        #                             [0,0,0,0.3,0,0,0,0,0,0,0,0,0,0,0],
        #                             [0,0,0,0,0.3,0,0,0,0,0,0,0,0,0,0],
        #                             [0,0,0,0,0,0.3,0,0,0,0,0,0,0,0,0],
        #                             [0,0,0,0,0,0,0.3,0,0,0,0,0,0,0,0],
        #                             [0,0,0,0,0,0,0,0.3,0,0,0,0,0,0,0],
        #                             [0,0,0,0,0,0,0,0,0.3,0,0,0,0,0,0],
        #                             [0,0,0,0,0,0,0,0,0,2,0,0,0,0,0],
        #                             [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0],
        #                             [0,0,0,0,0,0,0,0,0,0,0,2,0,0,0],
        #                             [0,0,0,0,0,0,0,0,0,0,0,0,2,0,0],
        #                             [0,0,0,0,0,0,0,0,0,0,0,0,0,2,0],
        #                             [0,0,0,0,0,0,0,0,0,0,0,0,0,0,2]]) 
        
        self.R_camera_top = np.matrix([[0.2,0,0,0,0,0],
                                [0,0.2,0,0,0,0],
                                [0,0,0.2,0,0,0],
                                [0,0,0,0.2,0,0],
                                [0,0,0,0,0.2,0],
                                [0,0,0,0,0,0.2]])

        # self.Q_camera_below = np.matrix([[0.3,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        #                                 [0,0.3,0,0,0,0,0,0,0,0,0,0,0,0,0],
        #                                 [0,0,0.3,0,0,0,0,0,0,0,0,0,0,0,0],
        #                                 [0,0,0,0.3,0,0,0,0,0,0,0,0,0,0,0],
        #                                 [0,0,0,0,0.3,0,0,0,0,0,0,0,0,0,0],
        #                                 [0,0,0,0,0,0.3,0,0,0,0,0,0,0,0,0],
        #                                 [0,0,0,0,0,0,0.3,0,0,0,0,0,0,0,0],
        #                                 [0,0,0,0,0,0,0,0.3,0,0,0,0,0,0,0],
        #                                 [0,0,0,0,0,0,0,0,0.3,0,0,0,0,0,0],
        #                                 [0,0,0,0,0,0,0,0,0,2,0,0,0,0,0],
        #                                 [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0],
        #                                 [0,0,0,0,0,0,0,0,0,0,0,2,0,0,0],
        #                                 [0,0,0,0,0,0,0,0,0,0,0,0,2,0,0],
        #                                 [0,0,0,0,0,0,0,0,0,0,0,0,0,2,0],
        #                                 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,2]]) 
        
        self.R_camera_below = np.matrix([[0.1,0,0,0,0,0],
                                [0,0.1,0,0,0,0],
                                [0,0,0.05,0,0,0],
                                [0,0,0,0.1,0,0],
                                [0,0,0,0,0.1,0],
                                [0,0,0,0,0,0.1]])

        self.initial_state_uav = np.matrix([[0,0,0.607,0,0,0,0,0,0,0,0,0,0,0,0]])
        pv=20
        self.P_uav = np.matrix([[pv,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                        [0,pv,0,0,0,0,0,0,0,0,0,0,0,0,0],
                        [0,0,pv,0,0,0,0,0,0,0,0,0,0,0,0],
                        [0,0,0,pv,0,0,0,0,0,0,0,0,0,0,0],
                        [0,0,0,0,pv,0,0,0,0,0,0,0,0,0,0],
                        [0,0,0,0,0,pv,0,0,0,0,0,0,0,0,0],
                        [0,0,0,0,0,0,pv,0,0,0,0,0,0,0,0],
                        [0,0,0,0,0,0,0,pv,0,0,0,0,0,0,0],
                        [0,0,0,0,0,0,0,0,pv,0,0,0,0,0,0],
                        [0,0,0,0,0,0,0,0,0,pv,0,0,0,0,0],
                        [0,0,0,0,0,0,0,0,0,0,pv,0,0,0,0],
                        [0,0,0,0,0,0,0,0,0,0,0,pv,0,0,0],
                        [0,0,0,0,0,0,0,0,0,0,0,0,pv,0,0],
                        [0,0,0,0,0,0,0,0,0,0,0,0,0,pv,0],
                        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,pv]]) 
        self.dt = 0.01
        self.uav_state = self.initial_state_uav.transpose()

        # initial_markers_qualysis = np.matrix([[607.5, 1565.6, 8.8],
        #                             [29,1998.7, 5.5],
        #                             [-1468.4, 2105.7, 3.4],
        #                             [-3074, 3399, 1],
        #                             [-2361.7, 1334.1, 2.6],
        #                             [-3543.4, 607.3, 0.5],
        #                             [-2062.4, 380.7, 4.5],
        #                             [55, -570.6, 10],
        #                             [-806.5, -366.9, 11.6],
        #                             [-887.2, 1373.4, 6.7],
        #                             [-471.7, 373.6, 10],
        #                             [-2984, 1456.5, 0.5],
        #                             [-2754, 2465.4, -1.2],
        #                             [543.5, 795.4, 4.6],
        #                             [-1242, 871, 7.3],
        #                             [-1315.9, -414.3, 13],
        #                             [-2811.7, -142, 2.5],
        #                             [-1707.6, 1465.5, 1.4],
        #                             [-537.2, 2189.9, 7.4],
        #                             [-168, 1096.4, 7.3]])
        # ref_markers = np.matrix([[607.5, 1565.6, 8.8]])
        # markers_ref_id1_qualysis = initial_markers_qualysis - ref_markers
        # beta = np.radians(90)
        # coordonne = np.zeros((20, 3))
        # R_qual_euler = np.matrix([[np.cos(beta), np.sin(beta), 0],
        #                         [-np.sin(beta), np.cos(beta), 0],
        #                         [0,0,0]])
        # for i in range(20):
        #     popo = np.transpose(np.matrix(markers_ref_id1_qualysis[i,:]))
        #     final_vecteur = R_qual_euler @ popo
        #     coordonne[i,:] = np.array([
        #         [np.transpose(final_vecteur)]
        #     ])
        
        # self.initial_markers = np.matrix(coordonne)
        self.initial_markers = np.matrix([[0,0,0],
                                          [0,0,0]])

        # self.P_markers = np.matrix([[1,0,0],
        #                     [0,1,0],
        #                     [0,0,1]])
        num_markers = 3
        self.P_markers_dict = {}
        for i in range(num_markers):
        # Simulation : Ajouter une ligne avec ID, numéro d'incrémentation, temps et valeurs aléatoires
            new_row = [i,  # ID
                    np.matrix([[1,0,0],
                            [0,1,0],
                            [0,0,1]])] # Valeurs (simulées ici) converties en liste

            # Extraire l'ID et le temps de la nouvelle ligne
            marker_id = new_row[0]

            # Vérifier si l'ID existe déjà dans le dictionnaire
            if marker_id not in self.P_markers_dict:
                self.P_markers_dict[marker_id] = {}

            # Ajouter la nouvelle ligne dans le dictionnaire interne correspondant à l'ID
            self.P_markers_dict[marker_id] = new_row

        self.state_markers = self.initial_markers

        self.Q_markers =np.matrix([[0.01,0,0],
                            [0,0.01,0],
                            [0,0,0.01]])

        self.dt_acceptable = 1

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

        self.offset_imu_camera_top = [0, 0.2, 0.05]
        self.offset_imu_camera_below = [0.05, 0.0, 0]
        self.offset_angle_top = 90
        self.offset_angle_below = 180
        self.offset_lidar = [0.09,0.03,0]

        self.target_angle_offset = target_angle_offset
        self.target_angle_offset = (0,-7,0)

        self.calibration_faite = 0

        # Structure de données partagée

        self.get_imu_meas = 0
        self.get_camera_top_meas = 0
        self.get_camera_below_meas = 0
        self.get_lidar_meas =0

        # print task init
        self.true_uav_position = [0,0,0.607]

        self.lock = threading.Lock()

    def plot_3d_curve(self, x, y, z):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x, y, z)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()    
    def state_function(self,state_uav, dt):
        
        #matrice rotation bodyframe->absolute
        position = np.transpose(np.matrix(state_uav[0:3]))
        velocity = np.transpose(np.matrix(state_uav[3:6]))
        euler = np.transpose(np.radians(np.matrix(state_uav[6:9])))
        euler_deg = np.transpose(np.matrix(state_uav[6:9]))
        angular_rate = np.transpose(np.matrix(state_uav[9:12]))
        linear_acceleration = np.matrix(state_uav[12:15])
        ax = linear_acceleration[0,0]
        ay = linear_acceleration[1,0]
        az = linear_acceleration[2,0]
        w1 = np.degrees(angular_rate[0,0])
        w2 = np.degrees(angular_rate[0,1])
        w3 = np.degrees(angular_rate[0,2])
        print("velocity dans l estimation de state", velocity)
        
        
        R_imu_abs = np.matrix([
                                    [np.cos(euler[0,1])*np.cos(euler[0,0]), np.cos(euler[0,1])*np.sin(euler[0,0]), -np.sin(euler[0,1])],
                                    [np.sin(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])-np.cos(euler[0,2])*np.sin(euler[0,0]),  np.sin(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])+np.cos(euler[0,2])*np.cos(euler[0,0]),  np.sin(euler[0,2])*np.cos(euler[0,1])],
                                    [np.cos(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])+np.sin(euler[0,2])*np.sin(euler[0,0]),  np.cos(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])-np.sin(euler[0,2])*np.cos(euler[0,0]),  np.cos(euler[0,2])*np.cos(euler[0,1])]
            ])
        
        #matrice pour passer de anglar rate a euler rate
        B_theta = np.matrix([
                            [0, np.sin(euler[0,2]), np.cos(euler[0,2])],
                            [0, np.cos(euler[0,1])*np.cos(euler[0,2]), -np.cos(euler[0,1])*np.sin(euler[0,2])],
                            [np.cos(euler[0,1]), np.sin(euler[0,1])*np.sin(euler[0,2]), np.sin(euler[0,1])*np.cos(euler[0,2])]
                            ])
        euler_dot = (1/(np.cos(euler[0,1])))*B_theta @ angular_rate.transpose()
        print("eleur dot a la main", euler_dot)
        position_e = position.transpose() + velocity.transpose()*dt

        velocity_e = velocity.transpose() + (R_imu_abs@linear_acceleration)*dt

        estimated_euler = np.degrees(euler.transpose() + euler_dot.transpose()*dt)
    
        estimated_state_uav = np.array([[position_e[0,0], position_e[1,0], position_e[2,0],velocity_e[0,0], velocity_e[1,0], velocity_e[2,0], estimated_euler[0,0], estimated_euler[1,0], estimated_euler[2,0], angular_rate[0,0], angular_rate[0,1], angular_rate[0,2],linear_acceleration[0,0], linear_acceleration[1,0], linear_acceleration[2,0]]])
        estimated_state_uav = estimated_state_uav.transpose()
        print("state estimé a la main",estimated_state_uav)
        F47 = (-np.cos(euler[0,1])*np.sin(euler[0,0])*ax + np.cos(euler[0,1])*np.cos(euler[0,0])*ay)*dt
        F48 = (-np.sin(euler[0,1])*np.cos(euler[0,0])*ax - np.sin(euler[0,1])*np.sin(euler[0,0])*ay - np.cos(euler[0,1])*az)*dt
        F413 = np.cos(euler[0,1])*np.cos(euler[0,0])*dt
        F414 = np.cos(euler[0,1])*np.sin(euler[0,0])*dt
        F415 = -np.sin(euler[0,1])*dt
        F57 = ((-np.sin(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])-np.cos(euler[0,2])*np.cos(euler[0,0]))*ax + (np.sin(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0]) - np.cos(euler[0,2])*np.sin(euler[0,0]))*ay)*dt
        F58 = ((np.sin(euler[0,2])*np.cos(euler[0,1])*np.cos(euler[0,0]))*ax + np.sin(euler[0,2])*np.cos(euler[0,1])*np.sin(euler[0,0])*ay - np.sin(euler[0,2])*np.sin(euler[0,1])*az)*dt
        F59 = ((np.cos(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0]) + np.sin(euler[0,2])*np.sin(euler[0,0]))*ax + (np.cos(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0]) - np.cos(euler[0,0])*np.sin(euler[0,2]))*ay + np.cos(euler[0,2])*np.cos(euler[0,1])*az)*dt
        F513 = (np.sin(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])-np.cos(euler[0,2])*np.sin(euler[0,0]))*dt
        F514 = (np.sin(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0]) + np.cos(euler[0,2])*np.cos(euler[0,0]))*dt
        F515 = np.sin(euler[0,2])*np.cos(euler[0,1])*dt
        F67 = ((-np.cos(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0]) + np.sin(euler[0,2])*np.cos(euler[0,0]))*ax + (np.cos(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0]) + np.sin(euler[0,2])*np.sin(euler[0,0]))*ay)*dt
        F68 = (np.cos(euler[0,2])*np.cos(euler[0,1])*np.cos(euler[0,0])*ax + np.cos(euler[0,2])*np.cos(euler[0,1])*np.sin(euler[0,0])*ay - np.cos(euler[0,2])*np.sin(euler[0,1])*az)*dt
        F69 = ((-np.sin(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0]) + np.cos(euler[0,2])*np.sin(euler[0,0]))*ax + (-np.sin(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0]) - np.cos(euler[0,2])*np.cos(euler[0,0]))*ay -np.sin(euler[0,2])*np.cos(euler[0,1])*az)*dt
        F613 = (np.cos(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0]) + np.sin(euler[0,2])*np.sin(euler[0,0]))*dt
        F614 = (np.cos(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0]) - np.sin(euler[0,2])*np.cos(euler[0,0]))*dt
        F615 = np.cos(euler[0,2])*np.cos(euler[0,1])*dt
        F78 = np.sin(euler[0,1])*(np.sin(euler[0,2])*w2 + np.cos(euler[0,2])*w3)*dt/(np.cos(euler[0,1])**2)
        F79 = (np.cos(euler[0,2])*w2 - np.sin(euler[0,2])*w3)*np.cos(euler[0,1])*dt/(np.cos(euler[0,1])**2)
        F711 = np.sin(euler[0,2])*dt/np.cos(euler[0,1])
        F712 = np.cos(euler[0,2])*dt/np.cos(euler[0,1])
        F88 = 1 + ((-np.sin(euler[0,1])*np.cos(euler[0,1])*w2 + np.sin(euler[0,1])*np.sin(euler[0,2])*w3)*np.cos(euler[0,1]) + np.sin(euler[0,1])*(np.cos(euler[0,1])*np.cos(euler[0,2])*w2 - np.cos(euler[0,1])*np.sin(euler[0,2])*w3))*dt/(np.cos(euler[0,1])**2)
        F89 = (-np.sin(euler[0,2])*np.cos(euler[0,1])*w2 - np.cos(euler[0,1])*np.cos(euler[0,2])*w3)*np.cos(euler[0,1])*dt/(np.cos(euler[0,1])**2)
        F811 = np.cos(euler[0,2])*dt
        F812 = -np.sin(euler[0,2])*dt
        F98 = ((-np.sin(euler[0,1])*w1 + np.cos(euler[0,1])*np.sin(euler[0,2])*w2 + np.cos(euler[0,1])*np.cos(euler[0,2])*w3)*np.cos(euler[0,1]) + np.sin(euler[0,1])*(np.cos(euler[0,1])*w1 + np.sin(euler[0,1])*np.sin(euler[0,2])*w2 + np.sin(euler[0,1])*np.cos(euler[0,2])*w3))*dt/(np.cos(euler[0,1])**2)
        F99 = 1 + ((np.sin(euler[0,1])*np.cos(euler[0,2])*w2 - np.sin(euler[0,1])*np.sin(euler[0,2])*w3)*np.cos(euler[0,1]))*dt/(np.cos(euler[0,1])**2)
        F911 = np.sin(euler[0,1])*np.sin(euler[0,2])*dt/np.cos(euler[0,1])
        F912 = np.sin(euler[0,1])*np.cos(euler[0,2])*dt/np.cos(euler[0,1])

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

        print("F a la main:",F)

        pos_x = position[0,0]
        pos_y = position[0,1]
        pos_z = position[0,2]
        vel_x = velocity[0,0]
        vel_y = velocity[0,1]
        vel_z = velocity[0,2]
        acc_x = linear_acceleration[0,0]
        acc_y = linear_acceleration[1,0]
        acc_z = linear_acceleration[2,0]
        omega_1 = angular_rate[0,0]
        omega_2 = angular_rate[0,1]
        omega_3 = angular_rate[0,2] 
        theta_1 = euler[0,0] 
        theta_2 = euler[0,1] 
        theta_3 = euler[0,2] 

        state_estimated = np.matrix([
            [pos_x + dt*vel_x],
            [ pos_y + dt*vel_y],
            [ pos_z + dt*vel_z],
            [ vel_x + dt*(acc_x*np.cos(theta_1)*np.cos(theta_2) - acc_z*np.sin(theta_2) + acc_y*np.cos(theta_2)*np.sin(theta_1))],
            [ vel_y + dt*(acc_y*(np.cos(theta_1)*np.cos(theta_3) + np.sin(theta_1)*np.sin(theta_2)*np.sin(theta_3)) - acc_x*(np.cos(theta_3)*np.sin(theta_1) - np.cos(theta_1)*np.sin(theta_2)*np.sin(theta_3)) + acc_z*np.cos(theta_2)*np.sin(theta_3))],
            [ vel_z + dt*(acc_x*(np.sin(theta_1)*np.sin(theta_3) + np.cos(theta_1)*np.cos(theta_2)*np.cos(theta_3)) - acc_y*(np.cos(theta_1)*np.sin(theta_3) - np.cos(theta_2)*np.cos(theta_3)*np.sin(theta_1)) + acc_z*np.cos(theta_2)*np.cos(theta_3))],
            [ np.degrees(theta_1 + dt*((omega_3*np.cos(theta_3))/np.cos(theta_2) + (omega_2*np.sin(theta_3))/np.cos(theta_2)))],
            [ np.degrees(theta_2 + dt*(omega_2*np.cos(theta_3) - omega_3*np.sin(theta_3)))],
            [ np.degrees(theta_3 + dt*(omega_1 + (omega_3*np.cos(theta_3)*np.sin(theta_2))/np.cos(theta_2) + (omega_2*np.sin(theta_2)*np.sin(theta_3))/np.cos(theta_2)))],
            [ omega_1],
            [ omega_2],
            [ omega_3],
            [ acc_x],
            [ acc_y],
            [ acc_z]
        ])
        F_matlab = np.matrix([
            [1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [ 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [ 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [ 0, 0, 0, 1, 0, 0, dt*(acc_y*np.cos(theta_1)*np.cos(theta_2) - acc_x*np.cos(theta_2)*np.sin(theta_1)), -dt*(acc_z*np.cos(theta_2) + acc_x*np.cos(theta_1)*np.sin(theta_2) + acc_y*np.sin(theta_1)*np.sin(theta_2)), 0, 0, 0, 0, dt*np.cos(theta_1)*np.cos(theta_2), dt*np.cos(theta_2)*np.sin(theta_1), -dt*np.sin(theta_2)],
            [ 0, 0, 0, 0, 1, 0, -dt*(acc_x*(np.cos(theta_1)*np.cos(theta_3) + np.sin(theta_1)*np.sin(theta_2)*np.sin(theta_3)) + acc_y*(np.cos(theta_3)*np.sin(theta_1) - np.cos(theta_1)*np.sin(theta_2)*np.sin(theta_3))), dt*(acc_x*np.cos(theta_1)*np.cos(theta_2)*np.sin(theta_3) - acc_z*np.sin(theta_2)*np.sin(theta_3) + acc_y*np.cos(theta_2)*np.sin(theta_1)*np.sin(theta_3)), dt*(acc_x*(np.sin(theta_1)*np.sin(theta_3) + np.cos(theta_1)*np.cos(theta_3)*np.sin(theta_2)) - acc_y*(np.cos(theta_1)*np.sin(theta_3) - np.cos(theta_3)*np.sin(theta_1)*np.sin(theta_2)) + acc_z*np.cos(theta_2)*np.cos(theta_3)), 0, 0, 0, -dt*(np.cos(theta_3)*np.sin(theta_1) - np.cos(theta_1)*np.sin(theta_2)*np.sin(theta_3)), dt*(np.cos(theta_1)*np.cos(theta_3) + np.sin(theta_1)*np.sin(theta_2)*np.sin(theta_3)), dt*np.cos(theta_2)*np.sin(theta_3)],
            [ 0, 0, 0, 0, 0, 1, dt*(acc_x*(np.cos(theta_1)*np.sin(theta_3) - np.cos(theta_2)*np.cos(theta_3)*np.sin(theta_1)) + acc_y*(np.sin(theta_1)*np.sin(theta_3) + np.cos(theta_1)*np.cos(theta_2)*np.cos(theta_3))), -dt*(acc_z*np.cos(theta_3)*np.sin(theta_2) + acc_x*np.cos(theta_1)*np.cos(theta_3)*np.sin(theta_2) + acc_y*np.cos(theta_3)*np.sin(theta_1)*np.sin(theta_2)), -dt*(acc_y*(np.cos(theta_1)*np.cos(theta_3) + np.cos(theta_2)*np.sin(theta_1)*np.sin(theta_3)) - acc_x*(np.cos(theta_3)*np.sin(theta_1) - np.cos(theta_1)*np.cos(theta_2)*np.sin(theta_3)) + acc_z*np.cos(theta_2)*np.sin(theta_3)), 0, 0, 0, dt*(np.sin(theta_1)*np.sin(theta_3) + np.cos(theta_1)*np.cos(theta_2)*np.cos(theta_3)), -dt*(np.cos(theta_1)*np.sin(theta_3) - np.cos(theta_2)*np.cos(theta_3)*np.sin(theta_1)), dt*np.cos(theta_2)*np.cos(theta_3)],
            [ 0, 0, 0, 0, 0, 0, 1, dt*((omega_3*np.cos(theta_3)*np.sin(theta_2))/np.cos(theta_2)**2 + (omega_2*np.sin(theta_2)*np.sin(theta_3))/np.cos(theta_2)**2), dt*((omega_2*np.cos(theta_3))/np.cos(theta_2) - (omega_3*np.sin(theta_3))/np.cos(theta_2)), 0, (dt*np.sin(theta_3))/np.cos(theta_2), (dt*np.cos(theta_3))/np.cos(theta_2), 0, 0, 0],
            [ 0, 0, 0, 0, 0, 0, 0, 1, -dt*(omega_3*np.cos(theta_3) + omega_2*np.sin(theta_3)), 0, dt*np.cos(theta_3), -dt*np.sin(theta_3), 0, 0, 0],
            [ 0, 0, 0, 0, 0, 0, 0, dt*(omega_3*np.cos(theta_3) + omega_2*np.sin(theta_3) + (omega_3*np.cos(theta_3)*np.sin(theta_2)**2)/np.cos(theta_2)**2 + (omega_2*np.sin(theta_2)**2*np.sin(theta_3))/np.cos(theta_2)**2), dt*((omega_2*np.cos(theta_3)*np.sin(theta_2))/np.cos(theta_2) - (omega_3*np.sin(theta_2)*np.sin(theta_3))/np.cos(theta_2)) + 1, dt, (dt*np.sin(theta_2)*np.sin(theta_3))/np.cos(theta_2), (dt*np.cos(theta_3)*np.sin(theta_2))/np.cos(theta_2), 0, 0, 0],
            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
        ])
        print("state estimé par matlab:", state_estimated)
        print("F de matlab",F_matlab)
        F_markers = np.matrix([[1,0,0],
                              [0,1,0],
                              [0,0,1]])
        
        print("etat estimé", estimated_state_uav)

        return estimated_state_uav, F, F_markers
    def EKF_filter(self,H, predicted_state_uav, F, Q, R, P, residual):

        # Pp= F @ P @ (F.transpose()) + Q
        S = H @ P @ (H.transpose()) + R
        K = (P @ H.transpose())*inv(S)
        

        #update
        update_state = predicted_state_uav + K*residual
        P = P - K @ S @ K.transpose()
        print("covariance", P)
        return update_state, P
    
    def residual_euler(self,measurement,estimation):
        residual = ((measurement - estimation + 180) % 360 - 180)
        if residual < -180:
            residual += 360
        elif residual > 180:
            residual -= 360
        return residual
        
    def imu_measurement_matrix(self,estimated_state_uav):

        h = np.array([
                        [estimated_state_uav[6,0], estimated_state_uav[7,0], estimated_state_uav[8,0], estimated_state_uav[9,0], estimated_state_uav[10,0], estimated_state_uav[11,0], estimated_state_uav[12,0], estimated_state_uav[13,0], estimated_state_uav[14,0] ]
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
    def lidar_measurement_matrix(self,estimated_state_uav, offset):
        # x_z = estimated_state_uav[2,0]
        # # print(x_z)
        # pitch = np.radians(estimated_state_uav[7,0])
        # roll = np.radians(estimated_state_uav[8,0])
        euler = np.radians(np.matrix(estimated_state_uav[6:9]))
        position = np.matrix(estimated_state_uav[0:3])
        offset_z = offset[2]
        offset_center = np.sqrt(offset[0]**2 + offset[1]**2)

        # h = (x_z/(np.cos(pitch)*np.cos(roll)))

        # H = np.array([
        #                 [0, 0, 1/(np.cos(pitch)*np.cos(roll)), 0, 0, 0, 0, np.cos(roll)*np.sin(pitch)/((np.cos(pitch)*np.cos(roll))**2), np.cos(pitch)*np.sin(roll)/((np.cos(pitch)*np.cos(roll))**2), 0, 0, 0, 0, 0, 0]
        #             ])
        
        print("test eue",euler[1,0])
        R_imu_abs = np.matrix([
            [np.cos(euler[1,0])*np.cos(euler[0,0]), np.cos(euler[1,0])*np.sin(euler[0,0]), -np.sin(euler[1,0])],
            [np.sin(euler[2,0])*np.sin(euler[1,0])*np.cos(euler[0,0])-np.cos(euler[2,0])*np.sin(euler[0,0]),  np.sin(euler[2,0])*np.sin(euler[1,0])*np.sin(euler[0,0])+np.cos(euler[2,0])*np.cos(euler[0,0]),  np.sin(euler[2,0])*np.cos(euler[1,0])],
            [np.cos(euler[2,0])*np.sin(euler[1,0])*np.cos(euler[0,0])+np.sin(euler[2,0])*np.sin(euler[0,0]),  np.cos(euler[2,0])*np.sin(euler[1,0])*np.sin(euler[0,0])-np.sin(euler[2,0])*np.cos(euler[0,0]),  np.cos(euler[2,0])*np.cos(euler[1,0])]
        ])

        A = np.transpose(np.matrix([[0,0,3]]))
        a = R_imu_abs@A
        alpha = np.degrees(np.arccos(a[2]/A[2]))
        beta = 90-alpha
        alpha_rad = np.radians(alpha)
        beta_rad = np.radians(beta)
        h_lidar = ((position[2,0] + (np.cos(beta_rad)*offset_center))/np.cos(alpha_rad))-offset_z
        print("estimation mesurement lidar", h_lidar)

        theta_1 = euler[0]
        theta_2 = euler[1,0]
        theta_3 = euler[2,0]
        pos_z = position[2,0]

        H_lidar = np.matrix([
            [0, 0, 1/(np.cos(theta_2)*np.cos(theta_3)), 0, 0, 0, 0, (np.sin(theta_2)*(pos_z + offset_center*np.cos(np.arccos(np.cos(theta_2)*np.cos(theta_3)) - 90)))/(np.cos(theta_2)**2*np.cos(theta_3)) - (offset_center*np.sin(np.arccos(np.cos(theta_2)*np.cos(theta_3)) - 90)*np.sin(theta_2))/(np.cos(theta_2)*np.sqrt(1 - np.cos(theta_2)**2*np.cos(theta_3)**2)), (np.sin(theta_3)*(pos_z + offset_center*np.cos(np.arccos(np.cos(theta_2)*np.cos(theta_3)) - 90)))/(np.cos(theta_2)*np.cos(theta_3)**2) - (offset_center*np.sin(np.arccos(np.cos(theta_2)*np.cos(theta_3)) - 90)*np.sin(theta_3))/(np.cos(theta_3)*np.sqrt(1 - np.cos(theta_2)**2*np.cos(theta_3)**2)), 0, 0, 0, 0, 0, 0]
        ])

        return h_lidar, H_lidar

    def camera_markers_measurement(self,state_uav, state_markers, offset, offset_angle) :
        # print(state_uav)
        # state_uav = np.squeeze(np.asarray(state_uav))
        # offset = np.squeeze(np.asarray(offset))
        # yaw = np.squeeze(np.asarray(state_uav[6]))
        # pitch = np.squeeze(np.asarray(state_uav[7]))
        # roll =np.squeeze(np.asarray(state_uav[8]))
        
        # euler_rad = np.array([yaw, pitch, roll])
        # euler = np.radians(euler_rad)
        euler = np.transpose(np.radians(np.matrix(state_uav[6:9])))
        yaw = euler[0,0]
        pitch = euler[0,1]
        roll = euler[0,2]
        position = np.transpose(np.matrix(state_uav[0:3]))
        
#         print("euler",euler)
#         print("test", [[np.cos(euler[0,1])*np.cos(euler[0,0]), np.cos(euler[0,1])*np.sin(euler[0,0]), -np.sin(euler[0,1])],
#                                     [np.sin(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])-np.cos(euler[0,2])*np.sin(euler[0,0]),  np.sin(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])+np.cos(euler[0,2])*np.cos(euler[0,0]),  np.sin(euler[0,2])*np.cos(euler[0,1])],
#                                     [np.cos(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])+np.sin(euler[0,2])*np.sin(euler[0,0]),  np.cos(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])-np.sin(euler[0,2])*np.cos(euler[0,0]),  np.cos(euler[0,2])*np.cos(euler[0,1])]
#  ] )
        R_imu_abs = np.matrix([
                                    [np.cos(euler[0,1])*np.cos(euler[0,0]), np.cos(euler[0,1])*np.sin(euler[0,0]), -np.sin(euler[0,1])],
                                    [np.sin(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])-np.cos(euler[0,2])*np.sin(euler[0,0]),  np.sin(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])+np.cos(euler[0,2])*np.cos(euler[0,0]),  np.sin(euler[0,2])*np.cos(euler[0,1])],
                                    [np.cos(euler[0,2])*np.sin(euler[0,1])*np.cos(euler[0,0])+np.sin(euler[0,2])*np.sin(euler[0,0]),  np.cos(euler[0,2])*np.sin(euler[0,1])*np.sin(euler[0,0])-np.sin(euler[0,2])*np.cos(euler[0,0]),  np.cos(euler[0,2])*np.cos(euler[0,1])]
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

        # print("state markers",state_markers)

        meas_x = position[0,0]+offset_x-state_markers[0,0]
        meas_y = position[0,1]+offset_y-state_markers[0,1]
        meas_z = position[0,2]+offset_z-state_markers[0,2]

        estimation = np.matrix([
            [meas_x, meas_y, meas_z, euler_cam_z, euler_cam_y, euler_cam_x]
        ])

        print("estimation des mesure de la cam", estimation)
    
        h_camera = estimation.transpose()
        H_camera = np.array([
                                [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, -(((np.real(np.cos(yaw)*np.cos(roll)) + np.imag(np.cos(pitch)*np.sin(yaw)) + np.real(np.sin(yaw)*np.sin(pitch)*np.sin(roll)))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw))) + ((np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw)))*(np.imag(np.cos(yaw)*np.cos(roll)) + np.imag(np.sin(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(pitch)*np.sin(yaw))))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))**2)*(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))**2)/((np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))**2 + (np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw)))**2), (((np.real(np.cos(yaw)*np.cos(pitch)*np.sin(roll)) - np.imag(np.cos(yaw)*np.sin(pitch)))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw))) + ((np.imag(np.cos(yaw)*np.cos(pitch)*np.sin(roll)) + np.real(np.cos(yaw)*np.sin(pitch)))*(np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw))))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))**2)*(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))**2)/((np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))**2 + (np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw)))**2), (((np.real(np.cos(yaw)*np.cos(roll)*np.sin(pitch)) + np.real(np.sin(yaw)*np.sin(roll)))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw))) + ((np.imag(np.cos(yaw)*np.cos(roll)*np.sin(pitch)) + np.imag(np.sin(yaw)*np.sin(roll)))*(np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw))))/(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))**2)*(np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))**2)/((np.real(np.cos(yaw)*np.cos(pitch)) - np.imag(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.imag(np.cos(roll)*np.sin(yaw)))**2 + (np.imag(np.cos(yaw)*np.cos(pitch)) + np.real(np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.real(np.cos(roll)*np.sin(yaw)))**2), 0, 0, 0, 0, 0, 0],
                                [ 0, 0, 0, 0, 0, 0, -(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))/np.sqrt(1 - (np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))**2), (np.cos(yaw)*np.cos(roll)*np.sin(pitch))/np.sqrt(1 - (np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))**2), -(np.cos(roll)*np.sin(yaw) - np.cos(yaw)*np.cos(pitch)*np.sin(roll))/np.sqrt(1 - (np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))**2), 0, 0, 0, 0, 0, 0],
                                [ 0, 0, 0, 0, 0, 0, (((np.real(np.cos(alpha)*(np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))) + np.imag(np.sin(alpha)*(np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll))) - ((np.imag(np.cos(alpha)*(np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))) - np.real(np.sin(alpha)*(np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.cos(pitch)*np.cos(roll))))*(np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll))))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2)*(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2)/((np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2 + (np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2), -(((np.real(np.cos(alpha)*np.cos(roll)*np.sin(yaw)*np.sin(pitch)) + np.imag(np.sin(alpha)*np.cos(roll)*np.sin(yaw)*np.sin(pitch)) + np.imag(np.cos(alpha)*np.cos(roll)*np.sin(pitch)) - np.real(np.sin(alpha)*np.cos(roll)*np.sin(pitch)))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll))) + ((np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))*(np.real(np.sin(alpha)*np.cos(roll)*np.sin(yaw)*np.sin(pitch)) - np.imag(np.cos(alpha)*np.cos(roll)*np.sin(yaw)*np.sin(pitch)) + np.real(np.cos(alpha)*np.cos(roll)*np.sin(pitch)) + np.imag(np.sin(alpha)*np.cos(roll)*np.sin(pitch))))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2)*(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2)/((np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2 + (np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2), -(((np.real(np.cos(alpha)*(np.cos(yaw)*np.cos(roll) + np.cos(pitch)*np.sin(yaw)*np.sin(roll))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.cos(roll) + np.cos(pitch)*np.sin(yaw)*np.sin(roll))) + np.imag(np.cos(alpha)*np.cos(pitch)*np.sin(roll)) - np.real(np.sin(alpha)*np.cos(pitch)*np.sin(roll)))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll))) + ((np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))*(np.real(np.sin(alpha)*(np.cos(yaw)*np.cos(roll) + np.cos(pitch)*np.sin(yaw)*np.sin(roll))) - np.imag(np.cos(alpha)*(np.cos(yaw)*np.cos(roll) + np.cos(pitch)*np.sin(yaw)*np.sin(roll))) + np.real(np.cos(alpha)*np.cos(pitch)*np.sin(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.sin(roll))))/(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2)*(np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2)/((np.imag(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.real(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.real(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.imag(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2 + (np.real(np.cos(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) + np.imag(np.sin(alpha)*(np.cos(yaw)*np.sin(roll) - np.cos(pitch)*np.cos(roll)*np.sin(yaw))) - np.imag(np.cos(alpha)*np.cos(pitch)*np.cos(roll)) + np.real(np.sin(alpha)*np.cos(pitch)*np.cos(roll)))**2), 0, 0, 0, 0, 0, 0]
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

    def slam(self,id_max, max_increment, camera_dict, uav_state, F_uav, P_uav, Q_uav, state_markers, P_markers, F_markers, Q_markers, R, offset, offset_angle):

        for value in id_max:
            # print("top dic cam:", camera_dict)
            # print("value",value)
            # print("max incrementation", max_increment)
            
            mesure = camera_dict[value][max_increment][3][0]
            z = np.array([
                                            [mesure[0], mesure[1], mesure[2], mesure[3], mesure[4], mesure[5]]
                                        ])
            #------------EKF SLAM----------------------------------------------------------
            #loop for each landmarks detected
            #markers estimation
            state_markers_fonction = state_markers[value-1,:]
            #Ppredicted_markers = F_markers @ P_markers[value][1] @ F_markers.transpose() + Q_markers
            h_markers, H_camera, H_markers = self.camera_markers_measurement(uav_state, state_markers_fonction, offset, offset_angle)
            S_markers = H_markers@P_markers@H_markers.transpose() + R
            K_markers = (P_markers@H_markers.transpose())@inv(S_markers)
            #residual, verifier comment on le fait
            residual_markers = z.transpose() - h_markers  #verifier a bien faire la transpose si necessaire

            residual_markers[3,0] = self.residual_euler(z[0,3], h_markers[3,0])
            residual_markers[4,0] = self.residual_euler(z[0,4], h_markers[4,0])
            residual_markers[5,0] = self.residual_euler(z[0,5], h_markers[5,0])
            # print("z",z)
            # print("h markers", h_markers)
            # print("k markers", K_markers)
            # print("state", state_markers[value,:])

            state_markers[value-1,:]= state_markers[value-1,:] + (K_markers@residual_markers).transpose()
            P_markers[value][1] = P_markers - K_markers@H_markers@P_markers.transpose()

            # print("apres transpose", state_markers[value,:])

            #uav estimation
            #Ppredicted_uav = F_uav@P_uav@F_uav.transpose() + Q_uav 
            h_camera, H_camera, H_markers = self.camera_markers_measurement(uav_state, state_markers_fonction, offset, offset_angle)
            S_uav = H_camera@P_uav@H_camera.transpose() + R
            K_uav = (P_uav@H_camera.transpose())@inv(S_uav)
            #faire residual uav
            residual_uav = z.transpose() - h_camera
            residual_uav[3,0] = self.residual_euler(z[0,3], h_camera[3,0])
            residual_uav[4,0] = self.residual_euler(z[0,4], h_camera[4,0])
            residual_uav[5,0] = self.residual_euler(z[0,5], h_camera[5,0])
            print('residual de la camera', residual_uav)
            uav_state = uav_state + K_uav*residual_uav
            P_uav = P_uav - K_uav*H_camera*P_uav.transpose()
            print("covariance apres le slam", P_uav)
            #end of the loop------------------------------------------------------
        return uav_state, P_uav, state_markers, P_markers

# ajouter def measurment pour camera ET pour markers

    def Start_measurement(self):
        self.imu_thread = threading.Thread(target=self.imu_task, args=[self.get_imu_meas,self.target_angle_offset])
        self.camera_top_thread = threading.Thread(target=self.camera_top_task, args=[self.get_camera_top_meas])
        self.camera_below_thread = threading.Thread(target=self.camera_below_task, args=[self.get_camera_top_meas])
        self.lidar_thread = threading.Thread(target=self.lidar_task, args=[self.get_lidar_meas])
        # self.print_thread = threading.Thread(target=self.print_task)

        self.imu_thread.start()
        self.camera_top_thread.start()
        self.camera_below_thread.start()
        self.lidar_thread.start()
        # self.print_thread.start()
        
    def Stop_measurement(self):
        self.saving_data = False
        self.save_data.save_all()
        time.sleep(10)
        self.stop_measurement_thread = True
        self.stop_main_sensor_thread = True
        self.imu_thread.join()
        self.camera_top_thread.join()
        self.camera_below_thread.join()
        self.lidar_thread.join()
    
    def save_data_to_file(self):
        self.save_data.save_all()
        
        

# Tâche Principale
    def main_task(self):
        P_uav = self.P_uav
        while not self.stop_main_sensor_thread:

            t1 = time.time()
            # print("lock", self.lock)
            with self.lock:
                # print("lock fermé", self.lock)
                
                if type(self.get_camera_top_meas) != int:
                    camera_top_dict = copy.deepcopy(self.get_camera_top_meas)
                else:
                    camera_top_dict = 0
                if type(self.get_camera_below_meas) != int:
                    try:
                        camera_below_dict = copy.deepcopy(self.get_camera_below_meas)
                    except Exception as e:
                        print("Une erreur s'est produite lors de la copie profonde :", e)
                        camera_below_dict = 0

                else:
                    camera_below_dict = 0
                if type(self.get_imu_meas) != int:
                    imu_measurement = self.get_imu_meas[:]
                else:
                    imu_measurement=0
                if type(self.get_lidar_meas) != int:
                    lidar_measurement = self.get_lidar_meas.copy()
                else:
                    lidar_measurement=0
            # camera_top_dict = 0
            # camera_below_dict = 0
            # imu_measurement = 0
            # lidar_measurement = 0
            # print("cam meas",camera_top_dict)
            # print("cam b meas", camera_below_dict)
            # print("imu meas",imu_measurement)
            # print("lidar meas",self.get_lidar_meas)
            # print("type 1",type(imu_measurement))
            # print("time meas", imu_measurement[0][0])

            if type(imu_measurement) != int:  
                
                if t1-self.dt_acceptable <= imu_measurement[0][0] <= t1+self.dt_acceptable: 
                    # print("sebastien ne sait pas")
                    imu_measurement = np.array([
                                                    [imu_measurement[1][0], imu_measurement[1][1], imu_measurement[1][2], imu_measurement[2][0], imu_measurement[2][1], imu_measurement[2][2], imu_measurement[3][0], imu_measurement[3][1], imu_measurement[3][2]]
                                                    ])
                    
                else:
                    imu_measurement = 0
            # print("meas imu apres if", imu_measurement)
            # print("type",type(imu_measurement))
                    
            ############################################## LIDAR ###################################################################

            if type(lidar_measurement) != int:  
                # print("temps lidar",lidar_measurement[1])
                if t1-self.dt_acceptable <= lidar_measurement[1] <= t1+self.dt_acceptable: 
                    lidar_measurement = lidar_measurement[0]
                else:
                    lidar_measurement = 0
            # print("lidar apres if", lidar_measurement)

            ############################################# CAMERA TOP  ############################################################       

            if type(camera_top_dict) != int and camera_top_dict != {}:
                max_increment_t = max((increment for inner_dict in camera_top_dict.values() for increment in inner_dict), default=None)

                # Trouver tous les IDs ayant la valeur maximale d'incrémentation
                id_max_top = [id for id, inner_dict in camera_top_dict.items() if max_increment_t in inner_dict]
            else:
                id_max_top = []
                
            if id_max_top != []:   
                t = camera_top_dict[id_max_top[0]][max_increment_t][2]

                if t1-self.dt_acceptable <= t <= t1+self.dt_acceptable:
                    camera_top_measurement = camera_top_dict
                else:
                    camera_top_measurement = 0  
            else:
                camera_top_measurement = 0
            ###########################################CAMERA BELOW #############################################################

            if type(camera_below_dict) != int and camera_below_dict != {}:

                max_increment_b = max((increment for inner_dict in camera_below_dict.values() for increment in inner_dict), default=None)

                # Trouver tous les IDs ayant la valeur maximale d'incrémentation
                id_max_b = [id for id, inner_dict in camera_below_dict.items() if max_increment_b in inner_dict]
            else:
                id_max_b = []

            if id_max_b != []:   
                t = camera_below_dict[id_max_b[0]][max_increment_b][2]

                if t1-self.dt_acceptable <= t <= t1+self.dt_acceptable:
                    camera_below_measurement = camera_below_dict
                else:
                    camera_below_measurement = 0  
            else:
                camera_below_measurement = 0

            #on defini les dt pour chaque cas
            if type(imu_measurement) != int and lidar_measurement != 0 and camera_below_measurement != 0 :
                dt = self.dt_imu_lidar_camera
                situation = 1
            elif type(imu_measurement) != int and lidar_measurement != 0 and camera_below_measurement == 0:
                dt = self.dt_imu_lidar 
                situation = 2
            elif type(imu_measurement) != int and lidar_measurement == 0 and camera_below_measurement != 0:  
                dt = self.dt_imu_camera
                situation = 3
            elif type(imu_measurement) == int and lidar_measurement != 0 and camera_below_measurement != 0:
                dt = self.dt_lidar_camera
                situation = 4
            elif type(imu_measurement) != int and lidar_measurement == 0 and camera_below_measurement == 0:
                dt = self.dt_imu
                situation = 5
            elif type(imu_measurement) == int and lidar_measurement != 0 and camera_below_measurement == 0:
                dt= self.dt_lidar
                situation = 6
            elif type(imu_measurement) == int and lidar_measurement == 0 and camera_below_measurement != 0:
                dt= self.dt_camera
                situation = 7
            else:
                situation = 0
                dt=self.dt
                camera_top_measurement = 0
            print("temps", dt)
            # print("avant fct state:",self.uav_state)
            
            if situation != 0:
                self.uav_state, F, F_markers = self.state_function(self.uav_state, dt)
                self.P_uav= F @ self.P_uav @ (F.transpose()) + self.Q  #at the begining, the covariance has to be estimated once 

            if type(imu_measurement) != int:

                h_imu, H_imu = self.imu_measurement_matrix(self.uav_state)
                print("mesure imu", imu_measurement)
                residual_imu =  imu_measurement.transpose() - h_imu.transpose()
                residual_imu[0,0] = self.residual_euler(imu_measurement[0,0], h_imu[0,0])
                residual_imu[1,0] = self.residual_euler(imu_measurement[0,1], h_imu[0,1])
                residual_imu[2,0] = self.residual_euler(imu_measurement[0,2], h_imu[0,2])
                print("residual imu", residual_imu)

                

                self.uav_state, self.P_uav = self.EKF_filter(H_imu, self.uav_state, F, self.Q, self.R_imu, self.P_uav, residual_imu)
                print("apres imu", self.uav_state)
            if type(lidar_measurement) != int:

                h_lidar, H_lidar = self.lidar_measurement_matrix(self.uav_state, self.offset_lidar)
                residual_lidar = lidar_measurement - h_lidar
                print("measurement lidar", lidar_measurement)
                print("residual lidar", residual_lidar)
                self.uav_state, self.P_uav = self.EKF_filter(H_lidar, self.uav_state, F, self.Q, self.R_lidar, self.P_uav, residual_lidar)  
                print("apres lidar", self.uav_state)


            if type(camera_below_measurement) != int:

                if type(camera_top_measurement) != int:
                    for value in set(id_max_b + id_max_top): 
                     prediction_covariance_markers = 1
                     self.P_markers_dict[value][1] = F_markers @ self.P_markers_dict[value][1] @ F_markers.transpose() + self.Q_markers  
                else:
                    for value in id_max_b: 
                        self.P_markers_dict[value][1] = F_markers @ self.P_markers_dict[value][1] @ F_markers.transpose() + self.Q_markers   


                # print("avantle slam",self.uav_state)
                print("camera dic avant le slam below", camera_below_measurement)
                self.uav_state, self.P_uav, self.state_markers, self.P_markers_dict = self.slam(id_max_b, max_increment_b, camera_below_measurement, self.uav_state, F, self.P_uav, self.Q, self.state_markers, self.P_markers_dict, F_markers, self.Q_markers, self.R_camera_below, self.offset_imu_camera_below, self.offset_angle_below )  
                print("apres cambelow", self.uav_state)


            if type(camera_top_measurement) != int: 

                if prediction_covariance_markers == 1:
                    prediction_covariance_markers = 0
                else:
                    for value in id_max_top: 
                        self.P_markers_dict[value][1] = F_markers @ self.P_markers_dict[value][1] @ F_markers.transpose() + self.Q_markers 

                Ppredicted_markers = F_markers @ self.P_markers_dict[value][1] @ F_markers.transpose() + self.Q_markers 
                
                self.uav_state, self.P_uav, self.state_markers, self.P_markers_dict = self.slam(id_max_top, max_increment_t, camera_top_measurement, self.uav_state, F, self.P_uav, self.Q, self.state_markers, self.P_markers_dict, F_markers, self.Q_markers, self.R_camera_top, self.offset_imu_camera_top, self.offset_angle_top )
                print("apres camtop", self.uav_state)
            #print("type camera", type(camera_below_measurement))   
                
            # Mettre à jour le temps pour la prochaine itération
            # self.uav_state = self.uav_state.transpose()
            self.true_uav_position = self.uav_state
            print("result",self.true_uav_position)
            if self.uav_state[0,0]>0.1 or self.uav_state[1,0]>0.1 or self.uav_state[2,0]>1 or np.absolute(self.uav_state[6,0])>1 or np.absolute(self.uav_state[7,0])>1 or np.absolute(self.uav_state[8,0])>1 :
                print("probem")
                time.sleep(30)
            self.plot_3d_curve(self.true_uav_position[0], self.true_uav_position[1], self.true_uav_position[2])
            # print("resultat",self.uav_state)

            imu_measurement = 0
            lidar_measurement = 0
            camera_top_measurement = 0
            camera_below_measurement = 0

            t2 = time.time()
            dt = t2-t1
            if situation==1:
                self.dt_imu_lidar_camera = dt
            elif situation ==2:
                self.dt_imu_lidar =dt
            elif situation ==3:
                self.dt_imu_camera = dt
            elif situation ==4:
                self.dt_lidar_camera = dt
            elif situation ==5:
                self.dt_imu = dt
            elif situation ==6:
                self.dt_lidar = dt
            elif situation ==7:
                self.dt_camera = dt
            
            # for threa in threading.enumerate():
            #     print(threa.name)
           #!/usr/bin/env python
                
    # #def print_task(self):
    #     while True:
    #         # print(self.true_uav_position)
    #         # time.sleep(1)
    #         pass

    def camera_top_task(self, get_camera_top_meas):
        #region setup
        # Dictionary that was used to generate the ArUco marker
        aruco_dictionary_name = "DICT_ARUCO_ORIGINAL"
        
        # The different ArUco dictionaries built into the OpenCV library. 
        ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }
        
        # Side length of the ArUco marker in meters 
        aruco_marker_side_length = 0.0785
        
        # Calibration parameters yaml file
        camera_calibration_parameters_filename = 'calibration_chessboard.yaml'
        
        def euler_from_quaternion(x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
                
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
                
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
                
            return roll_x, pitch_y, yaw_z # in radians
        

        # Check that we have a valid ArUco marker
        if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
            logging.info("ArUCo tag is not supported")
            

        # Load the camera parameters from the saved file
        cv_file = cv2.FileStorage(
            camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
        mtx = cv_file.getNode('K').mat()
        dst = cv_file.getNode('D').mat()
        cv_file.release()
        mtx = np.matrix([[5.682484410751633e+02,0,3.198526929019697e+02],          #aruco cam params
                   [0,5.689423158702836e+02,2.269844844918011e+02],
                   [0,0,1]])
        dst = np.matrix([[-0.234216845902543,0.052261480464537,0,0,-0.022931112954800]])


        # Load the ArUco dictionary
        logging.info("Detecting '{}' markers...".format(
            aruco_dictionary_name))
        # this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])
        # this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
        this_aruco_parameters =  cv2.aruco.DetectorParameters()
        
        # Start the video stream
        # cap = cv2.VideoCapture(0)
        cap = Picamera2(0)
        cap.start()
        sleep(2)
        n=0
        data_dict = {}
        measurement_dict = {}
        #endregion
        while not self.stop_measurement_thread:
        
            # Capture frame-by-frame
            # This method returns True/False as well
            # as the video frame.
            # ret, frame = cap.read() 
            # if not cap.isOpened():
            #  print("Error: Unable to open camera")
            # break
            # Capturer une image dans le flux
            # cap.capture_image("main")
            frame = cap.capture_array()
            t1 = time.time()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # display the image on screen and wait for a keypress
        

            # Décodez le tableau NumPy en une image OpenCV
            # frame = cv2.imdecode(array, 1)

            
            # Detect ArUco markers in the video frame
            (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
            frame, this_aruco_dictionary, parameters=this_aruco_parameters)
            n=n+1
            # Check that at least one ArUco marker was detected
            if marker_ids is not None:

                # Draw a square around detected markers in the video frame
                #cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
                
                # Get the rotation and translation vectors
                rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    aruco_marker_side_length,
                    mtx, 
                    dst)
                
                # Print the pose for the ArUco marker
                # The pose of the marker is with respect to the camera lens frame.
                # Imagine you are looking through the camera viewfinder, 
                # the camera lens frame's:
                # x-axis points to the right
                # y-axis points straight down towards your toes
                # z-axis points straight ahead away from your eye, out of the camera
                
                for i, marker_id in enumerate(marker_ids):
                
                    marker_id = marker_id[0]
                    if marker_id != 0:
                        # Store the translation (i.e. position) information
                        transform_translation_x = tvecs[i][0][0]
                        transform_translation_y = tvecs[i][0][1]
                        transform_translation_z = tvecs[i][0][2]

                        # Store the rotation information
                        rotation_matrix = np.eye(4)
                        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                        r = R.from_matrix(rotation_matrix[0:3, 0:3])
                        euler = r.as_euler('zxy', degrees=True)
                        quat = r.as_quat()   
                        
                        # Quaternion format     
                        transform_rotation_x = quat[0] 
                        transform_rotation_y = quat[1] 
                        transform_rotation_z = quat[2] 
                        transform_rotation_w = quat[3] 
                        
                        # Euler angle format in radians
                        roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
                                                                    transform_rotation_y, 
                                                                    transform_rotation_z, 
                                                                    transform_rotation_w)
                        
                        roll_x = math.degrees(roll_x)
                        pitch_y = math.degrees(pitch_y)
                        yaw_z = math.degrees(yaw_z)

                        camera_measurement = np.array([[transform_translation_x, transform_translation_y, transform_translation_z, roll_x, pitch_y, yaw_z]])
                        for_save = [t1, marker_id, n, camera_measurement]
                        if self.saving_data:
                            self.save_data.save_Camera_TOP(for_save)
                        new_row = [marker_id,  # ID
                            n,  # Numéro d'incrémentation
                            t1,  # Temps (simulé ici)
                            camera_measurement ] # Valeurs (simulées ici) converties en liste

                        # Extraire l'ID et le temps de la nouvelle ligne
                        current_id = new_row[0]
                        incrementation = new_row[1]
                        # Vérifier si l'ID existe déjà dans le dictionnaire
                        if current_id not in data_dict:
                            data_dict[current_id] = {}

                        # Ajouter la nouvelle ligne dans le dictionnaire interne correspondant à l'ID
                        data_dict[current_id][incrementation] = new_row

                        last_three_increments_exist = all(data_dict.get(marker_id, {}).get(n, None) is not None
                                                for n in range(n, n - 3, -1))

                        # Afficher le résultat
                        if last_three_increments_exist:
                            removed_data = measurement_dict.pop(marker_id, None)  
                            if current_id not in measurement_dict:
                                measurement_dict[current_id] = {}
                            measurement_dict[current_id][incrementation] = new_row
                        with self.lock:
                            self.get_camera_top_meas = measurement_dict

                        # print(marker_id)
                        # print("transform_translation_x: {}".format(transform_translation_x))
                        # print("transform_translation_y: {}".format(transform_translation_y))
                        # print("transform_translation_z: {}".format(transform_translation_z))
                        # print("roll_x: {}".format(roll_x))
                        # print("pitch_y: {}".format(pitch_y))
                        # print("yaw_z: {}".format(yaw_z))
                        # # print("roll_z: {}".format(euler[0]))
                        # # print("pitch_y: {}".format(euler[1]))
                        # # print("yaw_x: {}".format(euler[2]))
                        # print()
                        
                        # Draw the axes on the marker
                        #cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], aruco_marker_side_length, 1)
                
            # Display the resulting frame
            #cv2.imshow('frame',frame)
                
            # If "q" is pressed on the keyboard, 
            # exit this loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Close down the video stream
        cap.release()
        # cv2.destroyAllWindows()
            
        # if __name__ == '__main__':
        #   print(__doc__)
        #   main() 

    def camera_below_task(self, get_camera_below_meas):
        #region
        # Dictionary that was used to generate the ArUco marker
        aruco_dictionary_name = "DICT_ARUCO_ORIGINAL"

        # The different ArUco dictionaries built into the OpenCV library. 
        ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }
        
        # Side length of the ArUco marker in meters 
        aruco_marker_side_length = 0.0785
        
        # Calibration parameters yaml file
        camera_calibration_parameters_filename = 'calibration_chessboard.yaml'
        
        def euler_from_quaternion(x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
                
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
                
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
                
            return roll_x, pitch_y, yaw_z # in radians
        
        #def main():
        """
        Main method of the program.
        """
        if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
            logging.info("[INFO] ArUCo tag is not supported")
        # Load the camera parameters from the saved file
        cv_file = cv2.FileStorage(
            camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
        mtx = cv_file.getNode('K').mat()
        dst = cv_file.getNode('D').mat()
        cv_file.release()
        mtx = np.matrix([[5.706055532684599e+02,0,3.282198582514894e+02],     #camera noir wide params
        [0,5.690321322616674e+02,2.450640199750066e+02],
        [0,0,1]])
        dst = np.matrix([[-0.199154552455924,-0.111666325169893,0,0,0.198132380405577]])

        # print(mtx)
        # print(dst)   
        # Load the ArUco dictionary
        logging.info("Detecting '{}' markers...".format(aruco_dictionary_name))
        # this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])
        # this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
        this_aruco_parameters =  cv2.aruco.DetectorParameters()
        
        # Start the video stream
        # cap = cv2.VideoCapture(0)
        cap = Picamera2(1)
        cap.start()
        sleep(2)
        n=0
        data_dict = {}
        measurement_dict = {}
        #endregion
        while not self.stop_measurement_thread:
            # Capture frame-by-frame
            # This method returns True/False as well
            # as the video frame.
            # ret, frame = cap.read() 
            # if not cap.isOpened():
            #  print("Error: Unable to open camera")
            # break
            # Capturer une image dans le flux
            # cap.capture_image("main")
            frame = cap.capture_array()
            t1 = time.time()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # display the image on screen and wait for a keypress
        

            # Décodez le tableau NumPy en une image OpenCV
            # frame = cv2.imdecode(array, 1)

            
            # Detect ArUco markers in the video frame
            (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
            frame, this_aruco_dictionary, parameters=this_aruco_parameters)
            n=n+1
            # Check that at least one ArUco marker was detected
            if marker_ids is not None:

                # Draw a square around detected markers in the video frame
                #cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
                
                # Get the rotation and translation vectors
                rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    aruco_marker_side_length,
                    mtx, 
                    dst)
                    
                # Print the pose for the ArUco marker
                # The pose of the marker is with respect to the camera lens frame.
                # Imagine you are looking through the camera viewfinder, 
                # the camera lens frame's:
                # x-axis points to the right
                # y-axis points straight down towards your toes
                # z-axis points straight ahead away from your eye, out of the camera
                
                for i, marker_id in enumerate(marker_ids):
                    marker_id = marker_id[0]
                    if marker_id != 0:
                        # Store the translation (i.e. position) information
                        transform_translation_x = tvecs[i][0][0]
                        transform_translation_y = tvecs[i][0][1]
                        transform_translation_z = tvecs[i][0][2]

                        # Store the rotation information
                        rotation_matrix = np.eye(4)
                        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                        r = R.from_matrix(rotation_matrix[0:3, 0:3])
                        euler = r.as_euler('zxy', degrees=True)
                        quat = r.as_quat()   
                        
                        # Quaternion format     
                        transform_rotation_x = quat[0] 
                        transform_rotation_y = quat[1] 
                        transform_rotation_z = quat[2] 
                        transform_rotation_w = quat[3] 
                        
                        # Euler angle format in radians
                        roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
                                                                    transform_rotation_y, 
                                                                    transform_rotation_z, 
                                                                    transform_rotation_w)
                        
                        roll_x = math.degrees(roll_x)
                        pitch_y = math.degrees(pitch_y)
                        yaw_z = math.degrees(yaw_z)

                        camera_measurement = np.array([[transform_translation_x, transform_translation_y, transform_translation_z, roll_x, pitch_y, yaw_z]])
                        
                        for_save = [t1, marker_id, n, camera_measurement]
                        if self.saving_data:
                            self.save_data.save_Camera_BOT(for_save)

                        new_row = [marker_id,  # ID
                            n,  # Numéro d'incrémentation
                            t1,  # Temps (simulé ici)
                            camera_measurement ] # Valeurs (simulées ici) converties en liste

                        # Extraire l'ID et le temps de la nouvelle ligne
                        current_id = new_row[0]
                        incrementation = new_row[1]

                        # Vérifier si l'ID existe déjà dans le dictionnaire
                        if current_id not in data_dict:
                            data_dict[current_id] = {}

                        # Ajouter la nouvelle ligne dans le dictionnaire interne correspondant à l'ID
                        data_dict[current_id][incrementation] = new_row

                        last_three_increments_exist = all(data_dict.get(marker_id, {}).get(n, None) is not None
                                                for n in range(n, n - 3, -1))

                        # Afficher le résultat
                        
                        if last_three_increments_exist:
                            removed_data = measurement_dict.pop(marker_id, None)  
                            if current_id not in measurement_dict:
                                measurement_dict[current_id] = {}
                            measurement_dict[current_id][incrementation]= new_row
                        # print("lock camera", self.lock)
                        # print("patryck 2", self.lock.locked())
                        if self.lock.locked() != True:
                            # print("patryk ")
                            self.get_camera_below_meas = measurement_dict
                    # print("dic from cam", measurement_dict)

                    # print(marker_id)
                    # print("transform_translation_x: {}".format(transform_translation_x))
                    # print("transform_translation_y: {}".format(transform_translation_y))
                    # print("transform_translation_z: {}".format(transform_translation_z))
                    # print("roll_x: {}".format(roll_x))
                    # print("pitch_y: {}".format(pitch_y))
                    # print("yaw_z: {}".format(yaw_z))
                    # # print("roll_z: {}".format(euler[0]))
                    # # print("pitch_y: {}".format(euler[1]))
                    # # print("yaw_x: {}".format(euler[2]))
                    # print()
                    
                    # Draw the axes on the marker
                    #cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], aruco_marker_side_length, 1)
                
            # Display the resulting frame
            #cv2.imshow('frame',frame)
                
            # If "q" is pressed on the keyboard, 
            # exit this loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Close down the video stream
        cap.release()
        cv2.destroyAllWindows()
            
        # if __name__ == '__main__':
        #   print(__doc__)
        #   main()
                    

        # Exécution des threads en arrière-plan
    
    def imu_task(self, get_imu_meas, target_angle_offset):
        #region
        i2c = board.I2C()  # uses board.SCL and board.SDA  attention c'est celui la que j'ai commente
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
        sensor = adafruit_bno055.BNO055_I2C(i2c)

        # If you are going to use UART uncomment these lines
        # uart = board.UART()
        # sensor = adafruit_bno055.BNO055_UART(uart)

        


        # def temperature():
        #     global last_val  # pylint: disable=global-statement
        #     result = sensor.temperature
        #     if abs(result - last_val) == 128:
        #         result = sensor.temperature
        #         if abs(result - last_val) == 128:
        #             return 0b00111111 & result
        #     last_val = result
        #     return result

        #endregion
        while not self.stop_measurement_thread:
            t = time.time()
            sensor_euler = sensor.euler
            angular_rate =sensor.gyro
            acceleration = sensor.linear_acceleration
            # Adjust the Euler angle values with the target_angle_offset
            
            heading, roll, pitch = [position - target_angle_offset[idx] for idx, position in enumerate(sensor_euler)]
            euler = np.array([[heading, roll, pitch]])
            if (
                    euler[0][0] is not None and
                    euler[0][1] is not None and
                    euler[0][2] is not None and
                    angular_rate[0] is not None and
                    angular_rate[1] is not None and
                    angular_rate[2] is not None and
                    acceleration[0] is not None and
                    acceleration[1] is not None and
                    acceleration[2] is not None 
                ):
                    with self.lock:
                        if euler[0][0]>180:
                            euler[0][0] = euler[0][0] - 360
                        self.get_imu_meas = [
                                        [t],
                                        [euler[0][0], euler[0][1], euler[0][2]],
                                        [angular_rate[0], angular_rate[1], angular_rate[2]],
                                        [acceleration[0], acceleration[1], acceleration[2]]
                                        ]
                    self.save_get_imu_meas = [
                                    t,
                                    [euler[0][0], euler[0][1], euler[0][2]],
                                    [angular_rate[0], angular_rate[1], angular_rate[2]],
                                    [acceleration[0], acceleration[1], acceleration[2]]
                                    ]
                    if self.saving_data:
                        self.save_data.save_imu(self.save_get_imu_meas)
            # shared_data["imu_measurement"] = imu_data
            
    def lidar_task(self,get_lidar_meas):
        # print("lidar 11")
        GPIO.setmode(GPIO.BCM)
        
        # print "+-----------------------------------------------------------+"
        # print "|   Mesure de distance par le capteur ultrasonore HC-SR04   |"
        # print "+-----------------------------------------------------------+"

        Trig = 23          # Entree Trig du HC-SR04 branchee au GPIO 23
        Echo = 24         # Sortie Echo du HC-SR04 branchee au GPIO 24

        GPIO.setup(Trig,GPIO.OUT)
        GPIO.setup(Echo,GPIO.IN)
        
        GPIO.output(Trig, False)
        first_time = True
        time.sleep(5)
        while not self.stop_measurement_thread:
            # print("lidar 22")
            GPIO.output(Trig, True)
            time.sleep(0.00001)
            GPIO.output(Trig, False)
            p=0
            pp=0
            myStartingTime =time.time()
            while GPIO.input(Echo)==0:  ## Emission de l'ultrason
                calculation_starting_time = time.time()
                p=0
                if calculation_starting_time - myStartingTime > 1:
                    p=99
                    break
                start = time.time()

            if p != 99:

                while GPIO.input(Echo)==1:   ## Retour de l'Echo
                    calculation_starting_time = time.time()
                    pp=0
                    if calculation_starting_time - myStartingTime > 1:
                        pp=99
                        break
                    
                    end = time.time()

                if pp !=99:
                    if first_time:
                        time.sleep(0.2)
                        first_time = False
                   
                    distance = round((end - start) * 340 * 100 / 2, 1)  ## Vitesse du son = 340 m/s

                    # print("lidar 33")
                    t1 = time.time()
                    with self.lock:
                        self.get_lidar_meas = np.array([
                                                    [distance/100],
                                                    [t1]
                                                    ])
                    if self.saving_data:
                        self.save_data.save_Lidar(self.get_lidar_meas)
            
              
            #print ("La distance est de : ",distance," cm, mesure:",x)
        
        GPIO.cleanup()








