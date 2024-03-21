import numpy as np
import os
import matplotlib.pyplot as plt
import logging

class save_tuning_data:
    
    def __init__(self) -> None:
        self.Qualisys = np.array([],dtype=[(save_tuning_data.TIME,float),(save_tuning_data.DATA,float,(12,))])
        self.Lidar = np.array([])
        self.Camera_TOP = np.array([],dtype=[(save_tuning_data.TIME,float),(save_tuning_data.ID,float),(save_tuning_data.N,float),(save_tuning_data.DATA,float,(6))])
        self.Camera_BOT = np.array([],dtype=[(save_tuning_data.TIME,float),(save_tuning_data.ID,float),(save_tuning_data.N,float),(save_tuning_data.DATA,float,(6))])
        self.imu = np.array([],dtype=[(save_tuning_data.TIME,float),(save_tuning_data.EULER,float,(3)),(save_tuning_data.ANGULAR_RATE,float,(3)),(save_tuning_data.ACCELERATION,float,(3))])

        self.uav_state = []

        self.all_data = np.array([self.Qualisys,self.Lidar,self.Camera_TOP,self.Camera_BOT,self.imu])
    
    QUALISYS = 0
    LIDAR = 1
    CAMERA = 2
    IMU = 3

    TIME = 'time'
    DATA = 'data'
    ID = 'id'
    N = 'n'
    EULER = 'euler'
    ANGULAR_RATE = 'angular_rate'
    ACCELERATION = 'acceleration'

    #region Save
    def save_Qualisys(self,Qualisys = [0,0,0],angle =[0,0,0,0,0,0,0,0,0], time : float = 0):
        '''
        [position,rotation_matrix] et time
        '''
        try:
            temp = np.append(Qualisys,angle)
            self.Qualisys = np.append(self.Qualisys,np.array((time,temp),dtype=[(save_tuning_data.TIME,float),(save_tuning_data.DATA,float,(12,))]))
        except:
            print("grrr")

    def save_Lidar(self,Lidar_Time): 
        '''
        [valeur,temps] 
        '''
        self.Lidar = np.append(self.Lidar,(Lidar_Time))

    def save_Camera_TOP(self,Camera):
        '''
        [T, id, n, [x, y, z, yaw, pitch, roll]]
        '''
        self.Camera_TOP = np.append(self.Camera_TOP,np.array(tuple(Camera),dtype=[(save_tuning_data.TIME,float),(save_tuning_data.ID,float),(save_tuning_data.N,float),(save_tuning_data.DATA,float,(6))]))

    def save_Camera_BOT(self,Camera):
        '''
        [T, id, n, [x, y, z, yaw, pitch, roll]]
        '''
        self.Camera_BOT = np.append(self.Camera_BOT,np.array(tuple(Camera),dtype=[(save_tuning_data.TIME,float),(save_tuning_data.ID,float),(save_tuning_data.N,float),(save_tuning_data.DATA,float,(6))]))
    
    def save_imu(self, imu):
        '''
        [temps,[psi, theta, phi],[psi_dot, theta_dot, phi_dot],[acc_x, acc_y, acc_z]] 
        '''
        self.imu = np.append(self.imu,np.array(tuple(imu),dtype=[(save_tuning_data.TIME,float),(save_tuning_data.EULER,float,(3)),(save_tuning_data.ANGULAR_RATE,float,(3)),(save_tuning_data.ACCELERATION,float,(3))]))
    # endregion
        
    def save_Qualisys_to_file(self):
        try:
            print(self.Qualisys)
            np.save(r"\Qualisys_data",self.Qualisys)
            print(np.load(r"\Qualisys_data.npy"))
            print("\033[93msaved")
        except:
            print("Couldn't save Qualisys")
            logging("Couldn't save Qualisys")

    def save_all(self, folder_path : str = r"\Rocket_Software\saved_measurement"): 
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        try:
            np.save(folder_path + r"\Lidar_data.csv",self.Lidar)
        except:
            print("Couldn't save Lidar")
            logging("Couldn't save Lidar")
        try:
            np.save(folder_path + r"\Camera_data_top.csv",self.Camera_TOP)
        except:
            print("Couldn't save Camera top")
            logging("Couldn't save Camera top")
        try:
            np.save(folder_path + r"\Camera_data_bot.csv",self.Camera_BOT)
        except:
            print("Couldn't save Camera bot")
            logging("Couldn't save Camera bot")
        try:
            np.save(folder_path + r"\imu_data",self.imu)
        except:
            print("Couldn't save imu")
            logging("Couldn't save imu")
        

    def load_all(self, folder_path):
            self.Qualisys = np.load(folder_path + r"\Qualisys_data.csv")
            self.Camera = np.load(folder_path + r"\Camera_data_top.csv")
            self.Camera = np.load(folder_path + r"\Camera_data_bot.csv")
            self.Lidar = np.load(folder_path + r"\Lidar_data.csv")
            self.imu = np.load(folder_path + r"\imu_data.csv")
    
    def save_state_UAV(self,uav_state):
        np.append([self.uav_state],[uav_state],axis=0)

    def __str__(self,uav_state = None) -> str:
        if uav_state != None:
            plt.figure(1)
            plt.plot(self.Qualisys[:,0],label="Qualisys")
            plt.plot(uav_state[:,0])
            plt.xlabel("time")
            plt.ylabel("x(m)")
            plt.title("x position of Qualisys and the state")
            plt.legend()
            
            plt.figure(2)
            plt.plot(self.Qualisys[:,1],label="Qualisys")
            plt.plot(uav_state[:,1],label="state")
            plt.xlabel("time")
            plt.ylabel("y(m)")
            plt.title("y position of Qualisys and the state")
            plt.legend()
            
            plt.figure(3)
            plt.plot(self.Qualisys[:,2],label="Qualisys")
            plt.plot(uav_state[:,2],label="state")
            plt.xlabel("time")
            plt.ylabel("z(m)")
            plt.title("z position of Qualisys and the state")
            plt.legend()

            #psi theta phi
            plt.figure(4)
            plt.plot(self.Qualisys[:,3],label="Qualisys")
            plt.plot(uav_state[:,6],label="state")
            plt.xlabel("time")
            plt.ylabel("psi(deg)")
            plt.title("Psi position of Qualisys and the state")
            plt.legend()
            
            plt.figure(5)
            plt.plot(self.Qualisys[:,4],label="Qualisys")
            plt.plot(uav_state[:,7],label="state")
            plt.xlabel("time")
            plt.ylabel("theta(deg)")
            plt.title("Theta position of Qualisys and the state")
            plt.legend()
            
            plt.figure(6)
            plt.plot(self.Qualisys[:,5],label="Qualisys")
            plt.plot(uav_state[:,8],label="state")
            plt.xlabel("time")
            plt.ylabel("phi(deg)")
            plt.title("Phi position of Qualisys and the state")
            plt.legend()
            plt.show()

            plt.figure(7)
            plt.axes(projection='3d').plot3D(self.Qualisys[:,0],self.Qualisys[:,1],self.Qualisys[:,2],'green')
            plt.axes.set_title("Position from Qualisys and State")
            plt.axes(projection='3d').plot3D(self.uav_state[:,0],self.uav_state[:,1],self.uav_state[:,2],'blue')
        return("Qualisys: "+ str(self.Qualisys)+"\nLidar: "+str(self.Lidar)+"\nCamera top: "+str(self.Camera_TOP)+"\nCamera bot: "+str(self.Camera_BOT)+"\nImu : "+str(self.imu))

if __name__ == "__main__":
    save = save_tuning_data()
    save.save_Qualisys(np.array([1,3,1]),np.array([3,4,6,4,2,1,9,8,3]),0)
    save.save_Lidar(np.array([2,2]))
    save.save_Camera_TOP([3,3,4,[3,2,3,2,3,2]])
    save.save_Camera_BOT([3,3,4,[3,2,3,2,3,2]])
    save.save_imu([0,[1,4.1,4.1],[4.2,4.2,4.2],[4.3,4.3,4.3]])
    print(save)
    save.save_all()