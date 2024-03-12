import numpy as np
import os

class save_tuning_data:
    
    def __init__(self) -> None:
        self.Qualisys = np.array([],dtype=[(save_tuning_data.TIME,float),(save_tuning_data.DATA,float,(1,3))])
        self.Lidar = np.array([],dtype=[(save_tuning_data.TIME,float),(save_tuning_data.DATA,float,(1,2))])
        self.Camera = np.array([],dtype=[(save_tuning_data.TIME,float),(save_tuning_data.DATA,float,(2,3))])
        self.imu = np.array([],dtype=[(save_tuning_data.TIME,float),(save_tuning_data.DATA,float,(3,3))])

        self.all_data = np.array([self.Qualisys,self.Lidar,self.Camera,self.imu])
    
    QUALISYS = 0
    LIDAR = 1
    CAMERA = 2
    IMU = 3

    TIME = 'time'
    DATA = 'data'


    def save_Qualisys(self,Qualisys = [0,0,0], time : float = 0):
        self.Qualisys = np.append(self.Qualisys,np.array((time,Qualisys),dtype=[(save_tuning_data.TIME,float),(save_tuning_data.DATA,float,(1,3))]))

    def save_Lidar(self,Lidar = [0,0], time = 0):
        self.Lidar = np.append(self.Lidar,np.array((time,Lidar),dtype=[(save_tuning_data.TIME,float),(save_tuning_data.DATA,float,(1,2))]))

    def save_Camera(self,Camera = [[0,0,0],[0,0,0]], time = 0):
        self.Camera = np.append(self.Camera,np.array((time,Camera),dtype=[(save_tuning_data.TIME,float),(save_tuning_data.DATA,float,(2,3))]))

    def save_imu(self, imu = [[0,0,0],[0,0,0],[0,0,0]], time = 0):
        self.imu = np.append(self.imu,np.array((time,imu),dtype=[(save_tuning_data.TIME,float),(save_tuning_data.DATA,float,(3,3))]))

    def save_all(self, folder_path : str = "Rocket_Software\saved_measurement"): 
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        np.save(folder_path + "\Qualisys_data.csv",self.Qualisys)
        np.save(folder_path + "\Camera_data.csv",self.Camera)
        np.save(folder_path + "\Lidar_data.csv",self.Lidar)
        np.save(folder_path + "\imu_data.csv",self.imu)

    def load_all(self, folder_path):
            self.Qualisys = np.load(folder_path + "\Qualisys_data.csv",allow_pickle=True)
            self.Camera = np.load(folder_path + "\Camera_data.csv",allow_pickle=True)
            self.Lidar = np.load(folder_path + "\Lidar_data.csv",allow_pickle=True)
            self.imu = np.load(folder_path + "\imu_data.csv",allow_pickle=True)


    def __str__(self) -> str:
        return str(np.array([self.Qualisys,self.Lidar,self.Camera,self.imu]))

if __name__ == "__main__":
    save = save_tuning_data()
    save.save_Qualisys([1,1,1],0)
    save.save_Lidar([2,2],0)
    save.save_Camera([[3,3,3],[3.2,3.2,3.2]])
    save.save_imu([[4.1,4.1,4.1],[4.2,4.2,4.2],[4.3,4.3,4.3]])
    print(save)
    save.save_all()