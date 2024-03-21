import asyncio
import qtm
import socket
import threading
import logging
import time
import board
import adafruit_bno055
import numpy as np
from ESP32_Com import ESP32
import Sensor_fusion as Sensors
import Measurement_save as save


HOST, PORT = '0.0.0.0', 65000

class Server:
    
    __default_message_size = 32

    INFO, DEBUG, ERROR, WARNING, CRITICAL, GREEN = 0,10,20,30,40,50
    
    def __init__(self,host :str = '0.0.0.0',port : int = 65000 ,new_message_size = __default_message_size):
        '''
        Initialize the server with the define host and port, and a fixed size message
        '''
        # Create the log file
        logging.basicConfig(filename="GDP_retrun_server.log", level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%d-%b-%y %H:%M:%S',force=True)   

        # Create a TCP socket
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)     
        self.__server_address = (host, port)
        self.__message_size = new_message_size
              
        # Starting TCP server on {host} port {port}
        self.__sock.bind(self.__server_address)
        
        # Server settings
        self.isServerrunning = True
        self.__server_open_to_connect = True
        self.isServeropen = False
        self.max_clients = 4

        # Boolean of the clients
        self.ground_station : socket.socket = None
        self.isGround_station_connected = False
        self.print_all_received_data = True

        # QTM
        self.__connection_password = "gdp-return"
        self.__disconnect_QTM = asyncio.Event()
        self.__isQTM_connected = False
        self.__qtm_IP = "138.250.154.168"
        self.__tolerance = 0.1
        self.__qtm_position = []
        self.__qtm_rotation = []
        self.stop_save_qtm = False


        # ESP32
        try:
            self.ESP32 : ESP32 = ESP32()
            self.log_and_print("ESP32 connected to the raspberry",Server.ERROR)
        except:
            self.log_and_print("ESP32 not connected to the raspberry",Server.ERROR)
        self.ESP32_data = "data"

        # Path
        self.Emergency_landing = False
        
        # Sensors
        self.save_data = save.save_tuning_data()
        self.euler_offset = 0
        self.calibration = False
        self.calibration_done = False
        self.euler_angle = 0
        self.stop_sensor_fusion_thread = False
        self.sensor_offset : str
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.sensor_offset_done = False
        self.sensor_yaw_offset = False
        self.sensor_roll_offset = False
        self.sensor_pitch_offset = False
        self.sensor_offset_satisfied = False
        self.sensors_offset_done = False
        self.sensor_satisfied = False
        self.input_sensors_roll_set = False
        self.input_sensors_pitch_set = False
        self.input_sensors_yaw_set = False
        self.target_angle_offset = (0,0,0)

        self.start_server()

    # region Init Server
    def start_server(self):
        '''
        Start the different thread and functions we need
        '''
        self.Server_command_thread = threading.Thread(target=self.Server_command)
        self.Server_command_thread.daemon = True
        self.Server_command_thread.start()
        self.open_server()
        #self.Start_sensors()
        self.imu_calibration()
        self.Handle_server_activity()
        
        #self.start_QTM()
          
    def open_server(self):
        '''
        Manage the connection of the clients
        '''
        try:
            self.log_and_print(f"Opening the local server on port: {HOST}")
            self.__sock.listen(self.max_clients)
            self.log_and_print("Server open",Server.GREEN)
            self.isServeropen = True
        

            connected_client, address = self.__sock.accept()

            # Set the IP, port and the name
            self.IP = address[0]
            self.port = address[1]
            self.name = "Ground_station"
            self.log_and_print(f"Ground station connected with ip: {self.IP} on port: {self.port}",Server.INFO)
            self.ground_station = connected_client
            
            # Message to send
            self.msg_to_send : str = ""

            # Activity of the client
            self.isGround_station_connected = True

            # Thread bool
            self.__needs_to_stop_sending_gs = False
            self.__needs_to_stop_receiving_gs = False
            self.print_received_data = True
            
            # Strat sending/receiving
            self.start_receiving()
            self.start_sending()


        except Exception as ex:
            self.log_and_print(ex)
            self.log_and_print("Could not connect",Server.WARNING)
            self.open_server()
    # endregion
            
    # region Ground Station
    # region Sending message
    def start_sending(self):
        '''
        Initiate the sending message thread
        '''
        self.__needs_to_stop_sending_gs = False
        self.sending_thread = threading.Thread(target = self.send_message)
        self.sending_thread.start()
        self.log_and_print(f"Starting sending messages to {self.name}",Server.GREEN)

    def stop_sending(self):
        '''
        Stop the sending thread
        '''
        self.__needs_to_stop_sending_gs = True
        self.sending_thread.join()
        self.log_and_print(f"Stopped sending messages to {self.name}",Server.INFO)

    def send_message(self):
        '''
        Send message to the connected client
        '''
        try:
            while (not self.__needs_to_stop_sending_gs) and self.isServerrunning:            
                if self.msg_to_send != "":
                    self.ground_station.sendall(str(self.msg_to_send).encode())

        except:
                if self.__needs_to_stop_sending_gs and not self.isServerrunning:
                    self.log_and_print("Stopped sending",Server.INFO)
                elif self.isGround_station_connected:
                    self.log_and_print(f"Message couldn't be sent to {self.name}. Restarting sending...",Server.WARNING)
                    self.stop_sending()
                    self.start_sending()
                else:
                    self.log_and_print(f"Client {self.name} is not connected",Server.WARNING)
    #endregion
    
    # region Receiveing message
    def start_receiving(self):
        '''
        Initiate the sending message thread
        '''
        self.__needs_to_stop_receiving_gs = False
        self.receiving_thread = threading.Thread(target = self.receive_message)
        self.receiving_thread.start()
        self.log_and_print(f"Starting receiving messages from {self.name}",Server.GREEN)

    def stop_receiving(self):
        '''
        Stop the sending thread
        '''
        self.__needs_to_stop_receiving_gs = True
        self.receiving_thread.join()
        self.log_and_print(f"Stopped receiving messages from {self.name}",Server.INFO)   

    def receive_message(self):
        '''
        Receive the message and print it (or not print_received_data) and execute the msg received
        '''
        try:
            while (not self.__needs_to_stop_receiving_gs) and self.isServerrunning:
                #Check if the previous received message has been executed
                data = self.ground_station.recv(self.__message_size).decode()
                if self.isGround_station_connected:
                    self.execute_message(data)
        except Exception as ex:
            if not self.__needs_to_stop_receiving_gs:
                print(ex)
                self.log_and_print(f"Error in receiving a message from {self.name}",Server.WARNING)
                self.shutdown()

    def execute_message(self,msg : str):
        '''
        Execute the incoming message 
        '''
        match msg.split("==")[0]:

            case "start -ESP32":
                self.Start_ESP32()

            case "stop -ESP32":
                self.Stop_ESP32()

            case "stop -com":
                self.stop_sending()

            case "start -com":
                self.start_sending()

            case "exit" | "disconnect" | "shutdown":
                self.shutdown()

            case "ping":
                self.ground_station.sendall(str("ping").encode())

            case "start -QTM":
                self.start_QTM()
            
            case "stop -QTM":
                self.stop_QTM()

            case "start -Sensors":
                self.Start_sensors()
            case "stop -Sensors":
                self.Stop_sensors()
            case "input -sensors_offset":
                if msg.split("==")[1] == 'y':
                    self.sensor_offset_done = True   
                    self.sensor_offset = 'y'
                else:
                    self.sensor_offset = 'n'
                    self.sensor_offset_done = True

            case "sensor -Off_P":
                self.sensor_pitch_offset = float(msg.split("==")[1])
                self.input_sensors_pitch_set = True
            case "sensor -Off_R":
                self.sensor_roll_offset = float(msg.split("==")[1])
                self.input_sensors_roll_set = True
            case "sensor -Off_Y":
                self.sensor_yaw_offset = float(msg.split("==")[1])
                self.input_sensors_yaw_set = True
            
            case "input -sensors_satisfied":
                self.sensor_satisfied = True
                self.sensor_offset_satisfied = True
            case "input -sensors_not_satisfied":
                self.sensor_satisfied = True
            # example of msg to send: "euler -offset==1,-2,3"
            case "euler -offset":
                self.euler_offset = np.array([float(i) for i in msg.split("==")[1].split(",")])
                self.euler_angle += self.euler_offset
                self.ground_station.sendall(str(self.euler_angle).encode())

            case "control -calibration_done":
                self.calibration_done = True

            case "control -calibration":
                self.ground_station.sendall(str(self.euler_angle).encode())
            case "save -all":
                self.ground_station.sendall(str("save").encode())
                self.Stop_sensors()
                self.stop_save_qtm = True
                time.sleep(1)
                self.save_data.save_Qualisys_to_file()
                self.log_and_print("Saving the data")
                self.sensors.save_data_to_file()
            case "active thread":
                for thread in threading.enumerate():
                    print(thread)
            case _:
                if self.print_received_data and self.isGround_station_connected:
                    self.log_and_print(f"Message received from {self.name} : " + msg)

    # endregion
    # endregion

    # region Server Main thread here
    def Server_command(self):
        while self.isServerrunning:
            command = input("__________________________________\n\033[92mgdp-return: \033[00m")
            match command:
                case "shutdown":
                    self.shutdown()

    #### Main thread ####
    def Handle_server_activity(self):
        while True:
            if not self.isServeropen:
                self.log_and_print("Shutting down the server",Server.WARNING)
                if self.isGround_station_connected:
                    self.stop_sending()
                    self.stop_receiving()
                if self.__isQTM_connected:
                    self.stop_QTM()
                else:
                    socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((HOST,PORT))
                self.__sock.shutdown(socket.SHUT_RDWR)
                self.__sock.close()
                self.isServerrunning = False
                self.isGround_station_connected = False        
                self.isServerrunning = False    
                exit()
            time.sleep(2)
    
    def shutdown(self):
        '''
        Shutdwon the server
        '''
        try:
            self.isServeropen = False
            self.__needs_to_stop_receiving_gs = True
            # Disconnecting all the clients
            # if self.isGround_station_connected:
            #     self.ground_station.sendall(str("shutdown").encode())
            if not self.isGround_station_connected:
                self.Handle_server_activity()
        except:
            if not self.isGround_station_connected and not self.isServerrunning:
                exit()
            elif self.isGround_station_connected:
                self.log_and_print("Disconnecting client failed",Server.ERROR)
                again = input("Try again if not the server will force shutdown ? (Y/n)")
                while again.lower() not in ('y','n'):
                    again = input("Please only answer by 'Y' or 'n' only: ")
                if again.lower() == 'y':
                    exit()
                else:
                    self.shutdown()
            else:
                exit()
    # endregion

    # region Getting the data from qtm
                
    def start_QTM(self):
        self.log_and_print("Starting QTM connection")
        self.__disconnect_QTM.clear()
        self.qtm_thread = threading.Thread(target=self.Async_start_QTM,args=[self.__qtm_position])
        self.qtm_thread.start()

    def stop_QTM(self):
        self.__disconnect_QTM.set()
        self.qtm_thread.join()

    def Async_start_QTM(self,position):
        asyncio.run(self.get_QTM_data(position)) 

    async def get_QTM_data(self,qtm_position):
        # Connect to the QTM 
        connection = await qtm.connect(self.__qtm_IP)
        if connection is None:
            return -1
        else:
            self.__isQTM_connected = True
        print("Connected")
        # Take control of the QTM system
        async with qtm.TakeControl(connection, self.__connection_password):
            state = await connection.get_state()
            if state != qtm.QRTEvent.EventConnected:
                await connection.new()
                try:
                    await connection.await_event(qtm.QRTEvent.EventConnected, timeout=10)
                except asyncio.TimeoutError:
                    return -1
                
            # Start the measurement
            await connection.start()
            await connection.await_event(qtm.QRTEvent.EventCaptureStarted, timeout=10)

            def on_packet(packet):
                pos = []
                rot = []
                info, bodies = packet.get_6d()
                pos = np.array([bodies[0][0].x,bodies[0][0].y,bodies[0][0].z])
                rot = np.array(bodies[0][1].matrix)
                self.__qtm_position, self.__qtm_rotation = pos,rot
                if not self.stop_save_qtm:
                    self.save_data.save_Qualisys(self.__qtm_position,self.__qtm_rotation,time.time())
                


            await connection.stream_frames(components=["6d"], on_packet=on_packet)
            await self.__disconnect_QTM.wait()
            await connection.stream_frames_stop()
            # Stop the measurement
            await connection.stop()
            await connection.await_event(qtm.QRTEvent.EventCaptureStopped, timeout=10)
        connection.disconnect()
    # endregion
    

    # region Control
    def position_offset(self, position, offset_x : float = 0, offset_y : float = 0, offset_z : float = 0):
        new_position = np.array([position[0] + offset_x, position[1] + offset_y, position[2] + offset_z])
        return new_position
    
    def check_position(self, pos, tolerance = 0.1, x_min = 0, x_max = 100, y_min = 0, y_max = 100, z_min = 0, z_max = 100):
        min_values = [x_min,y_min,z_min]
        max_values = [x_max,y_max,z_max]
        for i in range(len(min_values)):
            if pos[i] < (1+tolerance) * min_values[i] or pos > (1-tolerance)*max_values[i]:
                self.Emergency_landing = True
    
    def update_position(self, position, rotation, offset_x : float = 0, offset_y : float = 0, offset_z : float = 0,tolerance = 0.1, x_min = 0, x_max = 100, y_min = 0, y_max = 100, z_min = 0, z_max = 100):
        true_position = self.position_offset(position)
        self.check_position(true_position)
        if self.Emergency_landing:
            pass
        return true_position, rotation

    # endregion

    # region Sensors
    def Start_sensors(self):
        self.log_and_print("Starting the sensor fusion",Server.GREEN)
        self.sensors = Sensors.sensor_fusion(self.target_angle_offset)
        self.sensors.Start_measurement()
        self.sensor_fusion_thread = threading.Thread(target=self.sensors.main_task)
        time.sleep(5)
        self.sensor_fusion_thread.start()
        self.log_and_print("Started",Server.GREEN)
    
    def Stop_sensors(self):
        self.sensors.Stop_measurement() 
        self.sensor_fusion_thread.join()
    
    def imu_calibration(self):
        # pylint: disable=too-few-public-methods
        class Mode:
            CONFIG_MODE = 0x00
            ACCONLY_MODE = 0x01
            MAGONLY_MODE = 0x02
            GYRONLY_MODE = 0x03
            ACCMAG_MODE = 0x04
            ACCGYRO_MODE = 0x05
            MAGGYRO_MODE = 0x06
            AMG_MODE = 0x07
            IMUPLUS_MODE = 0x08
            COMPASS_MODE = 0x09
            M4G_MODE = 0x0A
            NDOF_FMC_OFF_MODE = 0x0B
            NDOF_MODE = 0x0C


        # Uncomment these lines for UART interface connection
        # uart = board.UART()
        # sensor = adafruit_bno055.BNO055_UART(uart)

        # Instantiate I2C interface connection
        # i2c = board.I2C()  # For board.SCL and board.SDA
        i2c = board.I2C()  # uses board.SCL and board.SDA  attention c'est celui la que j'ai commente
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        sensor.mode = Mode.NDOF_MODE  # Set the sensor to NDOF_MODE

        self.print_client("sensor -text==magnetometer")
        while not sensor.calibration_status[3] == 3:
            # Calibration Dance Step One: Magnetometer
            #   Move sensor away from magnetic interference or shields
            #   Perform the figure-eight until calibrated
            self.print_client(f"Mag Calib Status: {100 / 3 * sensor.calibration_status[3]:3.0f}%")
            time.sleep(1)
        self.print_client("... CALIBRATED")
        time.sleep(1)

        self.print_client("sensor -text==accel")
        while not sensor.calibration_status[2] == 3:
            # Calibration Dance Step Two: Accelerometer
            #   Place sensor board into six stable positions for a few seconds each:
            #    1) x-axis right, y-axis up,    z-axis away
            #    2) x-axis up,    y-axis left,  z-axis away
            #    3) x-axis left,  y-axis down,  z-axis away
            #    4) x-axis down,  y-axis right, z-axis away
            #    5) x-axis left,  y-axis right, z-axis up
            #    6) x-axis right, y-axis left,  z-axis down
            #   Repeat the steps until calibrated
            self.print_client(f"Accel Calib Status: {100 / 3 * sensor.calibration_status[2]:3.0f}%")
            time.sleep(1)
        self.print_client("... CALIBRATED")
        time.sleep(1)
 
        self.print_client("sensor -text==gyro")
        while not sensor.calibration_status[1] == 3:
            # Calibration Dance Step Three: Gyroscope
            #  Place sensor in any stable position for a few seconds
            #  (Accelerometer calibration may also calibrate the gyro)
            self.print_client(f"Gyro Calib Status: {100 / 3 * sensor.calibration_status[1]:3.0f}%")
            time.sleep(1)
        self.print_client("... CALIBRATED")
        time.sleep(1)

        self.print_client("\nCALIBRATION COMPLETED")
        self.print_client("sensor -text==insert")
        self.print_client(f"sensor -Off_M=={sensor.offsets_magnetometer}")
        self.print_client(f"sensor -Off_G=={sensor.offsets_gyroscope}")
        self.print_client(f"sensor -Off_A=={sensor.offsets_accelerometer}")
        time.sleep(0.2)
        self.print_client("input -sensors_offset")
        while not self.sensor_offset_done:
            pass    
        if self.sensor_offset == 'y':
            temps_debut = time.time()
            satisfaction = "n"

            # Exécuter la tâche pendant 10 secondes
            self.print_client("Euler angle: ")
            while (time.time() - temps_debut) < 10:
                self.print_client(sensor.euler)
                time.sleep(1)

            while satisfaction == "n":
                self.print_client("input -sensors_yaw")
                while not self.input_sensors_yaw_set:
                    pass
                self.print_client("input -sensors_pitch")
                while not self.input_sensors_pitch_set:
                    pass
                self.print_client("input -sensors_roll")
                while not self.input_sensors_roll_set:
                    pass
                self.yaw += self.sensor_yaw_offset
                self.pitch += self.sensor_pitch_offset
                self.roll += self.sensor_roll_offset
                self.target_angle_offset = (self.yaw, self.pitch, self.roll)
                self.print_client("Euler angle: ")
                temps_debut = time.time()
                while (time.time() - temps_debut) < 10:
                    sensor_euler = sensor.euler
                    yaw_corrected, pitch_corrected, roll_corrected = [position - self.target_angle_offset[idx] for idx, position in enumerate(sensor_euler)]
                    euler = (yaw_corrected, pitch_corrected, roll_corrected)
                    self.print_client(euler)
                    time.sleep(1)

                self.print_client("input -sensors_satisfied")
                while not self.sensor_satisfied:
                    pass
                if self.sensor_offset_satisfied:
                    satisfaction = "y"
                    self.Start_sensors()
                else:
                    self.sensor_satisfied = False
                    self.input_sensors_yaw_set = False
                    self.input_sensors_pitch_set = False
                    self.input_sensors_roll_set = False
    
        elif self.sensor_offset == "n":
            self.target_angle_offset = (0, 0, 0)
            self.Start_sensors()

    
    # endregion
    # region ESP32
    def Start_ESP32(self):
        self.ESP32_com_thread = threading.Thread(target=self.ESP32.Start_sending,args=[self.ESP32_data])
        self.ESP32_com_thread.start()
        self.log_and_print(f"Start sending data to ESP32 from {self.port}...",Server.GREEN)

    
    def Stop_ESP32(self):
        self.ESP32.Stop_sending()
        self.ESP32_com_thread.join()
        self.log_and_print(f"Stopped sending data to ESP32 from {self.port}...",Server.GREEN)
    # endregion
    def print_client(self,msg,level: int = INFO):
        self.ground_station.sendall(str(msg).encode())
        self.log_and_print(msg,level,False)

    def log_and_print(self, txt: str, level: int = INFO, print_to_cl : bool = True):
        if print_to_cl:
            self.print_with_colors(txt,level)
        match level:
            case Server.DEBUG:
                logging.debug(txt)
            case Server.INFO:
                logging.info(txt)
            case Server.ERROR:
                logging.error(txt)
            case Server.WARNING:
                logging.warning(txt)    
            case Server.CRITICAL:
                logging.critical(txt)
            case Server.GREEN:
                logging.info(txt+" !!")

    def print_with_colors(self,txt:str,txt_type:int = INFO):
        match txt_type:
            case Server.DEBUG:
                print("debug" + txt)
            case Server.INFO:
                print(txt)
            case Server.ERROR:
                print(f"\033[91mError: {txt}\033[00m")
            case Server.WARNING:
                print(f"\033[93mWarning: {txt}\033[00m")
            case Server.CRITICAL:
                print(f"\033[93mCritical !!! {txt}\033[00m")
            case Server.GREEN:
                print(f"\033[92m{txt}\033[00m")



if __name__ == "__main__":
    server = Server(HOST,PORT)




