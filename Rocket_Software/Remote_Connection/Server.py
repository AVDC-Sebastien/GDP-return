import asyncio
import qtm
import socket
import threading
import logging
import time
import numpy as np
from Rocket_Software.ESP32.ESP32_Com import ESP32
import Rocket_Software.Sensors.SLAM_final.Sensor_fusion as Sensors


HOST, PORT = '0.0.0.0', 65000

class Server:
    
    __default_message_size = 32

    INFO, DEBUG, ERROR, WARNING, CRITICAL, GREEN = 0,10,20,30,40,50
    
    def __init__(self,host :str = '0.0.0.0',port : int = 65000 ,new_message_size = __default_message_size):
        '''
        Initialize the server with the define host and port, and a fixed size message
        '''
        # Create the log file
        logging.basicConfig(filename="GDP_retrun_server.log", level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%d-%b-%y %H:%M:%S',force=True)   

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

        # ESP32
        self.ESP32 : ESP32 = ESP32()
        self.ESP32_data = "data"

        # Path
        self.Emergency_landing = False
        
        # Sensors
        self.euler_offset = 0
        self.calibration = False
        self.calibration_done = False
        self.euler_angle = 0
        self.stop_sensor_fusion_thread = False
        

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
        self.Start_sensors()
        self.Handle_server_activity()
        
        #self.start_QTM()
          
    def open_server(self):
        '''
        Manage the connection of the clients
        '''
        try:
        
            log_and_print(f"Opening the local server on port: {HOST}")
            self.__sock.listen(self.max_clients)
            log_and_print("Server open",Server.GREEN)
            self.isServeropen = True
        

            connected_client, address = self.__sock.accept()

            # Set the IP, port and the name
            self.IP = address[0]
            self.port = address[1]
            self.name = "Ground_station"
            log_and_print(f"Ground station connected with ip: {self.IP} on port: {self.port}",Server.INFO)
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
            log_and_print(ex)
            log_and_print("Could not connect",Server.WARNING)
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
        log_and_print(f"Starting sending messages to {self.name}",Server.GREEN)

    def stop_sending(self):
        '''
        Stop the sending thread
        '''
        self.__needs_to_stop_sending_gs = True
        self.sending_thread.join()
        log_and_print(f"Stopped sending messages to {self.name}",Server.INFO)

    def send_message(self):
        '''
        Send message to the connected client
        '''
        try:
            while (not self.__needs_to_stop_sending_gs) and self.isServerrunning:            
                if self.msg_to_send != "":
                    self.ground_station.sendall(str(self.msg_to_send).encode())
                if self.__isQTM_connected:
                    self.test = self.__qtm_position
                    self.test = self.test * 4
                    print("true pos:",self.__qtm_position)
                    print("test",self.test)
        except:
                if self.__needs_to_stop_sending_gs and not self.isServerrunning:
                    log_and_print("Stopped sending",Server.INFO)
                elif self.isGround_station_connected:
                    log_and_print(f"Message couldn't be sent to {self.name}. Restarting sending...",Server.WARNING)
                    self.stop_sending()
                    self.start_sending()
                else:
                    log_and_print(f"Client {self.name} is not connected",Server.WARNING)
    #endregion
    
    # region Receiveing message
    def start_receiving(self):
        '''
        Initiate the sending message thread
        '''
        self.__needs_to_stop_receiving_gs = False
        self.receiving_thread = threading.Thread(target = self.receive_message)
        self.receiving_thread.start()
        log_and_print(f"Starting receiving messages from {self.name}",Server.GREEN)

    def stop_receiving(self):
        '''
        Stop the sending thread
        '''
        self.__needs_to_stop_receiving_gs = True
        self.receiving_thread.join()
        log_and_print(f"Stopped receiving messages from {self.name}",Server.INFO)   

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
                log_and_print(f"Error in receiving a message from {self.name}",Server.WARNING)
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
            # example of msg to send: "euler -offset==1,-2,3"
            case "euler -offset":
                self.euler_offset = np.array([float(i) for i in msg.split("==")[1].split(",")])
                self.euler_angle += self.euler_offset
                self.ground_station.sendall(str(self.euler_angle).encode())

            case "control -calibration_done":
                self.calibration_done = True

            case "control -calibration":
                self.ground_station.sendall(str(self.euler_angle).encode())

            case _:
                if self.print_received_data and self.isGround_station_connected:
                    log_and_print(f"Message received from {self.name} : " + msg)

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
                log_and_print("Shutting down the server",Server.WARNING)
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
                log_and_print("Disconnecting client failed",Server.ERROR)
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
        log_and_print("Starting QTM connection")
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
        self.senors = Sensors.sensor_fusion()
        self.senors.Start_measurement()
        self.sensor_fusion_thread = threading.Thread(target=self.senors.main_task)
        time.sleep(5)
        self.sensor_fusion_thread.start()
        print_with_colors("Started",Server.GREEN)
    
    def Stop_sensors(self):
        self.senors.Stop_measurement() 
        self.sensor_fusion_thread.join()
    # endregion
    # region ESP32
    def Start_ESP32(self):
        self.ESP32_com_thread = threading.Thread(target=self.ESP32.Start_sending,args=[self.ESP32_data])
        self.ESP32_com_thread.start()
        log_and_print(f"Start sending data to ESP32 from {self.port}...",Server.GREEN)

    
    def Stop_ESP32(self):
        self.ESP32.Stop_sending()
        self.ESP32_com_thread.join()
        log_and_print(f"Stopped sending data to ESP32 from {self.port}...",Server.GREEN)
    # endregion

def log_and_print(txt: str, level: int = Server.INFO):
    print_with_colors(txt,level)
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

def print_with_colors(txt:str,txt_type:str):
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




