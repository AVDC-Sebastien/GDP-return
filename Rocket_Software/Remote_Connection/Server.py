import asyncio
import qtm
import socket
import threading
import logging
import os
import json
import time

HOST, PORT = '0.0.0.0', 65000
logging.basicConfig(filename="GDP_retrun_server.log", level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%d-%b-%y %H:%M:%S')   

class Server:
    
    __default_message_size = 32

    INFO, DEBUG, ERROR, WARNING, CRITICAL, GREEN = 0,10,20,30,40,50
    
    def __init__(self,host :str = '0.0.0.0',port : int = 65000 ,new_message_size = __default_message_size):
        '''
        Initialize the server
        '''
        # Create the log file

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
        self.__disconnect_QTM = False
        self.__qtm_IP = "138.250.154.110"
        self.__tolerance = 0.1
        self.__qtm_position = []
        self.__qtm_rotation = []


        self.start_server()


    def start_server(self):
        '''
        Start the different thread and functions we need
        '''
        self.open_server()
        self.Server_command_thread = threading.Thread(target=self.Server_command)
        self.Server_command_thread.start()
        
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
            if ex == SystemExit:
                print("stopped")
                return
            if not self.__needs_to_stop_receiving_gs:
                log_and_print(f"Error in receiving a message from {self.name}",Server.WARNING)
                resend = input("Do you want to restart receiving again, if not the client will be disconnected (Y/n)?")
                while resend.lower() not in ('y','n'):
                    resend = input("Please only answer by 'Y' or 'n' only: ")
                if resend.lower() == 'y':
                    self.stop_receiving()
                    self.start_receiving()
                else:
                    print("line 367")                    
    
    def execute_message(self,msg : str):
        '''
        Execute the incoming message 
        '''
        match msg.split("==")[0]:
            case "stop":
                self.stop_sending()

            case "start":
                self.start_sending()

            case "exit" | "disconnect" | "shutdown":
                self.shutdown()

            case "ping":
                self.ground_station.sendall(str("ping").encode())

            case "Start -QTM":
                self.start_QTM = True
            
            case "Stop -QTM":
                self.start_QTM = False
            
            case _:
                if self.print_received_data and self.isGround_station_connected:
                        log_and_print(f"Message received from {self.name} : " + msg)

    # endregion
    # endregion

    def Server_command(self):
        while self.isServerrunning:
            command = input("__________________________________\n\033[92mgdp-return: \033[00m")
            match command:
                case "shutdown":
                    self.shutdown()

    def Handle_server_activity(self):
        while True:
            if not self.isServeropen:
                self.__sock.close()
                self.stop_sending()
                self.stop_receiving()
                self.isServerrunning = False
                self.Server_command_thread.join()
                self.isGround_station_connected = False        
                self.isServerrunning = False    
                exit()
            time.sleep(5)
    def shutdown(self):
        '''
        Shutdwon the server
        '''
        try:
            log_and_print("Shutting down the server",Server.WARNING)
            self.isServeropen = False
            # Disconnecting all the clients
            # if self.isGround_station_connected:
            #     self.ground_station.sendall(str("shutdown").encode())
            if threading.current_thread().name == threading.main_thread().name:
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
                print("weird")


    # region Getting the data from qtm
                
    def start_QTM(self):
        log_and_print("Starting QTM connection")
        self.__disconnect_QTM = False
        self.qtm_thread = threading.Thread(target=self.Async_start_QTM)
        self.qtm_thread.start()

    def stop_QTM(self):
        self.__disconnect_QTM = True
        self.qtm_thread.join()

    def Async_start_QTM(self):
        asyncio.get_event_loop().run_until_complete(self.get_QTM_data()) 

    async def get_QTM_data(self):
        # Connect to the QTM 
        connection = await qtm.connect(self.__qtm_IP)
        if connection is None:
            return -1

        self.update_clients("QTM_isrunning")
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
                info, bodies = packet.get_6d()
                for position, rotation in bodies:
                    self.update_position(position,rotation)
            log_and_print(self.check_position(self.__qtm_position,self.__tolerance))
            
            while True:
                try: 
                    if not self.__disconnect_QTM:
                        break
                    await connection.stream_frames(components=["6d"], on_packet=on_packet)
                except:
                    pass
            await connection.stream_frames_stop()
            # Stop the measurement
            await connection.stop()
            await connection.await_event(qtm.QRTEvent.EventCaptureStopped, timeout=10)
        connection.disconnect()
    
    def check_position(self, pos, tolerance = 0.1, x_min = 0, x_max = 100, y_min = 0, y_max = 100, z_min = 0, z_max = 100):
        min_values = [x_min,y_min,z_min]
        max_values = [x_max,y_max,z_max]
        for i in range(min_values):
            if pos[i] < (1+tolerance) * min_values[i] or pos > (1-tolerance)*max_values[i]:
                return "Out of Bound"
        return
    
    def update_position(self,pos,rot):
        self.__qtm_position = pos
        self.__qtm_rotation = rot

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
            logging.info(f"\033[92m{txt}\033[00m")

def print_with_colors(txt:str,txt_type:str):
    match txt_type:
        case "debug":
            print("debug" + txt)
        case "info":
            print(txt)
        case "error":
            print(f"\033[91mError: {txt}\033[00m")
        case "warning":
            print(f"\033[93mWarning: {txt}\033[00m")
        case "critical":
            print(f"\033[93mCritical !!! {txt}\033[00m")
        case "Green":
            print(f"\033[92m{txt}\033[00m")

def handle_json_file(file_path, default_data=None):
    if os.path.exists(file_path):
        # If the file exists, open and read it
        with open(file_path, 'r') as file:
            data = json.load(file)
        print(f"Existing JSON file found and loaded: {file_path}")
    else:
        # If the file does not exist, create a new one with default data
        data = default_data or {}
        with open(file_path, 'w') as file:
            json.dump(data, file, indent=2)
        print(f"New JSON file created: {file_path}")

    return data



if __name__ == "__main__":
    server = Server(HOST,PORT)




