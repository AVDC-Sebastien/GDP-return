import asyncio
import qtm
import socket
import threading
import logging
import os
import json

HOST, PORT = '0.0.0.0', 65000
logging.basicConfig(filename="GDP_retrun_server.log", level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%d-%b-%y %H:%M:%S')
logger = logging.getLogger("gdp-return")


class Client:
    def __init__(self,connected_client : socket.socket, address : list, isServerrunning : bool, message_size : int ):
        
        self.connected_client = connected_client

        # Set the IP, port and the name
        self.IP = address[0]
        self.port = address[1]
        self.name = str(self.IP)
        self.named = False
        log_and_print(f"Client connected with ip: {self.IP} on port: {self.port}",Server.INFO)
        # Size of the message
        self.__message_size = message_size

        # Message to send
        self.__msg_to_send = ""
        # Activity of the server
        self.isServerrunning = isServerrunning

        # Activity of the client
        self.client_isConnected = True

        # Thread bool
        self.__needs_to_stop_sending = False
        self.__needs_to_stop_receiving = False
        self.print_received_data = True

        # QTM
        self.isQTM_running = False
        self.start_QTM = False
        
        # Strat sending/receiving
        self.start_receiving()
        self.start_sending()

    # region miscellaneous
    def change_message_size(self,new_size):
        try:
            self.connected_client.sendall(str(f"gdp-return -new_message_size=={self.__message_size} ").encode())
        except Exception as e:
            log_and_print(e)
        log_and_print(f"Message size changed from {self.__message_size} to {new_size}",Server.INFO)
        self.__message_size = new_size
        return

    def change_name(self,new_name : str):
        try:
            if not self.named:
                self.name = new_name
                log_and_print(f"Name changed to {self.name} for {self.IP} ",Server.INFO)
                self.named = True
        except:
            log_and_print("The new name: '{new_name}' is not a valid name",Server.WARNING)
            self.connected_client.sendall(str(f"\033[93mWarning! The new name: '{new_name}' is not a valid name\033[00m").encode())
            return
# endregion

    # region Sending message
    def start_sending(self):
        '''
        Initiate the sending message thread
        '''
        self.needs_to_stop_sending = False
        self.sending_thread = threading.Thread(target = self.send_message)
        self.sending_thread.start()
        log_and_print(f"Starting sending messages to {self.name}",Server.GREEN)

    def stop_sending(self):
        '''
        Stop the sending thread
        '''
        self.__needs_to_stop_sending = True
        self.sending_thread.join()
        log_and_print(f"Stopped sending messages to {self.name}",Server.INFO)

    def send_message(self):
        '''
        Send message to the connected client
        '''
        try:
            while (not self.__needs_to_stop_sending) and self.isServerrunning:            
                if self.__msg_to_send != "":
                    self.connected_client.sendall(str(self.__msg_to_send).encode())
        except:
                if self.__needs_to_stop_sending and not self.isServerrunning:
                    log_and_print("Stopped sending",Server.INFO)
                elif self.client_isConnected:
                    log_and_print(f"Message couldn't be sent to {self.name}. Restarting sending...",Server.WARNING)
                    self.stop_sending()
                    self.start_sending()
                else:
                    log_and_print(f"Client {self.name} is not connected",Server.WARNING)
                    self.disconnect_client()        
    #endregion
    
    # region Receiveing message
    def start_receiving(self):
        '''
        Initiate the sending message thread
        '''
        self.__needs_to_stop_receiving = False
        self.receiving_thread = threading.Thread(target = self.receive_message)
        self.receiving_thread.start()
        log_and_print(f"Starting receiving messages from {self.name}",Server.GREEN)

    def stop_receiving(self):
        '''
        Stop the sending thread
        '''
        self.__needs_to_stop_receiving = True
        self.receiving_thread.join()
        log_and_print(f"Stopped receiving messages from {self.name}",Server.INFO)   

    def receive_message(self):
        '''
        Receive the message and print it (or not print_received_data) and execute the msg received
        '''
        try:
            while (not self.__needs_to_stop_receiving) and self.isServerrunning:
                #Check if the previous received message has been executed
                data = self.connected_client.recv(self.__message_size).decode()
                if self.client_isConnected:
                    self.execute_message(data)
        except:
            if not self.__needs_to_stop_receiving:
                log_and_print(f"Error in receiving a message from {self.name}",Server.WARNING)
                resend = input("Do you want to restart receiving again, if not the client will be disconnected (Y/n)?")
                while resend.lower() not in ('y','n'):
                    resend = input("Please only answer by 'Y' or 'n' only: ")
                if resend.lower() == 'y':
                    self.stop_receiving()
                    self.start_receiving()
                else:
                    self.disconnect_client()
                    
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
                self.disconnect_client()

            case "ping":
                self.connected_client.sendall(str("ping").encode())
            
            case "gdp-return -new_name":
                self.change_name(msg.split("==")[1])

            case "Start -QTM":
                self.start_QTM = True
            
            case "Stop -QTM":
                self.start_QTM = False
            
            case _:
                if self.print_received_data and self.client_isConnected:
                        log_and_print(f"Message received from {self.name} : " + msg)
            # case _:
            #      self.__msg_to_send = msg
    # endregion
                            
    def disconnect_client(self):
            try:
                log_and_print(f"Disconnecting {self.name} in the background")
                self.client_isConnected = False
            except Exception as ex:
                disconnect = input("\033[93mWarning! Coulnd't disconnect the client, try again (Y/n)?\033[00m")
                while disconnect.lower() not in ('y','n'):
                    disconnect = input("Please only answer by 'Y' or 'n' only: ")
                if disconnect.lower() == 'y':
                    self.disconnect_client()
                else:
                    log_and_print(ex)
                    log_and_print("The server will now shutdown")
                    exit()

    

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
        self.__connected_client : list[Client] = []
        self.sending_thread : list[threading.Thread] = []
        self.print_all_received_data = True
        self.__any_client_connected = False

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
        self.connecting_client_thread = threading.Thread(target = self.open_server)
        self.connecting_client_thread.start()
        self.Server_command()
        
        #self.start_QTM()
          
    def open_server(self):
        '''
        Manage the connection of the clients
        '''
        try:
            while self.__server_open_to_connect:
                if not self.isServeropen:
                    logger.info(f"Opening the local server on port: {HOST}")
                    log_and_print(f"Opening the local server on port: {HOST}")
                    self.__sock.listen(self.max_clients)
                    log_and_print("Server open",Server.GREEN)
                    self.isServeropen = True
                
                # Connect the client
                if len(self.__connected_client) < self.max_clients:
                    self.__full = False
                    connected_client, adress = self.__sock.accept()
                    # Create the new client identity
                    if adress[0] != "127.0.0.1":
                        self.__connected_client.append(Client(connected_client,adress,self.isServerrunning,self.__message_size))
                    self.__any_client_connected = True

                elif len(self.__connected_client) == self.max_clients:
                    if self.__full == True:
                        log_and_print(f"The server is full. Clients connected:")
                        for client in self.__connected_client:
                            log_and_print(f"- {client.name}")
                        self.__full = True

        except Exception as ex:
            log_and_print(ex)
            log_and_print("Could not connect",Server.WARNING)
            self.open_server()
    
    def Server_command(self):
        while self.isServerrunning:
            command = input("__________________________________\n\033[92mgdp-return: \033[00m")
            match command:
                case "shutdown":
                    self.shutdown()
                case "disconnect -all":
                    self.update_clients("disconnect")
                case "clients -list":
                    self.list_of_clients()
    
    def list_of_clients(self):
        log_and_print(self.__connected_client)

    def shutdown(self):
        '''
        Shutdwon the server
        '''
        try:
            log_and_print("shutting down the server",Server.WARNING)
            # Disconnecting all the clients
            self.__server_open_to_connect = False
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(self.__server_address)
            self.connecting_client_thread.join()
            print("1")
            if self.__any_client_connected:
                self.update_clients("disconnect")
            self.__any_client_connected = False
            print("2")
            # Closing the TCP server Connection
            self.isServerrunning = False
            exit()
        except:
            if not self.__any_client_connected and not self.isServerrunning:
                exit()
            elif self.__any_client_connected:
                log_and_print("Disconnecting client failed",Server.ERROR)
                again = input("Try again if not the server will force shutdown ? (Y/n)")
                while again.lower() not in ('y','n'):
                    again = input("Please only answer by 'Y' or 'n' only: ")
                if again.lower() == 'y':
                    self.shutdown()
                else:
                    exit()
            else:
                print("weird")

    def update_clients(self,command):
        match command:
            # Disconnecting all the clients
            case "disconnect":
                log_and_print("Disconnecting the clients...")
                for client in self.__connected_client:
                    client.disconnect_client()    
                log_and_print("Clients disconnected",Server.GREEN)
            # Update the QTM running status
            case "QTM_isrunning":
                for client in self.__connected_client:
                    client.isQTM_running = True
            case "QTM_isstoped":
                for client in self.__connected_client:
                    client.isQTM_running = False

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




