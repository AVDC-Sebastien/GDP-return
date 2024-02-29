import asyncio
import qtm
import socket
import threading

HOST, PORT = '0.0.0.0', 65000

class Server:
    
    __message_size = 32
    
    def __init__(self,host,port,new_message_size = __message_size):
        '''
        Initialize the server
        '''
        # region Create a TCP socket
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)     
        self.__server_address = (host, port)
        self.__message_size = new_message_size
        # endregion                
        # region Starting TCP server on {host} port {port}
        self.__sock.bind(self.__server_address)
        self.__isServerrunning = True
        self.__msg = "hi"
        self.__custom_message = ""
        #endregion
        # region Boolean of the client
        self.__connected_client = 0
        self.print_received_data = True
        self.__client_isConnected = False
        self.__needs_to_stop = False
        self.__stop_receiving_msg = False
        #endregion
        # region QTM
        self.__connection_password = "gdp-return"
        self.__disconnect_QTM = False
        self.__qtm_IP = "138.250.154.110"
        self.__tolerance = 0.1
        self.__qtm_position = []
        self.__qtm_rotation = []
        #endregion

        self.start_server()


    def start_server(self):
        '''
        Start the different thread and functions we need
        '''
        self.open_server()
        self.start_sending()
        self.receive_message()
        self.start_QTM()
          
    def open_server(self):
        '''
        Manage the connection of the clients
        '''
        try:
            self.__sock.listen()
            print("Server open")
            self.__connected_client, self.__addr = self.__sock.accept()
            print(f"client connected with {self.__addr}")
            self.__client_isConnected = True
        except:
            print("Could not connect")
    
    def disconnect(self):
        '''
        Disconnect the server
        '''
        try:
            self.print_received_data = False
            print("Disconnection...")
            self.__stop_receiving_msg = True
            self.__sock.close()
            self.__client_isConnected = False
            self.__isServerrunning = False
            print("Disconnected")
            exit()
        except:
            if not self.__client_isConnected and not self.__isServerrunning:
                return
            elif self.__client_isConnected:
                print("Disconnection failed")
            else:
                print("No server connected")

#region Sending message
    def start_sending(self):
        '''
        Initiate the sending message thread
        '''
        self.__needs_to_stop = False
        self.t1 = threading.Thread(target = self.send_message,args=[self.__msg])
        self.t1.start()
        print("starting sending")

    def stop_sending(self):
        '''
        Stop the sending thread
        '''
        self.__needs_to_stop = True
        self.t1.join()
        print("sending stopped")

    def send_message(self,message):
        '''
        Send message to the connected client
        '''
        try:
            while (not self.__needs_to_stop) and self.__isServerrunning:
                message = self.update_msg()
                if message != "Not sending any msg":
                    self.__connected_client.sendall(str(message).encode())
        except:
                if self.__needs_to_stop and not self.__isServerrunning:
                    print("Stopped sending")
                elif self.__client_isConnected:
                    print("Message couldn't be sent")
                else:
                    print("There is no client connected")

    def update_msg(self):
        '''
        Update the message if it has changed
        '''
        return self.custom_message()
    
    def custom_message(self):
        '''
        Change the message to send
        '''
        if self.__custom_message  == "exit":
            self.disconnect()
        if self.__custom_message  == "ping":
            self.__connected_client.sendall(str("ping").encode())
            self.__custom_message = "ping sent"
            print("ping sent")
        return "Not sending any msg"

#endregion
#region Receiveing message
    def receive_message(self):
        '''
        Receive the message and print it (or not print_received_data) and execute the msg received
        '''
        try:
            while (not self.__stop_receiving_msg) and self.__isServerrunning:
                data = self.__connected_client.recv(self.__message_size).decode()
                if self.print_received_data:
                    print("Message received : "+str(data))
                self.execute_message(data)
        except:
                if not self.__stop_receiving_msg:
                    print("Error in receiving a message")
    
    def execute_message(self,msg):
        '''
        Execute the incoming message 
        '''
        match msg:
            case "stop":
                self.stop_sending()

            case "start":
                self.start_sending()

            case "exit":
                self.disconnect()

            case "ping":
                self.__custom_message = "ping"
#endregion
#region Getting the data from qtm
                
    def start_QTM(self):
        print("Starting QTM connection")
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
            print(self.check_position(self.__qtm_position,self.__tolerance))
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


        



#endregion
if __name__ == "__main__":
    server = Server(HOST,PORT)




