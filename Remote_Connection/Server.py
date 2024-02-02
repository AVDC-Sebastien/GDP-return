
import socket
import threading

HOST, PORT = '0.0.0.0', 65000

class Server:
    
    __message_size = 32
    
    def __init__(self,host,port,new_message_size = __message_size):
        # Create a TCP socket
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)     
        self.__server_address = (host, port)
        self.__message_size = new_message_size
                
        # Starting TCP server on {host} port {port}
        self.__sock.bind(self.__server_address)
        self.__isServerrunning = True
        self.__msg = "hi"

        # Boolean of the client
        self.__connected_client = 0
        self.print_received_data = True
        self.__client_isConnected = False
        self.__needs_to_stop = False
        self.__stop_receiving_msg = False
        self.start_server()


    def start_server(self):
        self.open_server()
        self.start_sending()
        self.receive_message()
          

    def open_server(self):
        try:
            self.__sock.listen()
            print("Server open")
            self.__connected_client, self.__addr = self.__sock.accept()
            print(f"client connected with {self.__addr}")
            self.__client_isConnected = True
        except:
            print("Could not connect")
    
    def disconnect(self):
        try:
            print("Disconnection...")
            self.__sock.close()
            self.__client_isConnected = False
            self.__isServerrunning = False

        except:
            if self.__client_isConnected:
                print("Disconnection failed")
            else:
                print("No server connected")

#region Sending message
    def send_message(self,message):
        try:
            while (not self.__needs_to_stop) and self.__isServerrunning:
                message = self.update_msg()
                self.__connected_client.sendall(str(message).encode())
        except:
                if self.__client_isConnected:
                    print("Message couldn't be sent")
                else:
                    print("There is no client connected")

    def update_msg(self):
        return self.__msg
    
    def start_sending(self):
        self.t1 = threading.Thread(target = self.send_message,args=[self.__msg])
        self.t1.start()
        print("starting sending")

    def stop_sending(self):
        self.__needs_to_stop = True
        self.t1.join()
        print("sending stopped")
#endregion

    def receive_message(self):
        try:
            while (not self.__stop_receiving_msg) and self.__isServerrunning:
                data = self.__connected_client.recv(self.__message_size).decode()
                if self.print_received_data:
                    print(data)
                self.execute_message(data)
        except:
                print("Error in receiving a message")
    
    
    def execute_message(self,msg):
        match msg:
            case "stop":
                self.stop_sending()

            case "start":
                self.start_sending()

            case "exit":
                self.__stop_receiving_msg = True
                self.disconnect()
                exit()
    


server = Server(HOST,PORT)




