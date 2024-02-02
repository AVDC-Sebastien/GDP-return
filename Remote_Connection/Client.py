import socket
import threading

HOST, PORT = '138.250.149.163', 65000

class Client:
    
    __message_size = 32
    
    def __init__(self,host,port,new_message_size = __message_size):
        # Create a TCP socket
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)     
        self.__server_address = (host, port)
        self.__message_size = new_message_size
        self.__msg = 1

        # Boolean of the client
        self.__isClientrunning = True
        self.print_received_data = True
        self.__isConnected = False
        self.__needs_to_stop = False
        self.__stop_receiving_msg = False
        self.start()


    def start(self):
        self.connect()
        self.start_sending()
        self.receive_message()
          

    def connect(self):
        try:
            self.__sock.connect(self.__server_address)
            self.__isConnected = True
            print("Connection done!")
        except:
            print("Could not connect")
    
    def disconnect(self):
        try:
            print("Disconnection...")
            self.__isConnected = False
            self.__sock.close()
            self.__isClientrunning = False
        except:
            if self.__isConnected:
                print("Disconnection failed")
            else:
                print("No server connected")

#region Sending message
    def send_message(self,message):
        try:
            while (not self.__needs_to_stop) and self.__isClientrunning:
                message = self.update_msg()
                self.__sock.sendall(str(message).encode())
        except:
                if self.__isConnected:
                    print("Message couldn't be sent")
                else:
                    print("There is no server connected")
    
    def update_msg(self):
        return self.custom_message()

    def start_sending(self):
        self.t1 = threading.Thread(target = self.send_message, args=[self.__msg])
        self.t1.start()
        print("starting sending")

    def stop_sending(self):
        self.__needs_to_stop = True
        self.t1.join()
        print("Sending stopped")
#endregion
#region Receive message
    def receive_message(self):
        try:
            while (not self.__stop_receiving_msg) and self.__isClientrunning:
                data = self.__sock.recv(self.__message_size).decode()
                if self.print_received_data:
                    print(data)
                self.execute_message(data)
        except:
                print("Error in receiving a message")
#endregion
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
    
    def custom_message(self):
        msg = input("Message to send: ")
        if msg == "exit":
            self.disconnect()
        return msg


client = Client(HOST,PORT)





