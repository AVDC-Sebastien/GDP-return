import socket
import threading
import time

HOST, PORT = '138.250.151.144', 65000

class Client:
    
    __message_size = 32
    
    def __init__(self,host,port,new_message_size = __message_size):
        
        new_host = input("Change IP?(Y/n): ")
        while new_host.lower() not in ("y","n"):
            new_host = input("please type 'Y' or 'n': ")
        if new_host.lower() == "y":
            host = input("IP adress: ")
            print()
        # Create a TCP socket
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)     
        self.__server_address = (host, port)
        self.__message_size = new_message_size
        self.__msg = 1

        self.client_name = input("Client name: ")
        # Boolean of the client
        self.__isClientrunning = True
        self.print_received_data = True
        self.__isConnected = False
        self.__needs_to_stop_sending = False
        self.__needs_to_stop_receiving = False


        self.__sending_time = []
        self.start()


    def start(self):
        self.connect()

          

    def connect(self):
        try:
            print(f"Trying to connect to {self.__server_address[0]} on port {self.__server_address[1]}")
            self.__sock.connect(self.__server_address)
            self.__isConnected = True
            print_with_colors("Connection done!","Green")
            self.__sock.sendall(str("gdp-return -new_name==" + self.client_name).encode())
            self.start_sending()
            self.start_receiving()
        except:
            print_with_colors("Could not connect","Warning")
            try_again = input("Want to try again ?(Y/N): ")
            while try_again.lower() not in ("y","n"):
                try_again = input("Try 'Y' or 'N': ")
            if try_again.lower() == "y":
                self.connect()
            else:
                self.disconnect()
    
    def shutdown(self):
        try:
            if self.__isConnected:
                self.print_received_data = False
                print("Shutting down the server")
                self.__isConnected = False
                self.__sock.sendall(str("shutdown").encode())
                time.sleep(2)                
                self.__needs_to_stop_sending = True
                self.__needs_to_stop_receiving = True
                self.__sock.close()
                self.__isClientrunning = False
                print_with_colors("Disconnected","Green")
                exit()
            else:
                print_with_colors("Exiting","Green")
                exit()
        except Exception as e:
            if self.__isConnected and not self.__isClientrunning:
                return
            elif self.__isConnected:
                print("Disconnection failed")
            else:
                print("Error")
                      
    def disconnect(self):
        try:
            if self.__isConnected:
                self.print_received_data = False
                print("Disconnection...")
                self.__isConnected = False
                self.__sock.sendall(str("disconnect").encode())
                time.sleep(1)                
                self.__needs_to_stop_sending = True
                self.__needs_to_stop_receiving = True
                self.__sock.close()
                self.__isClientrunning = False
                print_with_colors("Disconnected","Green")
                exit()
            else:
                print_with_colors("Exiting","Green")
                exit()
        except Exception as e:
            if self.__isConnected and not self.__isClientrunning:
                return
            elif self.__isConnected:
                print("Disconnection failed")
            else:
                print("Error")
#region Sending message
    def send_message(self,message):
        try:
            while (not self.__needs_to_stop_sending) and self.__isClientrunning:
                message = self.update_msg()
                self.__sock.sendall(str(message).encode())
        except:
                if not self.__isConnected and not self.__isClientrunning:
                    return
                if self.__isConnected:
                    print("Message couldn't be sent")
                else:
                    print("There is no server connected")
    
    def update_msg(self):
        msg = input("Message to send: ")
        match msg:
            
            case "shutdown":
                self.shutdown()

            case "exit" | "disconnect":
                self.disconnect()
                
                
            case "ping":
                self.__sending_time.append(time.time())
                return "ping"
            
            case "ping -m":
                print(sum(self.__sending_time)/len(self.__sending_time))

            case "show_msg":
                self.print_received_data=False

        return msg

    def start_sending(self):
        self.t1 = threading.Thread(target = self.send_message, args=[self.__msg])
        self.t1.start()
        print("starting sending")

    def stop_sending(self):
        self.__needs_to_stop_sending = True
        self.t1.join()
        print("Sending stopped")
#endregion
#region Receive message
    def start_receiving(self):
        '''
        Initiate the sending message thread
        '''
        self.__needs_to_stop_receiving = False
        self.receiving_thread = threading.Thread(target = self.receive_message)
        self.receiving_thread.start()
        print_with_colors("Starting receiving messages !","Green")

    def stop_receiving(self):
        '''
        Stop the sending thread
        '''
        self.__needs_to_stop_receiving = True
        self.receiving_thread.join()
        print("Receiving stopped")   

    def receive_message(self):
        try:
            while (not self.__needs_to_stop_receiving) and self.__isClientrunning:
                data = self.__sock.recv(self.__message_size).decode()
                self.execute_message(data)
                if self.print_received_data:
                    print(data)
        except:
                if not self.__needs_to_stop_receiving:
                    print("Error in receiving a message")
#endregion
    def execute_message(self,msg):
        match msg:
            case "stop":
                self.stop_sending()

            case "start":
                self.start_sending()

            case "shutdown":
                self.__needs_to_stop_receiving = True
                self.disconnect()
                exit()
            case "ping":
                self.t = 1000 * (time.time() - self.__sending_time[-1])
                print("receiving time : " + str(self.t) +"ms")
                self.__sending_time[-1] = self.t


def print_with_colors(txt,txt_type):
    if txt_type == "Error":
        print("\033[91mError: {}\033[00m".format(txt))
    if txt_type == "Warning":
         print("\033[93mWarning: {}\033[00m".format(txt))
    if txt_type == "Green":
        print("\033[92m{}\033[00m".format(txt))

client = Client(HOST,PORT)





