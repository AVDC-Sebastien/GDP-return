# Server #
import socket

class Server:
    HOST = "138.250.184.188"
    PORT = 12345      
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.HOST, self.PORT))    

    def accept(self):
        self.sock.listen()
        c, a = self.sock.accept()
        self.rpi = c
        self.send()

    def send(self):
        self.rpi.send(str("damn").encode())
    

s = Server()