# Python program raising
# exceptions in a python
# thread

import threading

stop_threads = False

class send_receive_msg():
    def __init__(self,first_msg):
        self.__needs_to_stop = False
        self.__msg = first_msg
        self.__new_msg = ""
        
    def start_sending(self):
        self.t1 = threading.Thread(target = self.sending, args= [self.__msg])
        self.t1.start()

    def sending(self,msg):
        while not self.__needs_to_stop:
            msg = self.update_msg(msg)
            self.__msg = msg
    
    def update_msg(self,msg):
        if type(msg) == type(""):
            return msg + "1"
        else:
            return msg + 1            

    def receiving(self):
        while True:
            print(self.__msg)
        # msg = input("input")
        # if msg == "stop":
        #     self.__needs_to_stop = True


    def stop_sending(self):
        self.__needs_to_stop = True
        self.t1.join()
        print('thread killed')

    def stopping(self):
        while True:
            self.__new_msg = self.__msg
            print(self.__new_msg)
        self.msg = input("Kill ?")
        if self.msg == "yes":
            self.stop_sending()



            
test_1 = send_receive_msg(1)
test_1.start_sending()
test_2 = send_receive_msg("1")
test_2.start_sending()
test_1.stopping()
test_2.stopping()



