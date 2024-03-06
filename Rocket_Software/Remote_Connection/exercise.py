# Python program raising
# exceptions in a python
# thread

import threading
import time

stop_threads = False

class send_receive_msg():
    def __init__(self,first_msg):
        self.__needs_to_stop = False
        self.__msg = first_msg
        self.__new_msg = ""
        
    def RunThread(self):
        self.t1 = threading.Thread(target = self.ThreadAction)
        self.t1.start()

    def ThreadAction(self):
        while not self.__needs_to_stop:
            msg = self.get_update()
            print(msg)
            time.sleep(2)
            # msg = self.update_msg(msg)
            # self.__msg = msg

    def get_update(self):
        return self.__msg
    
    def update_msg(self,msg):
        if type(msg) == type(""):
            return msg + "1"
        else:
            return msg + 1            


    def stop_sending(self):
        self.__needs_to_stop = True
        self.t1.join()
        print('thread killed')

    def stopping(self):
        while True:
            self.__msg = self.update_msg(self.__msg)
            time.sleep(2)
            # self.__new_msg = self.__msg
            # print(self.__new_msg)
        self.msg = input("Kill ?")
        if self.msg == "yes":
            self.stop_sending()



            
test_1 = send_receive_msg(1)
test_1.RunThread()
test_1.stopping()
test_2 = send_receive_msg("1")
test_2.RunThread()
test_2.stopping()



