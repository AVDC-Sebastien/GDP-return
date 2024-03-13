import serial
import logging

class ESP32:
    def __init__(self,port : str = "COM6", bauderate : int = 115200, parity : str = serial.PARITY_ODD, stopbits : float = serial.STOPBITS_TWO, bytesize : int = serial.SEVENBITS ):
        '''
        Start the ESP32 communication
        '''
        logging.info("Starting ESP32 communication")
        
        # configure the serial connections (the parameters differs on the device you are connecting to)
        self.port = port
        self.bauderate = bauderate
        self.ESP32 = serial.Serial(self.port,self.bauderate)

        # Sending bool
        self.need_to_stop_sending = False

    def Start_sending(self,command : str):
        while not self.need_to_stop_sending:
            self.ESP32.write((command).encode())

    def Stop_sending(self):
        self.need_to_stop_sending=True
        self.ESP32.close()

if __name__ == "__main__":
    test_esp32 = ESP32()
    test_esp32.Start_sending("hi")