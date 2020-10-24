import serial
import time
def read():
    ser = serial.Serial(port = "com9", baudrate = 115200 , timeout=0.1)
    ser.rts = 0
    while(True):
        # time.sleep(0.5)
        # ser.write(b"a")
        line = ser.readline()  
        data = line.decode()[:len(line)-1]
        if(len(data) > 0):
            print(data)

if __name__ == '__main__':
    try:
        read()
    except KeyboardInterrupt:
        print("\n\n\n\nShutdown ...\n\n\n\n")