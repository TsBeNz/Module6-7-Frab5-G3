import serial
import time


def read():
    ser = serial.Serial(port="com9", baudrate=115200, timeout=0.1)
    ser.rts = 0
    while(True):
        # time.sleep(0.5)
        line = ser.readline()
        data = line.decode()[:len(line)-1]
        if(len(data) > 0):
            print(data)
            if(data == "set home finish"):
                break

    while(True):
        line = ser.readline()
        data = line.decode()[:len(line)-1]
        if(len(data) > 0):
            print(data)
            if(data == "ok"):
                x_in = int(input("x : \n"))
                y_in = int(input("y : \n"))
                while(x_in > 400 or y_in > 400):
                    print("data out of range")
                    x_in = int(input("x : \n"))
                    y_in = int(input("y : \n"))
                print("sent data "+str(x_in) + " " + str(y_in))
                ser.write(data_frame(x_in,y_in ))


def data_frame(x, y):
    return bytes([255, 255, x & 0xFF, x >> 8, y & 0xFF, y >> 8])


if __name__ == '__main__':
    try:
        read()
    except KeyboardInterrupt:
        print("\n\n\n\nShutdown ...\n\n\n\n")
