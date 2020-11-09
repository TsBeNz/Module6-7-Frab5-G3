import serial
import time
import psutil
import matplotlib.pyplot as plt
import communication
import numpy as np


def read():
    ser = serial.Serial(port="com9", baudrate=115200)
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
                z_in = int(input("z : \n"))
                thata_in = int(input("thata : \n"))
                while(x_in > 420 or y_in > 420 or z_in > 420 or thata_in > 420):
                    print("data out of range")
                    x_in = int(input("x : \n"))
                    y_in = int(input("y : \n"))
                    z_in = int(input("z : \n"))
                    thata_in = int(input("thata : \n"))
                print(str(x_in) + " " + str(y_in)+" " +
                      str(z_in)+" " + str(thata_in))
                ser.write(data_frame(x_in, y_in, z_in, thata_in))


def readtest():
    ser = serial.Serial(port="com9", baudrate=115200)
    ser.rts = 0
    while(True):
        # time.sleep(0.5)
        line = ser.readline()
        data = line.decode()[:len(line)-1]
        if(len(data) > 0):
            print(data)
            if(data == "set home finish"):
                break

    a = 0
    b = 0
    c = 0
    d = 0
    while(True):
        line = ser.readline()
        data = line.decode()[:len(line)-1]
        if(len(data) > 0):
            print(data)
            if(data == "ok"):
                # x_in = int(input("x : \n"))
                # y_in = int(input("y : \n"))
                # z_in = int(input("z : \n"))
                # thata_in = int(input("thata : \n"))
                # while(x_in > 420 or y_in > 420 or z_in > 420 or thata_in > 420):
                #     print("data out of range")
                #     x_in = int(input("x : \n"))
                #     y_in = int(input("y : \n"))
                #     z_in = int(input("z : \n"))
                #     thata_in = int(input("thata : \n"))
                # print(str(x_in) + " " + str(y_in)+" " + str(z_in)+" " + str(thata_in))
                print("input "+str(a) + " " + str(b)+" " + str(c)+" " + str(d))
                ser.write(data_frame(a, b, c, d))
        line = ser.readline()
        data = line.decode()[:len(line)-1]
        if(len(data) > 0):
            if(data == (str(a) + " " + str(b)+" " + str(c)+" " + str(d))):
                print("output "+str(data))
                print("pass")
                a = a+1
                b = b+1
                c = c+1
                d = d+1
                if(a == 512):
                    break
            else:
                print("error")
                print("input "+str(a) + " " + str(b)+" " + str(c)+" " + str(d))
                print("output "+str(data))
                break


# def data_frame(x, y):
#     return bytes([255, 255, x & 0xFF, x >> 8, y & 0xFF, y >> 8])

def data_frame(x=0, y=0, z=0, thata=0, type=0):
    return bytes([255, 255, x >> 1, (((x & 0x0001) << 7) | (y >> 2)), (((y & 0x0003) << 6) | (z >> 3)), (((z & 0x0007) << 5) | (thata >> 4)), (((thata & 0x000F) << 4) | (0x00 >> 4))])


def realtimeplot():
    # ser = serial.Serial(port="com9", baudrate=115200)
    # ser.rts = 0
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ay = fig.add_subplot(111)
    fig.show()
    i = 0
    x, y, y1 = [], [], []
    block = 50
    while True:
        buffer = psutil.cpu_percent()
        x.append(i)
        y.append(buffer)
        y1.append(buffer-10)
        ax.plot(x, y, color='b')
        ay.plot(x, y1, color='g')
        fig.canvas.draw()
        ax.set_xlim(left=max(0, i-block), right=i+block)
        ay.set_xlim(left=max(0, i-block), right=i+block)
        plt.pause(0.0001)
        # time.sleep(0.01)
        i += 1


def trajectory(t0=0, tf=8, q0=0, v0=0, q1=100, v1=0):
    M = np.array([[1, t0, t0**2,  t0**3],
                  [0, 1,  2*t0,   3*t0**2],
                  [1, tf, tf**2,  tf**3],
                  [0, 1,  2*tf,   3*(tf**2)]])
    b = np.array([[q0], [v0], [q1], [v1]])
    Minv = np.linalg.inv(M)
    co_trajectory = np.array([[0], [0], [0], [0]])
    co_trajectory = np.dot(Minv, b)
    output = (co_trajectory.tolist())
    a = int(round(float(output[0][0])*10000))
    b = int(round(float(output[1][0])*10000))
    c = int(round(float(output[2][0])*10000))
    d = int(round(float(output[3][0])*10000))
    print(str(a) + ' ' +str(b) + ' ' +str(c) + ' ' +str(d) )


if __name__ == '__main__':
    try:
        # readtest()
        # read()
        # realtimeplot()
        trajectory()
    except KeyboardInterrupt:
        print("\n\n\n\nShutdown ...\n\n\n\n")
