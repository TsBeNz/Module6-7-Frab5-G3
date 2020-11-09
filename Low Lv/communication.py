import serial
import time
import math


class communication:
    """
    Class for communication with dsPIC #FRAB 5 Module 6-7 G.3
    =========================================================
    Function List
    ==============
    String Readline()\n
    void ResetdsPIC()\n
    bool Connect()\n
    void Go2home()\n
    void Move2point(4 parameter)\n
    """

    def __init__(self, port="com9", baudrate=2000000):
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate)
            self.ser.rts = 0
            self.status = 1
            while(True):
                if(self.Readline() == "start"):
                    print("connect to dsPIC success")
                    break
        except:
            print("\n\nCommunication to dsPIC Error !!!!!!\nComport = " +
                  str(port)+" ?\n\n")
            self.status = 0

    def Readline(self):
        """
        Read data from dsPIC
        ====================
        return --> Data after decode form dsPIC
        """
        line = self.ser.readline()
        try:
            return line.decode()[:len(line)-1]
        except:
            print("Decode error")
            return -1

    def Connect(self):
        """
        Connection Test
        ==============
        return --> bool
            True in case can init uart
            False in case can't init uart
        """
        if(self.status):
            return True
        return False

    def Go2home(self):
        """
        Set Home Robot
        ==============
        """
        print("Go2home")
        buffer = bytes([255, 0, 0, 0, 0, 0, 0, 255])
        self.ser.write(buffer)
        status = 0
        while(True):
            data = self.Readline()
            if(data == "Z set" and status == 0):
                status = 1
            elif(data == "X set" and status == 1):
                status = 2
            elif(data == "Y set" and status == 2):
                status = 3
            elif(data == "set home finish" and status == 3):
                print("Home success!!")
                break

    def status_point(self):
        """
        read status from dsPIC
        ==============
        """
        buffer = bytes([255, 0, 0, 0, 0, 0, 0, 238])
        self.ser.write(buffer)
        data = (self.Readline().split(" "))
        return data

    def ResetdsPIC(self):
        """
        Reset dsPIC
        ============
        **not terminate Pyserial
        """
        self.ser.rts = 1
        time.sleep(0.2)
        self.ser.rts = 0
        time.sleep(1)

    def Move2point(self, x=0, y=0, z=0, theta=0):
        """
        Move By Trajectory Control
        ========================
        Move Robot to position with Position Control
        """
        self.ser.write(
            bytes([253, x >> 8, x & 0x00FF, y >> 8, y & 0x00FF, z >> 8, z & 0x00FF, 0]))

    def Path_list(self, Path=[]):
        """
        Path Move By Position Control
        ========================
        use Move2porint() function for move robot\n
        """
        if(PIC.Connect()):
            i = 0
            while(True):
                # self.Move2point(50,50,0,0)
                data = self.Readline()
                if(data == "ok"):
                    print("Move success!!")
                    if(i == len(Path)):
                        print("finish move!!")
                        break
                    print("Move to " + str(Path[i]))
                    self.Move2point(Path[i][0], Path[i][1], 0, 0)
                    i += 1


if __name__ == '__main__':
    try:
        i = 0
        inputtest = []
        PIC = communication()
        PIC.Go2home()
        print("eiei")
        while(True):
            x_in = int(input("x : \n"))
            y_in = int(input("y : \n"))
            z_in = int(input("z : \n"))
            thata_in = int(input("thata : \n"))
            while(x_in > 450 or y_in > 450 or z_in > 420 or thata_in > 420):
                print("data out of range")
                x_in = int(input("x : \n"))
                y_in = int(input("y : \n"))
                z_in = int(input("z : \n"))
                thata_in = int(input("thata : \n"))
            PIC.Move2point(x_in, y_in, z_in, thata_in)
            # while(True):
            #     print(PIC.status_point())
            #     time.sleep(0.5)
        
        # while(True):
        #     print(PIC.Readline())
        # while(True):
        #     i+=3
        #     x=round(170*(math.sin((i*math.pi / 180)-math.pi/2))+220)-1
        #     y=round(170*(math.cos((i*math.pi / 180)-math.pi/2))+220)-1
        #     inputtest.append([x,y,0,0])
        #     if(i >= 360):
        #         break
        # PIC.Path_list(inputtest)
        # inputtest = [[50,400,0,0],[400,400,0,0],[400,50,0,0],[50,50,0,0]]
        # PIC.Path_list(inputtest)
        # while(True):
        #     

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt!!!!\n\n\nShutdown ...\n\n\n\n")
