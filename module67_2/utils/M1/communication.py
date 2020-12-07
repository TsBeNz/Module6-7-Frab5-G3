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
    bool Connect()\n
    void Go2home()\n
    list Status_point()\n
    void Offset()\n
    void ResetdsPIC()\n
    void Move2point()\n
    void Griper()\n
    void Path_list()\n
    void Griping_Rod()\n
    void Manual_Control()\n
    """

    def __init__(self, port="com7", baudrate=500000):
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate)
            self.ser.rts = 0
            self.status = 1
            self.offsetxy = 0
            self.offsetz = 0
            while(True):
                if(self.Readline() == "start"):
                    print("connect to dsPIC success")
                    break
        except:
            print("\nCommunication to dsPIC Error !!!!!!\nComport = " +
                  str(port)+" ?\n")
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
        Set Home Square_Root
        ==============
        """
        print("Go2home")
        buffer = bytes([255, 17, 0, 0, 0, 0, 0, 0, 0])
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
                time.sleep(0.1)  # wait low lv update status
                while True:
                    if(int(self.Status_point()[4]) == 1):
                        break
                    time.sleep(0.1)
                print("Home success!!")
                break

    def Status_point(self):
        """
        read status from dsPIC
        ==============
        """
        buffer = bytes([255, 34, 0, 0, 0, 0, 0, 0, 0])
        self.ser.write(buffer)
        data = (self.Readline().split())
        return data

    def Offset(self, offsetxy=0, offsetz=0):
        self.offsetxy = offsetxy
        self.offsetz = offsetz

    def ResetdsPIC(self):
        """
        Reset dsPIC in Square_Root
        ==========================
        **not terminate Pyserial
        """
        self.ser.rts = 1
        time.sleep(0.2)
        self.ser.rts = 0
        time.sleep(1)

    def Move2point(self, x=0, y=0, z=0, theta=0, debug=0):
        """
        Move By Trajectory Control
        ========================
        Move Square_Root to position with\n
        trajectory && cascade control with velocity feedforward
        """
        x += self.offsetxy
        y += self.offsetxy
        buffer_theta =  theta
        if buffer_theta < 0:
            buffer_theta += 360
        if buffer_theta > 180:
            buffer_theta -= 180
        print(buffer_theta)
        self.ser.write(
            bytes([255, 51, x >> 8, x & 0x00FF, y >> 8, y & 0x00FF, z >> 8, z & 0x00FF, int(buffer_theta) & 0x00FF]))
        while (True):
            data = self.Readline()
            if(data == "can't move"):
                self.ser.write(
                    bytes([255, 52, x >> 8, x & 0x00FF, y >> 8, y & 0x00FF, z >> 8, z & 0x00FF, buffer_theta & 0x00FF]))
                if debug:
                    print("Status Error")
                time.sleep(0.05)
            else:
                data_list = data.split()
                if(int(data_list[0]) == x and int(data_list[1]) == y and int(data_list[2]) == z and int(data_list[3]) == buffer_theta):
                    self.ser.write(
                        bytes([255, 153, 0, 0, 0, 0, 0, 0, 0]))
                    break
                else:
                    self.ser.write(
                        bytes([255, 52, x >> 8, x & 0x00FF, y >> 8, y & 0x00FF, z >> 8, z & 0x00FF, buffer_theta & 0x00FF]))
                    if debug:
                        print("Data Error")

    def Griper(self, status=0):
        """
        Control Griper  
        ========================
        status 0 --> Not Griping
        status 1 --> Griping
        """
        if(status == 1):
            buffer = bytes([255, 68, 255, 255, 0, 0, 0, 0, 0])
        else:
            buffer = bytes([255, 68, 255, 0, 0, 0, 0, 0, 0])
        self.ser.write(buffer)

    def Path_list(self, Path=[], Home=False):
        """
        Path Move By Position Control
        ========================
        use Move2point() function for move Square_Root\n
        Home is use for Force Sethome
        """
        print("\n\n\n\n\n\n\n\nPath_list move is start\n")
        if(self.Connect()):
            if(Home):
                self.Go2home()
            i = 0
            while(True):
                if(i == len(Path)):
                    print("finish move!!")
                    break
                print("Move to " + str(Path[i]))
                self.Move2point(int(Path[i][0]), int(
                    Path[i][1]), int(Path[i][2]), int(Path[i][3]),  debug=1)
                time.sleep(0.1)  # wait low lv update status
                while True:
                    if(int(self.Status_point()[4]) == 1):
                        break
                    time.sleep(0.1)
                print("Move success!!")
                if(len(Path[i]) == 5):
                    time.sleep(0.1)
                    if(Path[i][4] == 1):
                        self.Griper(1)
                        while True:
                            data = self.Readline()
                            if(data == "griper true"):
                                break
                            else:
                                self.Griper(1)
                            time.sleep(0.1)
                    else:
                        self.Griper(0)
                        while True:
                            data = self.Readline()
                            if(data == "griper false"):
                                break
                            else:
                                self.Griper(0)
                            time.sleep(0.1)
                i += 1

    def Griping_Rod(self, point=0):
        """
        Square_Root was move to Griping Rod
        ========================
        Square_Root can move to Griping Rod in some conners\n
        """
        print("Moving to Griping Rod")
        if point == 0:
            print("Conner 0")
            inputtest = [[14, 60, 400, 0], [
                14, 60, 110, 0, 1], [14, 60, 400, 0]]
            self.Path_list(inputtest, Home=True)  # force Go2Home
        if point == 1:
            print("Going to Gragon")
            inputtest = [[0, 147, 400, 0], [
                0, 147, 135, 0, 1], [0, 147, 400, 0],[200,200,400,0]]
            self.Path_list(inputtest, Home=True)  # force Go2Home
        if point == 10:
            print("eiei Conner0")
            inputtest = [[10, 60, 400, 0,1], [
                10, 60, 110, 0, 0], [10, 60, 400, 0 ,0]]
            self.Path_list(inputtest, Home=True)  # force Go2Home
        time.sleep(0.5)

    def Velocity_max(self,velocity_max):
        if velocity_max > 90:
            velocity_max = 90
        if velocity_max <35:
            velocity_max = 35
        buffer = bytes([255, 85, 255, velocity_max & 0x00FF, 0, 0, 0, 0, 0])
        self.ser.write(buffer)

    def Manual_Control(self):
        """
        Square_Root was move by Manual
        ========================
        Read Command from command line\n
        """
        while(True):
            mode = input("\n\t1 for sent new position\n\t2 for read position\n\t3 Control Griper\n\t4 for set Velocity Trajectory Max\n\t9 for Set Home\n\n\tOther for exit from manu\n\nManu : ")
            try:
                if(int(mode) == 1):
                    print("input your position")
                    x_in = int(input("x : \n"))
                    while(x_in > 400):
                        print("Position Out of Range")
                        x_in = int(input("x : \n"))
                    y_in = int(input("y : \n"))
                    while(y_in > 400):
                        print("Position Out of Range")
                        y_in = int(input("y : \n"))
                    z_in = int(input("z : \n"))
                    while(z_in > 400):
                        print("Position Out of Range")
                        z_in = int(input("z : \n"))
                    theta_in = int(input("theta : \n"))
                    while(theta_in > 360 or theta_in < -360):
                        print("Position Out of Range")
                        theta_in = int(input("theta : \n"))
                    self.Move2point(x=x_in, y=y_in, z=z_in, theta=theta_in)
                elif(int(mode) == 2):
                    print(self.Status_point())
                elif(int(mode) == 3):
                    griper_buffer = int(input("Griping Rod? (y or N)").lower() == 'y')
                    self.Griper(griper_buffer)
                elif (int(mode) == 4):
                    velocity_max = int(input("Velocity Trajectory Max (Range 35-90 mm/s) : \n"))
                    self.Velocity_max(velocity_max)
                elif(int(mode) == 9):
                    self.Go2home()
                else:
                    if (input("exit manual mode? (y or N)").lower() == 'y'):
                        break
            except:
                if (input("exit manual mode? (y or N)").lower() == 'y'):
                    break


if __name__ == '__main__':
    try:
        Square_Root = communication(port="com7", baudrate=500000)
        Square_Root.ResetdsPIC()
        Square_Root.Offset(offsetxy=20, offsetz=0)
        # Square_Root.Go2home()
        # Square_Root.Manual_Control()
        # Square_Root.Velocity_max(80)
        Square_Root.Griping_Rod(point = 0)
        inputtest = [[55, 320, 400, 0], [55, 320, 250, 0], [192, 308, 250, 0], [
            315, 98, 150, 60]]
        Square_Root.Path_list(inputtest)
        Square_Root.Griping_Rod(point = 10)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt!!!!\n\n\nShutdown ...\n\n\n\n")