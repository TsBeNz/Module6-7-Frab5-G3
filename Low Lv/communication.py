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
    def __init__(self, port="com9", baudrate=115200):
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate)
            self.ser.rts = 0
            self.status = 1
        except:
            print("\n\nCommunication to dsPIC Error !!!!!!\nComport = "+str(port)+" ?\n\n")
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
        buffer = bytes([255, 255, 0,0,0,0,0])
        self.ser.write(buffer)
        
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

    def Move2point(self,x=0, y=0, z=0, theta=0):
        """
        Move By Position Control
        ========================
        Move Robot to position with Position Control
        """
        self.ser.write(bytes([255, 255, x >> 1, (((x & 0x0001) << 7) | (y >> 2)), (((y & 0x0003) << 6) | (z >> 3)), (((z & 0x0007) << 5) | (theta >> 4)), (((theta & 0x000F) << 4) | (0x00 >> 4))]))

    def Path_list(self,Path=[]):
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
                    self.Move2point(Path[i][0],Path[i][1],0,0)
                    i+=1
if __name__ == '__main__':
    try:
        i = 0
        inputtest = []
        PIC=communication()
        while(True):
            i+=3
            x=round(170*(math.sin((i*math.pi / 180)-math.pi/2))+220)-1
            y=round(170*(math.cos((i*math.pi / 180)-math.pi/2))+220)-1
            inputtest.append([x,y,0,0])
            if(i >= 360):
                break
        PIC.Path_list(inputtest)
        inputtest = [[50,400,0,0],[400,400,0,0],[400,50,0,0],[50,50,0,0]]
        PIC.Path_list(inputtest)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt!!!!\n\n\nShutdown ...\n\n\n\n")
