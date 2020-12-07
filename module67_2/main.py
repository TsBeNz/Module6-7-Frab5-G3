import os
from kivy.app import App
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.image import Image, AsyncImage
from kivy.uix.behaviors import ButtonBehavior
import kivy.properties as prop
from kivy.lang import Builder

from utils.get_map import get_map
from utils.get_path import get_path
from utils.M1.capture_map import capture_map, crop_sign
from utils.M1.communication import communication

Builder.load_file('main.kv')
PATH = os.path.dirname(os.path.abspath(__file__))

class ScreenCommand(Screen):
    
    def __init__(self, **kwargs):
        super(ScreenCommand, self).__init__(**kwargs)
        self.hsv_img = 0
        self.area_img = 0
        self.no_sign_img = 0
        self.sk_img = 0
        self.sign_pos = 0
        self.comport = "com4"
        # self.path = [[319.0, 341.0, 0, 0], [319, 227, 180.0, 0], [76.5, 85.0, 0, 0]]
        # self.path = [[319.0, 341.0, 103.53500485105982, 180.0], [319, 227, 120.6768271356224, -121.11609354847653], [70.5, 77.0, 120.6768271356224, -121.11609354847653]]
        self.path = [[319.0, 345.0, 240.99099040031433, 180.0], [319, 285, 240.99099040031433, 180.0], [319, 233, 247.29729890823364, 180.0], [319, 229, 247.29729890823364, -120.96375653207352], [314, 226, 244.5945918560028, -123.69006752597979], [134, 106, 156.30630627274513, -123.9234502658317], [77.5, 68.0, 156.30630627274513, -33.923450265831704]]
    def get_map(self):
        capture_map(communication,comport = self.comport)
        self.hsv_img, self.area_img, self.no_sign_img, self.sk_img, self.sign_pos = get_map()
        self.ids.command_img.source = PATH + '/utils/imgs/ui/get_map.png'
        print('getmap')

    def get_path(self):
        self.path = get_path(self.hsv_img, self.area_img, self.no_sign_img, self.sk_img, self.sign_pos)
        print('get_path')

    def start(self):
        if self.ids.start_stop.text == 'start':
            print('start')
            Square_Root = communication(port=self.comport, baudrate=500000)
            Square_Root.Offset(20)
            Square_Root.Griping_Rod(1)
            Square_Root.Path_list(self.path)
            self.ids.start_stop.text = 'stop'
        else:
            print('stop')
            self.ids.start_stop.text = 'start'

    def clear(self):
        print('clear')

class ScreenShow(Screen):
    def __init__(self, **kwargs):
        super(ScreenShow, self).__init__(**kwargs)
        self.count = 0
        self.comport = "com4"

    def select(self, fileName):
        try:
            self.ids.image.source = fileName[0]
        except:
            pass
    
    def enable_cropping(self):
        if self.ids.start_stop.text == 'enable':
            print(self.ids.image.pos)
            print(self.ids.image.size)
            self.ids.start_stop.text = 'unable'
        else:
            print('unable')
            self.ids.start_stop.text = 'enable'

    def crop(self):
        self.count += 1
        crop_sign(communication, self.count,self.comport,PATH)

    def cancel(self):
        os.remove(PATH + "/utils/imgs/raw_templates/{}.png".format(self.count))
        self.count -= 1

class ScreenSetting(Screen):
    pass

class DevManager(ScreenManager):
    pass

class DevApp(App):
    def build(self):
        return DevManager()

if __name__ == '__main__':
    app = DevApp()
    app.run()