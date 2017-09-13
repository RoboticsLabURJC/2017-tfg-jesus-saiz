import threading
import time
from datetime import datetime
import cv2
import numpy as np
from math import pi

from sensors.cameraFilter import CameraFilter
from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient


time_cycle = 80
detect = False

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra
        self.minError=0.01

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)


    def run (self):

        self.stop_event.clear()

        while (not self.kill_event.is_set()):

            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()

    def execute(self):
       # Add your code here
        global detec

        input_image = self.camera.getImage()

        if input_image is not None:
            blur = cv2.GaussianBlur(input_image, (3, 3), 0)
            color_HSV = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)

            H_max = 4.3*(180/(2*pi))
            H_min = 1.98*(180/(2*pi))
            S_max = 0.8*(255/1)
            S_min = 0.3*(255/1)
            V_max = 255.5
            V_min = 51.81

            bk_image = cv2.inRange(color_HSV, np.array([H_min,S_min,V_min]), np.array([H_max,S_max,V_max]))

            bk_image_cp = np.copy(bk_image)
            image, contours, hierarchy = cv2.findContours(bk_image_cp, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

            if contours is not None:
                cnt = contours[0]
                approx = cv2.approxPolyDP(cnt,0.1*cv2.arcLength(cnt,True),True)
                x,y,w,h = cv2.boundingRect(approx)
                image_contour = cv2.rectangle(blur,(x,y),(x+w,y+h),(0,255,0),2)
                self.camera.setColorImage(image_contour)
                detec = True
            else:
                self.camera.setColorImage(blur)

        if (detec == True):
            print ("Turtle Detected")
#           posicion central de la imagen en el filtro de color
            ini_pos = np.array([160, -120])
            print (ini_pos)
            coord_tur = np.array([x+w/2, -y+h/2])
            print (coord_tur)
            vect_1 = coord_tur - ini_pos
            vel_1 = vect_1*0.003
            print (vel_1)
            if abs(vel_1[0]) < self.minError and abs(vel_1[1]) < self.minError:
                self.cmdvel.sendCMDVel(0,0,0,0,0,0)
                print ("Turtle Stop")
            else:
                self.cmdvel.sendCMDVel(vel_1[0],vel_1[1],0,0,0,0)
                print ("Following turtle")