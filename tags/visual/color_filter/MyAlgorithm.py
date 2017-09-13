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

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

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

        input_image = self.camera.getImage()

        if input_image is not None:
            blur = cv2.GaussianBlur(input_image, (3, 3), 0)
            color_HSV = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)

            H_max = 6.28*(180/(2*pi))
            H_min = 4.19*(180/(2*pi))
            S_max = 1*(255/1)
            S_min = 0.56*(255/1)
            V_max = 255
            V_min = 136.05

    #        bk_image = cv2.inRange(color_HSV, min([H_min,S_min,V_min]), max([H_max,S_max,V_max]))
            bk_image = cv2.inRange(color_HSV, np.array([H_min,S_min,V_min]), np.array([H_max,S_max,V_max]))
    #        bk_image = cv2.inRange(color_HSV, np.array([H_min,100,50]), np.array([H_max,255,255]))
            bk_image_cp = np.copy(bk_image)
            image, contours, hierarchy = cv2.findContours(bk_image_cp, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

            if contours is not None:
                cnt = contours[0]
                approx = cv2.approxPolyDP(cnt,0.1*cv2.arcLength(cnt,True),True)
                x,y,w,h = cv2.boundingRect(approx)
                image_contour = cv2.rectangle(blur,(x,y),(x+w,y+h),(0,255,0),2)
                self.camera.setColorImage(image_contour)
            else:
                self.camera.setColorImage(blur)
            self.camera.setThresholdImage(bk_image)


            '''
            If you want show a threshold image (black and white image)
            self.camera.setThresholdImage(bk_image)
            '''
