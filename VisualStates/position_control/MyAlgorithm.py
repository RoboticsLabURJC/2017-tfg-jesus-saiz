import threading
import time
from datetime import datetime
import cv2
import numpy as np

import math
from math import pi
import jderobot
import colorsys
from PIL import Image, ImageOps
from Beacon import Beacon
from enum import Enum

from matplotlib import pyplot as plt

from parallelIce.cameraClient import CameraClient
from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient

from gui.machine import Machine

time_cycle = 80
i = 0
numVuelta=50

wSearch=0

timerW=10

initTime=0
initialTime=0

yanterior=0
xanterior=0

yanteriorTot=0
xanteriorTot=0
m=0
x_img=0
y_img=0


landed=0
turnland=0

numIteracionesOrange=0
numIteracionesGreen=0

changeCam = False;

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, machine, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

        self.beacons=[]
        self.initBeacons()
        self.minError=0.01

        self.machine = machine

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def initBeacons(self):
        self.beacons.append(Beacon('despegue',jderobot.Pose3DData(0,0,2,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('pos1',jderobot.Pose3DData(2,1,3,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('pos1',jderobot.Pose3DData(2,1,3,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('pos2',jderobot.Pose3DData(-5,2,3,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('pos2',jderobot.Pose3DData(-5,2,3,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('pos3',jderobot.Pose3DData(-5,9,3,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('pos4',jderobot.Pose3DData(9,9,3,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('pos5',jderobot.Pose3DData(9,2,3,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('aterrizaje',jderobot.Pose3DData(9,-3,3,0,0,0,0,0),False,False))

    def getNextBeacon(self):
        for beacon in self.beacons:
            if beacon.isReached() == False:
                return beacon
        return None

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

    def centroImagen(self, input_image,hsv):
        print("CALCULA EL CENTRO DE LA IMAGEN")
        m=input_image+1-1
        lower_img = np.array([0,0,0], dtype=np.uint8)
        upper_img = np.array([255,255,255], dtype=np.uint8)
        centroimg = cv2.inRange(hsv, lower_img, upper_img)
        momentsimg=cv2.moments(centroimg)
        areaimg= momentsimg['m00']
        global x_img
        global y_img
        x_img = int(momentsimg['m10']/momentsimg['m00'])
        y_img = int(momentsimg['m01']/momentsimg['m00'])

    def center(self,show_image,maskRGBOrange,maskRGBGreen):
        f = []
        i=0

        imgray2 = cv2.cvtColor(maskRGBOrange,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray2,255,255,255)
        _,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        areas = [cv2.contourArea(c) for c in contours]
        for extension in areas:
            if extension > 4000:
                img = np.zeros((y_img*2,x_img*2,3), np.uint8)
                actual = contours[i]
                approx = cv2.approxPolyDP(actual,0.05*cv2.arcLength(actual,True),True)
                cv2.drawContours(img,[actual],0,(0,30,0),12)
                f.append(img)
                i=i+1

        i=0
        imgray3 = cv2.cvtColor(maskRGBGreen,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray3,255,255,255)
        _,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        areas = [cv2.contourArea(c) for c in contours]
        for extension in areas:
           if extension > 4000:
               img = np.zeros((y_img*2,x_img*2,3), np.uint8)
               actual = contours[i]
               approx = cv2.approxPolyDP(actual,0.05*cv2.arcLength(actual,True),True)
               cv2.drawContours(img,[actual],0,(0,30,0),12)
               f.append(img)
               i=i+1

        kernel = np.ones((5,5),np.uint8)
        show_image2=show_image+1-1
        if(len(f)>0):
            f[0] = cv2.dilate(f[0],kernel,iterations = 4)
            show_image2=f[0]
            for k in range(len(f)-1):
                f[k+1] = cv2.dilate(f[k+1],kernel,iterations = 4)
                show_image2=show_image2+f[k+1]

        return show_image2,f

    def printAndCoord(self, show_image2,show_image,f):
                lower_green = np.array([0,80,0], dtype=np.uint8)
                upper_green = np.array([0, 255,0], dtype=np.uint8)
                maskSHI = cv2.inRange(show_image2, lower_green, upper_green)
                show_image2 = cv2.bitwise_and(show_image2,show_image2, mask= maskSHI)

                compare_image = np.zeros((y_img*2,x_img*2,3), np.uint8)
                diff_total = cv2.absdiff(compare_image, show_image2)

                imagen_gris = cv2.cvtColor(diff_total, cv2.COLOR_BGR2GRAY)
                _,contours,_ = cv2.findContours(imagen_gris,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

                positionXarr=[]
                positionYarr=[]
                positionX=-20
                positionY=-20
                for c in contours:
                    if(cv2.contourArea(c) >= 0):
                        posicion_x,posicion_y,ancho,alto = cv2.boundingRect(c)
                        cv2.rectangle(show_image,(posicion_x,posicion_y),(posicion_x+ancho,posicion_y+alto),(0,0,255),2)

                        positionX= (posicion_x+posicion_x+ancho)/2
                        positionY= (posicion_y+posicion_y+ancho)/2

                        positionXarr.append(positionX)
                        positionYarr.append(positionY)


                show_image2=f[0]
                for k in range(len(f)-1):
                    show_image2=show_image2+f[k+1]

                lower_green = np.array([0,90,0], dtype=np.uint8)
                upper_green = np.array([0, 255,0], dtype=np.uint8)
                maskSHI = cv2.inRange(show_image2, lower_green, upper_green)
                show_image2 = cv2.bitwise_and(show_image2,show_image2, mask= maskSHI)

                compare_image = np.zeros((y_img*2,x_img*2,3), np.uint8)
                diff_total = cv2.absdiff(compare_image, show_image2)

                imagen_gris = cv2.cvtColor(diff_total, cv2.COLOR_BGR2GRAY)
                _,contours,_ = cv2.findContours(imagen_gris,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

                for c in contours:
                    if(cv2.contourArea(c) >= 1000):
                        posicion_x,posicion_y,ancho,alto = cv2.boundingRect(c)
                        cv2.rectangle(show_image,(posicion_x,posicion_y),(posicion_x+ancho,posicion_y+alto),(255,0,0),2)

                return positionXarr,positionYarr, show_image

    def execute(self):
        global i
        while (len(self.beacons) > i):
            if i == 0:
                self.extra.takeoff()
                self.machine.setStateActive(0, True)
                i = i+1
            if i == 2:
                self.extra.toggleCam()
                i = i+1
            act_pos = np.array([self.pose.getPose3D().x, self.pose.getPose3D().y, self.pose.getPose3D().z])
            coord_b1 = np.array([self.beacons[i].getPose().x, self.beacons[i].getPose().y, self.beacons[i].getPose().z])
            vect_1 = coord_b1 - act_pos
            vel_1 = vect_1
            if abs(vect_1[0]) < self.minError and abs(vect_1[1]) < self.minError and abs(vect_1[2]) < self.minError :
                self.cmdvel.sendCMDVel(0,0,0,0,0,0)
                i = i+1
                if (i < len(self.beacons)):
                    print (self.beacons[i].getId(), i)
                else:
                    print ("Trayectoria Aterrizaje")
                    self.extra.toggleCam()
            else:
                self.cmdvel.sendCMDVel(vel_1[0],vel_1[1],vect_1[2],0,0,0)
        if (len(self.beacons) == i):
            input_image = self.camera.getImage()
            global initialTime
            global initTime

            if input_image is not None:
                hsv = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)
                show_image=input_image+1-1
                show_image2=input_image+1-1
                # define range of orange and green color in HSV
                # H_max = 0.12*(180/(2*pi))
                # H_min = 0.08*(180/(2*pi))
                # S_max = 1.1*(255/1)
                # S_min = 0.8*(255/1)
                # V_max = 255
                # V_min = 0
                lower_orange = np.array([0,131,0], dtype=np.uint8)                                         # 108,220,69 orange, CAMARA DELANTERA REAL 70,110,120 CAMARA ABAJO DRONE 100,100,80
                upper_orange = np.array([35,255,225], dtype=np.uint8)                                     #120, 255,110 orange,   CAMARA DELANTERA REAL 120, 200,255 CAMARA ABAJO DRONE 150, 255,255
                maskOrange = cv2.inRange(hsv, lower_orange, upper_orange)
                maskRGBOrange = cv2.bitwise_and(input_image,input_image, mask= maskOrange)
                momentsOrange = cv2.moments(maskOrange)
                areaOrange = momentsOrange['m00']
                print("AO=", areaOrange)

                # H_max = 0.3*(180/(2*pi))
                # H_min = 0.2*(180/(2*pi))
                # S_max = 0.56*(255/1)
                # S_min = 0.3*(255/1)
                # V_max = 255
                # V_min = 0
                lower_green = np.array([42,120,40], dtype=np.uint8) #20,193,65 DELANTERA REAL 10,20,0 ABAJO REAL 0,0,0
                upper_green = np.array([70,255,255], dtype=np.uint8) #70, 227,100 DELANTERA REAL 40, 200,255 ABAJO REAL 140,100,255
                maskGreen = cv2.inRange(hsv, lower_green, upper_green)
                maskRGBGreen = cv2.bitwise_and(input_image,input_image, mask= maskGreen)
                momentsGreen = cv2.moments(maskGreen)
                areaGreen = momentsGreen['m00']
                print("AG=", areaGreen)

                kernel = np.ones((3,3),np.uint8)

                maskRGBOrange = cv2.erode(maskRGBOrange,kernel,iterations = 4)
                maskRGBOrange = cv2.dilate(maskRGBOrange,kernel,iterations = 3)

                maskRGBGreen = cv2.erode(maskRGBGreen,kernel,iterations = 4)
                maskRGBGreen = cv2.dilate(maskRGBGreen,kernel,iterations = 3)

                kp=0.01
                kd=0.003

                maskRGBTot = maskRGBOrange+maskRGBGreen
                global x_img
                global y_img
                global yanterior
                global xanterior
                global landed
                global yanteriorTot
                global xanteriorTot
                global numIteracionesOrange
                global numIteracionesGreen
                if(time.time()-landed>4.5 and landed!=0):
                    self.machine.setStateActive(5, True)
                elif(-initialTime+time.time()<5 or initTime==0):
                    if(initTime==0):
                        initialTime=time.time()
                        initTime=1
                        # self.extra.takeoff()
                        self.centroImagen(input_image, hsv)

                    momentsTot = cv2.moments(maskGreen+maskOrange)
                    areaTot = areaGreen + areaOrange
# para que no divida entre cero si no empiza desde encima del coche le sumo 0.001
                    xTot = int(momentsTot['m10']/(momentsTot['m00']+0.001))
                    yTot = int(momentsTot['m01']/(momentsTot['m00']+0.001))
                    swi=show_image+1-1
                    getImage,f = self.center(show_image,maskRGBOrange,maskRGBGreen)
                    positionXarr=[]
                    if(len(f)>0):
                        positionXarr,positionYarr,show_image = self.printAndCoord(getImage,swi,f)
                    if(len(positionXarr)>0):
                        if(positionXarr[0] != -20 and positionYarr[0]!=-20):
                            vely = (y_img-positionYarr[0])
                            velx = (x_img-positionXarr[0])

                            vytot= vely*kp #0.01
                            vxtot= velx*kp #0.01

                            velxa=1-abs(xanterior-velx)/50 #10
                            if(velxa<0.1):
                                velxa=0.1

                            velya=1-abs(yanterior-vely)/50 #10
                            if(velya<0.1):
                                velya=0.1

                            #self.cmdvel.sendCMDVel(0,0,0,0,0,0)
                            if(y_img-positionYarr[0]<25):
                                vy=0
                            else:
                                vy=vytot*velya*1.4

                            if(x_img-positionXarr[0]<25):
                                vx=0
                            else:
                                vx=vxtot*velxa*1.4
    # cambio de la velocidad inicial para que no se vaya tan lejos
                            self.cmdvel.sendCMDVel(vy*0.001,vx*0.001,0,0,0,0)
                        else:
                            self.cmdvel.sendCMDVel(0,0,0,0,0,0)
                    else:
                        self.cmdvel.sendCMDVel(0,0,0,0,0,0)
    # cambio de la velocidad inicial para que no se vaya tan lejos
                    # self.machine.setStateActive(0, True)
                elif(-initialTime+time.time()>5 and -initialTime+time.time()<15):

                    self.cmdvel.sendCMDVel(0,0.5,0,0,0,0)
                    yanterior=0
                    xanterior=0
                else:
                    global wSearch
                    global numVuelta
                    global timerW
                    global yanterior
                    global xanterior
                    global var_beacon_status

                    if(areaOrange > 0 and areaGreen == 0):
                        numIteracionesOrange=numIteracionesOrange+1
                        self.machine.setStateActive(2, True)
                        print("orange")
                        xOrange = int(momentsOrange['m10']/momentsOrange['m00'])
                        yOrange = int(momentsOrange['m01']/momentsOrange['m00'])

                        vely = (y_img-yOrange)
                        velx = (x_img-xOrange)

                        vytot= vely*kp
                        vxtot= velx*kp

                        velxa=abs(xanterior-velx)*kd
                        velya=abs(yanterior-vely)*kd

                        vytot=(vytot+velya)
                        vxtot=(vxtot+velxa)

                        if(abs(vxtot-xanteriorTot)>0.3):
                            if(vxtot<xanteriorTot):
                                 vxtot = xanteriorTot-0.3
                            else:
                                vxtot = xanteriorTot+0.3

                            if(abs(vytot-yanteriorTot)>0.3):
                                 if(vytot<yanteriorTot):
                                     vytot = yanteriorTot-0.3
                                 else:
                                     vytot = yanteriorTot+0.3
                                 yanteriorTot=vytot
                                 xanteriorTot=vxtot
                                 self.cmdvel.sendCMDVel(vytot,vxtot,0,0,0,0)

                    elif(areaOrange > 0 and areaGreen>0):
                            momentsTot = cv2.moments(maskGreen+maskOrange)
                            areaTot = areaGreen + areaOrange
                            xTot = int(momentsTot['m10']/momentsTot['m00'])
                            yTot = int(momentsTot['m01']/momentsTot['m00'])
                            print("green and orange")

                            if((abs(y_img-yTot)<=6 and abs(x_img-xTot)<=6)):
                                global turnland
                                self.extra.land()
                                if(turnland==0):
                                    self.machine.setStateActive(4, True)
                                    if(areaTot>19272135.0):
                                        turnland=1
                                        landed=time.time()
                                    else:
                                        self.cmdvel.sendCMDVel(0,0,-0.5,0,0,0)
                                else:
                                    if(time.time()-landed>4.5):
                                        self.machine.setStateActive(5, True)
                                    else:
                                        self.machine.setStateActive(4, True)
                            elif(landed==0):
                                kernel = np.ones((3,3),np.uint8)
                                maskRGBTot = cv2.erode(maskRGBTot,kernel,iterations =2)
                                maskRGBTot = cv2.dilate(maskRGBTot,kernel,iterations =2)

                                self.machine.setStateActive(3, True)

                                vely = (y_img-yTot)
                                velx = (x_img-xTot)

                                vytot= vely*kp
                                vxtot= velx*kp

                                velxa=1-abs(xanterior-velx)/10
                                if(velxa<0.1):
                                    velxa=0.1

                                velya=1-abs(yanterior-vely)/10
                                if(velya<0.1):
                                    velya=0.1


                                yanterior = y_img-yTot
                                xanterior = x_img-xTot

                                swi=show_image+1-1
                                getImage,f = self.center(show_image,maskRGBOrange,maskRGBGreen)
                                show_image4=getImage
                                positionXarr=[]
                                if(len(f) >0):
                                    positionXarr,positionYarr,show_image = self.printAndCoord(getImage,swi,f)

                                blank_image = np.zeros((y_img*2,x_img*2,3), np.uint8)

                                positionX = -20
                                positionY = -20
                                if(len(positionXarr)>0):
                                    positionX=positionXarr[0]
                                    positionY=positionYarr[0]

                                    if(positionX != 0 ):
                                        vely = (y_img-positionYarr[0])
                                        velx = (x_img-positionXarr[0])

                                        vytot= vely*kp
                                        vxtot= velx*kp

                                        velxa=abs(xanterior-velx)*kd
                                        velya=abs(yanterior-vely)*kd


                                        if(abs(vxtot-xanteriorTot)>0.3):
                                           if(vxtot<xanteriorTot):
                                              vxtot = xanteriorTot-0.3
                                           else:
                                              vxtot = xanteriorTot+0.3

                                        if(abs(vytot-yanteriorTot)>0.3):
                                           if(vytot<yanteriorTot):
                                              vytot = yanteriorTot-0.3
                                           else:
                                              vytot = yanteriorTot+0.3
                                        yanterior=velya
                                        xanterior=velxa
                                        self.cmdvel.sendCMDVel(vytot,vxtot,0,0,0,0)
                                        yanteriorTot=vytot
                                        xanteriorTot=vxtot
                                    else:
                                        if(abs(vxtot-xanteriorTot)>0.3):
                                           if(vxtot<xanteriorTot):
                                              vxtot = xanteriorTot-0.3
                                           else:
                                              vxtot = xanteriorTot+0.3

                                        if(abs(vytot-yanteriorTot)>0.3):
                                           if(vytot<yanteriorTot):
                                              vytot = yanteriorTot-0.3
                                           else:
                                              vytot = yanteriorTot+0.3
                                        yanterior=velya
                                        xanterior=velxa
                                        self.cmdvel.sendCMDVel(vytot,vxtot,0,0,0,0)
                                        yanteriorTot=vytot
                                        xanteriorTot=vxtot

                                else:
                                    velxa=abs(xanterior-velx)*kd
                                    velya=abs(yanterior-vely)*kd
                                    print(vytot+velya)
                                    vytot=(vytot+velya)
                                    vxtot=(vxtot+velxa)


                                    if(abs(vxtot-xanteriorTot)>0.3):
                                       if(vxtot<xanteriorTot):
                                          vxtot = xanteriorTot-0.3
                                       else:
                                          vxtot = xanteriorTot+0.3

                                    if(abs(vytot-yanteriorTot)>0.3):
                                       if(vytot<yanteriorTot):
                                          vytot = yanteriorTot-0.3
                                       else:
                                          vytot = yanteriorTot+0.3
                                    yanteriorTot=vytot
                                    xanteriorTot=vxtot
                                    self.cmdvel.sendCMDVel(vytot,vxtot,0,0,0,0)
                    elif(areaOrange == 0 and areaGreen>0):
                        numIteracionesGreen=numIteracionesGreen+1
                        self.machine.setStateActive(2, True)
                        var_beacon_status = 1
                        print("green")

                        xGreen = int(momentsGreen['m10']/momentsGreen['m00'])
                        yGreen = int(momentsGreen['m01']/momentsGreen['m00'])
                        if(yanterior==0 and xanterior==0):
                            yanterior = (y_img-yGreen)*0.02
                            xanterior = (x_img-xGreen)*0.02
                            self.cmdvel.sendCMDVel(yanterior,xanterior,0,0,0,0)
                            vely = yanterior
                            velx = xanterior
                        else:
                            vely = (y_img-yGreen)
                            velx = (x_img-xGreen)

                        vytot= vely*kp
                        vxtot= velx*kp

                        velxa=abs(xanterior-velx)*kd
                        velya=abs(yanterior-vely)*kd

                        vytot=(vytot+velya)
                        vxtot=(vxtot+velxa)

                        if(abs(vxtot-xanteriorTot)>0.3):
                            if(vxtot<xanteriorTot):
                                 vxtot = xanteriorTot-0.3
                            else:
                                vxtot = xanteriorTot+0.3

                            if(abs(vytot-yanteriorTot)>0.3):
                                 if(vytot<yanteriorTot):
                                     vytot = yanteriorTot-0.3
                                 else:
                                     vytot = yanteriorTot+0.3
                                 yanteriorTot=vytot
                                 xanteriorTot=vxtot
                                 self.cmdvel.sendCMDVel(vytot,vxtot,0,0,0,0)

                    else:
                        self.machine.setStateActive(1, True)
                        numVuelta=numVuelta+1
                        if(numVuelta>100 and numVuelta < 120):
                            self.cmdvel.sendCMDVel(1.8+wSearch,0,0,0,0,-1.5)
                            timerW=timerW+(timerW/8)
                            if(numVuelta==119):
                                numVuelta=0
                            if(wSearch<1 and numVuelta==101):
                               wSearch=wSearch+0.2
                               numIteracionesGreen=0
                               numIteracionesOrange=0
                        else:
                            self.cmdvel.sendCMDVel(1.8+wSearch,0,0,0,0,1.5 - wSearch)


                # self.camera.setColorImage(maskGreen)

        pass
