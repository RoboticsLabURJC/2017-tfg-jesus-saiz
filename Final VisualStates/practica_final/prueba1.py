#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys, threading, time, signal
sys.path.append("${CMAKE_INSTALL_PREFIX}/lib/python2.7")
sys.path.append("${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py")
from codegen.python.state import State
from codegen.python.temporaltransition import TemporalTransition
from codegen.python.conditionaltransition import ConditionalTransition
from codegen.python.runtimegui import RunTimeGui
from PyQt5.QtWidgets import QApplication
import config, comm
import cv2
import math
import jderobot
import colorsys
import numpy as np


class State0(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		pass

class State1(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		#self.interfaces.myDroneExtra.takeoff()
		#"""
		time_cycle = 80
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
		turntake=0
		numIteracionesOrange=0
		numIteracionesGreen=0
		
		input_image = self.interfaces.myCamera.getImage()
		
		while input_image is not None and turntake == 0:
		    input_image = self.interfaces.myCamera.getImage()
		    hsv = cv2.cvtColor(input_image.data, cv2.COLOR_BGR2HSV)
		    show_image=input_image.data+1-1
		    show_image2=input_image.data+1-1
			
		    lower_orange = np.array([0,131,0], dtype=np.uint8)                                         # 108,220,69 orange, CAMARA DELANTERA REAL 70,110,120 CAMARA ABAJO DRONE 100,100,80
		    upper_orange = np.array([35,255,225], dtype=np.uint8)                                     #120, 255,110 orange,   CAMARA DELANTERA REAL 120, 200,255 CAMARA ABAJO DRONE 150, 255,255
		    maskOrange = cv2.inRange(hsv, lower_orange, upper_orange)
		    maskRGBOrange = cv2.bitwise_and(input_image.data,input_image.data, mask= maskOrange)
		    momentsOrange = cv2.moments(maskOrange)
		    areaOrange = momentsOrange['m00']
		#    print("AO=", areaOrange)
		
		    lower_green = np.array([42,120,40], dtype=np.uint8) #20,193,65 DELANTERA REAL 10,20,0 ABAJO REAL 0,0,0
		    upper_green = np.array([70,255,255], dtype=np.uint8) #70, 227,100 DELANTERA REAL 40, 200,255 ABAJO REAL 140,100,255
		    maskGreen = cv2.inRange(hsv, lower_green, upper_green)
		    maskRGBGreen = cv2.bitwise_and(input_image.data,input_image.data, mask= maskGreen)
		    momentsGreen = cv2.moments(maskGreen)
		    areaGreen = momentsGreen['m00']
		#    print("AG=", areaGreen)
		
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
		
		    if(-initialTime+time.time()<5 or initTime==0):
		        if(initTime==0):
		            initialTime=time.time()
		            initTime=1
		            self.interfaces.centroImagen(input_image, hsv)
		            self.interfaces.myDroneExtra.takeoff()
		        momentsTot = cv2.moments(maskGreen+maskOrange)
		        areaTot = areaGreen + areaOrange
			# para que no divida entre cero si no empiza desde encima del coche le sumo 0.001
		        xTot = int(momentsTot['m10']/(momentsTot['m00']+0.001))
		        yTot = int(momentsTot['m01']/(momentsTot['m00']+0.001))
		        swi=show_image+1-1
		        getImage,f = self.interfaces.center(show_image,maskRGBOrange,maskRGBGreen)
		        positionXarr=[]
		        self.interfaces.myCMDVel.sendCMDVel(0,0,0.1,0,0,0)
		        if(len(f)>0):
		            positionXarr,positionYarr,show_image = self.interfaces.printAndCoord(getImage,swi,f)
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
		
		                if(y_img-positionYarr[0]<25):
		                    vy=0
		                else:
		                    vy=vytot*velya*1.4
		
		                if(x_img-positionXarr[0]<25):
		                    vx=0
		                else:
		                    vx=vxtot*velxa*1.4
		    # cambio de la velocidad inicial para que no se vaya tan lejos
		                self.interfaces.myCMDVel.sendCMDVel(vy*0.001,vx*0.001,0,0,0,0)
		            else:
		                self.interfaces.myCMDVel.sendCMDVel(0,0,0.1,0,0,0)
		        else:
		            self.interfaces.myCMDVel.sendCMDVel(0,0,0.1,0,0,0)  
		
		    else:
		        global wSearch 
		        global numVuelta
		        global timerW
		        global yanterior
		        global xanterior
		        global var_beacon_status
		#        print(time_cycle, numVuelta, wSearch, timerW, initTime, initialTime, yanterior, xanterior) 
		#        print(yanteriorTot, xanteriorTot, m, x_img, y_img, numIteracionesOrange, numIteracionesGreen)
		        if(areaOrange > 0 and areaGreen == 0):
		            numIteracionesOrange=numIteracionesOrange+1
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
		                    self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
		
		        elif(areaOrange > 0 and areaGreen>0):
		            momentsTot = cv2.moments(maskGreen+maskOrange)
		            areaTot = areaGreen + areaOrange
		            xTot = int(momentsTot['m10']/momentsTot['m00'])
		            yTot = int(momentsTot['m01']/momentsTot['m00'])
		            print("green and orange")
		
		            if((abs(y_img-yTot)<=6 and abs(x_img-xTot)<=6)):
		                if(turntake==0):
		                    if(areaTot<9272135.0):
		                        self.interfaces.myDroneExtra.takeoff()
		                        turntake=1
		                        landed=time.time()
		                    else:
		                        self.interfaces.myCMDVel.sendCMDVel(0,0,0.1,0,0,0)
		                        print(areaTot)
		            elif(landed==0):
		                kernel = np.ones((3,3),np.uint8)
		                maskRGBTot = cv2.erode(maskRGBTot,kernel,iterations =2)
		                maskRGBTot = cv2.dilate(maskRGBTot,kernel,iterations =2)
		
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
		                getImage,f = self.interfaces.center(show_image,maskRGBOrange,maskRGBGreen)
		                show_image4=getImage
		                positionXarr=[]
		                if(len(f) >0):
		                    positionXarr,positionYarr,show_image = self.interfaces.printAndCoord(getImage,swi,f)
		
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
		                        self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
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
		                        self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
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
		                    self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
		        elif(areaOrange == 0 and areaGreen>0):
		            numIteracionesGreen=numIteracionesGreen+1
		            var_beacon_status = 1
		            print("green")
		
		            xGreen = int(momentsGreen['m10']/momentsGreen['m00'])
		            yGreen = int(momentsGreen['m01']/momentsGreen['m00'])
		            if(yanterior==0 and xanterior==0):
		                yanterior = (y_img-yGreen)*0.02
		                xanterior = (x_img-xGreen)*0.02
		                self.interfaces.myCMDVel.sendCMDVel(yanterior*0.5,xanterior*0.5,0,0,0,0)
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
		                    self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
		        else:
		            turntake == 1
		#"""

class State7(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		# self.interfaces.myDroneExtra.land()
		
		time_cycle = 80
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
		
		self.interfaces.myDroneExtra.toggleCam()
		input_image = self.interfaces.myCamera.getImage()
		
		while input_image is not None and turnland == 0:
		    input_image = self.interfaces.myCamera.getImage()
		    hsv = cv2.cvtColor(input_image.data, cv2.COLOR_BGR2HSV)
		    show_image=input_image.data+1-1
		    show_image2=input_image.data+1-1
			
		    lower_orange = np.array([0,131,0], dtype=np.uint8)                                         # 108,220,69 orange, CAMARA DELANTERA REAL 70,110,120 CAMARA ABAJO DRONE 100,100,80
		    upper_orange = np.array([35,255,225], dtype=np.uint8)                                     #120, 255,110 orange,   CAMARA DELANTERA REAL 120, 200,255 CAMARA ABAJO DRONE 150, 255,255
		    maskOrange = cv2.inRange(hsv, lower_orange, upper_orange)
		    maskRGBOrange = cv2.bitwise_and(input_image.data,input_image.data, mask= maskOrange)
		    momentsOrange = cv2.moments(maskOrange)
		    areaOrange = momentsOrange['m00']
		#    print("AO=", areaOrange)
		
		    lower_green = np.array([42,120,40], dtype=np.uint8) #20,193,65 DELANTERA REAL 10,20,0 ABAJO REAL 0,0,0
		    upper_green = np.array([70,255,255], dtype=np.uint8) #70, 227,100 DELANTERA REAL 40, 200,255 ABAJO REAL 140,100,255
		    maskGreen = cv2.inRange(hsv, lower_green, upper_green)
		    maskRGBGreen = cv2.bitwise_and(input_image.data,input_image.data, mask= maskGreen)
		    momentsGreen = cv2.moments(maskGreen)
		    areaGreen = momentsGreen['m00']
		#    print("AG=", areaGreen)
		
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
		
		    if(-initialTime+time.time()<5 or initTime==0):
		        if(initTime==0):
		            initialTime=time.time()
		            initTime=1
		            self.interfaces.centroImagen(input_image, hsv)
		
		        momentsTot = cv2.moments(maskGreen+maskOrange)
		        areaTot = areaGreen + areaOrange
			# para que no divida entre cero si no empiza desde encima del coche le sumo 0.001
		        xTot = int(momentsTot['m10']/(momentsTot['m00']+0.001))
		        yTot = int(momentsTot['m01']/(momentsTot['m00']+0.001))
		        swi=show_image+1-1
		        getImage,f = self.interfaces.center(show_image,maskRGBOrange,maskRGBGreen)
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
		
		                if(y_img-positionYarr[0]<25):
		                    vy=0
		                else:
		                    vy=vytot*velya*1.4
		
		                if(x_img-positionXarr[0]<25):
		                    vx=0
		                else:
		                    vx=vxtot*velxa*1.4
		    # cambio de la velocidad inicial para que no se vaya tan lejos
		                self.interfaces.myCMDVel.sendCMDVel(vy*0.001,vx*0.001,0,0,0,0)
		            else:
		                self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
		        else:
		            self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)  
		
		    else:
		        global wSearch 
		        global numVuelta
		        global timerW
		        global yanterior
		        global xanterior
		        global var_beacon_status
		#        print(time_cycle, numVuelta, wSearch, timerW, initTime, initialTime, yanterior, xanterior) 
		#        print(yanteriorTot, xanteriorTot, m, x_img, y_img, numIteracionesOrange, numIteracionesGreen)
		        if(areaOrange > 0 and areaGreen == 0):
		            numIteracionesOrange=numIteracionesOrange+1
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
		                    self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
		
		        elif(areaOrange > 0 and areaGreen>0):
		            momentsTot = cv2.moments(maskGreen+maskOrange)
		            areaTot = areaGreen + areaOrange
		            xTot = int(momentsTot['m10']/momentsTot['m00'])
		            yTot = int(momentsTot['m01']/momentsTot['m00'])
		            print("green and orange")
		
		            if((abs(y_img-yTot)<=6 and abs(x_img-xTot)<=6)):
		                if(turnland==0):
		                    if(areaTot>10272135.0):
		                        self.interfaces.myDroneExtra.land()
		                        turnland=1
		                        landed=time.time()
		                    else:
		                        self.interfaces.myCMDVel.sendCMDVel(0,0,-0.3,0,0,0)
		                        print(areaTot)
		            elif(landed==0):
		                kernel = np.ones((3,3),np.uint8)
		                maskRGBTot = cv2.erode(maskRGBTot,kernel,iterations =2)
		                maskRGBTot = cv2.dilate(maskRGBTot,kernel,iterations =2)
		
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
		                getImage,f = self.interfaces.center(show_image,maskRGBOrange,maskRGBGreen)
		                show_image4=getImage
		                positionXarr=[]
		                if(len(f) >0):
		                    positionXarr,positionYarr,show_image = self.interfaces.printAndCoord(getImage,swi,f)
		
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
		                        self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
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
		                        self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
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
		                    self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
		        elif(areaOrange == 0 and areaGreen>0):
		            numIteracionesGreen=numIteracionesGreen+1
		            var_beacon_status = 1
		            print("green")
		
		            xGreen = int(momentsGreen['m10']/momentsGreen['m00'])
		            yGreen = int(momentsGreen['m01']/momentsGreen['m00'])
		            if(yanterior==0 and xanterior==0):
		                yanterior = (y_img-yGreen)*0.02
		                xanterior = (x_img-xGreen)*0.02
		                self.interfaces.myCMDVel.sendCMDVel(yanterior*0.5,xanterior*0.5,0,0,0,0)
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
		                    self.interfaces.myCMDVel.sendCMDVel(vytot*0.5,vxtot*0.5,0,0,0,0)
		
		        else:
		            numVuelta=numVuelta+1
		            if(numVuelta>100 and numVuelta < 120):
		                self.interfaces.myCMDVel.sendCMDVel(1.8+wSearch,0,0,0,0,-1.5)
		                timerW=timerW+(timerW/8)
		                if(numVuelta==119):
		                    numVuelta=0
		                if(wSearch<1 and numVuelta==101):
		                    wSearch=wSearch+0.2
		                    numIteracionesGreen=0
		                    numIteracionesOrange=0
		                else:
		                    self.interfaces.myCMDVel.sendCMDVel(0.2+wSearch,0,0,0,0,0.55 - wSearch)
		

class State8(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		i = 0
		minError = 0.02
		changeCam = True
		while (len(self.interfaces.beacons) > i):
		    if i == 0 and changeCam:
		        self.interfaces.myDroneExtra.toggleCam()
		        changeCam = False
		    act_pos = self.interfaces.myPose3d.getPose3d()
		    coord_b = self.interfaces.beacons[i]
		    vect_x = (coord_b[0] - act_pos.x)*0.05
		    vect_y = (coord_b[1] - act_pos.y)*0.05
		    vect_z = (coord_b[2] - act_pos.z)*0.05
		    vel_1 = [vect_x, vect_y, vect_z]
		    if abs(vect_x) < minError and abs(vect_y) < minError and abs(vect_z) < minError :
		        self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
		        i = i+1
		        if (i < len(self.interfaces.beacons)):
		            print ("Baliza", i)
		        elif (i > len(self.interfaces.beacons)):
		            self.interfaces.myDroneExtra.toggleCam()
		            changeCam = True
		        else:
		            print ("Trayectoria Aterrizaje")
		    else:
		        self.interfaces.myCMDVel.sendCMDVel(vel_1[0],vel_1[1],vel_1[2],0,0,0)

class State9(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		pass

class Tran19(TemporalTransition):

	def runCode(self):
		pass

class Tran16(TemporalTransition):

	def runCode(self):
		pass

class Tran15(TemporalTransition):

	def runCode(self):
		pass

class Interfaces():
	def __init__(self):
		self.jdrc = None
		self.myCamera = None
		self.myCMDVel = None
		self.myPose3d = None
		self.myDroneExtra = None
		self.one_points = [[0,0], [1,1], [1,-4]]
		self.zero_points = [[2,-4], [4,-4], [4,1], [2,1]]
		self.closing_loop = False
		self.beacons = [[0,0,2], [2,1,3], [-5,2,3], [-5,12,3], [2,12,3], [10.7,12,3], [10,1,2.5]]
		#self.time_cycle = 80
		#self.numVuelta=50
		#self.wSearch=0
		#self.timerW=10
		#self.initTime=0
		#self.initialTime=0
		#self.yanterior=0
		#self.xanterior=0
		#self.yanteriorTot=0
		#self.xanteriorTot=0
		#self.m=0
		#self.x_img=0
		#self.y_img=0
		#self.landed=0
		#self.turnland=0
		#self.numIteracionesOrange=0
		#self.numIteracionesGreen=0

		self.connectProxies()

	def connectProxies(self):
		cfg = config.load(sys.argv[1])
		self.jdrc = comm.init(cfg, "prueba1")
		self.myCamera = self.jdrc.getCameraClient("prueba1.myCamera")
		if not self.myCamera:
			raise Exception("could not create client with name:myCamera")
		print("myCamera is connected")
		self.myCMDVel = self.jdrc.getCMDVelClient("prueba1.myCMDVel")
		if not self.myCMDVel:
			raise Exception("could not create client with name:myCMDVel")
		print("myCMDVel is connected")
		self.myPose3d = self.jdrc.getPose3dClient("prueba1.myPose3d")
		if not self.myPose3d:
			raise Exception("could not create client with name:myPose3d")
		print("myPose3d is connected")
		self.myDroneExtra = self.jdrc.getArDroneExtraClient("prueba1.myDroneExtra")
		if not self.myDroneExtra:
			raise Exception("could not create client with name:myDroneExtra")
		print("myDroneExtra is connected")

	def destroyProxies(self):
		if self.jdrc is not None:
			self.jdrc.destroy()

	def centroImagen(self,input_image,hsv):

	    print("CALCULA EL CENTRO DE LA IMAGEN")

	    m=input_image.data+1-1

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

	

displayGui = False
guiThread = None
gui = None
state0 = None

def signal_handler(signal, frame):
	global gui
	print("SIGINT is captured. The program exits")
	if gui is not None:
		gui.close()
	global state0
	state0.stop()

def readArgs():
	global displayGui
	for arg in sys.argv:
		splitedArg = arg.split('=')
		if splitedArg[0] == '--displaygui':
			if splitedArg[1] == 'True' or splitedArg[1] == 'true':
				displayGui = True
				print('runtime gui enabled')
			else:
				displayGui = False
				print('runtime gui disabled')

def runGui():
	global gui
	app = QApplication(sys.argv)
	gui = RunTimeGui()
	gui.show()
	app.exec_()

if __name__ == "__main__":
	interfaces = Interfaces()

	readArgs()
	if displayGui:
		guiThread = threading.Thread(target=runGui)
		guiThread.start()


	if displayGui:
		while(gui is None):
			time.sleep(0.1)

		gui.addState(0, "root", True, 0.0, 0.0, None)
		gui.addState(1, "Precise Takeoff", True, 889.0, 968.0, 0)
		gui.addState(7, "Precise Landing", False, 1173.0, 951.0, 0)
		gui.addState(8, "Beacons", False, 1017.0, 824.0, 0)
		gui.addState(9, "Landed", False, 1042.0, 1086.0, 0)

		gui.addTransition(19, "transition 1", 1, 8, 910.0, 876.5)
		gui.addTransition(16, "transition 3", 7, 9, 1130.0, 1012.5)
		gui.addTransition(15, "transition 2", 8, 7, 1117.0, 870.5)

	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	state0 = State0(0, True, interfaces, 100, None, gui)
	state1 = State1(1, True, interfaces, 100, state0, gui)
	state7 = State7(7, False, interfaces, 100, state0, gui)
	state8 = State8(8, False, interfaces, 100, state0, gui)
	state9 = State9(9, False, interfaces, 100, state0, gui)

	tran19 = Tran19(19, 8, 5000)
	state1.addTransition(tran19)

	tran16 = Tran16(16, 9, 2000)
	state7.addTransition(tran16)

	tran15 = Tran15(15, 7, 5000)
	state8.addTransition(tran15)

	try:
		state0.startThread()
		signal.signal(signal.SIGINT, signal_handler)
		signal.pause()
		state0.join()
		if displayGui:
			guiThread.join()

		interfaces.destroyProxies()
	except:
		state0.stop()
		if displayGui:
			gui.close()
			guiThread.join()

		state0.join()
		interfaces.destroyProxies()
		sys.exit(1)
