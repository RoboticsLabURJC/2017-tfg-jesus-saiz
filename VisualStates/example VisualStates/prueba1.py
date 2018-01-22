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
		self.interfaces.myDroneExtra.takeoff()

class State7(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		# self.interfaces.myDroneExtra.land()
		
		input_image = self.interfaces.myCamera.getImage()
		global initialTime
		global initTime
		if input_image is not None:
		    hsv = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)
		    show_image=input_image+1-1
		    show_image2=input_image+1-1
			
		    lower_orange = np.array([0,131,0], dtype=np.uint8)                                         # 108,220,69 orange, CAMARA DELANTERA REAL 70,110,120 CAMARA ABAJO DRONE 100,100,80
		    upper_orange = np.array([35,255,225], dtype=np.uint8)                                     #120, 255,110 orange,   CAMARA DELANTERA REAL 120, 200,255 CAMARA ABAJO DRONE 150, 255,255
		    maskOrange = cv2.inRange(hsv, lower_orange, upper_orange)
		    maskRGBOrange = cv2.bitwise_and(input_image,input_image, mask= maskOrange)
		    momentsOrange = cv2.moments(maskOrange)
		    areaOrange = momentsOrange['m00']
		    print("AO=", areaOrange)
		
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
		
		    if(-initialTime+time.time()<5 or initTime==0):
		        if(initTime==0):
		            initialTime=time.time()
		            initTime=1
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
		                    if(areaTot>19272135.0):
		                        turnland=1
		                        landed=time.time()
		                    else:
		                        self.cmdvel.sendCMDVel(0,0,-0.5,0,0,0)
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
		
		pass

class State8(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		i = 0
		minError = 0.01
		changeCam = True
		while (len(self.interfaces.beacons) > i):
		    if i == 2 and changeCam:
		        self.interfaces.myDroneExtra.toggleCam()
		        changeCam = False
		    act_pos = self.interfaces.myPose3d.getPose3d()
		    coord_b = self.interfaces.beacons[i]
		    vect_x = coord_b[0] - act_pos.x
		    vect_y = coord_b[1] - act_pos.y
		    vect_z = coord_b[2] - act_pos.z
		    vel_1 = [vect_x, vect_y, vect_z]
		    if abs(vect_x) < minError and abs(vect_y) < minError and abs(vect_z) < minError :
		        self.interfaces.myCMDVel.sendCMDVel(0,0,0,0,0,0)
		        i = i+1
		        if (i < len(self.interfaces.beacons)):
		            print ("Baliza", i)
		        else:
		            print ("Trayectoria Aterrizaje")
		        self.interfaces.myDroneExtra.toggleCam()
		    else:
		        self.interfaces.myCMDVel.sendCMDVel(vel_1[0],vel_1[1],vel_1[2],0,0,0)

class Tran14(TemporalTransition):

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
		self.beacons = [[0,0,2], [2,1,3], [-5,2,3], [-5,9,3], [9,9,3], [9,2,3], [9,-3,3]]
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
		gui.addState(1, "Takeoff", True, 831.0, 926.0, 0)
		gui.addState(7, "Land", False, 1138.0, 930.0, 0)
		gui.addState(8, "Beacons", False, 984.0, 833.0, 0)

		gui.addTransition(14, "transition 14", 1, 8, 907.5, 879.5)
		gui.addTransition(15, "transition 15", 8, 7, 1061.0, 881.5)

	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	state0 = State0(0, True, interfaces, 100, None, gui)
	state1 = State1(1, True, interfaces, 100, state0, gui)
	state7 = State7(7, False, interfaces, 100, state0, gui)
	state8 = State8(8, False, interfaces, 100, state0, gui)

	tran14 = Tran14(14, 8, 5000)
	state1.addTransition(tran14)

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
