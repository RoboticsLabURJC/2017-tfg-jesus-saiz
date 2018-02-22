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
		

class Tran16(TemporalTransition):

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
		self.jdrc = comm.init(cfg, "prueba_cam")
		self.myCamera = self.jdrc.getCameraClient("prueba_cam.myCamera")
		if not self.myCamera:
			raise Exception("could not create client with name:myCamera")
		print("myCamera is connected")
		self.myCMDVel = self.jdrc.getCMDVelClient("prueba_cam.myCMDVel")
		if not self.myCMDVel:
			raise Exception("could not create client with name:myCMDVel")
		print("myCMDVel is connected")
		self.myPose3d = self.jdrc.getPose3dClient("prueba_cam.myPose3d")
		if not self.myPose3d:
			raise Exception("could not create client with name:myPose3d")
		print("myPose3d is connected")
		self.myDroneExtra = self.jdrc.getArDroneExtraClient("prueba_cam.myDroneExtra")
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
		gui.addState(7, "Color Filter", False, 1138.0, 930.0, 0)

		gui.addTransition(16, "transition 16", 1, 7, 984.5, 928.0)

	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	state0 = State0(0, True, interfaces, 100, None, gui)
	state1 = State1(1, True, interfaces, 100, state0, gui)
	state7 = State7(7, False, interfaces, 100, state0, gui)

	tran16 = Tran16(16, 7, 5000)
	state1.addTransition(tran16)

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
