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

class State2(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		pass

class State3(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		pass

class State7(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.myDroneExtra.land()

class State4(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.moveToPoint(self.interfaces.one_points[0][0], self.interfaces.one_points[0][1])

class State5(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.moveToPoint(self.interfaces.one_points[1][0], self.interfaces.one_points[1][1])

class State6(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.moveToPoint(self.interfaces.one_points[2][0], self.interfaces.one_points[2][1])

class State8(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.moveToPoint(self.interfaces.zero_points[0][0], self.interfaces.zero_points[0][1])

class State9(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.moveToPoint(self.interfaces.zero_points[1][0], self.interfaces.zero_points[1][1])

class State10(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.moveToPoint(self.interfaces.zero_points[2][0], self.interfaces.zero_points[2][1])

class State11(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.moveToPoint(self.interfaces.zero_points[3][0], self.interfaces.zero_points[3][1])

class State12(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.closing_loop = True
		self.interfaces.moveToPoint(self.interfaces.zero_points[0][0], self.interfaces.zero_points[0][1])

class Tran1(TemporalTransition):

	def runCode(self):
		pass

class Tran2(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.reachedToPoint(self.interfaces.one_points[2][0], self.interfaces.one_points[2][1])

	def runCode(self):
		pass

class Tran3(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.reachedToPoint(self.interfaces.zero_points[0][0], self.interfaces.zero_points[0][1]) and self.interfaces.closing_loop

	def runCode(self):
		pass

class Tran7(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.reachedToPoint(self.interfaces.one_points[0][0], self.interfaces.one_points[0][1])

	def runCode(self):
		pass

class Tran8(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.reachedToPoint(self.interfaces.one_points[1][0], self.interfaces.one_points[1][1])

	def runCode(self):
		pass

class Tran9(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.reachedToPoint(self.interfaces.zero_points[0][0], self.interfaces.zero_points[0][1])

	def runCode(self):
		pass

class Tran10(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.reachedToPoint(self.interfaces.zero_points[1][0], self.interfaces.zero_points[1][1])

	def runCode(self):
		pass

class Tran11(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.reachedToPoint(self.interfaces.zero_points[2][0], self.interfaces.zero_points[2][1])

	def runCode(self):
		pass

class Tran12(ConditionalTransition):
	def __init__(self, id, destinationId, interfaces):
		ConditionalTransition.__init__(self, id, destinationId)
		self.interfaces = interfaces

	def checkCondition(self):
		return self.interfaces.reachedToPoint(self.interfaces.zero_points[3][0], self.interfaces.zero_points[3][1])

	def runCode(self):
		pass

class Interfaces():
	def __init__(self):
		self.jdrc = None
		self.myCMDVel = None
		self.myPose3d = None
		self.myDroneExtra = None
		self.one_points = [[0,0], [1,1], [1,-4]]
		self.zero_points = [[2,-4], [4,-4], [4,1], [2,1]]
		self.closing_loop = False

		self.connectProxies()

	def connectProxies(self):
		cfg = config.load(sys.argv[1])
		self.jdrc = comm.init(cfg, "ArDrone")
		self.myCMDVel = self.jdrc.getCMDVelClient("ArDrone.myCMDVel")
		if not self.myCMDVel:
			raise Exception("could not create client with name:myCMDVel")
		print("myCMDVel is connected")
		self.myPose3d = self.jdrc.getPose3dClient("ArDrone.myPose3d")
		if not self.myPose3d:
			raise Exception("could not create client with name:myPose3d")
		print("myPose3d is connected")
		self.myDroneExtra = self.jdrc.getArDroneExtraClient("ArDrone.myDroneExtra")
		if not self.myDroneExtra:
			raise Exception("could not create client with name:myDroneExtra")
		print("myDroneExtra is connected")

	def destroyProxies(self):
		if self.jdrc is not None:
			self.jdrc.destroy()

	def reachedToPoint(self, x, y):

		reachTresh = 0.1

		pose = self.myPose3d.getPose3d()

		diffx = x-pose.x

		diffy = y-pose.y

		maxDiff = abs(diffx)

		if abs(diffy) > maxDiff:

			maxDiff = abs(diffy)

		if maxDiff < reachTresh:

			return True

		return False

	

	

	def moveToPoint(self, x, y):

		if self.reachedToPoint(x, y):

			self.myCMDVel.setVX(0)

			self.myCMDVel.setVY(0)

		else:

			pose = self.myPose3d.getPose3d()

			diffx = x-pose.x

			diffy = y-pose.y

			maxDiff = abs(diffx)

			if abs(diffy) > maxDiff:

				maxDiff = abs(diffy)

			self.myCMDVel.setVX(diffx / (maxDiff*2))

			self.myCMDVel.setVY(diffy / (maxDiff*2))

		

		self.myCMDVel.sendVelocities()

displayGui = False
guiThread = None
gui = None
state0 = None
state2 = None
state3 = None

def signal_handler(signal, frame):
	global gui
	print("SIGINT is captured. The program exits")
	if gui is not None:
		gui.close()
	global state0
	state0.stop()
	global state2
	state2.stop()
	global state3
	state3.stop()

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
		gui.addState(1, "takeoff", True, 831.0, 926.0, 0)
		gui.addState(2, "draw_one", False, 965.0, 926.0, 0)
		gui.addState(3, "draw_zero", False, 1113.0, 940.0, 0)
		gui.addState(7, "land", False, 1220.0, 937.0, 0)
		gui.addState(4, "move_to_first", True, 833.0, 899.0, 2)
		gui.addState(5, "move_to_second", False, 973.0, 892.0, 2)
		gui.addState(6, "move_to_third", False, 1126.0, 894.0, 2)
		gui.addState(8, "move_to_first", True, 843.0, 915.0, 3)
		gui.addState(9, "move_to_second", False, 978.0, 911.0, 3)
		gui.addState(10, "move_to_third", False, 1137.0, 912.0, 3)
		gui.addState(11, "move_to_fourth", False, 1263.0, 915.0, 3)
		gui.addState(12, "close_the_loop", False, 1382.0, 911.0, 3)

		gui.addTransition(1, "transition 1", 1, 2, 903.0, 840.0)
		gui.addTransition(2, "one_completed", 2, 3, 1038.0, 827.0)
		gui.addTransition(3, "zero_completed", 3, 7, 1172.0, 842.5)
		gui.addTransition(7, "reached_to_first", 4, 5, 898.5, 827.5)
		gui.addTransition(8, "reached_to_second", 5, 6, 1056.5, 807.0)
		gui.addTransition(9, "reached_first", 8, 9, 914.5, 828.0)
		gui.addTransition(10, "reached_second", 9, 10, 1051.5, 840.5)
		gui.addTransition(11, "reached_third", 10, 11, 1201.0, 821.5)
		gui.addTransition(12, "reached_four", 11, 12, 1326.5, 820.0)

	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	state0 = State0(0, True, interfaces, 100, None, gui)
	state1 = State1(1, True, interfaces, 100, state0, gui)
	state2 = State2(2, False, interfaces, 100, state0, gui)
	state3 = State3(3, False, interfaces, 100, state0, gui)
	state7 = State7(7, False, interfaces, 100, state0, gui)
	state4 = State4(4, True, interfaces, 100, state2, gui)
	state5 = State5(5, False, interfaces, 100, state2, gui)
	state6 = State6(6, False, interfaces, 100, state2, gui)
	state8 = State8(8, True, interfaces, 100, state3, gui)
	state9 = State9(9, False, interfaces, 100, state3, gui)
	state10 = State10(10, False, interfaces, 100, state3, gui)
	state11 = State11(11, False, interfaces, 100, state3, gui)
	state12 = State12(12, False, interfaces, 100, state3, gui)

	tran1 = Tran1(1, 2, 5000)
	state1.addTransition(tran1)

	tran2 = Tran2(2, 3, interfaces)
	state2.addTransition(tran2)

	tran3 = Tran3(3, 7, interfaces)
	state3.addTransition(tran3)

	tran7 = Tran7(7, 5, interfaces)
	state4.addTransition(tran7)

	tran8 = Tran8(8, 6, interfaces)
	state5.addTransition(tran8)

	tran9 = Tran9(9, 9, interfaces)
	state8.addTransition(tran9)

	tran10 = Tran10(10, 10, interfaces)
	state9.addTransition(tran10)

	tran11 = Tran11(11, 11, interfaces)
	state10.addTransition(tran11)

	tran12 = Tran12(12, 12, interfaces)
	state11.addTransition(tran12)

	try:
		state0.startThread()
		state2.startThread()
		state3.startThread()
		signal.signal(signal.SIGINT, signal_handler)
		signal.pause()
		state0.join()
		state2.join()
		state3.join()
		if displayGui:
			guiThread.join()

		interfaces.destroyProxies()
	except:
		state0.stop()
		state2.stop()
		state3.stop()
		if displayGui:
			gui.close()
			guiThread.join()

		state0.join()
		state2.join()
		state3.join()
		interfaces.destroyProxies()
		sys.exit(1)
