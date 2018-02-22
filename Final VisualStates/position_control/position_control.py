#!/usr/bin/python3
#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Alberto Martin Florido <almartinflorido@gmail.com>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#

import sys
from MyAlgorithm import MyAlgorithm
import easyiceconfig as EasyIce
from gui.threadGUI import ThreadGUI
from gui.threadGUI2 import ThreadGUI as TG
from parallelIce.cameraClient import CameraClient
from sensors.cameraFilter import CameraFilter
from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient
from gui.GUI import MainWindow
from PyQt5.QtWidgets import QApplication


from gui.machine import Machine
from gui.Machinegui import Window

import signal


signal.signal(signal.SIGINT, signal.SIG_DFL)

if __name__ == '__main__':

    machine = Machine(6)
    machine.setStateName(0, 'Take off')
    machine.setStateName(1, 'Search')
    machine.setStateName(2, 'Possibly \n beacon')
    machine.setStateName(3, 'Centering \n beacon')
    machine.setStateName(4, 'Landing')
    machine.setStateName(5, 'Landed')

    machine.addTransition(0, 1,'')
    machine.addTransition(1, 2,'')
    machine.addTransition(2, 3,'')
    machine.addTransition(3, 4,'')
    machine.addTransition(4, 5,'')
    machine.addTransition(5, 0,'')


    ic = EasyIce.initialize(sys.argv) 
    cameraCli = CameraClient(ic, "Introrob.Camera", True)
    camera = CameraFilter(cameraCli)
    navdata = NavDataClient(ic, "Introrob.Navdata", True)
    pose = Pose3DClient(ic, "Introrob.Pose3D", True)
    cmdvel = CMDVel(ic, "Introrob.CMDVel")
    extra = Extra(ic, "Introrob.Extra")

    algorithm=MyAlgorithm(camera, navdata, pose, cmdvel, machine, extra)


    app = QApplication(sys.argv)
    frame = MainWindow()
    frame.setCamera(camera)
    frame.setNavData(navdata)
    frame.setPose3D(pose)
    frame.setCMDVel(cmdvel)
    frame.setExtra(extra)
    #frame.setExtra(machine)
    frame.setAlgorithm(algorithm)
    frame.show()


    myGUI = Window(machine, algorithm)
    myGUI.show()

    t3 = TG(myGUI)
    t3.daemon=True
    t3.start()

    t2 = ThreadGUI(frame)
    t2.daemon=True
    t2.start()

    sys.exit(app.exec_())
