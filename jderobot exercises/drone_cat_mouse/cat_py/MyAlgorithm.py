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
bool_velYaw = False
bool_velZ = False
bool_velX = False

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra
        self.minError=0.05

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
        # Add
         global detec

         input_image = self.camera.getImage()

# Realizacion del filtro de color
         if input_image is not None:
             blur = cv2.GaussianBlur(input_image, (3, 3), 0)
             color_HSV = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)

             H_max = 0.42*(180/(2*pi))  # 0.05
             H_min = 0.0*(180/(2*pi))   # 0.0
             S_max = 1.0*(255/1)        # 1.0
             S_min = 0.00*(255/1)       # 0.05
             V_max = 238.00             # 170.06
             V_min = 40.00               # 0.00

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

# Seguimiento del raton
         if (detec == True):
             print ("Mouse Detected")
#            posicion central de la imagen en el filtro de color (cambio: rojo por debajo del centro)
             ini_pos = np.array([160, -140])

# Cambio en yaw y en z
             coord_mou = np.array([x+w/2, -y+h/2])
             print ("Coord Rat:", coord_mou)
             vect_1 = ini_pos - coord_mou
             vel_yaw = vect_1[0]*0.01
             vel_z = vect_1[1]*(-0.01)
             print ("Vel yaw y z:", vel_yaw, vel_z)

             # yaw
             if vel_yaw > 0.25:
                vel_yaw = 0.99
             elif vel_yaw < -0.25:
                vel_yaw = -0.99

             if abs(vel_yaw) < self.minError:
                 bool_velYaw = False
                #  self.cmdvel.sendCMDVel(0,0,0,0,0,0)
                 print ("mouse on yaw good")
             else:
                 bool_velYaw = True
                #  self.cmdvel.sendCMDVel(0,vel_yaw,0,0,0,0)
                 print("cambia yaw", vel_yaw)

             # z
             if vel_z > 0.25:
                vel_z = 0.99
             elif vel_z < -0.25:
                vel_z = -0.99

             if abs(vel_z) < self.minError:
                 bool_velZ = False
                #  self.cmdvel.sendCMDVel(0,0,0,0,0,0)
                 print ("mouse on z good")
             else:
                 bool_velZ = True
                #  self.cmdvel.sendCMDVel(0,0,vel_z,0,0,0)
                 print("cambia z", vel_z)

# Cambio en la velocidad en x
             area_mou = w*h
             print ("Area mouse:", area_mou)
             ini_area = 9*9
             vel_x = (ini_area - area_mou)*0.01
             print ("Vel x", vel_x)
             if vel_x > 0.3:
                 vel_x = 0.99
                 print ("Far mouse")
             elif vel_x < -0.3:
                 vel_x = -0.99
                 print ("Near mouse")

             if abs(vel_x) < (self.minError*3):
                 bool_velX = False
                #  self.cmdvel.sendCMDVel(0,0,0,0,0,0)
                 print ("mouse on x good")
             else:
                 bool_velX = True
                #  self.cmdvel.sendCMDVel(vel_x,0,0,0,0,0)
                 print ("Cambia x", vel_x)

# Enviamos el comando de cambio de velocidades
             if (bool_velX == True and bool_velZ == True and bool_velYaw == True):
                self.cmdvel.sendCMDVel(vel_x,vel_yaw,vel_z,0,0,0)
                print("Cambio las tres velocidades")
                print()
             elif (bool_velX == True and bool_velYaw == True and (bool_velZ == False)):
                self.cmdvel.sendCMDVel(vel_x,vel_yaw,0,0,0,0)
                print("Cambio VX y VY")
                print()
             elif (bool_velX == True and bool_velZ == True and (bool_velYaw == False)):
                self.cmdvel.sendCMDVel(vel_x,0,vel_z,0,0,0)
                print("Cambio VX y VZ")
                print()
             elif (bool_velYaw == True and bool_velZ == True and (bool_velX == False)):
                self.cmdvel.sendCMDVel(0,vel_yaw,vel_z,0,0,0)
                print("Cambio VY y VZ")
                print()
             elif(bool_velX == True and bool_velYaw == False and bool_velZ == False):
                self.cmdvel.sendCMDVel(vel_x,0,0,0,0,0)
                print("Cambio VX")
                print()
             elif(bool_velYaw == True and bool_velX == False and bool_velZ == False):
                self.cmdvel.sendCMDVel(0,vel_yaw,0,0,0,0)
                print("Cambio VY")
                print()
             elif(bool_velZ == True and bool_velYaw == False and bool_velX == False):
                self.cmdvel.sendCMDVel(0,0,vel_z,0,0,0)
                print("Cambio VZ")
                print()
             else:
                self.cmdvel.sendCMDVel(0,0,0,0,0,0)
                print("Raton quieto")
                print()


#              if abs(vel_1[0]) < self.minError and abs(vel_1[1]) < self.minError:
#                  self.cmdvel.sendCMDVel(0,0,0,0,0,0)
#                  print ("Mouse Stop")
#              else:
#                  if area_mou > ini_area:
#                      print ("Near mouse")
#                      self.cmdvel.sendCMDVel(0,vel_x,-vel_1[1],vel_1[0],0,0)
# #                     self.cmdvel.sendCMDVel(0,0,-vel_1[1],vel_1[0],0,0)
#                  elif area_mou <= ini_area:
#                      print ("Far mouse")
#                      self.cmdvel.sendCMDVel(0,vel_x,-vel_1[1],vel_1[0],0,0)
# #                     self.cmdvel.sendCMDVel(0,0,-vel_1[1],vel_1[0],0,0)
#                  print ("Following mouse")



#             if area_mou > ini_area:
#                 print ("Near mouse")
#                 self.cmdvel.setVX(-vel_x)
#                 self.cmdvel.sendVelocities( )
#             elif area_mou < ini_area:
#                 print ("Far mouse")
#                 self.cmdvel.setVX(vel_x)
#                 self.cmdvel.sendVelocities( )
#             else:
#                 self.cmdvel.setVX(0)
#                 self.cmdvel.sendVelocities( )



#        tmp = self.navdata.getNavdata()
#        if tmp is not None:
#            print ("State: " +str(tmp.state))
#            print ("Altitude: " +str(tmp.altd))
#            print ("Vehicle: " +str(tmp.vehicle))
#            print ("Battery %: " +str(tmp.batteryPercent))
