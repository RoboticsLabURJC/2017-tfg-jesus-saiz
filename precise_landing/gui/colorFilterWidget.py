#
#  Copyright (C) 1997-2015 JDE Developers Team
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
#

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtGui import QImage, QPixmap
import numpy as np
import cv2

class ColorFilterWidget(QWidget):
    IMAGE_COLS_MAX=640
    IMAGE_ROWS_MAX=360
    
    imageUpdate=pyqtSignal()
    
    def __init__(self,winParent):      
        super(ColorFilterWidget, self).__init__()
        self.winParent=winParent
        self.imageUpdate.connect(self.updateImage)
        self.initUI()
        
    def initUI(self):

        self.setWindowTitle("Color filter")

        self.setMinimumSize(600,400)
        self.setMaximumSize(1340,400)#1340

        self.imgLabelColor=QLabel(self)
        self.imgLabelColor.resize(self.IMAGE_COLS_MAX,self.IMAGE_ROWS_MAX)
        self.imgLabelColor.move(130,20)
        self.imgLabelColor.show()
        '''
        print(self.IMAGE_ROWS_MAX)

        self.imgLabelBlackWhite=QLabel(self)
        self.imgLabelBlackWhite.resize(self.IMAGE_COLS_MAX,self.IMAGE_ROWS_MAX)
        self.imgLabelBlackWhite.move(90+self.IMAGE_ROWS_MAX,20) #40 + self.IMAGE_COLS_MAX,20
        self.imgLabelBlackWhite.show()
        
        self.imgPrueba=QLabel(self)
        self.imgPrueba.resize(self.IMAGE_COLS_MAX,self.IMAGE_ROWS_MAX)
        self.imgPrueba.move(800,20) 
        self.imgPrueba.show()
        '''
    def setColorImage(self):
        img = self.winParent.getCamera().getColorImage()

        if img is not None:
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            self.imgLabelColor.setPixmap(QPixmap.fromImage(image))
    '''
    def setThresoldImage(self):
        img = self.winParent.getCamera().getThresoldImage()

        if img is not None:
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            self.imgPrueba.setPixmap(QPixmap.fromImage(image))

        #if img is not None:
        #    image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1], QImage.Format_Indexed8)
        #    self.imgLabelBlackWhite.setPixmap(QPixmap.fromImage(image))
         
    def setThresoldImage2(self):
        img = self.winParent.getCamera().getColorImage() #img = self.winParent.getCamera().getThresoldImage2()

        if img is not None:
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            self.imgLabelBlackWhite.setPixmap(QPixmap.fromImage(image))
    '''
    def updateImage(self):
        self.setColorImage()
        #self.setThresoldImage()
        #self.setThresoldImage2()
        
    def closeEvent(self, event):
        self.winParent.closeColorFilterWidget()
