#!/usr/bin/env python

from __future__ import division

import sys
import numpy as np
import cv2
import imutils
import sys
import roslib
import rospy
import rospkg
import tf

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.uic import loadUi
from test2 import test2

class setupMainWindow(QMainWindow):
    def refereeSetup(self, parent, xyzw, refereeCommand):
        super(setupMainWindow, self).__init__(parent)
        self.form_widget = referee(self, xyzw, refereeCommand)
        self.setCentralWidget(self.form_widget)

    def strikerSetup(self, parent):
        super(setupMainWindow, self).__init__(parent)
        self.form_widget = striker(self)
        self.setCentralWidget(self.form_widget)

    def defenderSetup(self, parent):
        super(setupMainWindow, self).__init__(parent)
        self.form_widget = defender(self)
        self.setCentralWidget(self.form_widget)

    def goalkeeperSetup(self, parent):
        super(setupMainWindow, self).__init__(parent)
        self.form_widget = goalkeeper(self)
        self.setCentralWidget(self.form_widget)

    def closeEvent(self, event):
        self.parent().setEnabled(True)

class referee(QWidget):
    def __init__(self, parent, xyzw, refereeCommand):
        super(referee, self).__init__(parent)
        self.path = rospkg.RosPack().get_path('rostu_gui')
        loadUi(self.path + '/qt_ui/refereeUi.ui', self)
        self.setFixedSize(680, 500)
        self.xy_coor.setText(xyzw) 
        self.x.setValidator(QDoubleValidator(-100, 100, 10, self))
        self.y.setValidator(QDoubleValidator(-100, 100, 10, self))
        self.z.setValidator(QDoubleValidator(-100, 100, 10, self))
        self.w.setValidator(QDoubleValidator(-100, 100, 10, self))
        self.refereeCommand = refereeCommand

        self.param = [None, None, None]

        self.initButton()

    def initButton(self):
        self.exit.clicked.connect(self.closeIt)
        self.striker.clicked.connect(self.strikerParam)
        self.defender.clicked.connect(self.defenderParam)
        self.goalkeeper.clicked.connect(self.goalkeeperParam)

        self.save.clicked.connect(self.saveCoor)
        self.load.clicked.connect(self.loadCoor)

        #Referee Button
        self.kickoffBtnM.clicked.connect(self.kickoffM)
        self.freekickBtnM.clicked.connect(self.freekickM)
        self.goalkickBtnM.clicked.connect(self.goalkickM)
        self.cornerBtnM.clicked.connect(self.cornerM)
        self.penaltyBtnM.clicked.connect(self.penaltyM)
        self.dropballBtnM.clicked.connect(self.dropballM)

        self.kickoffBtnC.clicked.connect(self.kickoffC)
        self.freekickBtnC.clicked.connect(self.freekickC)
        self.goalkickBtnC.clicked.connect(self.goalkickC)
        self.cornerBtnC.clicked.connect(self.cornerC)
        self.penaltyBtnC.clicked.connect(self.penaltyC)
        self.dropballBtnC.clicked.connect(self.dropballC)


    def closeIt(self):
        self.parent().close()

    def strikerParam(self):
        self.referee.setEnabled(True)
        self.coordinates.setEnabled(False)
        self.param[0] = 'S'
        self.set.setText('Set : ' + self.param[0]) 
    def defenderParam(self):
        self.referee.setEnabled(True)
        self.coordinates.setEnabled(False)
        self.param[0] = 'D'
        self.set.setText('Set : ' + self.param[0]) 
    def goalkeeperParam(self):
        self.referee.setEnabled(True)
        self.coordinates.setEnabled(False)
        self.param[0] = 'G'
        self.set.setText('Set : ' + self.param[0]) 

    def saveCoor(self):
        self.parent().parent().writeRefereeCommand(self.param[2] + "_" + self.param[1] + "_" + self.param[0], 
                                                   [float(self.x.text()), float(self.y.text()), float(self.w.text()), float(self.z.text())])
        self.referee.setEnabled(False)
        self.coordinates.setEnabled(False)
        self.param = [None, None, None]
        self.set.setText('Set : ')

    def loadCoor(self):
        if self.xy_coor != "X & Y & Z & W : ":
            string = self.xy_coor.text()
            substr = string[17:-1]
            string = ["", "", "", ""]
            idx = 0
            for i in range(0, len(substr)):
                if substr[i] != " " and substr[i] != ":":
                    string[idx] = string[idx] + substr[i]
                elif substr[i] == ":":
                    idx = idx + 1
            self.x.setText(string[0])
            self.y.setText(string[1])
            self.z.setText(string[2])
            self.w.setText(string[3])

    def kickoffM(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'M'
        self.param[2] = 'kickoff'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))
    def freekickM(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'M'
        self.param[2] = 'freekick'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))
    def goalkickM(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'M'
        self.param[2] = 'goalkick'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))
    def cornerM(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'M'
        self.param[2] = 'corner'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))
    def penaltyM(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'M'
        self.param[2] = 'penalty'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))
    def dropballM(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'M'
        self.param[2] = 'dropball'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))

    def kickoffC(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'C'
        self.param[2] = 'kickoff'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))
    def freekickC(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'C'
        self.param[2] = 'freekick'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))
    def goalkickC(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'C'
        self.param[2] = 'goalkick'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))
    def cornerC(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'C'
        self.param[2] = 'corner'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))
    def penaltyC(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'C'
        self.param[2] = 'penalty'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))
    def dropballC(self):
        self.coordinates.setEnabled(True)
        self.param[1] = 'C'
        self.param[2] = 'dropball'
        self.set.setText('Set : ' + self.param[0] + ' >> ' + self.param[1] + ' >> ' + self.param[2]) 
        coor = self.refereeCommand[self.param[2] + "_" + self.param[1] + "_" + self.param[0]]
        self.x.setText(str(coor[0]))
        self.y.setText(str(coor[1]))
        self.z.setText(str(coor[2]))
        self.w.setText(str(coor[3]))

class striker(QWidget):
    def __init__(self, parent):
        super(striker, self).__init__(parent)
        self.path = rospkg.RosPack().get_path('rostu_gui')
        # loadUi(self.path + '/qt_ui/refereeUi.ui', self)
        # self.setFixedSize(680, 500)

class defender(QWidget):
    def __init__(self, parent):
        super(defender, self).__init__(parent)
        self.path = rospkg.RosPack().get_path('rostu_gui')
        # loadUi(self.path + '/qt_ui/refereeUi.ui', self)
        # self.setFixedSize(680, 500)

class goalkeeper(QWidget):
    def __init__(self, parent):
        super(goalkeeper, self).__init__(parent)
        self.path = rospkg.RosPack().get_path('rostu_gui')
        # loadUi(self.path + '/qt_ui/refereeUi.ui', self)
        # self.setFixedSize(680, 500)

