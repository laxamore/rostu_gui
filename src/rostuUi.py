#!/usr/bin/env python

from __future__ import division

import numpy as np
import cv2
import imutils
import sys
import roslib
import rospy
import rospkg
import tf
import yaml

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import PoseWithCovarianceStamped 
from geometry_msgs.msg import Pose 
from actionlib_msgs.msg import GoalID
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.uic import loadUi
from setup import setupMainWindow

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.form_widget = rostuWidget(self) 
        self.setCentralWidget(self.form_widget)

class Worker(QRunnable):
    def __init__(self, fn):
        super(Worker, self).__init__()
        self.fn = fn
        
    def run(self):
        result = self.fn()

class rostuWidget(QWidget):
    def __init__(self, parent=None):
        super(rostuWidget, self).__init__(parent)
        self.refereeCommand = None
        self.loadRefereeCommand()
        self.path = rospkg.RosPack().get_path('rostu_gui')

        self.listener = tf.TransformListener()
        self.strikerRobotPose = [[0, 0, 0], [0, 0, 0, 0]]
        self.defenderRobotPose = [[0, 0, 0], [0, 0, 0, 0]]
        self.goalkeeperRobotPose = [[0, 0, 0], [0, 0, 0, 0]]

        self.initUI()
        self.initButton()

        self.setMouseTracking(True)
        self.mouseClickInFieldArea = False
        self.mouseClickPos = [0, 0]
        self.mouseReleasePos = [0, 0]
        self.mouseMovePos = [0, 0]
        self.robotTeam = 'M'
        self.robotNav = False
        self.robotPoseEstimate = False
        self.strikerNav = False
        self.strikerPoseEstimate = False
        self.defenderNav = False
        self.defenderPoseEstimate = False
        self.goalkeeperNav = False
        self.goalkeeperPoseEstimate = False

        self.updateFieldTimer = QTimer()
        self.updateFieldTimer.timeout.connect(self.updateFieldThread)
        self.updateFieldTimer.start(1)
        
        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())

    def loadRefereeCommand(self):
        stream = open("/home/laxamore/catkin_ws/src/rostu_gui/cfg/refereeCommand.yaml", 'r')
        self.refereeCommand = yaml.load(stream)
    
    def writeRefereeCommand(self, param, value):
        stream = open("/home/laxamore/catkin_ws/src/rostu_gui/cfg/refereeCommand.yaml", 'w')
        self.refereeCommand[param] = value
        yaml.dump(self.refereeCommand, stream)

    def initUI(self):
        #Load Design UI Dari QtDesign
        loadUi(self.path + '/qt_ui/rostuUi.ui', self)
        #Setting Fixed Frame Window
        self.setFixedSize(1280, 680)

    def initButton(self):
        self.penaltyUp.setVisible(False)
        self.penaltyBot.setVisible(False)
        self.cornerLeft.setVisible(False)
        self.cornerRight.setVisible(False)

        self.refereeSetup.clicked.connect(self.openRefereeSetup)

        self.teamCyan.clicked.connect(self.changeTeam)
        self.teamMagenta.clicked.connect(self.changeTeam)
        self.strikerNavBtn.clicked.connect(self.strikerNavCall)
        self.strikerPosBtn.clicked.connect(self.strikerPoseCall)
        self.defenderNavBtn.clicked.connect(self.defenderNavCall)
        self.defenderPosBtn.clicked.connect(self.defenderPoseCall)
        self.goalkeeperNavBtn.clicked.connect(self.goalkeeperNavCall)
        self.goalkeeperPosBtn.clicked.connect(self.goalkeeperPoseCall)

        #Referee Button
        self.kickoffBtnM.clicked.connect(self.kickoffM)
        self.freekickBtnM.clicked.connect(self.freekickM)
        self.goalkickBtnM.clicked.connect(self.goalkickM)
        self.cornerBtnM.clicked.connect(self.cornerM)
        self.penaltyBtnM.clicked.connect(self.penaltyM)
        self.goalBtnM.clicked.connect(self.goalM)

        self.kickoffBtnC.clicked.connect(self.kickoffC)
        self.freekickBtnC.clicked.connect(self.freekickC)
        self.goalkickBtnC.clicked.connect(self.goalkickC)
        self.cornerBtnC.clicked.connect(self.cornerC)
        self.penaltyBtnC.clicked.connect(self.penaltyC)
        self.goalBtnC.clicked.connect(self.goalC)

        self.startBtn.clicked.connect(self.startReferee)
        self.stopBtn.clicked.connect(self.stopReferee)
        self.dropballBtn.clicked.connect(self.dropballReferee)
        self.endBtn.clicked.connect(self.endReferee)
        self.resetBtn.clicked.connect(self.resetReferee)

    def openRefereeSetup(self):
        self.setEnabled(False)
        self.window = setupMainWindow()
        self.window.refereeSetup(self, self.xy_coor.text(), self.refereeCommand)
        self.window.setWindowTitle('Referee Setup')
        self.window.show()

    # def openStrikerSetup(self):
    #     self.setEnabled(False)
    #     self.window = setupMainWindow()
    #     self.window.strikerSetup(self)
    #     self.window.setWindowTitle('Striker Setup')
    #     self.window.show()

    # def openDefenderSetup(self):
    #     self.setEnabled(False)
    #     self.window = setupMainWindow()
    #     self.window.defenderSetup(self)
    #     self.window.setWindowTitle('Defender Setup')
    #     self.window.show()

    # def openGoalkeeperSetup(self):
    #     self.setEnabled(False)
    #     self.window = setupMainWindow()
    #     self.window.goalkeeperSetup(self)
    #     self.window.setWindowTitle('Goalkeeper Setup')
    #     self.window.show()

    def kickoffM(self):
        coorStriker = [0, 0, 0, 0]
        coorDefender = [0, 0, 0, 0]
        coorGoalkeeper = [0, 0, 0, 0]
        if self.robotTeam == "M":
            coorStriker = self.refereeCommand["kickoff_M_S_T"]
            coorDefender = self.refereeCommand["kickoff_M_D_T"]
            coorGoalkeeper = self.refereeCommand["kickoff_M_G_T"]
        elif self.robotTeam == "C":
            coorStriker = self.refereeCommand["kickoff_M_S_E"]
            coorDefender = self.refereeCommand["kickoff_M_D_E"]
            coorGoalkeeper = self.refereeCommand["kickoff_M_G_E"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
        self.robotGoalPublish("defender", coorDefender[0], coorDefender[1], coorDefender[2], coorDefender[3])
        # self.robotGoalPublish("goalkeeper", coorGoalkeeper[0], coorGoalkeeper[1], coorGoalkeeper[2], coorGoalkeeper[3])
    def freekickM(self):
        coorStriker = [0, 0, 0, 0]
        coorDefender = [0, 0, 0, 0]
        coorGoalkeeper = [0, 0, 0, 0]
        if self.robotTeam == "M":
            coorStriker = self.refereeCommand["freekick_M_S_T"]
            coorDefender = self.refereeCommand["freekick_M_D_T"]
            coorGoalkeeper = self.refereeCommand["freekick_M_G_T"]
        elif self.robotTeam == "C":
            coorStriker = self.refereeCommand["freekick_M_S_E"]
            coorDefender = self.refereeCommand["freekick_M_D_E"]
            coorGoalkeeper = self.refereeCommand["freekick_M_G_E"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
        self.robotGoalPublish("defender", coorDefender[0], coorDefender[1], coorDefender[2], coorDefender[3])
        # self.robotGoalPublish("goalkeeper", coorGoalkeeper[0], coorGoalkeeper[1], coorGoalkeeper[2], coorGoalkeeper[3])
    def goalkickM(self):
        coorStriker = [0, 0, 0, 0]
        coorDefender = [0, 0, 0, 0]
        coorGoalkeeper = [0, 0, 0, 0]
        if self.robotTeam == "M":
            coorStriker = self.refereeCommand["goalkick_M_S_T"]
            coorDefender = self.refereeCommand["goalkick_M_D_T"]
            coorGoalkeeper = self.refereeCommand["goalkick_M_G_T"]
        elif self.robotTeam == "C":
            coorStriker = self.refereeCommand["goalkick_M_S_E"]
            coorDefender = self.refereeCommand["goalkick_M_D_E"]
            coorGoalkeeper = self.refereeCommand["goalkick_M_G_E"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
        self.robotGoalPublish("defender", coorDefender[0], coorDefender[1], coorDefender[2], coorDefender[3])
        # self.robotGoalPublish("goalkeeper", coorGoalkeeper[0], coorGoalkeeper[1], coorGoalkeeper[2], coorGoalkeeper[3])
    def cornerM(self):
        coorStriker = [0, 0, 0, 0]
        coorDefender = [0, 0, 0, 0]
        coorGoalkeeper = [0, 0, 0, 0]
        if self.robotTeam == "M":
            coorStriker = self.refereeCommand["corner_M_S_T"]
            coorDefender = self.refereeCommand["corner_M_D_T"]
            coorGoalkeeper = self.refereeCommand["corner_M_G_T"]
        elif self.robotTeam == "C":
            coorStriker = self.refereeCommand["corner_M_S_E"]
            coorDefender = self.refereeCommand["corner_M_D_E"]
            coorGoalkeeper = self.refereeCommand["corner_M_G_E"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
        self.robotGoalPublish("defender", coorDefender[0], coorDefender[1], coorDefender[2], coorDefender[3])
        # self.robotGoalPublish("goalkeeper", coorGoalkeeper[0], coorGoalkeeper[1], coorGoalkeeper[2], coorGoalkeeper[3])
    def penaltyM(self):
        coorStriker = [0, 0, 0, 0]
        if self.robotTeam == "M":
            coorStriker = self.refereeCommand["penalty_M_S_"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
    def goalM(self):
        print("goalM")
    
    def kickoffC(self):
        coorStriker = [0, 0, 0, 0]
        coorDefender = [0, 0, 0, 0]
        coorGoalkeeper = [0, 0, 0, 0]
        if self.robotTeam == "C":
            coorStriker = self.refereeCommand["kickoff_C_S_T"]
            coorDefender = self.refereeCommand["kickoff_C_D_T"]
            coorGoalkeeper = self.refereeCommand["kickoff_C_G_T"]
        elif self.robotTeam == "M":
            coorStriker = self.refereeCommand["kickoff_C_S_E"]
            coorDefender = self.refereeCommand["kickoff_C_D_E"]
            coorGoalkeeper = self.refereeCommand["kickoff_C_G_E"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
        self.robotGoalPublish("defender", coorDefender[0], coorDefender[1], coorDefender[2], coorDefender[3])
        # self.robotGoalPublish("goalkeeper", coorGoalkeeper[0], coorGoalkeeper[1], coorGoalkeeper[2], coorGoalkeeper[3])
    def freekickC(self):
        coorStriker = [0, 0, 0, 0]
        coorDefender = [0, 0, 0, 0]
        coorGoalkeeper = [0, 0, 0, 0]
        if self.robotTeam == "C":
            coorStriker = self.refereeCommand["freekick_C_S_T"]
            coorDefender = self.refereeCommand["freekick_C_D_T"]
            coorGoalkeeper = self.refereeCommand["freekick_C_G_T"]
        elif self.robotTeam == "M":
            coorStriker = self.refereeCommand["freekick_C_S_E"]
            coorDefender = self.refereeCommand["freekick_C_D_E"]
            coorGoalkeeper = self.refereeCommand["freekick_C_G_E"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
        self.robotGoalPublish("defender", coorDefender[0], coorDefender[1], coorDefender[2], coorDefender[3])
        # self.robotGoalPublish("goalkeeper", coorGoalkeeper[0], coorGoalkeeper[1], coorGoalkeeper[2], coorGoalkeeper[3])
    def goalkickC(self):
        coorStriker = [0, 0, 0, 0]
        coorDefender = [0, 0, 0, 0]
        coorGoalkeeper = [0, 0, 0, 0]
        if self.robotTeam == "C":
            coorStriker = self.refereeCommand["goalkick_C_S_T"]
            coorDefender = self.refereeCommand["goalkick_C_D_T"]
            coorGoalkeeper = self.refereeCommand["goalkick_C_G_T"]
        elif self.robotTeam == "M":
            coorStriker = self.refereeCommand["goalkick_C_S_E"]
            coorDefender = self.refereeCommand["goalkick_C_D_E"]
            coorGoalkeeper = self.refereeCommand["goalkick_C_G_E"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
        self.robotGoalPublish("defender", coorDefender[0], coorDefender[1], coorDefender[2], coorDefender[3])
        # self.robotGoalPublish("goalkeeper", coorGoalkeeper[0], coorGoalkeeper[1], coorGoalkeeper[2], coorGoalkeeper[3])
    def cornerC(self):
        coorStriker = [0, 0, 0, 0]
        coorDefender = [0, 0, 0, 0]
        coorGoalkeeper = [0, 0, 0, 0]
        if self.robotTeam == "C":
            coorStriker = self.refereeCommand["corner_C_S_T"]
            coorDefender = self.refereeCommand["corner_C_D_T"]
            coorGoalkeeper = self.refereeCommand["corner_C_G_T"]
        elif self.robotTeam == "M":
            coorStriker = self.refereeCommand["corner_C_S_E"]
            coorDefender = self.refereeCommand["corner_C_D_E"]
            coorGoalkeeper = self.refereeCommand["corner_C_G_E"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
        self.robotGoalPublish("defender", coorDefender[0], coorDefender[1], coorDefender[2], coorDefender[3])
        # self.robotGoalPublish("goalkeeper", coorGoalkeeper[0], coorGoalkeeper[1], coorGoalkeeper[2], coorGoalkeeper[3])
    def penaltyC(self):
        coorStriker = [0, 0, 0, 0]
        if self.robotTeam == "C":
            coorStriker = self.refereeCommand["penalty_C_S_"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
    def goalC(self):
        print("Cyan Goal")

    def startReferee(self):
        print("startReferee")
    def stopReferee(self):
        strikerGoalCancel = GoalID()
        defenderGoalCancel = GoalID()
        goalkeeperGoalCancel = GoalID()
        striker_cancel_goal.publish(strikerGoalCancel)
        defender_cancel_goal.publish(defenderGoalCancel)
        goalkeeper_cancel_goal.publish(goalkeeperGoalCancel)
    def dropballReferee(self):
        coorStriker = [0, 0, 0, 0]
        if self.robotTeam == "C":
            coorStriker = self.refereeCommand["dropball_C_S_"]
            coorDefender = self.refereeCommand["dropball_C_D_"]
            coorGoalkeeper = self.refereeCommand["dropball_C_G_"]
        elif self.robotTeam == "M":
            coorStriker = self.refereeCommand["dropball_M_S_"]
            coorDefender = self.refereeCommand["dropball_M_D_"]
            coorGoalkeeper = self.refereeCommand["dropball_M_G_"]
        self.robotGoalPublish("striker", coorStriker[0], coorStriker[1], coorStriker[2], coorStriker[3])
        self.robotGoalPublish("defender", coorDefender[0], coorDefender[1], coorDefender[2], coorDefender[3])
        # self.robotGoalPublish("goalkeeper", coorGoalkeeper[0], coorGoalkeeper[1], coorGoalkeeper[2], coorGoalkeeper[3])
    def endReferee(self):
        print("endReferee")
    def resetReferee(self):
        print("resetReferee")

    def changeTeam(self):
        if self.teamCyan.isChecked():
            self.robotTeam = 'C'
        elif self.teamMagenta.isChecked():
            self.robotTeam = 'M'
            
    def updateFieldThread(self):
        worker = Worker(self.updateField)
        self.threadpool.start(worker) 

    def updateField(self):
        self.image = cv2.imread(self.path + '/media/field.png')
        height, width = self.image.shape[:2]
        
        if self.mouseClickInFieldArea:
            self.image = cv2.line(self.image, (self.mouseClickPos[1], self.mouseClickPos[0]), (self.mouseMovePos[1], self.mouseMovePos[0]), (0, 0, 255), 20)
        
        

        #Draw Striker Robot
        try:
            self.strikerRobotPose = self.listener.lookupTransform('/map', 'striker/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            None

        strikerOrientation_list = [self.strikerRobotPose[1][0], self.strikerRobotPose[1][1], self.strikerRobotPose[1][2], self.strikerRobotPose[1][3]]
        (strikerR, strikerP, strikerY) = euler_from_quaternion(strikerOrientation_list)
        
        strikerHeading = ((strikerY / (2 * np.pi)) * 360)
        strikerCosHeading = np.cos(np.radians(strikerHeading))
        strikerSinHeading = np.sin(np.radians(strikerHeading))
        strikerPosX = int(self.strikerRobotPose[0][1] / 0.0025)
        strikerPosY = int(self.strikerRobotPose[0][0] / 0.0025)
        
        self.image = cv2.resize(self.image, (int(width / 7), int(height / 7)))
        self.image = cv2.circle(self.image, (int(strikerPosX / 7), int(strikerPosY / 7)), int(100 / 7), (0, 0, 255), -1)
        self.image = cv2.line(self.image, (int(strikerPosX / 7), int(strikerPosY / 7)), (int(strikerPosX / 7) + int(strikerSinHeading * int(100 / 7)), int(strikerPosY / 7) + int(strikerCosHeading * int(100 / 7))), (255, 255, 255), int(20 / 7))
        ##########################

        #Draw Defender Robot
        try:
            self.defenderRobotPose = self.listener.lookupTransform('/map', 'defender/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            None

        defenderOrientation_list = [self.defenderRobotPose[1][0], self.defenderRobotPose[1][1], self.defenderRobotPose[1][2], self.defenderRobotPose[1][3]]
        (defenderR, defenderP, defenderY) = euler_from_quaternion(defenderOrientation_list)
        
        defenderHeading = ((defenderY / (2 * np.pi)) * 360)
        defenderCosHeading = np.cos(np.radians(defenderHeading))
        defenderSinHeading = np.sin(np.radians(defenderHeading))
        defenderPosX = int(self.defenderRobotPose[0][1] / 0.0025)
        defenderPosY = int(self.defenderRobotPose[0][0] / 0.0025)
        
        self.image = cv2.resize(self.image, (int(width / 7), int(height / 7)))
        self.image = cv2.circle(self.image, (int(defenderPosX / 7), int(defenderPosY / 7)), int(100 / 7), (100, 200, 0), -1)
        self.image = cv2.line(self.image, (int(defenderPosX / 7), int(defenderPosY / 7)), (int(defenderPosX / 7) + int(defenderSinHeading * int(100 / 7)), int(defenderPosY / 7) + int(defenderCosHeading * int(100 / 7))), (255, 255, 255), int(20 / 7))
        ##########################

        # #Draw Goalkeeper Robot
        # self.image = cv2.resize(self.image, (int(width / 7), int(height / 7)))
        # self.image = cv2.circle(self.image, (int(posX / 7), int(posY / 7)), int(100 / 7), (0, 0, 255), -1)
        # self.image = cv2.line(self.image, (int(posX / 7), int(posY / 7)), (int(posX / 7) + int(sinHeading * int(100 / 7)), int(posY / 7) + int(cosHeading * int(100 / 7))), (255, 255, 255), int(20 / 7))
        # ##########################

        self.fieldImage(self.image, 1)

    def fieldImage(self, img, window=1):
        qformat = QImage.Format_Indexed8

        if len(img.shape) == 3:
            if img.shape[2] == 4:
                qformat = QImage.Format_RGBA8888
            else:
                qformat = QImage.Format_RGB888

        outImage = QImage(img, img.shape[1], img.shape[0], img.strides[0], qformat)
        outImage = outImage.rgbSwapped()

        if window == 1:
            self.field.setPixmap(QPixmap.fromImage(outImage))
            self.field.installEventFilter(self)

    def robotGoalPublish(self, robot, x, y, z, w):
        goal_publish_msg = PoseStamped()
        goal_publish_msg.header.frame_id = "map"
        goal_publish_msg.pose.position.x = x
        goal_publish_msg.pose.position.y = y
        goal_publish_msg.pose.orientation.z = z
        goal_publish_msg.pose.orientation.w = w
        if robot == "striker":
            striker_publish_goal.publish(goal_publish_msg)
        elif robot == "defender":
            defender_publish_goal.publish(goal_publish_msg)
        # elif robot == "goalkeeper":
            # goalkeeper_publish_goal.publish(goal_publish_msg)

    def robotPoseEstimation(self, robot, x, y, z, w):
        pose_estimate_publish_msg = PoseWithCovarianceStamped()
        pose_estimate_publish_msg.header.frame_id = "map"
        pose_estimate_publish_msg.pose.pose.position.x = x
        pose_estimate_publish_msg.pose.pose.position.y = y
        pose_estimate_publish_msg.pose.pose.orientation.z = z
        pose_estimate_publish_msg.pose.pose.orientation.w = w
        if robot == "striker":
            striker_pose_estimate.publish(pose_estimate_publish_msg)
        elif robot == "defender":
            defender_pose_estimate.publish(pose_estimate_publish_msg)
        # elif robot == "goalkeeper":
        #     goalkeeper_pose_estimate.publish(pose_estimate_publish_msg)

    def strikerNavCall(self):
        self.robotNav = True
        self.robotPoseEstimate = False
        self.strikerNav = True
        self.strikerPoseEstimate = False
        self.defenderNav = False
        self.defenderPoseEstimate = False
        self.goalkeeperNav = False
        self.goalkeeperPoseEstimate = False
        self.striker.setStyleSheet("""
            #striker{
                color: rgb(255, 0, 0);
                background-color: rgb(0, 170, 0);
            }
        """)
        self.defender.setStyleSheet("""
            #defender{
                color: rgb(0, 255, 0);
            }
        """)
        self.goalkeeper.setStyleSheet("""
            #goalkeeper{
                color: rgb(0, 0, 255);
            }
        """)
    def strikerPoseCall(self):
        self.robotNav = False
        self.robotPoseEstimate = True
        self.strikerPoseEstimate = True
        self.strikerNav = False
        self.defenderPoseEstimate = False
        self.defenderNav = False
        self.goalkeeperPoseEstimate = False
        self.goalkeeperNav = False
        self.striker.setStyleSheet("""
            #striker{
                color: rgb(255, 0, 0);
                background-color: rgb(0, 170, 0);
            }
        """)
        self.defender.setStyleSheet("""
            #defender{
                color: rgb(0, 255, 0);
            }
        """)
        self.goalkeeper.setStyleSheet("""
            #goalkeeper{
                color: rgb(0, 0, 255);
            }
        """)
    def defenderNavCall(self):
        self.robotNav = True
        self.robotPoseEstimate = False
        self.defenderNav = True
        self.defenderPoseEstimate = False
        self.strikerNav = False
        self.strikerPoseEstimate = False
        self.goalkeeperNav = False
        self.goalkeeperPoseEstimate = False
        self.defender.setStyleSheet("""
            #defender{
                color: rgb(0, 255, 0);
                background-color: rgb(0, 170, 0);
            }
        """)
        self.striker.setStyleSheet("""
            #striker{
                color: rgb(255, 0, 0);
            }
        """)
        self.goalkeeper.setStyleSheet("""
            #goalkeeper{
                color: rgb(0, 0, 255);
            }
        """)
    def defenderPoseCall(self):
        self.robotNav = False
        self.robotPoseEstimate = True
        self.defenderPoseEstimate = True
        self.defenderNav = False
        self.strikerPoseEstimate = False
        self.strikerNav = False
        self.goalkeeperPoseEstimate = False
        self.goalkeeperNav = False
        self.defender.setStyleSheet("""
            #defender{
                color: rgb(0, 255, 0);
                background-color: rgb(0, 170, 0);
            }
        """)
        self.striker.setStyleSheet("""
            #striker{
                color: rgb(255, 0, 0);
            }
        """)
        self.goalkeeper.setStyleSheet("""
            #goalkeeper{
                color: rgb(0, 0, 255);
            }
        """)
    def goalkeeperNavCall(self):
        self.robotNav = True
        self.robotPoseEstimate = False
        self.goalkeeperNav = True
        self.goalkeeperPoseEstimate = False
        self.strikerNav = False
        self.strikerPoseEstimate = False
        self.defenderNav = False
        self.defenderPoseEstimate = False
        self.goalkeeper.setStyleSheet("""
            #goalkeeper{
                color: rgb(0, 0, 255);
                background-color: rgb(0, 170, 0);
            }
        """)
        self.striker.setStyleSheet("""
            #striker{
                color: rgb(255, 0, 0);
            }
        """)
        self.defender.setStyleSheet("""
            #defender{
                color: rgb(0, 255, 0);
            }
        """)
    def goalkeeperPoseCall(self):
        self.robotNav = False
        self.robotPoseEstimate = True
        self.goalkeeperPoseEstimate = True
        self.goalkeeperNav = False
        self.strikerPoseEstimate = False
        self.strikerNav = False
        self.defenderPoseEstimate = False
        self.defenderNav = False
        self.goalkeeper.setStyleSheet("""
            #goalkeeper{
                color: rgb(0, 0, 255);
                background-color: rgb(0, 170, 0);
            }
        """)
        self.striker.setStyleSheet("""
            #striker{
                color: rgb(255, 0, 0);
            }
        """)
        self.defender.setStyleSheet("""
            #defender{
                color: rgb(0, 255, 0);
            }
        """)

    def turnFalse(self):
        self.striker.setStyleSheet("""
            #striker{
                color: rgb(255, 0, 0);
            }
        """)
        self.defender.setStyleSheet("""
            #defender{
                color: rgb(0, 255, 0);
            }
        """)
        self.goalkeeper.setStyleSheet("""
            #goalkeeper{
                color: rgb(0, 0, 255);
            }
        """)
        self.robotNav = False
        self.robotPoseEstimate = False
        self.strikerNav = False
        self.strikerPoseEstimate = False
        self.defenderNav = False
        self.defenderPoseEstimate = False
        self.goalkeeperNav = False
        self.goalkeeperPoseEstimate = False
        self.mouseClickInFieldArea = False

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.turnFalse()

    def eventFilter(self, object, event):
        if event.type() == QEvent.Enter:
            return True
        return False

    def mousePressEvent(self, QMouseEvent):
        self.mouseClickPos = [(QMouseEvent.y() - 50) * 7, (QMouseEvent.x() - 10) * 7]
        self.mouseMovePos[0] = self.mouseClickPos[0]
        self.mouseMovePos[1] = self.mouseClickPos[1]
        if self.mouseClickPos[0] >= 0 and self.mouseClickPos[1] >= 0 and self.mouseClickPos[0] <= 4201 and self.mouseClickPos[1] <= 3060:
            self.mouseClickInFieldArea = True
        else:
            self.mouseClickInFieldArea = False

    def mouseReleaseEvent(self, QMouseEvent):
        self.mouseReleasePos = [(QMouseEvent.y() - 50) * 7, (QMouseEvent.x() - 10) * 7]
        if self.mouseClickInFieldArea:
            delta_x = self.mouseReleasePos[0] - self.mouseClickPos[0]
            delta_y = self.mouseReleasePos[1] - self.mouseClickPos[1]
            orientation = quaternion_from_euler(0, 0, np.arctan2(delta_y, delta_x))
            if self.robotNav:
                if self.strikerNav:
                    self.robotGoalPublish("striker", self.mouseClickPos[0] * 0.0025, self.mouseClickPos[1] * 0.0025, orientation[2], orientation[3])
                elif self.defenderNav:
                    self.robotGoalPublish("defender", self.mouseClickPos[0] * 0.0025, self.mouseClickPos[1] * 0.0025, orientation[2], orientation[3])
                # elif self.goalkeeperNav:
                #     self.robotGoalPublish("goalkeeper", self.mouseClickPos[0] * 0.0025, self.mouseClickPos[1] * 0.0025, orientation[2], orientation[3])
            if self.robotPoseEstimate:
                if self.strikerPoseEstimate:
                    self.robotPoseEstimation("striker", self.mouseClickPos[0] * 0.0025, self.mouseClickPos[1] * 0.0025, orientation[2], orientation[3])
                elif self.defenderPoseEstimate:
                    self.robotPoseEstimation("defender", self.mouseClickPos[0] * 0.0025, self.mouseClickPos[1] * 0.0025, orientation[2], orientation[3])
                # elif self.goalkeeperPoseEstimate:
                #     self.robotPoseEstimation("striker", self.mouseClickPos[0] * 0.0025, self.mouseClickPos[1] * 0.0025, orientation[2], orientation[3])
        self.turnFalse()

    def mouseMoveEvent(self, event):
        if self.mouseClickInFieldArea == True:
            if (event.x() - 10) * 7 > 3060:
                self.mouseMovePos[1] = 3060
            elif (event.x() - 10) * 7 < 0:
                self.mouseMovePos[1] = 0
            else:
                self.mouseMovePos[1] = (event.x() - 10) * 7
            
            if (event.y() - 50) * 7 > 4201:
                self.mouseMovePos[0] = 4201
            elif (event.y() - 50) * 7 < 0:
                self.mouseMovePos[0] = 0
            else:
                self.mouseMovePos[0] = (event.y() - 50) * 7

            delta_x = self.mouseMovePos[0] - self.mouseClickPos[0]
            delta_y = self.mouseMovePos[1] - self.mouseClickPos[1]
            orientation = quaternion_from_euler(0, 0, np.arctan2(delta_y, delta_x))
            self.xy_coor.setText('X & Y & Z & W : ( %f : %f : %f : %f )' % (self.mouseClickPos[0] * 0.0025, self.mouseClickPos[1] * 0.0025, orientation[2], orientation[3])) 

if __name__ == "__main__":
    rospy.init_node('rostu_gui')
    striker_publish_goal = rospy.Publisher('/striker/move_base_simple/goal', PoseStamped, queue_size=1)  
    striker_pose_estimate = rospy.Publisher('/striker/initialpose', PoseWithCovarianceStamped, queue_size=1)   
    defender_publish_goal = rospy.Publisher('/defender/move_base_simple/goal', PoseStamped, queue_size=1)  
    defender_pose_estimate = rospy.Publisher('/defender/initialpose', PoseWithCovarianceStamped, queue_size=1)   
    # goalkeeper_publish_goal = rospy.Publisher('/goalkeeper/move_base_simple/goal', PoseStamped, queue_size=1)  
    # goalkeeper_pose_estimate = rospy.Publisher('/goalkeeper/initialpose', PoseWithCovarianceStamped, queue_size=1)    

    striker_cancel_goal = rospy.Publisher('/striker/move_base/cancel', GoalID, queue_size=1)
    defender_cancel_goal = rospy.Publisher('/defender/move_base/cancel', GoalID, queue_size=1)
    goalkeeper_cancel_goal = rospy.Publisher('/goalkeeper/move_base/cancel', GoalID, queue_size=1)

    app = QApplication(sys.argv)
    window = MainWindow()
    window.setWindowTitle('ROSTU')
    window.show()
    
    sys.exit(app.exec_())