#!/usr/bin/env python

from __future__ import division

import numpy as np
import cv2
import imutils
import sys
import roslib
import rospy
import rospkg

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import PoseWithCovarianceStamped 
from geometry_msgs.msg import Pose 
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.uic import loadUi

class Worker(QRunnable):
    def __init__(self, fn):
        super(Worker, self).__init__()
        self.fn = fn
        
    def run(self):
        result = self.fn()

class rostu(QWidget):
    def __init__(self):
        super(rostu, self).__init__()
        self.path = rospkg.RosPack().get_path('rostu_gui')
        
        self.robotPose = Pose()

        self.initSub()
        self.initUI()

        self.setMouseTracking(True)
        self.mouseClickInFieldArea = False
        self.mouseClickPos = [0, 0]
        self.mouseReleasePos = [0, 0]
        self.mouseMovePos = [0, 0]
        self.robotNav = False
        self.robotPoseEstimate = False
        self.strikerNav = False
        self.strikerPoseEstimate = False
        self.defenderNav = False
        self.defenderPoseEstimate = False
        self.goalkeeperNav = False
        self.goalkeeperPoseEstimate = False

        self.strikerNavBtn.clicked.connect(self.strikerNavCall)
        self.strikerPosBtn.clicked.connect(self.strikerPoseCall)
        self.defenderNavBtn.clicked.connect(self.defenderNavCall)
        self.defenderPosBtn.clicked.connect(self.defenderPoseCall)
        self.goalkeeperNavBtn.clicked.connect(self.goalkeeperNavCall)
        self.goalkeeperPosBtn.clicked.connect(self.goalkeeperPoseCall)

        self.updateFieldTimer = QTimer()
        self.updateFieldTimer.timeout.connect(self.updateFieldThread)
        self.updateFieldTimer.start(1)
        
        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())

    def initUI(self):
        #Load Design UI Dari QtDesign
        loadUi(self.path + '/qt_ui/rostuUi.ui', self)
        #Setting Fixed Frame Window
        self.setFixedSize(1280, 680)

    def initSub(self):
        self.s_p_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.striker_pose_callback)

    def striker_pose_callback(self, data):
        self.robotPose = data.pose.pose

    def updateField(self):
        self.image = cv2.imread(self.path + '/media/field.png')
        height, width = self.image.shape[:2]
        
        if self.mouseClickInFieldArea and (self.robotNav or self.robotPoseEstimate):
            self.image = cv2.line(self.image, (self.mouseClickPos[1], self.mouseClickPos[0]), (self.mouseMovePos[1], self.mouseMovePos[0]), (0, 0, 255), 20)
        
        orientation_list = [self.robotPose.orientation.x, self.robotPose.orientation.y, self.robotPose.orientation.z, self.robotPose.orientation.w]
        (r, p, y) = euler_from_quaternion(orientation_list)
        
        heading = ((y / (2 * np.pi)) * 360)
        cosHeading = np.cos(np.radians(heading))
        sinHeading = np.sin(np.radians(heading))
        posX = int(self.robotPose.position.y / 0.0025)
        posY = int(self.robotPose.position.x / 0.0025)

        self.image = cv2.circle(self.image, (posX, posY), 100, (255, 0, 0), -1)
        self.image = cv2.line(self.image, (posX, posY), (posX + int(sinHeading * 100), posY + int(cosHeading * 100)), (0, 0, 255), 20)
        self.image = cv2.resize(self.image, (int(width / 7), int(height / 7)))

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

    def updateFieldThread(self):
        worker = Worker(self.updateField)
        self.threadpool.start(worker) 

    def strikerNavCall(self):
        self.robotNav = True
        self.robotPoseEstimate = False
        self.strikerNav = True
        self.striker.setStyleSheet("""
            #striker{
                background-color: rgb(0, 170, 0)
            }
        """)

    def strikerPoseCall(self):
        self.robotNav = False
        self.robotPoseEstimate = True
        self.strikerPoseEstimate = True
        self.striker.setStyleSheet("""
            #striker{
                background-color: rgb(0, 170, 0)
            }
        """)

    def defenderNavCall(self):
        self.robotNav = True
        self.robotPoseEstimate = False
        self.defenderNav = True
        self.defender.setStyleSheet("""
            #defender{
                background-color: rgb(0, 170, 0)
            }
        """)

    def defenderPoseCall(self):
        self.robotNav = False
        self.robotPoseEstimate = True
        self.defenderPoseEstimate = True
        self.defender.setStyleSheet("""
            #defender{
                background-color: rgb(0, 170, 0)
            }
        """)

    def goalkeeperNavCall(self):
        self.robotNav = True
        self.robotPoseEstimate = False
        self.goalkeeperNav = True
        self.goalkeeper.setStyleSheet("""
            #goalkeeper{
                background-color: rgb(0, 170, 0)
            }
        """)

    def goalkeeperPoseCall(self):
        self.robotNav = False
        self.robotPoseEstimate = True
        self.goalkeeperPoseEstimate = True
        self.goalkeeper.setStyleSheet("""
            #goalkeeper{
                background-color: rgb(0, 170, 0)
            }
        """)

    def turnFalse(self):
        self.striker.setStyleSheet("""""")
        self.defender.setStyleSheet("""""")
        self.goalkeeper.setStyleSheet("""""")
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
                goal_publish_msg = PoseStamped()
                goal_publish_msg.header.frame_id = "map"
                goal_publish_msg.pose.position.x = self.mouseClickPos[0] * 0.0025
                goal_publish_msg.pose.position.y = self.mouseClickPos[1] * 0.0025
                goal_publish_msg.pose.orientation.z = orientation[2]
                goal_publish_msg.pose.orientation.w = orientation[3]
                if self.strikerNav:
                    striker_publish_goal.publish(goal_publish_msg)
            if self.robotPoseEstimate:
                pose_estimate_publish_msg = PoseWithCovarianceStamped()
                pose_estimate_publish_msg.header.frame_id = "map"
                pose_estimate_publish_msg.pose.pose.position.x = self.mouseClickPos[0] * 0.0025
                pose_estimate_publish_msg.pose.pose.position.y = self.mouseClickPos[1] * 0.0025
                pose_estimate_publish_msg.pose.pose.orientation.z = orientation[2]
                pose_estimate_publish_msg.pose.pose.orientation.w = orientation[3]
                if self.strikerPoseEstimate:
                    striker_pose_estimate.publish(pose_estimate_publish_msg)
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

            self.xy_coor.setText('X & Y : ( %d : %d )' % (self.mouseMovePos[0], self.mouseMovePos[1]))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = rostu()
    window.setWindowTitle('ROSTU')
    window.show()

    rospy.init_node('rostu_gui')
    striker_publish_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)  
    striker_pose_estimate = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)    

    sys.exit(app.exec_())