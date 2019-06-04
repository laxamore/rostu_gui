#!/usr/bin/env python

import rospkg

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
    path = rospkg.RosPack().get_path('rostu_gui')

    def __init__(self):
        super(rostu, self).__init__()
        self.initUI()
        self.setMouseTracking(True)
        self.fieldHover = False

        self.testBtn.clicked.connect(self.testFunc)
        
        self.updateFieldTimer = QTimer()
        self.updateFieldTimer.timeout.connect(self.updateFieldThread)
        self.updateFieldTimer.start(1)
        
        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())
        
    def updateField(self):
        self.image = cv2.imread(self.path + '/media/field.png')
        height, width, byteValue = self.image.shape
        byteValue = byteValue * width

        cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB, self.image)
        self.mQImage = QImage(self.image, width, height, byteValue, QImage.Format_RGB888)

        pixmap = QPixmap(self.mQImage)
        pixmap = pixmap.scaled(pixmap.width() / 7, pixmap.height() / 7)
        self.field.setPixmap(pixmap)
        self.field.resize(pixmap.width(), pixmap.height())
        self.field.installEventFilter(self)

    def updateFieldThread(self):
        worker = Worker(self.updateField)
        self.threadpool.start(worker) 

    def initUI(self):
        #Load Design UI Dari QtDesign
        loadUi(self.path + '/qt_ui/rostuUi.ui', self)
        #Setting Fixed Frame Window
        self.setFixedSize(1280, 680)

    def eventFilter(self, object, event):
        if event.type() == QEvent.Enter:
            self.fieldHover = True
            return True
        elif event.type() == QEvent.Leave:
            self.fieldHover = False
        return False

    def mouseMoveEvent(self, event):
        if self.fieldHover == True:
            xPos = 0
            yPos = 0
            if (event.x() - 10) * 7 > 3060:
                yPos = 3060
            elif (event.x() - 10) * 7 < 0:
                yPos = 0
            else:
                yPos = (event.x() - 10) * 7
            
            if (event.y() - 50) * 7 > 4201:
                xPos = 4201
            elif (event.y() - 50) * 7 < 0:
                xPos = 0
            else:
                xPos = (event.y() - 50) * 7

            self.xy_coor.setText('X & Y : ( %d : %d )' % (xPos, yPos))

    def testFunc(self):
        goal_publish_msg = PoseStamped()
        goal_publish_msg.header.frame_id = "map"
        goal_publish_msg.pose.position.x = 5.25402927399
        goal_publish_msg.pose.position.y = 4.76900291443
        goal_publish_msg.pose.orientation.z = -0.711607695391
        goal_publish_msg.pose.orientation.w = 0.702577033399
        publish_goal.publish(goal_publish_msg)
        # rate.sleep()


        
if __name__ == "__main__":
    import cv2
    import sys
    import rospy

    from geometry_msgs.msg import PoseStamped 

    rospy.init_node('rostu_gui')
    publish_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    # rate = rospy.Rate(30)

    app = QApplication(sys.argv)
    window = rostu()
    window.setWindowTitle('ROSTU')
    window.show()
    sys.exit(app.exec_())
    



