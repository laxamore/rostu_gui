from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.uic import loadUi
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QEvent

class rostu(QWidget):
    def __init__(self):
        super(rostu, self).__init__()
        self.initUI()
        self.setMouseTracking(True)
        self.fieldHover = False

    def initUI(self):
        #Load Design UI Dari QtDesign
        loadUi('rostuUi.ui', self)
        #Setting Fixed Frame Window
        self.setFixedSize(1280, 720)

        pixmap = QPixmap('field.png')
        pixmap = pixmap.scaled(pixmap.width() / 6, pixmap.height() / 6)
        self.label.setPixmap(pixmap)
        self.label.resize(pixmap.width(), pixmap.height())
        self.label.installEventFilter(self)

    def eventFilter(self, object, event):
        if event.type() == QEvent.Enter:
            print(True)
            self.fieldHover = True
            return True
        elif event.type() == QEvent.Leave:
            print(False)
            self.fieldHover = False
        return False

    def mouseMoveEvent(self, event):
        if self.fieldHover == True:
            xPos = 0
            yPos = 0
            if (event.x() - 50) * 6 > 4201:
                xPos = 4201
            elif (event.x() - 50) * 6 < 0:
                xPos = 0
            else:
                xPos = (event.x() - 50) * 6
            
            if (event.y() - 50) * 6 > 3060:
                yPos = 3060
            elif (event.y() - 50) * 6 < 0:
                yPos = 0
            else:
                yPos = (event.y() - 50) * 6

            print('Mouse coords: ( %d : %d )' % (xPos, yPos))
        
if __name__ == "__main__":
    import sys
    import rospy

    app = QApplication(sys.argv)
    window = rostu()
    window.setWindowTitle('ROSTU')
    window.show()
    sys.exit(app.exec_())
    



