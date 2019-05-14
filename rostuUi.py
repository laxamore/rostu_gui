from PyQt5.QtWidgets import QApplication, QDialog
from PyQt5.uic import loadUi

class rostu(QDialog):
    def __init__(self):
        super(rostu, self).__init__()
        #Load Design UI Dari QtDesign
        loadUi('rostuUi.ui', self)
        #Setting Fixed Frame Window
        self.setFixedSize(153, 133)
        
if __name__ == "__main__":
    import sys
    import rospy

    app = QApplication(sys.argv)
    window = rostu()
    window.setWindowTitle('ROSTU')
    window.show()
    sys.exit(app.exec_())
    



