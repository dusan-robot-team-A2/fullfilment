import sys

from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import pyqtSlot, QFile, QTextStream, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage

from .widgets.main_window import Ui_MainWindow
import threading

class MainWindow(QMainWindow):
    job_signal = pyqtSignal(int, int, int)
    conveyor_signal = pyqtSignal(bool)

    def __init__(self):
        super(MainWindow, self).__init__()

        #Variable init
        self.robot_status:int = None
        self.conveyor_status:bool = None

        #Ui default settings
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.global_cam = self.ui.globalCam
        self.robot_cam = self.ui.robotCam

        self.first_job_button = self.ui.firstJobButton
        self.second_job_button = self.ui.secondJobButton
        self.third_job_button = self.ui.thirdJobButton

        self.conveyor_play_button = self.ui.convayorPlayButton
        self.conveyor_stop_button = self.ui.convayorStopButton
        self.collect_button = self.ui.collectButton
        
        self.status_info = self.ui.status_info

        self.first_job_button.clicked.connect(lambda: self.send_job(2,1,1))
        self.second_job_button.clicked.connect(lambda: self.send_job(1,2,2))
        self.third_job_button.clicked.connect(lambda: self.send_job(0,1,3))

        self.conveyor_play_button.clicked.connect(lambda: self.send_conveyor_request(True))
        self.conveyor_stop_button.clicked.connect(lambda: self.send_conveyor_request(False))

    def render_status(self):
        text = f"Robot Status: {self.robot_status}"
        text += f"\nConveyor Status: {self.conveyor_status}"
        self.status_info.setText(text)

    def set_robot_status(self, status:int):
        self.robot_status = status
        self.render_status()
    
    def set_conveyor_status(self, status:bool):
        self.conveyor_status = status
        self.render_status()

    def set_global_image(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qt_image)

        target_width = self.global_cam.width()
        target_height = self.global_cam.height()
        scaled_pixmap = pixmap.scaled(target_width, target_height, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        self.global_cam.setPixmap(scaled_pixmap)
    
    def set_robot_image(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qt_image)

        target_width = self.robot_cam.width()
        target_height = self.robot_cam.height()
        scaled_pixmap = pixmap.scaled(target_width, target_height, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        self.robot_cam.setPixmap(scaled_pixmap)
    
    def send_job(self,red_num, blue_num, goal_num):
        self.job_signal.emit(red_num, blue_num, goal_num)

    def send_conveyor_request(self, status):
        self.conveyor_signal.emit(status)

if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())