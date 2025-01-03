import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import sys
from datetime import datetime
from PyQt5.QtCore import QThread, pyqtSignal, QVariant
from PyQt5.QtWidgets import QApplication

from .app import MainWindow
from .sm_node import SystemMonitoringNode

class ROS2Thread(QThread):
    robot_status_signal = pyqtSignal(int)
    conveyor_status_signal = pyqtSignal(int)
    global_cam_signal = pyqtSignal(QVariant)
    robot_cam_signal = pyqtSignal(QVariant)

    def __init__(self, node: SystemMonitoringNode):
        super().__init__()
        self.node = node
        from cv_bridge import CvBridge
        bridge = CvBridge()
        self.node.init_qthread_variables(bridge,self.robot_status_signal,self.conveyor_status_signal, self.global_cam_signal, self.robot_cam_signal)

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        # 스레드를 종료할 때 spin 종료
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    sm_node = SystemMonitoringNode()
    ros2_thread = ROS2Thread(sm_node)
    main_window = MainWindow(sm_node, ros2_thread)
    
    # ros2 thread 실행
    ros2_thread.start()

    # UI 표시
    main_window.show()

    # 애플리케이션 실행
    exit_code = app.exec_()

    # 앱 종료 후 스레드 종료 및 ROS2 정리
    ros2_thread.stop()
    rclpy.shutdown()

    sys.exit(exit_code)
    

if __name__ == "__main__":
    main()
