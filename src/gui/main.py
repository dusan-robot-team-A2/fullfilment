import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import sys
from datetime import datetime
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication

from .app import MainWindow
from .sm_node import SystemMonitoringNode

class ROS2Thread(QThread):
    def __init__(self, node:Node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        # 스레드를 종료할 때 spin 종료
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    main_window = MainWindow()    
    sm_node = SystemMonitoringNode(main_window=main_window)

    #ros2 thread 실행
    ros2_thread = ROS2Thread(sm_node)
    ros2_thread.start()

    #ui표시
    main_window.show()

    sys.exit(app.exec_())

    ros2_thread.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()