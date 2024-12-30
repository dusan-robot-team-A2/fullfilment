import sys
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton
from .sm_node import SystemMonitoringNode


class Worker(QThread):
    # 백그라운드 작업이 끝났을 때 메인 UI로 신호를 보내기 위한 시그널
    result_signal = pyqtSignal(str)

    def run(self):
        # 백그라운드 작업 (예: 3초 대기)
        import time
        time.sleep(3)
        # 작업 완료 후 메인 UI로 결과를 전달
        self.result_signal.emit("작업 완료!")


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PyQt Thread Test")
        self.setGeometry(100, 100, 300, 150)

        # UI 구성
        self.label = QLabel("작업을 시작하려면 버튼을 클릭하세요.", self)
        self.label.setAlignment(Qt.AlignCenter)

        self.button = QPushButton("작업 시작", self)
        self.button.clicked.connect(self.start_work)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.button)

        self.setLayout(layout)

        # 워커 스레드 인스턴스
        self.worker = Worker()
        # 워커 스레드의 결과가 메인 UI로 전달될 때 처리하는 슬롯
        self.worker.result_signal.connect(self.update_label)

    def start_work(self):
        # 작업 시작
        self.label.setText("작업 중...")
        self.worker.start()  # 백그라운드 작업 시작

    def update_label(self, result):
        # 작업 완료 후 레이블 업데이트
        self.label.setText(result)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
