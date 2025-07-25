import sys
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,  # widgets
    QPushButton,
)
from PyQt5.QtGui import QIcon, QFont


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Ros test Interface")
        self.setGeometry(700, 300, 3000, 2000)
        self.button = QPushButton("click", self)
        self.initUI()

        # label1.setAlignment(Qt.AlignTop)        # v. top
        # label1.setAlignment(Qt.AlignBottom)   # v.bottom
        # label1.setAlignment(Qt.AlignVCenter)   # v. cenetr
        # label1.setAlignment(Qt.AlignRight)   # horizontal right
        # label1.setAlignment(Qt.AlignLeft)   # horizontal right
        # label1.setAlignment(Qt.AlignHCenter)   # horizontal right

    def initUI(self):
        """creating panels:
        1. create widget  2.  add layout manager  3. add the widget to main window"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.labels = {
            "front": QLabel("Front Camera", self),
            "back": QLabel("Back Camera", self),
            "left": QLabel("Left Camera", self),
            "right": QLabel("Right Camera", self),
        }

        for label in self.labels.values():
            label.setScaledContents(True)
            label.setMinimumSize(320, 240)
            label.setFont(QFont("Arial", 12))
            label.setStyleSheet(
                "color:red;"
                "background-color : ;"
                "font-weight: bold;"
                "font-style: italic;"
            )

        # 4 cameras
        # creating labels
        label1 = QLabel("front camera", self)
        label1.setFont(QFont("Arial", 40))
        label1.setGeometry(0, 0, 200, 100)
        label1.setStyleSheet(
            "color:red;"
            "background-color : ;"
            "font-weight: bold;"
            "font-style: italic;"
        )
        label2 = QLabel("back camera", self)
        label3 = QLabel("right camera", self)
        label4 = QLabel("left camera", self)

        # creating grid with grid layout manager
        grid = QGridLayout()

        # assigning labels in grid
        # row  #column
        grid.addWidget(label1, 0, 0)
        grid.addWidget(label2, 0, 1)
        grid.addWidget(label3, 0, 2)
        grid.addWidget(label4, 0, 3)

        central_widget.setLayout(grid)

        """creating pushbutton"""
        self.button.setGeometry(150, 200, 200, 100)
        self.button.clicked.connect(self.button_1_click)  # set signal

    def button_1_click(self):
        self.button.setText("clicked")


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
