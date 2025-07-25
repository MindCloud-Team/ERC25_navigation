import sys
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QWidget,
    QVBoxLayout,
    QGridLayout,
    QFrame,
    QScrollArea,
)

from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QImage, QPixmap

from sensor_msgs.msg import Image


class ImageSignal(QObject):
    # Signal to send image updates to the GUI thread
    update = pyqtSignal(str, QPixmap)


class CameraSubscriber(Node):
    def __init__(self, image_signal: ImageSignal):
        super().__init__("camera_gui_node")
        self.bridge = CvBridge()
        self.image_signal = image_signal

        self.create_subscription(
            Image, "/lights/channel_1_frame", self.callback_factory("front"), 10
        )
        self.create_subscription(
            Image, "/back_camera/image_raw", self.callback_factory("back"), 10
        )
        self.create_subscription(
            Image, "/left_camera/image_raw", self.callback_factory("left"), 10
        )
        self.create_subscription(
            Image, "/right_camera/image_raw", self.callback_factory("right"), 10
        )

    def callback_factory(self, key):
        """creates callback function for each subscription
        it converts the recieved images into video by
        1. convert the incoming ROS Image message to an OpenCV-compatible format
        2. Converts the OpenCV image into a QImage
        3. Converts the QImage to a QPixmap to be shown with QLabel
        """

        def callback(msg):
            try:
                # convert the incoming ROS Image message to an OpenCV-compatible format.
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                h, w, ch = cv_image.shape
                bytes_per_line = (
                    ch * w
                )  # Calculates bytes of one row, required for QImage

                # Converts the OpenCV image into a QImage
                qt_image = QImage(
                    cv_image.data, w, h, bytes_per_line, QImage.Format_BGR888
                )

                # Converts the QImage to a QPixmap
                pixmap = QPixmap.fromImage(qt_image)

                # sends signal to update the image on the gui
                self.image_signal.update.emit(key, pixmap)

            except Exception as e:
                self.get_logger().error(f"Error processing {key} image: {e}")

        return callback


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS Test GUI")
        self.setGeometry(200, 200, 1600, 800)

        self.feeds = {}  # Dictionary of camera labels
        self.grid = QGridLayout()
        self.initUI()

    def initUI(self):
        """Setup all camera blocks and a sensor placeholder"""
        # Creating the 2×2 camera grid layout
        self.create_camera_block("Front Camera", 0, 0)
        self.create_camera_block("Back Camera", 0, 1)
        self.create_camera_block("Left Camera", 1, 0)
        self.create_camera_block("Right Camera", 1, 1)

        # creating the main layout (vertical)
        layout = QVBoxLayout()
        # adding the cameras grid at top
        layout.addLayout(self.grid)

        sensor_placeholder = QLabel("Sensor data will appear here.")
        sensor_placeholder.setAlignment(Qt.AlignCenter)
        sensor_placeholder.setStyleSheet(
            "color: gray; font-size: 16px; margin-top: 20px;"
        )

        # adding the sensor data widgets at bottom
        layout.addWidget(sensor_placeholder)

        # adding the layout to a central widget
        central_widget = QWidget()
        central_widget.setLayout(layout)

        # add scroll area to support overflow
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(central_widget)

        # Setting this full layout as the main GUI
        self.setCentralWidget(scroll)

    def create_camera_block(self, name, row, col):
        """
        Create an individual camera display box
        it contains title and image viewer box
        """
        title = QLabel(name)
        title.setStyleSheet("font-weight: bold; font-size: 18px;")
        title.setAlignment(Qt.AlignHCenter)

        image = QLabel()
        image.setFrameStyle(QFrame.Box)
        image.setScaledContents(True)
        image.setMinimumSize(320, 240)

        # creating vertical layout to hold title and image below it
        vbox = QVBoxLayout()
        vbox.addWidget(title)
        vbox.addWidget(image)

        # adding the layout to a Qwidget
        frame = QWidget()
        frame.setLayout(vbox)

        # adds the images widgets to the grid
        self.grid.addWidget(frame, row, col)
        self.feeds[name.lower().split()[0]] = image

    def update_feed(self, key, pixmap):
        if key in self.feeds:
            self.feeds[key].setPixmap(pixmap)


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()

    app = QApplication(sys.argv)
    window = MainWindow()

    # Signal for image updates
    image_signal = ImageSignal()
    image_signal.update.connect(window.update_feed)

    # Launch ROS node in a separate thread
    ros_node = CameraSubscriber(image_signal)
    thread = QThread()
    thread.run = lambda: rclpy.spin(ros_node)
    thread.start()

    window.show()
    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
