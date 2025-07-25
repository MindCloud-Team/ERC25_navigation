#!/usr/bin/env python3

import sys
import math
import time
from queue import Queue, Empty
import rclpy
import vtk
import cv2

# --- MODIFICATION: Added specific QoS imports ---
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import BatteryState, Imu, PointCloud2, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped, Twist

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
    QGridLayout,
    QFrame,
    QSizePolicy,
    QGroupBox,
    QProgressBar,
    QSplitter,
    QPushButton,
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt5.QtGui import (
    QImage,
    QPixmap,
    QFont,
    QPainter,
    QColor,
    QBrush,
    QLinearGradient,
)

from sensor_msgs_py import point_cloud2
import vtkmodules.all as vtk
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


class ResizablePixmapLabel(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._full_pixmap = None
        self.setMinimumSize(240, 180)
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_full_pixmap(self, pixmap):
        self._full_pixmap = pixmap
        self.rescale_pixmap()

    def resizeEvent(self, event):
        if self._full_pixmap:
            self.rescale_pixmap()
        super().resizeEvent(event)

    def rescale_pixmap(self):
        if self._full_pixmap:
            self.setPixmap(
                self._full_pixmap.scaled(
                    self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
            )


# --- MODIFICATION: Consolidated ImageProcessor threads into one ---
class CombinedImageProcessor(QThread):
    """A single thread to process images from all cameras."""

    pixmap_ready = pyqtSignal(str, QPixmap)

    def __init__(self):
        super().__init__()
        # The queue will now store tuples of (camera_key, message)
        self.image_queue = Queue(maxsize=10)  # Increased size slightly
        self.is_running = True

    def add_image(self, camera_key, msg):
        if not self.image_queue.full():
            self.image_queue.put((camera_key, msg))

    def run(self):
        bridge = CvBridge()
        PRESCALE_WIDTH = 640
        while self.is_running:
            try:
                # Dequeue the camera key and the message
                camera_key, msg = self.image_queue.get(timeout=1)

                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                if cv_image.shape[1] > PRESCALE_WIDTH:
                    scale = PRESCALE_WIDTH / cv_image.shape[1]
                    dims = (PRESCALE_WIDTH, int(cv_image.shape[0] * scale))
                    cv_image = cv2.resize(cv_image, dims, interpolation=cv2.INTER_AREA)

                h, w, ch = cv_image.shape
                bpl = ch * w
                qt_image = QImage(cv_image.data, w, h, bpl, QImage.Format_RGB888)

                # Emit the key along with the pixmap
                self.pixmap_ready.emit(camera_key, QPixmap.fromImage(qt_image))
            except Empty:
                continue
            except Exception as e:
                print(f"Error in CombinedImageProcessor: {e}")

    def stop(self):
        self.is_running = False
        self.quit()


def quaternion_to_euler(x, y, z, w):
    t0 = 2 * (w * x + y * z)
    t1 = 1 - 2 * (x * x + y * y)
    r = math.atan2(t0, t1)
    t2 = 2 * (w * y - z * x)
    t2 = max(min(t2, 1), -1)
    p = math.asin(t2)
    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    y = math.atan2(t3, t4)
    return r, p, y


def generate_placeholder_pixmap(t, w=320, h=240):
    i = QImage(w, h, QImage.Format_RGB32)
    p = QPainter(i)
    g = QLinearGradient(0, 0, w, h)
    g.setColorAt(0, QColor(45, 45, 45))
    g.setColorAt(1, QColor(25, 25, 25))
    p.fillRect(0, 0, w, h, QBrush(g))
    p.setPen(QColor(70, 130, 180))
    p.drawRect(0, 0, w - 1, h - 1)
    p.setPen(QColor(200, 200, 200))
    f = QFont("Arial", 8, QFont.Bold)
    p.setFont(f)
    p.drawText(i.rect(), Qt.AlignCenter, t)
    p.end()
    return QPixmap.fromImage(i)


class StatusIndicator(QFrame):
    def __init__(self, p=None):
        super().__init__(p)
        self.setFixedSize(20, 20)
        self.c = QColor(128, 128, 128)

    def set_status(self, s):
        self.c = {
            "good": QColor(76, 175, 80),
            "warning": QColor(255, 193, 7),
            "error": QColor(244, 67, 54),
            "inactive": QColor(128, 128, 128),
        }.get(s, QColor(128, 128, 128))
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QBrush(self.c))
        p.setPen(Qt.NoPen)
        p.drawEllipse(3, 3, 16, 16)


class ImageSignal(QObject):
    update_lidar = pyqtSignal(list)
    raw_image_received = pyqtSignal(str, object)


class PantherSensorNode(Node):
    def __init__(self, image_signal):
        super().__init__("panther_dashboard_gui")
        self.image_signal = image_signal
        self.battery = self.imu = self.odom = self.cmd = self.e_stop = None
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(
            BatteryState, "/battery/battery_status", self.battery_cb, 10
        )
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)
        self.create_subscription(Odometry, "/odometry/wheels", self.odom_cb, 10)

        # --- MODIFICATION: Using a more robust QoS for the E-Stop status ---
        # This profile ensures we get the last published state upon connecting.
        estop_qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, "/hardware/e_stop", self.estop_cb, estop_qos_profile
        )

        self.create_subscription(TwistStamped, "/cmd_vel", self.cmd_cb, 10)
        cam_topics = {
            "front": "/front_cam/zed_node/rgb/image_rect_color",
            "back": "/back_cam/zed_node/rgb/image_rect_color",
            "left": "/left_cam/zed_node/rgb/image_rect_color",
            "right": "/right_cam/zed_node/rgb/image_rect_color",
        }
        for k, t in cam_topics.items():
            self.create_subscription(
                Image, t, self.callback_factory(k), qos_profile_sensor_data
            )
        self.create_subscription(
            PointCloud2, "/lidar/velodyne_points", self.lidar_cb, 10
        )

    def battery_cb(self, m):
        self.battery = m

    def imu_cb(self, m):
        self.imu = m

    def odom_cb(self, m):
        self.odom = m

    def estop_cb(self, m):
        self.e_stop = m.data

    def cmd_cb(self, m):
        self.cmd = m

    def lidar_cb(self, msg):
        try:
            point_iterator = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=False
            )
            valid_points = [p for p in point_iterator if all(map(math.isfinite, p))]
            if valid_points:
                self.image_signal.update_lidar.emit(valid_points)
        except Exception as e:
            self.get_logger().error(f"LiDAR filtering error:{e}")

    def callback_factory(self, k):
        def c(m):
            self.image_signal.raw_image_received.emit(k, m)

        return c


class PantherDashboard(QMainWindow):
    def __init__(self, node, image_signal):
        super().__init__()
        self.last_lidar_update_time = 0.0
        self._lidar_camera_initialized = False
        self.setFocusPolicy(Qt.StrongFocus)
        self.pressed_keys = set()

        # --- MODIFICATION: Create a single image processor ---
        self.image_processor = CombinedImageProcessor()
        self.image_processor.pixmap_ready.connect(self.update_camera_feed)
        self.image_processor.start()

        self.node = node
        self.setWindowTitle("Panther Robot Dashboard")
        self.setMinimumSize(1200, 800)
        self.setStyleSheet(
            """
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2C3E50, stop:1 #34495E);
            }
            QGroupBox {
                font-size: 16px;
                font-weight: bold;
                color: #ECF0F1;
                border: 2px solid #3498DB;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 20px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #34495E, stop:1 #2C3E50);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 2px 10px;
                background-color: #3498DB;
                color: #FFFFFF;
                border-radius: 4px;
            }
            QPushButton {
                background-color: #3498DB;
                color: white;
                border-radius: 5px;
                padding: 8px 12px;
                font-size: 14px;
                font-weight: bold;
                border: 1px solid #2980B9;
            }
            QPushButton:hover {
                background-color: #2980B9;
            }
            QProgressBar {
                border: 2px solid #555;
                border-radius: 10px;
                text-align: center;
                background: #2C3E50;
                color: white;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #E74C3C, stop:0.4 #F39C12, stop:1 #27AE60);
                border-radius: 8px;
            }
            QLabel {
                color: #BDC3C7;
                font-size: 14px;
                background: transparent;
                border: none;
            }
            QLabel[isTitle="true"] {
                 color: #ECF0F1;
                 font-size: 16px;
                 font-weight: bold;
            }
            """
        )
        self.image_signal = image_signal
        self.image_signal.update_lidar.connect(self.update_lidar_vtk)
        self.image_signal.raw_image_received.connect(self.distribute_raw_image)
        self.feeds, self.status_indicators, self.sensor_labels = {}, {}, {}
        self.setup_ui()
        self.drive_timer = QTimer()
        self.drive_timer.timeout.connect(self.send_cmd_vel)
        self.drive_timer.start(100)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_labels)
        self.update_timer.start(500)

    def distribute_raw_image(self, camera_key, msg):
        # --- MODIFICATION: Pass image to the single processor ---
        self.image_processor.add_image(camera_key, msg)

    def update_camera_feed(self, k, p):
        if k in self.feeds:
            self.feeds[k].set_full_pixmap(p)
            self.status_indicators[k].set_status("good")

    def update_lidar_vtk(self, points):
        current_time = time.time()
        if current_time - self.last_lidar_update_time < 0.1:
            return
        self.last_lidar_update_time = current_time
        if not hasattr(self, "vtk_points"):
            return
        self.vtk_points.SetNumberOfPoints(len(points))
        for i, p in enumerate(points):
            self.vtk_points.SetPoint(i, p[0], p[1], p[2])
        self.vtk_verts.Reset()
        [self.vtk_verts.InsertNextCell(1, [i]) for i in range(len(points))]
        self.vtk_points.Modified()
        self.vtk_verts.Modified()
        self.lidar_polydata.Modified()

        if not self._lidar_camera_initialized and points:
            self.renderer.ResetCamera(self.lidar_polydata.GetBounds())
            self.renderer.GetActiveCamera().Dolly(1.5)
            self._lidar_camera_initialized = True

        self.vtk_widget.GetRenderWindow().Render()

    def reset_lidar_view(self):
        self._lidar_camera_initialized = False
        print("LiDAR view reset requested. Will apply on next data frame.")

    def closeEvent(self, e):
        print("Stopping threads...")
        # --- MODIFICATION: Stop the single image processor ---
        self.image_processor.stop()
        self.image_processor.wait()
        super().closeEvent(e)

    def keyPressEvent(self, e):
        self.pressed_keys.add(e.key())

    def keyReleaseEvent(self, e):
        self.pressed_keys.discard(e.key())

    def send_cmd_vel(self):
        twist = TwistStamped()
        twist.header.stamp = self.node.get_clock().now().to_msg()
        speed, turn = 1.0, 0.5
        if Qt.Key_W in self.pressed_keys:
            twist.twist.linear.x += speed
        if Qt.Key_S in self.pressed_keys:
            twist.twist.linear.x -= speed
        if Qt.Key_A in self.pressed_keys:
            twist.twist.angular.z += turn
        if Qt.Key_D in self.pressed_keys:
            twist.twist.angular.z -= turn
        self.node.cmd_vel_pub.publish(twist)

    def setup_ui(self):
        cw = QWidget()
        self.setCentralWidget(cw)
        ms = QSplitter(Qt.Vertical)
        tc = QWidget()
        tl = QHBoxLayout(tc)
        tl.addWidget(self.create_camera_section(), 3)
        tl.addWidget(self.create_sensor_section(), 1)
        ms.addWidget(tc)
        ms.addWidget(self.create_lidar_section())
        ms.setSizes([500, 300])
        ml = QVBoxLayout(cw)
        ml.addWidget(ms)

    def create_camera_section(self):
        g = QGroupBox("Camera Feeds")
        l = QGridLayout(g)
        # --- MODIFICATION: No change needed here, logic is robust ---
        for r, c, n, k in [
            (0, 0, "Front", "front"),
            (0, 1, "Back", "back"),
            (1, 0, "Left", "left"),
            (1, 1, "Right", "right"),
        ]:
            l.addWidget(self.create_camera_widget(n, k), r, c)
        return g

    def create_camera_widget(self, n, k):
        f = QFrame()
        f.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        f.setStyleSheet(
            "QFrame{border:2px solid #4A90E2;border-radius:10px;padding:5px;}"
        )
        v = QVBoxLayout(f)
        h = QHBoxLayout()
        t = QLabel(f"{n} Camera")
        t.setStyleSheet("color:#ECF0F1;font-size:16px;font-weight:bold;border:none;")
        s = StatusIndicator()
        s.set_status("inactive")
        self.status_indicators[k] = s
        h.addWidget(t)
        h.addStretch()
        h.addWidget(s)
        v.addLayout(h)
        il = ResizablePixmapLabel()
        il.set_full_pixmap(generate_placeholder_pixmap(f"Waiting for {n}..."))
        v.addWidget(il)
        self.feeds[k] = il
        return f

    def create_sensor_section(self):
        g = QGroupBox("Sensor Data")
        l = QVBoxLayout(g)
        l.addWidget(self.create_battery_widget())
        for k, t, d in [
            ("imu", "IMU", "--"),
            ("odom", "Odometry", "--"),
            ("cmd", "Velocity", "--"),
            ("e_stop", "E-Stop", "--"),
        ]:
            f, lb = self.create_sensor_widget(t, d)
            self.sensor_labels[k] = lb
            l.addWidget(f)
        l.addStretch()
        return g

    def create_battery_widget(self):
        f = QFrame()
        f.setStyleSheet(
            "QFrame{background:#3A4A5C;border:1px solid #4A90E2;border-radius:10px;padding:10px;}"
        )
        l = QVBoxLayout(f)
        h = QHBoxLayout()
        t = QLabel("Battery")
        t.setStyleSheet("color:#ECF0F1;font-size:16px;font-weight:bold;border:none;")
        self.battery_status = StatusIndicator()
        self.battery_status.set_status("inactive")
        h.addWidget(t)
        h.addStretch()
        h.addWidget(self.battery_status)
        l.addLayout(h)
        self.battery_progress = QProgressBar()
        self.battery_progress.setRange(0, 100)
        self.battery_progress.setValue(0)
        self.battery_progress.setStyleSheet(
            "QProgressBar{border:2px solid #555;border-radius:10px;text-align:center;background:#2C3E50;color:white;}QProgressBar::chunk{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #E74C3C,stop:0.5 #F39C12,stop:1 #27AE60);border-radius:8px;}"
        )
        l.addWidget(self.battery_progress)
        self.battery_voltage = QLabel("Voltage: --")
        self.battery_voltage.setStyleSheet("color:#BDC3C7;font-size:14px;border:none;")
        l.addWidget(self.battery_voltage)
        return f

    def create_sensor_widget(self, t, d):
        f = QFrame()
        f.setStyleSheet(
            "QFrame{background:#3A4A5C;border:1px solid #4A90E2;border-radius:10px;padding:5px;}"
        )
        l = QVBoxLayout(f)
        tl = QLabel(t)
        tl.setStyleSheet("color:#ECF0F1;font-size:16px;font-weight:bold;border:none;")
        l.addWidget(tl)
        dl = QLabel(d)
        dl.setStyleSheet("color:#BDC3C7;font-size:14px;border:none;")
        dl.setWordWrap(True)
        l.addWidget(dl)
        return f, dl

    def create_lidar_section(self):
        group = QGroupBox("LiDAR Point Cloud")
        layout = QVBoxLayout(group)

        top_layout = QVBoxLayout()
        self.vtk_widget = QVTKRenderWindowInteractor(group)
        top_layout.addWidget(self.vtk_widget)

        bottom_layout = QHBoxLayout()
        reset_button = QPushButton("Reset View")
        reset_button.setStyleSheet(
            "QPushButton { background-color: #3498DB; color: white; border-radius: 5px; padding: 5px; } QPushButton:hover { background-color: #2980B9; }"
        )
        reset_button.clicked.connect(self.reset_lidar_view)
        bottom_layout.addStretch()
        bottom_layout.addWidget(reset_button)
        bottom_layout.addStretch()

        layout.addLayout(top_layout)
        layout.addLayout(bottom_layout)

        self.renderer = vtk.vtkRenderer()
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self.renderer.SetBackground(0.1, 0.1, 0.15)
        self.renderer.AddActor(self._add_grid())
        self.vtk_points = vtk.vtkPoints()
        self.vtk_verts = vtk.vtkCellArray()
        self.lidar_polydata = vtk.vtkPolyData()
        self.lidar_polydata.SetPoints(self.vtk_points)
        self.lidar_polydata.SetVerts(self.vtk_verts)
        m = vtk.vtkPolyDataMapper()
        m.SetInputData(self.lidar_polydata)
        self.lidar_actor = vtk.vtkActor()
        self.lidar_actor.SetMapper(m)
        self.lidar_actor.GetProperty().SetPointSize(0.1)
        self.lidar_actor.GetProperty().SetColor(0.2, 0.8, 1.0)
        self.renderer.AddActor(self.lidar_actor)
        c = self.renderer.GetActiveCamera()
        c.SetPosition(0, -10, 5)
        c.SetFocalPoint(0, 0, 0)
        c.SetViewUp(0, 0, 1)
        self.vtk_widget.Initialize()
        self.vtk_widget.Start()

        return group

    def _add_grid(self, s=1.0, e=10.0):
        p, l = vtk.vtkPoints(), vtk.vtkCellArray()
        ic = 0
        nl = int(2 * e / s) + 1
        for i in range(nl):
            pos = -e + i * s
            p.InsertNextPoint(pos, -e, 0)
            p.InsertNextPoint(pos, e, 0)
            l.InsertNextCell(2)
            l.InsertCellPoint(ic)
            l.InsertCellPoint(ic + 1)
            ic += 2
            p.InsertNextPoint(-e, pos, 0)
            p.InsertNextPoint(e, pos, 0)
            l.InsertNextCell(2)
            l.InsertCellPoint(ic)
            l.InsertCellPoint(ic + 1)
            ic += 2
        gpd = vtk.vtkPolyData()
        gpd.SetPoints(p)
        gpd.SetLines(l)
        m = vtk.vtkPolyDataMapper()
        m.SetInputData(gpd)
        a = vtk.vtkActor()
        a.SetMapper(m)
        a.GetProperty().SetColor(0.3, 0.5, 0.7)
        return a

    def update_labels(self):
        n = self.node
        if n.battery:
            p = n.battery.percentage * 100
            self.battery_progress.setValue(int(p))
            self.battery_voltage.setText(f"V: {n.battery.voltage:.1f}V")
            self.battery_status.set_status(
                "good" if p > 50 else "warning" if p > 20 else "error"
            )
        if n.imu:
            q = n.imu.orientation
            r, p, y = quaternion_to_euler(q.x, q.y, q.z, q.w)
            self.sensor_labels["imu"].setText(
                f"R:{math.degrees(r):.1f}° P:{math.degrees(p):.1f}° Y:{math.degrees(y):.1f}°"
            )
        if n.odom:
            pos = n.odom.pose.pose.position
            self.sensor_labels["odom"].setText(f"X:{pos.x:.2f}m, Y:{pos.y:.2f}m")
        if n.cmd:
            lin, ang = n.cmd.twist.linear, n.cmd.twist.angular
            self.sensor_labels["cmd"].setText(
                f"LinX:{lin.x:.2f}m/s, AngZ:{ang.z:.2f}r/s"
            )
        # --- MODIFICATION: This logic is correct, the QoS change makes it work ---
        if n.e_stop is not None:
            s = "ACTIVE" if n.e_stop else "INACTIVE"
            self.sensor_labels["e_stop"].setText(s)
            self.sensor_labels["e_stop"].setStyleSheet(
                f"color:{'#E74C3C'if n.e_stop else'#27AE60'}; font-weight: bold; border: none;"
            )


class RosSpinThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)


def main():
    rclpy.init()
    image_signal = ImageSignal()
    node = PantherSensorNode(image_signal)
    spin_thread = RosSpinThread(node)
    spin_thread.start()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = PantherDashboard(node, image_signal)
    gui.show()
    QTimer.singleShot(100, lambda: gui.vtk_widget.GetRenderWindow().Render())
    exit_code = app.exec_()
    gui.closeEvent(None)
    node.get_logger().info("GUI closed, shutting down ROS.")
    rclpy.shutdown()
    spin_thread.wait()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
