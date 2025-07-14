#!/usr/bin/env python3

import sys
import math
import rclpy
import vtk

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import BatteryState, Imu, CompressedImage, PointCloud2, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout,
    QWidget, QGridLayout, QFrame, QScrollArea, QSizePolicy, QGroupBox,
    QProgressBar, QSplitter
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QImage, QPixmap, QFont, QPainter, QColor, QPalette, QLinearGradient, QBrush

from sensor_msgs_py import point_cloud2
import vtkmodules.all as vtk
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

def quaternion_to_euler(x, y, z, w):
    """calculates the yaw angle, the roll of x,the pitch of y and return them """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, +1.0), -1.0)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def generate_placeholder_pixmap(text, width=320, height=240):
    """Returns a QPixmap with modern gradient background and styled text"""
    image = QImage(width, height, QImage.Format_RGB32)
    
    # Create gradient background
    painter = QPainter(image)
    gradient = QLinearGradient(0, 0, width, height)
    gradient.setColorAt(0, QColor(45, 45, 45))
    gradient.setColorAt(1, QColor(25, 25, 25))
    painter.fillRect(0, 0, width, height, QBrush(gradient))
    
    # Draw border
    painter.setPen(QColor(70, 130, 180))
    painter.drawRect(0, 0, width-1, height-1)
    
    # Draw text
    painter.setPen(QColor(200, 200, 200))
    font = QFont("Arial", 14, QFont.Bold)
    painter.setFont(font)
    painter.drawText(image.rect(), Qt.AlignCenter, text)
    painter.end()

    return QPixmap.fromImage(image)


class AnimatedLabel(QLabel):
    """Custom label with fade animation for value changes"""
    def __init__(self, text="", parent=None):
        super().__init__(text, parent)
        self.animation = None
        self.pending_text = ""
    
    def animate_update(self, new_text):
        """Animate text change with fade effect"""
        if self.text() != new_text:
            self.pending_text = new_text
            if self.animation is not None:
                self.animation.stop()
                self.animation.deleteLater()
            
            self.animation = QPropertyAnimation(self, b"windowOpacity")
            self.animation.setDuration(200)
            self.animation.setStartValue(1.0)
            self.animation.setEndValue(0.7)
            self.animation.finished.connect(self._finish_animation)
            self.animation.start()
    
    def _finish_animation(self):
        """Complete the animation by setting new text and fading back in"""
        if self.pending_text:
            self.setText(self.pending_text)
            self.pending_text = ""
        
        if self.animation is not None:
            self.animation.stop()
            self.animation.deleteLater()
        
        self.animation = QPropertyAnimation(self, b"windowOpacity")
        self.animation.setDuration(200)
        self.animation.setStartValue(0.7)
        self.animation.setEndValue(1.0)
        self.animation.start()


class StatusIndicator(QFrame):
    """Custom status indicator with colored dot"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(20, 20)
        self.status_color = QColor(128, 128, 128)  # Default gray
        
    def set_status(self, status):
        """Set status color: 'good', 'warning', 'error', 'inactive'"""
        colors = {
            'good': QColor(76, 175, 80),      # Green
            'warning': QColor(255, 193, 7),   # Yellow
            'error': QColor(244, 67, 54),     # Red
            'inactive': QColor(128, 128, 128) # Gray
        }
        self.status_color = colors.get(status, colors['inactive'])
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setBrush(QBrush(self.status_color))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(2, 2, 16, 16)


class ImageSignal(QObject):
    """Signal to send image updates to the GUI thread"""
    update = pyqtSignal(str, QPixmap)
    update_lidar = pyqtSignal(list)  # emits list of (x,y,z)


class PantherSensorNode(Node):
    """class for creating nodes and subscriptions"""
    def __init__(self, image_signal):
        super().__init__('panther_dashboard_gui')
        #camera data
        self.bridge = CvBridge()
        self.image_signal = image_signal

        #sensor data
        self.battery = None
        self.imu = None
        self.odom = None
        self.cmd = None
        self.e_stop = None

        #lidar data
        self.lidar_data= []

        #sensors subscriptions
        self.create_subscription(BatteryState, '/battery/battery_status', self.battery_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Odometry, '/odometry/wheels', self.odom_cb, 10)
        self.create_subscription(Bool, '/hardware/e_stop', self.estop_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 10)

        self.create_subscription(Image, '/front_cam/zed_node/rgb/image_rect_color', self.callback_factory('front'), 10)
        self.create_subscription(Image, '/back_cam/zed_node/rgb/image_rect_color', self.callback_factory('back'), 10)
        self.create_subscription(Image, '/left_cam/zed_node/rgb/image_rect_color', self.callback_factory('left'), 10)
        self.create_subscription(Image, '/right_cam/zed_node/rgb/image_rect_color', self.callback_factory('right'), 10)

        #lidar subscription
        self.create_subscription( PointCloud2, '/lidar/velodyne_points', self.lidar_cb,10)


    def battery_cb(self, msg): self.battery = msg
    def imu_cb(self, msg):     self.imu = msg
    def odom_cb(self, msg):    self.odom = msg
    def estop_cb(self, msg):   self.e_stop = msg.data
    def cmd_cb(self, msg):     self.cmd = msg

    def lidar_cb(self, msg):
        """extract lidar msg as points"""
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.image_signal.update_lidar.emit(points)


    def callback_factory(self, key):
        """creates callback function for each image subscription"""
        def callback(msg):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                h, w, ch = cv_image.shape
                bytes_per_line = ch * w
               
                qt_image = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_BGR888)
                pixmap = QPixmap.fromImage(qt_image)
                self.image_signal.update.emit(key, pixmap)

            except Exception as e:
                self.get_logger().error(f'Error processing {key} image: {e}')
        return callback


class PantherDashboard(QMainWindow):
    """Enhanced GUI class with modern styling"""
    def __init__(self, node, image_signal):
        super().__init__()
        self.MAX_IMG_WIDTH = 400
        self.MAX_IMG_HEIGHT = 300
        self.MIN_IMG_WIDTH = 320
        self.MIN_IMG_HEIGHT = 240

        self.node = node
        self.setWindowTitle("🐾 Panther Robot Dashboard")
        self.setMinimumSize(1400, 900)
        
        # Set modern dark theme
        self.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #2C3E50, stop:1 #34495E);
            }
            QGroupBox {
                font-size: 16px;
                font-weight: bold;
                color: #ECF0F1;
                border: 2px solid #3498DB;
                border-radius: 10px;
                margin: 10px 0px;
                padding-top: 20px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #34495E, stop:1 #2C3E50);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 10px 0 10px;
                color: #3498DB;
            }
        """)
        
        self.image_signal = image_signal
        self.image_signal.update.connect(self.update_feed)
        self.image_signal.update_lidar.connect(self.update_lidar_vtk)

        self.camera_labels = {}
        self.feeds = {}
        self.status_indicators = {}

        self.setup_ui()

        # Timers
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_labels)
        self.update_timer.start(500)

        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self.spin_once)
        self.spin_timer.start(50)

    def setup_ui(self):
        """Setup the main UI layout"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main splitter for resizable panels
        main_splitter = QSplitter(Qt.Vertical)
        
        # Top section: Cameras and Sensors
        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)
        
        # Camera section
        camera_group = self.create_camera_section()
        top_layout.addWidget(camera_group, 2)  # 2/3 of width
        
        # Sensor section
        sensor_group = self.create_sensor_section()
        top_layout.addWidget(sensor_group, 1)  # 1/3 of width
        
        main_splitter.addWidget(top_widget)
        
        # Bottom section: LiDAR
        lidar_group = self.create_lidar_section()
        main_splitter.addWidget(lidar_group)
        
        # Set initial sizes
        main_splitter.setSizes([500, 400])
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        main_layout.addWidget(main_splitter)
        main_layout.setContentsMargins(10, 10, 10, 10)

    def create_camera_section(self):
        """Create the camera feeds section"""
        group = QGroupBox("📹 Camera Feeds")
        layout = QGridLayout(group)
        layout.setSpacing(15)
        
        cameras = [
            ("Front Camera", 0, 0, "front"),
            ("Back Camera", 0, 1, "back"),
            ("Left Camera", 1, 0, "left"),
            ("Right Camera", 1, 1, "right")
        ]
        
        for name, row, col, key in cameras:
            frame = self.create_camera_widget(name, key)
            layout.addWidget(frame, row, col)
            
        return group

    def create_camera_widget(self, name, key):
        """Create an individual camera display with modern styling"""
        frame = QFrame()
        frame.setFrameStyle(QFrame.Box)
        frame.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #3A4A5C, stop:1 #2C3E50);
                border: 2px solid #4A90E2;
                border-radius: 15px;
                padding: 10px;
            }
        """)
        
        layout = QVBoxLayout(frame)
        
        # Header with title and status
        header = QHBoxLayout()
        
        title = QLabel(name)
        title.setStyleSheet("""
            QLabel {
                color: #ECF0F1;
                font-size: 14px;
                font-weight: bold;
                background: transparent;
                border: none;
            }
        """)
        title.setAlignment(Qt.AlignLeft)
        
        status = StatusIndicator()
        status.set_status('inactive')
        self.status_indicators[key] = status
        
        header.addWidget(title)
        header.addStretch()
        header.addWidget(status)
        
        layout.addLayout(header)
        
        # Image display
        image_label = QLabel()
        image_label.setAlignment(Qt.AlignCenter)
        image_label.setScaledContents(True)
        image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        image_label.setMinimumSize(self.MIN_IMG_WIDTH, self.MIN_IMG_HEIGHT)
        image_label.setStyleSheet("""
            QLabel {
                border: 1px solid #555;
                border-radius: 8px;
                background: #1A1A1A;
            }
        """)
        image_label.setPixmap(generate_placeholder_pixmap(f"Waiting for {name}..."))
        
        layout.addWidget(image_label)
        self.feeds[key] = image_label
        
        return frame

    def create_sensor_section(self):
        """Create the sensor data section"""
        group = QGroupBox("📊 Sensor Data")
        layout = QVBoxLayout(group)
        layout.setSpacing(10)
        
        # Battery with progress bar
        battery_frame = self.create_battery_widget()
        layout.addWidget(battery_frame)
        
        # Other sensors
        self.sensor_labels = {}
        sensors = [
            ('imu', '🧭 IMU', 'IMU: --'),
            ('odom', '📍 Odometry', 'Odometry: --'),
            ('cmd', '🎮 Velocity', 'Velocity: --'),
            ('e_stop', '🛑 Emergency Stop', 'Emergency Stop: --')
        ]
        
        for key, title, default_text in sensors:
            # This line now correctly receives the two items
            frame, label = self.create_sensor_widget(title, default_text)
            
            # Store the label for later updates
            self.sensor_labels[key] = label
            
            # Add the container frame to the layout
            layout.addWidget(frame)
            
        layout.addStretch()
        return group

    def create_battery_widget(self):
        """Create battery widget with progress bar"""
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #3A4A5C, stop:1 #2C3E50);
                border: 2px solid #4A90E2;
                border-radius: 10px;
                padding: 10px;
            }
        """)
        
        layout = QVBoxLayout(frame)
        
        # Header
        header = QHBoxLayout()
        title = QLabel("🔋 Battery")
        title.setStyleSheet("color: #ECF0F1; font-size: 14px; font-weight: bold;")
        
        self.battery_status = StatusIndicator()
        self.battery_status.set_status('inactive')
        
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.battery_status)
        layout.addLayout(header)
        
        # Progress bar
        self.battery_progress = QProgressBar()
        self.battery_progress.setRange(0, 100)
        self.battery_progress.setValue(0)
        self.battery_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555;
                border-radius: 5px;
                text-align: center;
                background: #2C3E50;
                color: white;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #E74C3C, stop:0.5 #F39C12, stop:1 #27AE60);
                border-radius: 3px;
            }
        """)
        layout.addWidget(self.battery_progress)
        
        # Voltage label
        self.battery_voltage = QLabel("Voltage: --")
        self.battery_voltage.setStyleSheet("color: #BDC3C7; font-size: 12px;")
        layout.addWidget(self.battery_voltage)
        
        return frame

    def create_sensor_widget(self, title, default_text):
        """Create a sensor display widget"""
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #3A4A5C, stop:1 #2C3E50);
                border: 2px solid #4A90E2;
                border-radius: 10px;
                padding: 10px;
                margin: 2px;
            }
        """)
        
        layout = QVBoxLayout(frame)
        
        # Title
        title_label = QLabel(title)
        title_label.setStyleSheet("""
            QLabel {
                color: #ECF0F1;
                font-size: 14px;
                font-weight: bold;
                background: transparent;
                border: none;
            }
        """)
        layout.addWidget(title_label)
        
        # Data label
        data_label = QLabel(default_text)
        data_label.setStyleSheet("""
            QLabel {
                color: #BDC3C7;
                font-size: 12px;
                background: transparent;
                border: none;
                padding: 5px;
            }
        """)
        data_label.setWordWrap(True)
        layout.addWidget(data_label)
        
        # *** THIS IS THE LINE YOU NEED TO CHANGE ***
        # It must return both the frame and the label.
        return frame, data_label

    def create_lidar_section(self):
        """Create the LiDAR visualization section"""
        group = QGroupBox("📡 LiDAR Point Cloud")
        layout = QVBoxLayout(group)
        
        # VTK widget setup
        self.vtk_widget = QVTKRenderWindowInteractor(group)
        self.vtk_widget.setMinimumHeight(300)
        self.renderer = vtk.vtkRenderer()
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        
        # Setup renderer
        self.renderer.SetBackground(0.1, 0.1, 0.15)  # Dark blue background
        
        # Add grid
        self.grid_actor = self._add_grid()
        self.renderer.AddActor(self.grid_actor)
        
        # Add placeholder text
        text_actor = vtk.vtkTextActor()
        text_actor.SetInput("Waiting for LiDAR data...")
        text_actor.GetTextProperty().SetFontSize(18)
        text_actor.GetTextProperty().SetColor(0.8, 0.8, 0.8)
        text_actor.SetDisplayPosition(50, 50)
        self.renderer.AddActor2D(text_actor)
        self.placeholder_text_actor = text_actor
        
        self.vtk_widget.Initialize()
        self.vtk_widget.Start()
        
        # Render timer
        self.vtk_timer = QTimer()
        self.vtk_timer.timeout.connect(self.vtk_widget.GetRenderWindow().Render)
        self.vtk_timer.start(1000)
        
        layout.addWidget(self.vtk_widget)
        return group

    def _add_grid(self, spacing=1.0, extent=10.0):
        """Creates a VTK grid actor on the XY plane with better styling"""
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        num_lines = int(2 * extent / spacing) + 1
        start = -extent
        end = extent
        id_counter = 0

        for i in range(num_lines):
            pos = start + i * spacing
            # X-direction lines
            points.InsertNextPoint(pos, start, 0)
            points.InsertNextPoint(pos, end, 0)
            lines.InsertNextCell(2)
            lines.InsertCellPoint(id_counter)
            lines.InsertCellPoint(id_counter + 1)
            id_counter += 2

            # Y-direction lines
            points.InsertNextPoint(start, pos, 0)
            points.InsertNextPoint(end, pos, 0)
            lines.InsertNextCell(2)
            lines.InsertCellPoint(id_counter)
            lines.InsertCellPoint(id_counter + 1)
            id_counter += 2

        grid_poly = vtk.vtkPolyData()
        grid_poly.SetPoints(points)
        grid_poly.SetLines(lines)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(grid_poly)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.3, 0.5, 0.7)  # Blue-ish grid
        actor.GetProperty().SetLineWidth(1)
        actor.GetProperty().SetOpacity(0.6)

        return actor

    def update_labels(self):
        """Update all sensor labels with current data"""
        n = self.node

        # Battery
        if n.battery is not None:
            percent = max(0, n.battery.percentage * 100 if n.battery.percentage >= 0 else 0)
            voltage = n.battery.voltage
            
            self.battery_progress.setValue(int(percent))
            self.battery_voltage.setText(f"Voltage: {voltage:.1f} V")
            
            # Update status based on battery level
            if percent > 50:
                self.battery_status.set_status('good')
            elif percent > 20:
                self.battery_status.set_status('warning')
            else:
                self.battery_status.set_status('error')
        else:
            self.battery_progress.setValue(0)
            self.battery_voltage.setText("Voltage: --")
            self.battery_status.set_status('inactive')

        # IMU
        if n.imu is not None:
            q = n.imu.orientation
            roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
            text = f"Roll: {math.degrees(roll):.1f}°\nPitch: {math.degrees(pitch):.1f}°\nYaw: {math.degrees(yaw):.1f}°"
            self.sensor_labels['imu'].setText(text)
        else:
            self.sensor_labels['imu'].setText("IMU: --")

        # Odometry
        if n.odom is not None:
            x = n.odom.pose.pose.position.x
            y = n.odom.pose.pose.position.y
            z = n.odom.pose.pose.position.z
            or_x = n.odom.pose.pose.orientation.x
            or_y = n.odom.pose.pose.orientation.y
            or_z = n.odom.pose.pose.orientation.z
            text = f"Position:\nX: {x:.2f} m, Y: {y:.2f} m, Z: {z:.2f} m\n\nOrientation:\nX: {or_x:.2f}, Y: {or_y:.2f}, Z: {or_z:.2f}"
            self.sensor_labels['odom'].setText(text)
        else:
            self.sensor_labels['odom'].setText("Odometry: --")

        # Command velocity
        if n.cmd is not None:
            lin_x = n.cmd.twist.linear.x
            lin_y = n.cmd.twist.linear.y
            lin_z = n.cmd.twist.linear.z
            ang_x = n.cmd.twist.angular.x
            ang_y = n.cmd.twist.angular.y
            ang_z = n.cmd.twist.angular.z
            text = f"Linear Velocity:\nX: {lin_x:.2f} m/s, Y: {lin_y:.2f} m/s, Z: {lin_z:.2f} m/s\n\nAngular Velocity:\nX: {ang_x:.2f} rad/s, Y: {ang_y:.2f} rad/s, Z: {ang_z:.2f} rad/s"
            self.sensor_labels['cmd'].setText(text)
        else:
            self.sensor_labels['cmd'].setText("Velocity: --")

        # Emergency Stop
        if n.e_stop is not None:
            status = "🛑 ACTIVE" if n.e_stop else "✅ INACTIVE"
            color = "#E74C3C" if n.e_stop else "#27AE60"
            self.sensor_labels['e_stop'].setText(f"Status: {status}")
            self.sensor_labels['e_stop'].setStyleSheet(f"""
                QLabel {{
                    color: {color};
                    font-size: 14px;
                    font-weight: bold;
                    background: transparent;
                    border: none;
                    padding: 5px;
                }}
            """)
        else:
            self.sensor_labels['e_stop'].setText("Emergency Stop: --")

    def update_lidar_vtk(self, points):
        """Update the VTK renderer with LiDAR data"""
        # Remove placeholder if it exists
        if hasattr(self, 'placeholder_text_actor'):
            self.renderer.RemoveActor2D(self.placeholder_text_actor)
            del self.placeholder_text_actor

        # Remove previous lidar actor if exists
        if hasattr(self, 'lidar_actor'):
            self.renderer.RemoveActor(self.lidar_actor)

        # Create vtkPoints
        vtk_points = vtk.vtkPoints()
        for x, y, z in points:
            vtk_points.InsertNextPoint(x, y, z)

        # Create vtkPolyData
        poly_data = vtk.vtkPolyData()
        poly_data.SetPoints(vtk_points)

        vertex_filter = vtk.vtkVertexGlyphFilter()
        vertex_filter.SetInputData(poly_data)
        vertex_filter.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(vertex_filter.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(3)
        actor.GetProperty().SetColor(0.2, 0.8, 1.0)  # Bright cyan
        actor.GetProperty().SetOpacity(0.8)

        self.renderer.AddActor(actor)
        self.renderer.ResetCamera()
        self.lidar_actor = actor

        self.vtk_widget.GetRenderWindow().Render()

    def update_feed(self, key, pixmap):
        """Update camera feed with status indicator"""
        if key in self.feeds:
            label = self.feeds[key]
            
            # Scale image
            scaled_pixmap = pixmap.scaled(
                self.MAX_IMG_WIDTH,
                self.MAX_IMG_HEIGHT,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            label.setPixmap(scaled_pixmap)
            
            # Update status indicator
            if key in self.status_indicators:
                self.status_indicators[key].set_status('good')

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)


def main():
    rclpy.init()
    image_signal = ImageSignal()
    node = PantherSensorNode(image_signal)

    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    gui = PantherDashboard(node, image_signal)
    gui.show()
    
    # Force VTK render after window appears
    QTimer.singleShot(100, lambda: gui.vtk_widget.GetRenderWindow().Render())

    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
