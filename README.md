# ERC25_navigation
# Introduction
This is the official repo for Mind cloud team ERC25 on-stie competition code.

# System requirement
This repo will work on **Ubuntu24** with **ROS2 Jazzy**
- [Ubuntu24 ISO](https://ubuntu.com/download/desktop/thank-you?version=24.04.2&architecture=amd64&lts=true)
- [ROS2 Jazzy installation](https://docs.ros.org/en/jazzy/Installation.html)

Instead of downlaoding docker compose repo is avaliable to use:
[ROS2 Jazzy docker repo](https://github.com/MindCloud-Team/ROS2_Jazzy_docker)

# Git Workflow Guidelines
## Branch Structure

Our repository follows a structured branching model to ensure stability and organize development:

- **`main`**: The last stable version of the system. This branch is protected and only receives updates from `devel` after thorough testing.
- **`devel`**: The active development branch where all feature work is merged after review.
- **`other`**: Individual task branches where developers work on specific task.

```
main (stable) ← devel (integration) ← task branches (features/fixes)
```

## Workflow Process

1. All development work happens in task-specific branches
2. Changes are merged into `devel` via pull requests
3. After testing, `devel` is merged upstream to `main`

## Task Branch Guidelines

- Each task should have its own branch

## Pull Request Process

Pull requests to `devel` will be evaluated based on:
- Code style adherence
- Complete and proper documentation
- Logical correctness
- Test coverage

## Example: Working with Task Branches

### Creating and pushing to a task branch

```bash
# Ensure you're starting from the latest devel
git checkout devel
git pull origin devel

# Create and switch to a new task branch
git checkout <your-branch>

# Make your changes, then commit them
git add .
git commit -m "ADD: Login form component and validation"

# Push your branch to remote
git push -u origin <your-branch>
```

### Creating a Pull Request (GitHub)

1. Go to the repository on GitHub
2. Click on "Pull requests" tab
3. Click the "New pull request" button
4. Set "base" branch to `devel`
5. Set "compare" branch to your task branch
6. Click "Create pull request"
7. Add a descriptive title and detailed description
8. Request reviews from team members
9. Submit the pull request

### Standard Commit Types

- **ADD:** Adding new features or functionality
- **FIX:** Bug fixes
- **DOCS:** Documentation changes only
- **STYLE:** Code style changes (formatting, missing semi-colons, etc.)
- **REFACTOR:** Code changes that neither fix bugs nor add features
- **TEST:** Adding or modifying tests
- **CHORE:** Changes to build process, dependencies, etc.
- **PERF:** Performance improvements

### Examples of Good Commit Messages

```
ADD: User authentication module

Implements login, registration, and password reset functionality
```

```
FIX: Correct calculation in tax processing function

Resolves issue #42 where tax calculation was incorrect for certain income ranges
```

```
REFACTOR: Improve error handling in API client
```

```
DOCS: Update installation instructions
```

Remember that good commit messages make it easier to track changes and understand the project history.

# Python and ROS2 Code Style and Documentation Guidelines

## Code Style

We follow PEP 8 guidelines with some additional project-specific conventions as outlined in the Mind Cloud System Guidelines.

### Naming Conventions

- **Classes**: Use `PascalCase` (e.g., `UserProfile`, `SensorManager`)
- **Functions/Methods**: Use `snake_case` (e.g., `calculate_total`, `publish_data`)
- **Variables**: Use `snake_case` (e.g., `user_name`, `lidar_data`)
- **Constants**: Use `UPPER_SNAKE_CASE` (e.g., `MAX_RETRY_COUNT`, `TAX_RATE`)
- **Modules**: Use short, `lowercase` names (e.g., `utils.py`, `perception.py`)
- **Private attributes/methods**: Prefix with underscore (e.g., `_internal_method`, `_private_var`)

### Formatting

- Maximum line length: 79 characters
- Add blank lines between functions and classes (2 lines)
- Add blank lines between logical sections of code (1 line)
- Use spaces around operators (`x = 1 + 2`, not `x=1+2`)

### Code Cleanliness

- Keep functions short and focused on a single responsibility
- Avoid deep nesting of loops and conditionals
- Use proper indentation consistently
- Follow the DRY principle (Don't Repeat Yourself)
- Avoid hardcoded values; use constants or configuration files instead

## Docstrings for Python and ROS2

All modules, classes, methods, and functions must include docstrings following the format specified in the Mind Cloud System Guidelines.

### Module Docstrings

```python
"""
Brief summary of the module.

Detailed description of the module and its functionality.
"""
```

### Class Docstrings

```python
class PerceptionNode:
    """
    Brief summary of the class.
    
    Detailed description of the class and its behavior, if necessary.
    
    Attributes:
        attribute1 (type): Description of attribute1.
        attribute2 (type): Description of attribute2.
    
    ROS Parameters:
        ~param_name (type): Description of the ROS parameter.
    
    ROS Publishers:
        /topic_name (msg_type): Description of the publisher topic.
    
    ROS Subscribers:
        /topic_name (msg_type): Description of the subscriber topic.
    
    ROS Service Clients:
        /service_name (srv_type): Description of the service client.
    
    ROS Service Servers:
        /service_name (srv_type): Description of the service server.
    
    Examples:
        Example usage of the class, if applicable.
    """
```

### Method and Function Docstrings

```python
def process_lidar_data(scan_data, filter_threshold=0.5):
    """
    Brief summary of the function.
    
    Detailed description of the function and its behavior, if necessary.
    
    Args:
        scan_data (sensor_msgs.msg.LaserScan): The raw lidar scan data.
        filter_threshold (float): Threshold for filtering noise. Defaults to 0.5.
    
    Returns:
        numpy.ndarray: Processed point cloud data.
    
    Raises:
        ValueError: If scan_data is empty or invalid.
    
    ROS Parameters:
        ~param_name (type): Description of the ROS parameter.
    
    ROS Publishers:
        /topic_name (msg_type): Description of the publisher topic.
    
    ROS Subscribers:
        /topic_name (msg_type): Description of the subscriber topic.
    
    ROS Service Clients:
        /service_name (srv_type): Description of the service client.
    
    ROS Service Servers:
        /service_name (srv_type): Description of the service server.
    
    Examples:
        Example usage of the function, if applicable.
    """
```

> **Note**: Only include sections that are relevant. If a function doesn't have ROS parameters, publishers, etc., those sections can be omitted. At a minimum, the docstring must include a brief summary.

## Example ROS2 Node with Complete Documentation

```python
#!/usr/bin/env python3
"""
LiDAR perception module for obstacle detection.

This module processes LiDAR data to detect obstacles in the rover's path
and publishes the detected obstacles for use by the planning layer.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import numpy as np


class ObstacleDetectionNode(Node):
    """
    ROS2 node for detecting obstacles using LiDAR data.
    
    This node subscribes to LaserScan messages, processes the data to
    identify potential obstacles, and publishes the positions of detected
    obstacles.
    
    Attributes:
        scan_sub (rclpy.subscription.Subscription): Subscription to LaserScan topic
        obstacle_pub (rclpy.publisher.Publisher): Publisher for detected obstacles
        min_distance (float): Minimum distance threshold for obstacle detection
        max_distance (float): Maximum distance threshold for obstacle detection
    
    ROS Parameters:
        ~min_distance (float): Minimum distance to consider for obstacles
        ~max_distance (float): Maximum distance to consider for obstacles
        ~detection_threshold (float): Threshold value for obstacle detection
    
    ROS Publishers:
        /perception/obstacles (geometry_msgs/PointStamped): Publishes detected obstacle positions
    
    ROS Subscribers:
        /scan (sensor_msgs/LaserScan): Subscribes to LiDAR scan data
    """
    
    def __init__(self):
        """
        Initialize the ObstacleDetectionNode.
        
        Sets up ROS publishers, subscribers, and parameters.
        """
        super().__init__('obstacle_detection_node')
        
        # Declare and get parameters
        self.declare_parameter('min_distance', 0.2)
        self.declare_parameter('max_distance', 5.0)
        self.declare_parameter('detection_threshold', 0.5)
        
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        
        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )
        
        # Create publishers
        self.obstacle_pub = self.create_publisher(
            PointStamped,
            '/perception/obstacles',
            10
        )
        
        self.get_logger().info('Obstacle detection node initialized')
    
    def _scan_callback(self, msg):
        """
        Process incoming LiDAR scan messages and detect obstacles.
        
        This method identifies potential obstacles in the LiDAR scan data
        and publishes their positions.
        
        Args:
            msg (sensor_msgs.msg.LaserScan): The LiDAR scan message
        """
        obstacles = self._detect_obstacles(msg)
        
        for obstacle in obstacles:
            point_msg = PointStamped()
            point_msg.header = msg.header
            point_msg.point.x = obstacle[0]
            point_msg.point.y = obstacle[1]
            point_msg.point.z = 0.0
            
            self.obstacle_pub.publish(point_msg)
    
    def _detect_obstacles(self, scan_msg):
        """
        Detect obstacles from LiDAR scan data.
        
        Args:
            scan_msg (sensor_msgs.msg.LaserScan): The LiDAR scan message
        
        Returns:
            list: List of (x, y) coordinates of detected obstacles
        """
        obstacles = []
        angle = scan_msg.angle_min
        
        for i, distance in enumerate(scan_msg.ranges):
            # Skip invalid measurements and those outside our distance thresholds
            if (distance < self.min_distance or 
                distance > self.max_distance or 
                not np.isfinite(distance)):
                angle += scan_msg.angle_increment
                continue
            
            # Convert polar to cartesian coordinates
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            
            # Apply detection threshold
            if distance < self.detection_threshold:
                obstacles.append((x, y))
            
            angle += scan_msg.angle_increment
        
        return obstacles


def main(args=None):
    """
    Main entry point for the obstacle detection node.
    
    Args:
        args: Command-line arguments
    """
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## ROS2-Specific Documentation Best Practices

1. **Node Documentation**: Clearly document the purpose of the node and its role within the system architecture (Perception, Planning, Control, etc.).

2. **Topic Documentation**: For publishers and subscribers, document:
   - Topic name
   - Message type
   - Purpose of the data
   - Frequency of publishing (if relevant)

3. **Parameter Documentation**: Document all parameters:
   - Parameter name
   - Data type
   - Default value
   - Valid ranges
   - Purpose of the parameter

4. **Service Documentation**: For services, document:
   - Service name
   - Service type
   - Expected input
   - Expected output
   - Error handling

5. **Action Documentation**: For ROS2 actions, document:
   - Action name
   - Action type
   - Goal, feedback, and result information
   - Timeout behavior

## Code Review Checklist

Before submitting a pull request, ensure your code meets these requirements:

- ✅ Code follows the style guidelines
- ✅ All functions, classes, and methods have appropriate docstrings
- ✅ ROS2 topics, services, and parameters are properly documented
- ✅ Type hints are used where appropriate
- ✅ No unnecessary commented-out code
- ✅ No debugging print statements
- ✅ Error handling is appropriate and specific
- ✅ Variable names are clear and descriptive
- ✅ Code is properly structured according to the PPC (Perception-Planning-Control) architecture
