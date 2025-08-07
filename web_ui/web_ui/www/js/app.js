class RoverWebUI {
    constructor() {
        this.socket = null;
        this.connected = false;
        this.mapCanvas = null;
        this.mapContext = null;
        this.imuCanvas = null;
        this.imuContext = null;
        this.cameraMode = 'auto';
        this.currentLinearVelocity = 0;
        this.cameraImages = {};
        this.keyStates = {};
        this.gamepadConnected = false;
        this.gamepadIndex = null;
        this.r2Pressed = false;
        this.r1Pressed = false;
        this.lastGamepadInput = false;
        
        this.initWebSocket();
        this.loadConfiguration();
    }

    async loadConfiguration() {
        try {
            const response = await fetch('topics.json');
            this.config = await response.json();
        } catch (error) {
            console.error('❌ Failed to load configuration:', error);
            // Fallback configuration
            this.config = {
                topics: {
                    cameras: {
                        front: "/front_cam/zed_node/rgb/image_rect_color",
                        back: "/back_cam/zed_node/rgb/image_rect_color", 
                        left: "/left_cam/zed_node/rgb/image_rect_color",
                        right: "/right_cam/zed_node/rgb/image_rect_color"
                    },
                    sensors: {
                        imu: "/imu/data",
                        battery: "/battery/battery_status",
                        estop: "/hardware/e_stop",
                        odometry: "/odometry/filtered"
                    },
                    navigation: {
                        map: "/costmap",
                        cmd_vel: "/cmd_vel",
                        goal_pose: "/goal_pose"
                    }
                }
            };
        }
    }

    initWebSocket() {
        
        // Connect to our custom WebSocket server
        this.socket = io();
        
        this.socket.on('connect', () => {
            this.connected = true;
            this.updateConnectionStatus(true);
        });
        
        this.socket.on('disconnect', () => {
            this.connected = false;
            this.updateConnectionStatus(false);
        });
        
        // Handle map data
        this.socket.on('map_data', (data) => {
            this.updateMapData(data);
        });
        
        // Handle IMU data
        this.socket.on('imu_data', (data) => {
            this.updateIMUData(data);
        });
        
        // Handle battery data
        this.socket.on('battery_data', (data) => {
            this.updateBatteryData(data);
        });
        
        // Handle E-stop data
        this.socket.on('estop_data', (data) => {
            this.updateEStopStatus(data.pressed);
        });
        
        // Handle odometry data
        this.socket.on('odometry_data', (data) => {
            this.updateOdometryData(data);
        });
        
        // Handle robot pose data
        this.socket.on('robot_pose', (data) => {
            this.updateRobotPosition(data);
        });
        
        // Handle camera data
        this.socket.on('camera_data', (data) => {
            this.updateCameraData(data);
        });
        
        // Handle latest data (comprehensive update)
        this.socket.on('latest_data', (data) => {
            this.updateAllData(data);
        });
    }

    updateConnectionStatus(connected) {
        const statusElement = document.getElementById('connection-status');
        const icon = statusElement.querySelector('i');
        const text = statusElement.querySelector('span');
        
        if (connected) {
            icon.className = 'fas fa-wifi';
            icon.style.color = '#00ff00';
            text.textContent = 'Connected';
            statusElement.style.color = '#00ff00';
        } else {
            icon.className = 'fas fa-wifi-slash';
            icon.style.color = '#ff0000';
            text.textContent = 'Disconnected';
            statusElement.style.color = '#ff0000';
        }
    }

    updateAllData(data) {
        if (data.map) this.updateMapData(data.map);
        if (data.imu) this.updateIMUData(data.imu);
        if (data.battery) this.updateBatteryData(data.battery);
        if (data.estop !== undefined) this.updateEStopStatus(data.estop.pressed || data.estop);
        if (data.odometry) this.updateOdometryData(data.odometry);
        if (data.robot_pose) {
            this.updateRobotPosition(data.robot_pose);
        }
        if (data.cameras) {
            Object.values(data.cameras).forEach(cameraData => {
                this.updateCameraData(cameraData);
            });
        }
    }
    


    updateIMUData(data) {
        // Store IMU data for arrow drawing
        this.imuData = data;
        
        // Check if data has the required properties - backend sends 'accel' and 'gyro'
        if (!data || !data.accel || !data.gyro) {
            return;
        }
        
        // Calculate magnitudes for display
        const accelMagnitude = Math.sqrt(
            (data.accel.x || 0) ** 2 + 
            (data.accel.y || 0) ** 2 + 
            (data.accel.z || 0) ** 2
        );
        
        const gyroMagnitude = Math.sqrt(
            (data.gyro.x || 0) ** 2 + 
            (data.gyro.y || 0) ** 2 + 
            (data.gyro.z || 0) ** 2
        );
        
        // Update IMU values display
        const accelElement = document.getElementById('imu-accel');
        const gyroElement = document.getElementById('imu-gyro');
        
        if (accelElement) {
            accelElement.textContent = `Accel: ${accelMagnitude.toFixed(1)}`;
        }
        if (gyroElement) {
            gyroElement.textContent = `Gyro: ${gyroMagnitude.toFixed(1)}`;
        }
        
        // Update IMU arrow
        this.drawIMUArrow();
    }
    
    updateBatteryData(data) {
        if (!data || typeof data.percentage !== 'number') return;
        
        // Handle both decimal (0.01 = 1%) and percentage (70 = 70%) formats
        let percentage;
        if (data.percentage <= 1.0) {
            // If it's a decimal (0.01 to 1.0), convert to percentage
            percentage = Math.round(data.percentage * 100);
        } else {
            // If it's already a percentage (0-100), use as is
            percentage = Math.round(data.percentage);
        }
        
        percentage = Math.max(0, Math.min(100, percentage));
        
        // Update battery display with percentage and bar
        const batteryElement = document.getElementById('battery-status');
        if (batteryElement) {
            // Determine battery icon based on percentage
            let batteryIcon = 'battery-full';
            if (percentage <= 20) batteryIcon = 'battery-empty';
            else if (percentage <= 40) batteryIcon = 'battery-quarter';
            else if (percentage <= 60) batteryIcon = 'battery-half';
            else if (percentage <= 80) batteryIcon = 'battery-three-quarters';
            
            // Determine battery color based on level
            let batteryColor = '#00ff00'; // Green
            if (percentage <= 20) {
                batteryColor = '#ff0000'; // Red
            } else if (percentage <= 50) {
                batteryColor = '#ffff00'; // Yellow
            }
            
            batteryElement.innerHTML = `
                <i class="fas fa-${batteryIcon}"></i>
                <div class="battery-bar-container">
                    <div class="battery-bar-fill" style="width: ${percentage}%; background-color: ${batteryColor};"></div>
                </div>
                <span>${percentage}%</span>
            `;
            batteryElement.style.color = batteryColor;
        }
    }
    
    updateEStopStatus(isPressed) {
        console.log('🛑 E-Stop update called with:', isPressed, typeof isPressed);
        
        const estopElement = document.getElementById('e-stop-status');
        if (!estopElement) {
            console.error('❌ E-Stop element not found');
            return;
        }
        
        // Convert to boolean if needed
        const pressed = Boolean(isPressed);
        
        estopElement.innerHTML = `
            <i class="fas fa-hand-paper"></i>
            <span>E-STOP: ${pressed ? 'PRESSED' : 'OK'}</span>
        `;
        estopElement.style.color = pressed ? '#ff0000' : '#00ff00';
        
        console.log('🛑 E-Stop updated to:', pressed ? 'PRESSED' : 'OK');
    }
    
    updateOdometryData(data) {
        if (!data || !data.pose || !data.twist) return;
        
        this.currentLinearVelocity = data.twist.linear || 0;
        
        // Update robot position for map
        this.updateRobotPosition(data.pose);
    }
    
    updateCameraData(data) {
        const cameraName = data.camera;
        const base64Image = `data:image/jpeg;base64,${data.image}`;
        
        // Store image for camera switching
        this.cameraImages[cameraName] = base64Image;
        
        // Get the current active camera
        const centerCamera = document.getElementById('center-camera');
        const currentCamera = centerCamera.getAttribute('data-camera') || 'front';
        
        // Only update if this is the currently active camera
        if ((cameraName === 'front_cam' && currentCamera === 'front') ||
            (cameraName === 'back_cam' && currentCamera === 'back')) {
            
            const imgElement = document.getElementById('center-cam-img');
            const overlayElement = imgElement.nextElementSibling;
            
            imgElement.src = base64Image;
            imgElement.onload = () => {
                overlayElement.style.display = 'none';
                imgElement.style.display = 'block';
            };
        }
        
        // Handle side cameras
        if (cameraName === 'left_cam') {
            const imgElement = document.getElementById('left-cam-img');
            const overlayElement = imgElement.nextElementSibling;
            
            imgElement.src = base64Image;
            imgElement.onload = () => {
                overlayElement.style.display = 'none';
                imgElement.style.display = 'block';
            };
        } else if (cameraName === 'right_cam') {
            const imgElement = document.getElementById('right-cam-img');
            const overlayElement = imgElement.nextElementSibling;
            
            imgElement.src = base64Image;
            imgElement.onload = () => {
                overlayElement.style.display = 'none';
                imgElement.style.display = 'block';
            };
        }
    }
    
    updateCameraLayout(primary) {
        const centerCamera = document.getElementById('center-camera');
        const centerLabel = document.getElementById('center-camera-label');
        const centerIcon = document.getElementById('center-camera-icon');
        const centerImg = document.getElementById('center-cam-img');
        const centerOverlay = centerImg.nextElementSibling;
        
        if (primary === 'front') {
            // Show front camera
            centerLabel.textContent = 'Front Camera';
            centerIcon.className = 'fas fa-arrow-up';
            centerCamera.setAttribute('data-camera', 'front');
        } else {
            // Show back camera
            centerLabel.textContent = 'Back Camera';
            centerIcon.className = 'fas fa-arrow-down';
            centerCamera.setAttribute('data-camera', 'back');
        }
        
        // Clear current image and show overlay
        centerImg.style.display = 'none';
        centerOverlay.style.display = 'flex';
        
        // Update camera image if available
        if (this.cameraImages[primary]) {
            centerImg.src = this.cameraImages[primary];
            centerImg.onload = () => {
                centerOverlay.style.display = 'none';
                centerImg.style.display = 'block';
            };
        }
    }
    
    initMapVisualization() {
        
        // Get the canvas element
        this.mapCanvas = document.getElementById('map-canvas');
        if (!this.mapCanvas) {
            console.error('❌ Map canvas not found');
            return;
        }
        
        this.mapContext = this.mapCanvas.getContext('2d');
        
        // Set canvas size
        this.mapCanvas.width = 600;
        this.mapCanvas.height = 400;
        
        // Map view state
        this.mapView = {
            scale: 8.0, // Much larger scale for RViz-like clarity
            offsetX: 0,
            offsetY: 0,
            isDragging: false,
            lastMouseX: 0,
            lastMouseY: 0
        };
        
        // Initialize with empty map
        this.drawEmptyMap();
        
        // Add map interaction listeners
        this.initMapInteractions();
    }
    
    initMapInteractions() {
        const canvas = this.mapCanvas;
        
        // Mouse wheel zoom
        canvas.addEventListener('wheel', (event) => {
            event.preventDefault();
            
            const rect = canvas.getBoundingClientRect();
            const mouseX = event.clientX - rect.left;
            const mouseY = event.clientY - rect.top;
            
            // Calculate zoom factor (inverted for natural zoom direction)
            const zoomFactor = event.deltaY > 0 ? 1.1 : 0.9;
            const newScale = Math.max(0.1, Math.min(20.0, this.mapView.scale * zoomFactor));
            
            // Zoom towards mouse position
            const scaleChange = newScale / this.mapView.scale;
            this.mapView.offsetX = mouseX - (mouseX - this.mapView.offsetX) * scaleChange;
            this.mapView.offsetY = mouseY - (mouseY - this.mapView.offsetY) * scaleChange;
            this.mapView.scale = newScale;
            
            this.drawMap();
        });
        
        // Mouse drag pan
        canvas.addEventListener('mousedown', (event) => {
            this.mapView.isDragging = true;
            this.mapView.lastMouseX = event.clientX;
            this.mapView.lastMouseY = event.clientY;
            canvas.style.cursor = 'grabbing';
        });
        
        canvas.addEventListener('mousemove', (event) => {
            if (this.mapView.isDragging) {
                const deltaX = event.clientX - this.mapView.lastMouseX;
                const deltaY = event.clientY - this.mapView.lastMouseY;
                
                this.mapView.offsetX += deltaX;
                this.mapView.offsetY += deltaY;
                
                this.mapView.lastMouseX = event.clientX;
                this.mapView.lastMouseY = event.clientY;
                
                this.drawMap();
            }
        });
        
        canvas.addEventListener('mouseup', () => {
            this.mapView.isDragging = false;
            canvas.style.cursor = 'grab';
        });
        
        canvas.addEventListener('mouseleave', () => {
            this.mapView.isDragging = false;
            canvas.style.cursor = 'grab';
        });
        
        // Set initial cursor
        canvas.style.cursor = 'grab';
    }
    
    drawEmptyMap() {
        if (!this.mapContext) return;
        
        // Clear canvas
        this.mapContext.fillStyle = '#1a1a2e';
        this.mapContext.fillRect(0, 0, this.mapCanvas.width, this.mapCanvas.height);
        
        // Draw grid
        this.mapContext.strokeStyle = '#0066cc';
        this.mapContext.lineWidth = 1;
        
        const gridSize = 20;
        for (let x = 0; x <= this.mapCanvas.width; x += gridSize) {
            this.mapContext.beginPath();
            this.mapContext.moveTo(x, 0);
            this.mapContext.lineTo(x, this.mapCanvas.height);
            this.mapContext.stroke();
        }
        
        for (let y = 0; y <= this.mapCanvas.height; y += gridSize) {
            this.mapContext.beginPath();
            this.mapContext.moveTo(0, y);
            this.mapContext.lineTo(this.mapCanvas.width, y);
            this.mapContext.stroke();
        }
        
        // Draw "Waiting for map data" text
        this.mapContext.fillStyle = '#cccccc';
        this.mapContext.font = '16px Arial';
        this.mapContext.textAlign = 'center';
        this.mapContext.fillText('Waiting for map data...', 
            this.mapCanvas.width / 2, this.mapCanvas.height / 2);
    }
    
    initIMUArrow() {
        
        this.imuCanvas = document.getElementById('imu-arrow');
        if (!this.imuCanvas) {
            console.error('❌ IMU arrow canvas not found');
            return;
        }
        
        this.imuContext = this.imuCanvas.getContext('2d');
        this.drawIMUArrow();
        
    }
    
    drawIMUArrow() {
        if (!this.imuContext) {
            return;
        }
        
        const canvas = this.imuCanvas;
        const ctx = this.imuContext;
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const radius = 35; // Increased radius for better visibility
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw circle
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
        ctx.strokeStyle = '#0066cc';
        ctx.lineWidth = 3;
        ctx.stroke();
        
        // Get current orientation from IMU data
        if (this.imuData && this.imuData.orientation) {
            // Convert quaternion to euler angles to get yaw
            const q = this.imuData.orientation;
            const yaw = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            
            // Fix the yaw direction (flip left/right)
            const correctedYaw = -yaw; // Flip the yaw direction
            
            // Calculate arrow direction based on yaw
            const arrowLength = radius * 0.6;
            const startX = centerX;
            const startY = centerY;
            const endX = centerX + arrowLength * Math.sin(correctedYaw);
            const endY = centerY - arrowLength * Math.cos(correctedYaw);
            
            // Draw arrow
            ctx.beginPath();
            ctx.strokeStyle = '#00ffff';
            ctx.lineWidth = 4;
            ctx.moveTo(startX, startY);
            ctx.lineTo(endX, endY);
            ctx.stroke();
            
            // Draw arrowhead
            const headLength = 10;
            const angle = Math.atan2(endY - startY, endX - startX);
            
            ctx.beginPath();
            ctx.moveTo(endX, endY);
            ctx.lineTo(endX - headLength * Math.cos(angle - Math.PI / 6), 
                      endY - headLength * Math.sin(angle - Math.PI / 6));
            ctx.moveTo(endX, endY);
            ctx.lineTo(endX - headLength * Math.cos(angle + Math.PI / 6), 
                      endY - headLength * Math.sin(angle + Math.PI / 6));
            ctx.stroke();
            
            // Draw cardinal directions
            ctx.fillStyle = '#0066cc';
            ctx.font = '10px Arial';
            ctx.textAlign = 'center';
            
            // North
            ctx.fillText('N', centerX, centerY - radius - 5);
            // East
            ctx.fillText('E', centerX + radius + 10, centerY);
            // South
            ctx.fillText('S', centerX, centerY + radius + 15);
            // West
            ctx.fillText('W', centerX - radius - 10, centerY);
        }
    }
    
    updateMapData(mapMessage) {
        
        this.mapData = mapMessage;
        this.mapResolution = mapMessage.resolution;
        this.mapOrigin = {
            x: mapMessage.origin.x,
            y: mapMessage.origin.y
        };
        
        this.drawMap();
    }
    
    updateRobotPosition(pose) {
        this.robotPosition = pose;
        if (this.mapData) {
            this.drawMap();
        }
    }
    
    drawMap() {
        if (!this.mapContext || !this.mapData) return;
        
        const ctx = this.mapContext;
        const canvas = this.mapCanvas;
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Apply view transformations
        ctx.save();
        ctx.translate(this.mapView.offsetX, this.mapView.offsetY);
        ctx.scale(this.mapView.scale, this.mapView.scale);
        
        // Calculate map bounds using actual map dimensions
        const mapWidth = this.mapData.width;
        const mapHeight = this.mapData.height;
        const resolution = this.mapData.resolution;
        
        // Calculate the actual world size of the map
        const worldWidth = mapWidth * resolution;
        const worldHeight = mapHeight * resolution;
        
        // Scale the map to fit the canvas properly - now this will be affected by zoom
        const canvasWidth = canvas.width;
        const canvasHeight = canvas.height;
        
        // Calculate scale to fit map in canvas - this will now scale with zoom
        const scaleX = canvasWidth / worldWidth;
        const scaleY = canvasHeight / worldHeight;
        const mapScale = Math.min(scaleX, scaleY) * 8.0; // Much larger scale factor for RViz-like clarity
        
        // Center the map
        const scaledMapWidth = worldWidth * mapScale;
        const scaledMapHeight = worldHeight * mapScale;
        const centerX = (canvasWidth - scaledMapWidth) / 2;
        const centerY = (canvasHeight - scaledMapHeight) / 2;
        
        // Draw map as bitmap with proper scaling - now affected by zoom
        for (let y = 0; y < mapHeight; y++) {
            for (let x = 0; x < mapWidth; x++) {
                const index = y * mapWidth + x;
                const cellValue = this.mapData.data[index];
                
                if (cellValue !== -1) { // Skip unknown cells
                    const cellX = centerX + x * resolution * mapScale;
                    const cellY = centerY + y * resolution * mapScale;
                    const cellSize = resolution * mapScale;
                    
                    // Set color based on cell value with better contrast
                    if (cellValue === 0) {
                        ctx.fillStyle = '#ffffff'; // Free space - white
                    } else if (cellValue === 100) {
                        ctx.fillStyle = '#000000'; // Occupied - black for better contrast
                    } else {
                        ctx.fillStyle = '#888888'; // Unknown - medium gray
                    }
                    
                    // Draw larger cells for better visibility
                    if (cellSize > 1) {
                        ctx.fillRect(cellX, cellY, cellSize, cellSize);
                    } else {
                        // For very small cells, draw as single pixels
                        ctx.fillRect(cellX, cellY, 1, 1);
                    }
                }
            }
        }
        
        // Draw robot position if available
        if (this.robotPosition) {
            this.drawRobot(centerX, centerY, resolution * mapScale);
        }
        
        // Draw grid overlay
        this.drawGrid(centerX, centerY, resolution * mapScale, mapWidth, mapHeight);
        
        // Restore context
        ctx.restore();
        
        // No interactive box - removed to fix double box issue
    }
    
    drawGrid(offsetX, offsetY, cellSize, mapWidth, mapHeight) {
        const ctx = this.mapContext;
        
        // Grid settings
        ctx.strokeStyle = 'rgba(100, 100, 100, 0.3)';
        ctx.lineWidth = 0.5;
        
        // Draw vertical grid lines
        for (let x = 0; x <= mapWidth; x++) {
            const gridX = offsetX + x * cellSize;
            ctx.beginPath();
            ctx.moveTo(gridX, offsetY);
            ctx.lineTo(gridX, offsetY + mapHeight * cellSize);
            ctx.stroke();
        }
        
        // Draw horizontal grid lines
        for (let y = 0; y <= mapHeight; y++) {
            const gridY = offsetY + y * cellSize;
            ctx.beginPath();
            ctx.moveTo(offsetX, gridY);
            ctx.lineTo(offsetX + mapWidth * cellSize, gridY);
            ctx.stroke();
        }
        
        // Draw coordinate labels every 5 cells
        ctx.fillStyle = 'rgba(100, 100, 100, 0.7)';
        ctx.font = '10px Arial';
        ctx.textAlign = 'center';
        
        for (let x = 0; x <= mapWidth; x += 5) {
            const gridX = offsetX + x * cellSize;
            ctx.fillText(x.toString(), gridX, offsetY - 5);
        }
        
        for (let y = 0; y <= mapHeight; y += 5) {
            const gridY = offsetY + y * cellSize;
            ctx.fillText(y.toString(), offsetX - 10, gridY + 3);
        }
    }
    
    drawRobot(offsetX, offsetY, scale) {
        if (!this.mapData || !this.robotPosition) return;
        
        // Convert world coordinates to map coordinates
        const mapX = (this.robotPosition.x - this.mapOrigin.x) / this.mapResolution;
        const mapY = (this.robotPosition.y - this.mapOrigin.y) / this.mapResolution;
        
        // Convert to canvas coordinates using the new scaling
        const canvasX = offsetX + mapX * scale;
        const canvasY = offsetY + (this.mapData.height - mapY) * scale;
        
        const ctx = this.mapContext;
        
        // Robot body - larger circle
        const robotRadius = Math.max(8, scale * 1.2); // Increased from 0.5 to 1.2
        ctx.beginPath();
        ctx.arc(canvasX, canvasY, robotRadius, 0, 2 * Math.PI);
        ctx.fillStyle = '#00ffff';
        ctx.fill();
        ctx.strokeStyle = '#0066cc';
        ctx.lineWidth = Math.max(2, scale * 0.5); // Increased from 0.3 to 0.5
        ctx.stroke();
        
        // Robot orientation arrow - larger and more visible
        const arrowLength = Math.max(12, scale * 1.8); // Increased from 1.0 to 1.8
        const arrowWidth = Math.max(3, scale * 0.6); // Increased from 0.3 to 0.6
        
        ctx.beginPath();
        ctx.strokeStyle = '#0066cc';
        ctx.lineWidth = arrowWidth;
        ctx.moveTo(canvasX, canvasY);
        ctx.lineTo(canvasX + arrowLength * Math.sin(this.robotPosition.theta),
                   canvasY - arrowLength * Math.cos(this.robotPosition.theta));
        ctx.stroke();
        
        // Arrow head
        const headLength = Math.max(6, scale * 0.8); // Increased from default
        const angle = this.robotPosition.theta;
        ctx.beginPath();
        ctx.moveTo(canvasX + arrowLength * Math.sin(angle),
                   canvasY - arrowLength * Math.cos(angle));
        ctx.lineTo(canvasX + (arrowLength - headLength) * Math.sin(angle - Math.PI / 6),
                   canvasY - (arrowLength - headLength) * Math.cos(angle - Math.PI / 6));
        ctx.moveTo(canvasX + arrowLength * Math.sin(angle),
                   canvasY - arrowLength * Math.cos(angle));
        ctx.lineTo(canvasX + (arrowLength - headLength) * Math.sin(angle + Math.PI / 6),
                   canvasY - (arrowLength - headLength) * Math.cos(angle + Math.PI / 6));
        ctx.stroke();
    }

    publishCmdVel(linearX, linearY, angularZ) {
        if (!this.connected || !this.socket) return;
        
        this.socket.emit('cmd_vel', {
            linear_x: linearX,
            linear_y: linearY,
            angular_z: angularZ
        });
    }

    initUI() {
        // Initialize UI elements
        this.updateConnectionStatus(false);
        
        // Don't initialize E-stop status here - let it be handled by the first data received
        // This prevents showing wrong initial state
    }

    initControls() {
        this.initKeyboardControls();
        this.initVirtualJoystick();
        this.initRealJoystick();
        this.initCameraSwitching();
    }

    initKeyboardControls() {
        // Track key states
        this.keyStates = {};
        
        // Initialize all keys as not pressed
        ['w', 'a', 's', 'd', 'q', 'e'].forEach(key => {
            this.keyStates[key] = false;
        });
        
        // Continuous movement while keys are held
        const updateMovement = () => {
            let linearX = 0, angularZ = 0;
            const speed = 3.0;
            const angularSpeed = 2.5;
            
            if (this.keyStates['w']) linearX += speed;
            if (this.keyStates['s']) linearX -= speed;
            if (this.keyStates['a']) angularZ += angularSpeed;
            if (this.keyStates['d']) angularZ -= angularSpeed;
            
            // Only publish if there's actual input
            if (linearX !== 0 || angularZ !== 0) {
                this.publishCmdVel(linearX, 0, angularZ);
            }
            // Don't publish zeros - let other sources control the robot
        };
        
        setInterval(updateMovement, 50); // 20Hz updates
        
        // Keyboard events for physical keyboard
        document.addEventListener('keydown', (event) => {
            const key = event.key.toLowerCase();
            if (['w', 'a', 's', 'd', 'q', 'e'].includes(key)) {
                event.preventDefault();
                // Only set to true if it wasn't already pressed (avoid repeat)
                if (!this.keyStates[key]) {
                    this.keyStates[key] = true;
                    this.updateKeyVisual(key, true);
                }
            } else if (key === ' ') {
                event.preventDefault();
                this.toggleFrontBack();
            }
        });
        
        document.addEventListener('keyup', (event) => {
            const key = event.key.toLowerCase();
            if (['w', 'a', 's', 'd', 'q', 'e'].includes(key)) {
                // Only handle if key was actually pressed
                if (this.keyStates[key]) {
                    this.keyStates[key] = false;
                    this.updateKeyVisual(key, false);
                    
                    // Send a single stop command when any movement key is released
                    if (['w', 'a', 's', 'd'].includes(key)) {
                        this.publishCmdVel(0, 0, 0);
                    }
                }
            }
        });
    }

    initVirtualJoystick() {
        const joystickArea = document.getElementById('joystick-area');
        const joystickKnob = document.getElementById('joystick-knob');
        const linearDisplay = document.getElementById('linear-display');
        const angularDisplay = document.getElementById('angular-display');
        
        if (!joystickArea || !joystickKnob) {
            console.error('❌ Joystick elements not found!');
            return;
        }
        
        let isDragging = false;
        let centerX = 60; // Half of joystick area width (120px / 2)
        let centerY = 60; // Half of joystick area height (120px / 2)
        let maxDistance = 40; // Maximum distance from center
        
        // Initialize joystick position
        function initJoystickPosition() {
            if (!joystickArea) return;
            
            // Get the actual dimensions of the joystick area
            const rect = joystickArea.getBoundingClientRect();
            centerX = rect.width / 2;
            centerY = rect.height / 2;
            maxDistance = Math.min(centerX, centerY) - 20; // Leave some margin
            
            resetJoystick();
        }
        
        const updateJoystick = (x, y) => {
            if (!joystickKnob) return;
            
            const distance = Math.sqrt(x * x + y * y);
            
            if (distance > maxDistance) {
                const angle = Math.atan2(y, x);
                x = Math.cos(angle) * maxDistance;
                y = Math.sin(angle) * maxDistance;
            }
            
            // Position knob correctly using transform with the -50% offset
            joystickKnob.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
            
            // Convert to velocity commands with deadzone
            const deadzone = 0.1; // 10% deadzone
            let linearX = -y / maxDistance; // Forward/backward (up/down)
            let angularZ = -x / maxDistance; // Left/right turn (left/right)
            
            // Apply deadzone
            if (Math.abs(linearX) < deadzone) linearX = 0;
            if (Math.abs(angularZ) < deadzone) angularZ = 0;
            
            if (linearDisplay) linearDisplay.textContent = linearX.toFixed(2);
            if (angularDisplay) angularDisplay.textContent = angularZ.toFixed(2);
            
            // Publish velocity command
            if (this.connected) {
                if (Math.abs(linearX) > 0.01 || Math.abs(angularZ) > 0.01) {
                    this.publishCmdVel(linearX * 2.0, 0, angularZ * 2.0);
                }
                // Don't publish zeros - let other sources control the robot
            }
        };
        
        const resetJoystick = () => {
            if (!joystickKnob) return;
            // Center the knob properly
            joystickKnob.style.transform = 'translate(-50%, -50%)';
            
            if (linearDisplay) linearDisplay.textContent = '0.0';
            if (angularDisplay) angularDisplay.textContent = '0.0';
            
            // Send stop command when joystick is released
            this.publishCmdVel(0, 0, 0);
        };
        
        // Initialize position on load with delay
        setTimeout(initJoystickPosition, 100);
        
        joystickArea.addEventListener('mousedown', (event) => {
            isDragging = true;
            const rect = joystickArea.getBoundingClientRect();
            const x = event.clientX - rect.left - centerX;
            const y = event.clientY - rect.top - centerY;
            updateJoystick(x, y);
        });
        
        document.addEventListener('mousemove', (event) => {
            if (isDragging && joystickArea) {
                const rect = joystickArea.getBoundingClientRect();
                const x = event.clientX - rect.left - centerX;
                const y = event.clientY - rect.top - centerY;
                updateJoystick(x, y);
            }
        });
        
        document.addEventListener('mouseup', () => {
            if (isDragging) {
                isDragging = false;
                resetJoystick();
            }
        });
        
        // Touch events for mobile
        joystickArea.addEventListener('touchstart', (event) => {
            event.preventDefault();
            isDragging = true;
            const rect = joystickArea.getBoundingClientRect();
            const touch = event.touches[0];
            const x = touch.clientX - rect.left - centerX;
            const y = touch.clientY - rect.top - centerY;
            updateJoystick(x, y);
        });
        
        document.addEventListener('touchmove', (event) => {
            if (isDragging && joystickArea) {
                event.preventDefault();
                const rect = joystickArea.getBoundingClientRect();
                const touch = event.touches[0];
                const x = touch.clientX - rect.left - centerX;
                const y = touch.clientY - rect.top - centerY;
                updateJoystick(x, y);
            }
        });
        
        document.addEventListener('touchend', () => {
            if (isDragging) {
                isDragging = false;
                resetJoystick();
            }
        });
    }

    initRealJoystick() {
        // Check if Gamepad API is supported
        if (!navigator.getGamepads) {
            console.log('Gamepad API not supported');
            return;
        }
        
        // Track gamepad state
        this.gamepadConnected = false;
        this.gamepadIndex = null;
        this.r2Pressed = false;
        this.r1Pressed = false; // Track R1 button state
        this.lastGamepadInput = false; // Track if there was input in the previous frame
        
        // Update joystick status display
        this.updateJoystickStatus = (connected) => {
            const statusElement = document.getElementById('joystick-status');
            if (statusElement) {
                if (connected) {
                    statusElement.textContent = '🎮 Connected';
                    statusElement.style.color = '#00ff00';
                } else {
                    statusElement.textContent = '🎮 Disconnected';
                    statusElement.style.color = '#ff0000';
                }
            }
        };
        
        // Gamepad connection events
        window.addEventListener('gamepadconnected', (event) => {
            console.log('🎮 Gamepad connected:', event.gamepad.id);
            this.gamepadConnected = true;
            this.gamepadIndex = event.gamepad.index;
            this.updateJoystickStatus(true);
        });
        
        window.addEventListener('gamepaddisconnected', (event) => {
            console.log('🎮 Gamepad disconnected:', event.gamepad.id);
            this.gamepadConnected = false;
            this.gamepadIndex = null;
            this.updateJoystickStatus(false);
        });
        
        // Gamepad polling function
        const pollGamepad = () => {
            if (!this.gamepadConnected || this.gamepadIndex === null) {
                return;
            }
            
            const gamepads = navigator.getGamepads();
            const gamepad = gamepads[this.gamepadIndex];
            
            if (!gamepad) {
                return;
            }
            
            // Get joystick axes (usually axes 0 and 1 for left stick)
            const leftStickX = gamepad.axes[0] || 0; // Left/Right
            const leftStickY = gamepad.axes[1] || 0; // Forward/Backward
            
            // Apply larger deadzone (0.2)
            const deadzone = 0.2;
            let linearX = 0, angularZ = 0;
            
            if (Math.abs(leftStickY) > deadzone) {
                linearX = -leftStickY * 3.0; // Forward/Backward
            }
            
            if (Math.abs(leftStickX) > deadzone) {
                angularZ = leftStickX * 2.5; // Left/Right turn
            }
            
            // Update GUI joystick position
            this.updateGUIJoystick(leftStickX, leftStickY);
            
            // Track if there was input in this frame
            const hasInput = Math.abs(linearX) > 0.01 || Math.abs(angularZ) > 0.01;
            
            // Publish commands only if there's input
            if (hasInput) {
                this.publishCmdVel(linearX, 0, angularZ);
                this.lastGamepadInput = true;
            } else if (this.lastGamepadInput) {
                // Send stop command only once when joystick is released
                this.publishCmdVel(0, 0, 0);
                this.lastGamepadInput = false;
            }
            
            // Handle buttons
            if (gamepad.buttons[0] && gamepad.buttons[0].pressed) {
                // A button - toggle camera
                this.toggleFrontBack();
            }
            
            // R1 button (usually button 5) - toggle front/back only on press down
            const r1Pressed = gamepad.buttons[5] && gamepad.buttons[5].pressed;
            if (r1Pressed && !this.r1Pressed) {
                // R1 just pressed - toggle camera
                this.toggleFrontBack();
                this.r1Pressed = true;
            } else if (!r1Pressed && this.r1Pressed) {
                // R1 just released - reset state
                this.r1Pressed = false;
            }
            
            // R2 button (usually button 7) - hold for back camera, release for front
            const r2Pressed = gamepad.buttons[7] && gamepad.buttons[7].pressed;
            if (r2Pressed && !this.r2Pressed) {
                // R2 just pressed - show back camera
                this.updateCameraLayout('back');
                this.r2Pressed = true;
            } else if (!r2Pressed && this.r2Pressed) {
                // R2 just released - show front camera
                this.updateCameraLayout('front');
                this.r2Pressed = false;
            }
        };
        
        // Poll gamepad at 60Hz
        setInterval(pollGamepad, 16);
    }
    
    updateGUIJoystick(x, y) {
        const joystickKnob = document.getElementById('joystick-knob');
        const linearDisplay = document.getElementById('linear-display');
        const angularDisplay = document.getElementById('angular-display');
        
        if (!joystickKnob) return;
        
        const centerX = 60; // Half of joystick area width
        const centerY = 60; // Half of joystick area height
        const maxDistance = 40; // Maximum distance from center
        
        // Apply deadzone
        const deadzone = 0.2;
        if (Math.abs(x) < deadzone) x = 0;
        if (Math.abs(y) < deadzone) y = 0;
        
        // Scale to joystick area
        const scaledX = x * maxDistance;
        const scaledY = y * maxDistance;
        
        // Position knob
        joystickKnob.style.transform = `translate(calc(-50% + ${scaledX}px), calc(-50% + ${scaledY}px))`;
        
        // Update displays
        if (linearDisplay) linearDisplay.textContent = (-y * 3.0).toFixed(2);
        if (angularDisplay) angularDisplay.textContent = (x * 2.5).toFixed(2);
    }

    updateKeyVisual(key, pressed) {
        const keyElements = document.querySelectorAll('.key');
        keyElements.forEach(element => {
            if (element.getAttribute('data-key') === key) {
                if (pressed) {
                    element.classList.add('active');
                } else {
                    element.classList.remove('active');
                }
            }
        });
    }

    toggleFrontBack() {
        // Check which camera is currently active
        const centerCamera = document.getElementById('center-camera');
        const primary = centerCamera.getAttribute('data-camera');
        
        if (primary === 'front') {
            // Currently showing front, switch to back
            this.updateCameraLayout('back');
        } else {
            // Currently showing back, switch to front
            this.updateCameraLayout('front');
        }
    }

    initCameraSwitching() {
        // Set initial camera layout to front
        this.updateCameraLayout('front');
    }


}

// Initialize the application when the page loads
let roverUI;
document.addEventListener('DOMContentLoaded', () => {
    roverUI = new RoverWebUI();
    
    // Initialize UI and controls after the object is created
    setTimeout(() => {
        roverUI.initUI();
        roverUI.initControls();
        roverUI.initMapVisualization();
        roverUI.initIMUArrow();
    }, 100);
});

