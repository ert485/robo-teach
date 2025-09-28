// RoboTech Web App - Frontend Logic
class RobotController {
    constructor() {
        this.socket = null;
        this.connected = false;
        this.mapCanvas = null;
        this.mapCtx = null;
        this.lidarCanvas = null;
        this.lidarCtx = null;
        this.robotPosition = { x: 0, y: 0, angle: 0 };
        this.mapData = [];
        this.lidarData = [];
        this.mapScale = 50; // pixels per meter
        this.mapOffset = { x: 200, y: 200 };
        this.connectionTime = null;
        this.connectionTimer = null;
        this.currentSpeed = 1.0;
        // AR feature state
        this.showFOV = true;
        this.showBreadcrumbs = true;
        this.showSafety = true;
        this.waypointMode = false;
        this.safetyRadius = 0.7; // meters
        this.breadcrumbs = [];
        this.waypoints = [];
        this.lastBreadcrumbTime = 0;
    this.mapClickBound = false;
        
        this.initializeApp();
    }
    
    initializeApp() {
        this.setupWebSocket();
        this.setupCanvas();
        this.setupEventListeners();
        this.updateUI();
    }
    
    setupWebSocket() {
        // Initialize Socket.IO connection
        this.socket = io();
        
        this.socket.on('connect', () => {
            console.log('Connected to server');
        });
        
        this.socket.on('disconnect', () => {
            console.log('Disconnected from server');
        });
        
        this.socket.on('status_update', (data) => {
            this.updateRobotStatus(data);
        });
        
        this.socket.on('position_update', (position) => {
            this.robotPosition = position;
            this.updatePositionDisplay();
            this.drawMap();
        });
        
        this.socket.on('lidar_update', (lidarPoints) => {
            this.lidarData = lidarPoints;
            this.updateLidarDisplay();
            this.drawLidarScan();
            this.updateARFeatures();
        });
        
        this.socket.on('map_update', (mapPoints) => {
            this.mapData = mapPoints;
            this.drawMap();
        });
    }
    
    setupCanvas() {
        // Setup canvas after dashboard is shown (defer if needed)
        this.initializeCanvas();
    }
    
    initializeCanvas() {
        // Map Canvas - try to find it
        this.mapCanvas = document.getElementById('mapCanvas');
        if (this.mapCanvas) {
            this.mapCtx = this.mapCanvas.getContext('2d');
            // Map click handler for waypoint mode (bind once)
            if (!this.mapClickBound) {
                this.mapCanvas.addEventListener('click', (e) => this.onMapClick(e));
                this.mapClickBound = true;
            }
        }
        
        // LiDAR Canvas - try to find it
        this.lidarCanvas = document.getElementById('lidarCanvas');
        if (this.lidarCanvas) {
            this.lidarCtx = this.lidarCanvas.getContext('2d');
        }
        
        // Draw initial state if canvas exists
        if (this.mapCtx) this.drawMap();
        if (this.lidarCtx) this.drawLidarScan();
    }
    
    setupEventListeners() {
        // Connection buttons
        document.getElementById('connectBtn').addEventListener('click', () => this.connectRobot());
        document.getElementById('disconnectBtn').addEventListener('click', () => this.disconnectRobot());
        
        // Control buttons
        document.getElementById('forwardBtn').addEventListener('click', () => this.sendCommand('forward'));
        document.getElementById('backwardBtn').addEventListener('click', () => this.sendCommand('backward'));
        document.getElementById('leftBtn').addEventListener('click', () => this.sendCommand('left'));
        document.getElementById('rightBtn').addEventListener('click', () => this.sendCommand('right'));
        document.getElementById('stopBtn').addEventListener('click', () => this.sendCommand('stop'));
        
        // Shutdown button
        document.getElementById('shutdownBtn').addEventListener('click', () => this.shutdownRobot());
        
        // Emergency stop button
        document.getElementById('emergencyStopBtn').addEventListener('click', () => this.emergencyStop());
        
        // System control buttons
        document.getElementById('resetPositionBtn').addEventListener('click', () => this.resetPosition());
        document.getElementById('calibrateBtn').addEventListener('click', () => this.calibrateSystem());
        
        // Speed control
        const speedSlider = document.getElementById('speedSlider');
        if (speedSlider) {
            speedSlider.addEventListener('input', (e) => this.updateSpeed(e.target.value));
        }
        
        // Map controls
        document.getElementById('clearMapBtn').addEventListener('click', () => this.clearMap());
        document.getElementById('centerMapBtn').addEventListener('click', () => this.centerMap());
        
        // Additional map controls
        const saveMapBtn = document.getElementById('saveMapBtn');
        if (saveMapBtn) {
            saveMapBtn.addEventListener('click', () => this.saveMap());
        }
        
        // AR controls
        document.getElementById('toggleArBtn').addEventListener('click', () => this.toggleAR());
        document.getElementById('calibrateBtn').addEventListener('click', () => this.calibrateAR());
    const toggleFOV = document.getElementById('toggleFOV');
    const toggleBreadcrumbs = document.getElementById('toggleBreadcrumbs');
    const toggleSafety = document.getElementById('toggleSafety');
    const toggleWaypointMode = document.getElementById('toggleWaypointMode');
    const safetyRadius = document.getElementById('safetyRadius');
    const safetyRadiusValue = document.getElementById('safetyRadiusValue');
    const clearWaypointsBtn = document.getElementById('clearWaypointsBtn');
    const clearBreadcrumbsBtn = document.getElementById('clearBreadcrumbsBtn');
    if (toggleFOV) toggleFOV.addEventListener('change', (e) => { this.showFOV = e.target.checked; this.drawMap(); });
    if (toggleBreadcrumbs) toggleBreadcrumbs.addEventListener('change', (e) => { this.showBreadcrumbs = e.target.checked; this.drawMap(); });
    if (toggleSafety) toggleSafety.addEventListener('change', (e) => { this.showSafety = e.target.checked; this.drawMap(); });
    if (toggleWaypointMode) toggleWaypointMode.addEventListener('change', (e) => { this.waypointMode = e.target.checked; this.showMessage(this.waypointMode ? '🧭 Waypoint mode ON (click map)' : '🧭 Waypoint mode OFF', 'info'); });
    if (safetyRadius) safetyRadius.addEventListener('input', (e) => { this.safetyRadius = parseFloat(e.target.value); if (safetyRadiusValue) safetyRadiusValue.textContent = `${this.safetyRadius.toFixed(1)} m`; this.drawMap(); });
    if (clearWaypointsBtn) clearWaypointsBtn.addEventListener('click', () => { this.waypoints = []; this.drawMap(); this.showMessage('Waypoints cleared', 'info'); this.updateNavReadouts(); });
    if (clearBreadcrumbsBtn) clearBreadcrumbsBtn.addEventListener('click', () => { this.breadcrumbs = []; this.drawMap(); this.showMessage('Breadcrumbs cleared', 'info'); });
        
        // Auto navigate button
        const autoNavigateBtn = document.getElementById('autoNavigateBtn');
        if (autoNavigateBtn) {
            autoNavigateBtn.addEventListener('click', () => this.toggleAutoNavigate());
        }
        
        // Keyboard controls
        document.addEventListener('keydown', (e) => this.handleKeyPress(e));
        document.addEventListener('keyup', (e) => this.handleKeyRelease(e));
    }
    
    async connectRobot() {
        try {
            console.log('Attempting to connect...');
            // Show connecting state
            this.updateConnectionState('connecting');
            
            const response = await fetch('/api/connect', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            const result = await response.json();
            
            console.log('Connection response:', result);
            
            if (result.success) {
                this.connected = true;
                console.log('Connection successful, updating state...');
                this.updateConnectionState('connected');
                this.showMessage('Connected to robot successfully!', 'success');
                this.showMainFeatures();
                this.updateUI();
            } else {
                console.log('Connection failed:', result.message);
                this.updateConnectionState('disconnected');
                this.showMessage('Failed to connect: ' + result.message, 'error');
            }
        } catch (error) {
            console.error('Connection error:', error);
            this.updateConnectionState('disconnected');
            this.showMessage('Connection error: ' + error.message, 'error');
        }
    }
    
    async disconnectRobot() {
        try {
            const response = await fetch('/api/disconnect', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            const result = await response.json();
            
            if (result.success) {
                this.connected = false;
                this.updateConnectionState('disconnected');
                this.showMessage('Disconnected from robot', 'info');
                this.hideMainFeatures();
                this.updateUI();
            }
        } catch (error) {
            this.showMessage('Disconnection error: ' + error.message, 'error');
        }
    }
    
    updateConnectionState(state) {
        const statusIndicator = document.getElementById('statusIndicator');
        const statusText = document.getElementById('statusText');
        const connectBtn = document.getElementById('connectBtn');
        const disconnectBtn = document.getElementById('disconnectBtn');
        const connectionInfo = document.getElementById('connectionInfo');
        
        // Remove all state classes
        statusIndicator.className = 'status-indicator';
        
        switch(state) {
            case 'connecting':
                statusIndicator.classList.add('connecting');
                statusText.textContent = 'Connecting...';
                connectBtn.style.display = 'none';
                disconnectBtn.style.display = 'none';
                connectionInfo.style.display = 'none';
                break;
                
            case 'connected':
                statusIndicator.classList.add('connected');
                statusText.textContent = 'Connected';
                connectBtn.style.display = 'none';
                disconnectBtn.style.display = 'inline-block';
                connectionInfo.style.display = 'block';
                break;
                
            case 'disconnected':
            default:
                statusIndicator.classList.add('disconnected');
                statusText.textContent = 'Disconnected';
                connectBtn.style.display = 'inline-block';
                disconnectBtn.style.display = 'none';
                connectionInfo.style.display = 'none';
                break;
        }
    }
    
    showMainFeatures() {
        const mainFeatures = document.getElementById('mainFeatures');
        const welcomeSection = document.getElementById('welcomeSection');
        const controlDashboard = document.getElementById('controlDashboard');
        
        console.log('Showing main features...');
        console.log('controlDashboard element:', controlDashboard);
        
        // Hide welcome section
        if (welcomeSection) {
            welcomeSection.style.display = 'none';
        }
        
        // Show control dashboard instead of basic features
        if (controlDashboard) {
            controlDashboard.style.display = 'block';
            controlDashboard.style.animation = 'fadeInUp 0.8s ease-out';
            
            // Reinitialize canvas after dashboard is shown
            setTimeout(() => {
                this.initializeCanvas();
            }, 100);
            
            // Start connection timer
            this.connectionTime = new Date();
            this.startConnectionTimer();
        } else {
            console.error('Control dashboard element not found!');
        }
        
        // Hide basic main features (keep as fallback)
        if (mainFeatures) {
            mainFeatures.style.display = 'none';
        }
        
        // Show success message
        setTimeout(() => {
            this.showMessage('🎉 Control Dashboard Activated! Full robot control now available.', 'success');
        }, 500);
    }
    
    hideMainFeatures() {
        const mainFeatures = document.getElementById('mainFeatures');
        const welcomeSection = document.getElementById('welcomeSection');
        const controlDashboard = document.getElementById('controlDashboard');
        
        // Hide control dashboard
        if (controlDashboard) {
            controlDashboard.style.display = 'none';
        }
        
        // Hide main features
        if (mainFeatures) {
            mainFeatures.style.display = 'none';
        }
        
        // Show welcome section
        if (welcomeSection) {
            welcomeSection.style.display = 'block';
        }
        
        // Stop connection timer
        this.stopConnectionTimer();
    }
    
    startConnectionTimer() {
        this.connectionTimer = setInterval(() => {
            if (this.connectionTime) {
                const now = new Date();
                const elapsed = Math.floor((now - this.connectionTime) / 1000);
                const minutes = Math.floor(elapsed / 60);
                const seconds = elapsed % 60;
                const timeString = `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
                
                const timeDisplay = document.getElementById('dashboardConnectionTime');
                if (timeDisplay) {
                    timeDisplay.textContent = timeString;
                }
            }
        }, 1000);
    }
    
    stopConnectionTimer() {
        if (this.connectionTimer) {
            clearInterval(this.connectionTimer);
            this.connectionTimer = null;
        }
        this.connectionTime = null;
    }
    
    updateSpeed(value) {
        this.currentSpeed = parseFloat(value);
        const speedDisplay = document.getElementById('speedValue');
        if (speedDisplay) {
            speedDisplay.textContent = `${value} m/s`;
        }
    }
    
    emergencyStop() {
        this.showMessage('🚨 EMERGENCY STOP ACTIVATED!', 'error');
        this.sendCommand('stop');
        
        // Visual feedback
        const btn = document.getElementById('emergencyStopBtn');
        const original = btn.style.background;
        btn.style.background = '#ff0000';
        setTimeout(() => {
            btn.style.background = original;
        }, 1000);
    }
    
    resetPosition() {
        if (confirm('Reset robot position to origin (0,0)?')) {
            this.robotPosition = { x: 0, y: 0, angle: 0 };
            this.updateDashboardStatus();
            this.showMessage('📍 Robot position reset to origin', 'info');
        }
    }
    
    calibrateSystem() {
        this.showMessage('⚙️ Calibrating system...', 'info');
        
        // Simulate calibration process
        let progress = 0;
        const calibrationInterval = setInterval(() => {
            progress += 20;
            this.showMessage(`⚙️ Calibrating... ${progress}%`, 'info');
            
            if (progress >= 100) {
                clearInterval(calibrationInterval);
                this.showMessage('✅ System calibration complete!', 'success');
            }
        }, 500);
    }
    
    saveMap() {
        // Simulate saving map data
        this.showMessage('💾 Saving map data...', 'info');
        setTimeout(() => {
            this.showMessage('✅ Map saved successfully!', 'success');
        }, 1000);
    }
    
    toggleAutoNavigate() {
        const btn = document.getElementById('autoNavigateBtn');
        if (btn.textContent.includes('Auto Navigate')) {
            btn.innerHTML = '⏹️ Stop Auto';
            this.showMessage('🤖 Auto navigation enabled', 'info');
        } else {
            btn.innerHTML = '🤖 Auto Navigate';
            this.showMessage('👤 Manual control restored', 'info');
        }
    }
    
    updateDashboardStatus() {
        // Update position displays
        const dashPosition = document.getElementById('dashPosition');
        const dashHeading = document.getElementById('dashHeading');
        
        if (dashPosition) {
            dashPosition.textContent = `X: ${this.robotPosition.x.toFixed(1)}, Y: ${this.robotPosition.y.toFixed(1)}`;
        }
        
        if (dashHeading) {
            dashHeading.textContent = `${this.robotPosition.angle.toFixed(0)}°`;
        }
        
        // Update AR heading
        const arHeading = document.getElementById('arHeading');
        if (arHeading) {
            arHeading.textContent = `${this.robotPosition.angle.toFixed(0)}°`;
        }
        
        // Update compass needle
        const compassNeedle = document.getElementById('compassNeedle');
        if (compassNeedle) {
            compassNeedle.style.transform = `rotate(${this.robotPosition.angle}deg)`;
        }
        // Add breadcrumb periodically
        const now = Date.now();
        if (now - this.lastBreadcrumbTime > 800) {
            this.breadcrumbs.push({ x: this.robotPosition.x, y: this.robotPosition.y });
            if (this.breadcrumbs.length > 200) this.breadcrumbs.shift();
            this.lastBreadcrumbTime = now;
        }
        this.updateNavReadouts();
    }
    
    async sendCommand(action) {
        if (!this.connected) {
            this.showMessage('Robot not connected!', 'error');
            return;
        }
        
        try {
            const response = await fetch(`/api/control/${action}`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            const result = await response.json();
            
            if (!result.success) {
                this.showMessage('Command failed: ' + result.message, 'error');
            }
        } catch (error) {
            this.showMessage('Command error: ' + error.message, 'error');
        }
    }
    
    async shutdownRobot() {
        if (!confirm('⚠️ Are you sure you want to turn off RoboTech?\n\nThis will shutdown the robot and close the application.')) {
            return;
        }
        
        try {
            this.showMessage('Shutting down RoboTech system...', 'warning');
            
            const response = await fetch('/api/shutdown', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            const result = await response.json();
            
            if (result.success) {
                this.showMessage('✅ RoboTech system shutdown initiated', 'success');
                
                // Show countdown
                let countdown = 3;
                const countdownInterval = setInterval(() => {
                    this.showMessage(`🔌 Shutting down in ${countdown}...`, 'warning');
                    countdown--;
                    
                    if (countdown < 0) {
                        clearInterval(countdownInterval);
                        this.showMessage('👋 RoboTech shutdown complete. You can close this window.', 'success');
                        
                        // Disconnect WebSocket
                        if (this.socket) {
                            this.socket.close();
                        }
                        
                        // Disable all controls
                        document.querySelectorAll('button').forEach(btn => {
                            btn.disabled = true;
                        });
                        
                        // Hide main features
                        this.hideMainFeatures();
                    }
                }, 1000);
            } else {
                this.showMessage(`❌ Shutdown failed: ${result.message}`, 'error');
            }
        } catch (error) {
            console.error('Shutdown error:', error);
            this.showMessage('❌ Failed to communicate with server', 'error');
        }
    }
    
    handleKeyPress(e) {
        if (!this.connected) return;
        
        switch(e.code) {
            case 'ArrowUp':
            case 'KeyW':
                e.preventDefault();
                this.sendCommand('forward');
                break;
            case 'ArrowDown':
            case 'KeyS':
                e.preventDefault();
                this.sendCommand('backward');
                break;
            case 'ArrowLeft':
            case 'KeyA':
                e.preventDefault();
                this.sendCommand('left');
                break;
            case 'ArrowRight':
            case 'KeyD':
                e.preventDefault();
                this.sendCommand('right');
                break;
            case 'Space':
                e.preventDefault();
                this.sendCommand('stop');
                break;
        }
    }
    
    handleKeyRelease(e) {
        // Stop movement when key is released (for smoother control)
        if (this.connected && ['ArrowUp', 'ArrowDown', 'KeyW', 'KeyS'].includes(e.code)) {
            setTimeout(() => this.sendCommand('stop'), 100);
        }
    }
    
    updateUI() {
        // Connection state is now handled by updateConnectionState method
        // This method can be used for other UI updates if needed
        if (this.connected) {
            this.updatePositionDisplay();
        }
    }
    
    updateRobotStatus(status) {
        this.connected = status.connected;
        this.robotPosition = status.position;
        
        // Update connection state
        this.updateConnectionState(this.connected ? 'connected' : 'disconnected');
        
        // Update position displays
        document.getElementById('robotPosition').textContent = 
            `X: ${this.robotPosition.x.toFixed(1)}, Y: ${this.robotPosition.y.toFixed(1)}`;
        document.getElementById('robotAngle').textContent = 
            `${this.robotPosition.angle.toFixed(0)}°`;
        document.getElementById('robotStatus').textContent = status.status;
        
        // Show/hide features based on connection
        const controlDashboard = document.getElementById('controlDashboard');
        if (this.connected && controlDashboard && controlDashboard.style.display === 'none') {
            this.showMainFeatures();
        } else if (!this.connected && controlDashboard && controlDashboard.style.display !== 'none') {
            this.hideMainFeatures();
        }
        
        // Update dashboard status
        this.updateDashboardStatus();
    }
    
    updatePositionDisplay() {
        document.getElementById('robotPosition').textContent = 
            `X: ${this.robotPosition.x.toFixed(1)}, Y: ${this.robotPosition.y.toFixed(1)}`;
        document.getElementById('robotAngle').textContent = 
            `${this.robotPosition.angle.toFixed(0)}°`;
    }
    
    updateLidarDisplay() {
        if (this.lidarData.length > 0) {
            const pointCount = document.getElementById('lidarPointCount');
            if (pointCount) {
                pointCount.textContent = this.lidarData.length;
            }
        }
    }
    
    drawMap() {
        const canvas = this.mapCanvas;
        const ctx = this.mapCtx;
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw grid
        this.drawGrid(ctx, canvas);
        
        // Draw breadcrumbs
        if (this.showBreadcrumbs && this.breadcrumbs.length > 1) {
            ctx.strokeStyle = 'rgba(42,82,152,0.6)';
            ctx.lineWidth = 2;
            ctx.beginPath();
            this.breadcrumbs.forEach((p, i) => {
                const sx = this.mapOffset.x + p.x * this.mapScale;
                const sy = this.mapOffset.y - p.y * this.mapScale;
                if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
            });
            ctx.stroke();
        }

        // Draw map points (obstacles)
        ctx.fillStyle = '#ff4444';
        this.mapData.forEach(point => {
            if (point.type === 'obstacle') {
                const screenX = this.mapOffset.x + point.x * this.mapScale;
                const screenY = this.mapOffset.y - point.y * this.mapScale;
                ctx.beginPath();
                ctx.arc(screenX, screenY, 2, 0, 2 * Math.PI);
                ctx.fill();
            }
        });
        // Draw waypoints and path
        this.drawWaypoints(ctx);
        this.drawPathToWaypoint(ctx);

        // Draw robot
        this.drawRobot(ctx);

        // Draw safety bubble
        if (this.showSafety) this.drawSafetyBubble(ctx);

        // Draw FOV cone on map
        if (this.showFOV) this.drawFOV(ctx);
    }
    
    drawGrid(ctx, canvas) {
        ctx.strokeStyle = '#e0e0e0';
        ctx.lineWidth = 1;
        
        // Vertical lines
        for (let x = 0; x <= canvas.width; x += this.mapScale) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, canvas.height);
            ctx.stroke();
        }
        
        // Horizontal lines
        for (let y = 0; y <= canvas.height; y += this.mapScale) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(canvas.width, y);
            ctx.stroke();
        }
        
        // Center lines
        ctx.strokeStyle = '#cccccc';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(this.mapOffset.x, 0);
        ctx.lineTo(this.mapOffset.x, canvas.height);
        ctx.moveTo(0, this.mapOffset.y);
        ctx.lineTo(canvas.width, this.mapOffset.y);
        ctx.stroke();
    }
    
    drawRobot(ctx) {
        const screenX = this.mapOffset.x + this.robotPosition.x * this.mapScale;
        const screenY = this.mapOffset.y - this.robotPosition.y * this.mapScale;
        const angle = this.robotPosition.angle * Math.PI / 180;
        
        ctx.save();
        ctx.translate(screenX, screenY);
        ctx.rotate(-angle);
        
        // Robot body
        ctx.fillStyle = '#4CAF50';
        ctx.fillRect(-10, -8, 20, 16);
        
        // Robot direction indicator
        ctx.fillStyle = '#2E7D32';
        ctx.beginPath();
        ctx.moveTo(10, 0);
        ctx.lineTo(15, -5);
        ctx.lineTo(15, 5);
        ctx.closePath();
        ctx.fill();
        
        ctx.restore();
    }

    drawSafetyBubble(ctx) {
        const screenX = this.mapOffset.x + this.robotPosition.x * this.mapScale;
        const screenY = this.mapOffset.y - this.robotPosition.y * this.mapScale;
        ctx.save();
        ctx.strokeStyle = 'rgba(244,67,54,0.6)';
        ctx.fillStyle = 'rgba(244,67,54,0.1)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(screenX, screenY, this.safetyRadius * this.mapScale, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
        ctx.restore();
    }

    drawFOV(ctx) {
        // 60-degree FOV cone ahead of robot
        const screenX = this.mapOffset.x + this.robotPosition.x * this.mapScale;
        const screenY = this.mapOffset.y - this.robotPosition.y * this.mapScale;
        const angle = this.robotPosition.angle * Math.PI / 180;
        const fov = 60 * Math.PI / 180;
        const range = 3.0 * this.mapScale; // 3 meters visual range
        ctx.save();
        ctx.translate(screenX, screenY);
        ctx.rotate(-angle);
        ctx.fillStyle = 'rgba(102,126,234,0.15)';
        ctx.strokeStyle = 'rgba(102,126,234,0.4)';
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.arc(0, 0, range, -fov/2, fov/2);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
    }

    drawWaypoints(ctx) {
        if (!this.waypoints.length) return;
        ctx.save();
        this.waypoints.forEach((wp, i) => {
            const sx = this.mapOffset.x + wp.x * this.mapScale;
            const sy = this.mapOffset.y - wp.y * this.mapScale;
            ctx.fillStyle = i === 0 ? '#ff9800' : '#03a9f4';
            ctx.beginPath();
            ctx.arc(sx, sy, 5, 0, 2 * Math.PI);
            ctx.fill();
            // label
            ctx.fillStyle = '#333';
            ctx.font = '12px Segoe UI';
            ctx.fillText(i === 0 ? 'Goal' : `W${i}`, sx + 6, sy - 6);
        });
        ctx.restore();
    }

    drawPathToWaypoint(ctx) {
        if (!this.waypoints.length) return;
        const points = [{ x: this.robotPosition.x, y: this.robotPosition.y }, ...this.waypoints];
        ctx.save();
        ctx.strokeStyle = 'rgba(3,169,244,0.9)';
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 6]);
        ctx.beginPath();
        points.forEach((p, i) => {
            const sx = this.mapOffset.x + p.x * this.mapScale;
            const sy = this.mapOffset.y - p.y * this.mapScale;
            if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
        });
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.restore();
    }

    onMapClick(e) {
        if (!this.waypointMode) return;
        const rect = this.mapCanvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        // Convert to world coordinates
        const wx = (x - this.mapOffset.x) / this.mapScale;
        const wy = (this.mapOffset.y - y) / this.mapScale;
        this.waypoints.push({ x: wx, y: wy });
        this.showMessage(`Waypoint added at (${wx.toFixed(1)}, ${wy.toFixed(1)})`, 'success');
        this.drawMap();
        this.updateNavReadouts();
    }

    updateNavReadouts() {
        const goalEl = document.getElementById('goalDistance');
        const etaEl = document.getElementById('eta');
        const turnEl = document.getElementById('nextTurn');
        if (!goalEl || !etaEl || !turnEl) return;
        if (!this.waypoints.length) {
            goalEl.textContent = '-- m';
            etaEl.textContent = '--';
            turnEl.textContent = '--';
            return;
        }
        const goal = this.waypoints[0];
        const dx = goal.x - this.robotPosition.x;
        const dy = goal.y - this.robotPosition.y;
        const dist = Math.hypot(dx, dy);
        goalEl.textContent = `${dist.toFixed(1)} m`;
        const speed = Math.max(this.currentSpeed, 0.1);
        const sec = dist / speed;
        const mm = Math.floor(sec / 60);
        const ss = Math.floor(sec % 60);
        etaEl.textContent = `${mm}:${ss.toString().padStart(2, '0')}`;
        // Heading difference
        const targetAngle = (Math.atan2(dy, dx) * 180 / Math.PI + 360) % 360;
        let diff = ((targetAngle - this.robotPosition.angle + 540) % 360) - 180; // [-180,180]
        const dir = diff > 10 ? 'Turn Right' : diff < -10 ? 'Turn Left' : 'On Course';
        turnEl.textContent = `${dir} (${Math.abs(diff).toFixed(0)}°)`;
    }
    
    drawLidarScan() {
        const canvas = this.lidarCanvas;
        const ctx = this.lidarCtx;
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const maxRadius = Math.min(centerX, centerY) - 20;
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw range circles
        ctx.strokeStyle = '#e0e0e0';
        ctx.lineWidth = 1;
        for (let r = maxRadius / 5; r <= maxRadius; r += maxRadius / 5) {
            ctx.beginPath();
            ctx.arc(centerX, centerY, r, 0, 2 * Math.PI);
            ctx.stroke();
        }
        
        // Draw angle lines
        for (let angle = 0; angle < 360; angle += 30) {
            const rad = angle * Math.PI / 180;
            ctx.beginPath();
            ctx.moveTo(centerX, centerY);
            ctx.lineTo(
                centerX + Math.cos(rad) * maxRadius,
                centerY + Math.sin(rad) * maxRadius
            );
            ctx.stroke();
        }
        
        // Draw LiDAR points
        ctx.fillStyle = '#ff4444';
        this.lidarData.forEach(point => {
            const normalizedDistance = Math.min(point.distance / 5.0, 1.0);
            const radius = normalizedDistance * maxRadius;
            const rad = point.angle * Math.PI / 180;
            
            const x = centerX + Math.cos(rad) * radius;
            const y = centerY + Math.sin(rad) * radius;
            
            ctx.beginPath();
            ctx.arc(x, y, 3, 0, 2 * Math.PI);
            ctx.fill();
        });
        
        // Draw robot center
        ctx.fillStyle = '#4CAF50';
        ctx.beginPath();
        ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI);
        ctx.fill();
    }
    
    updateARFeatures() {
        if (this.lidarData.length === 0) return;
        
        // Find nearest obstacle
        let minDistance = Infinity;
        this.lidarData.forEach(point => {
            if (point.distance < minDistance) {
                minDistance = point.distance;
            }
        });
        
        // Update multiple displays
        const nearestDistance = document.getElementById('nearestDistance');
        const obstacleDistance = document.getElementById('obstacleDistance');
        
        const distanceText = minDistance === Infinity ? '-- m' : `${minDistance.toFixed(1)} m`;
        
        if (nearestDistance) {
            nearestDistance.textContent = distanceText;
        }
        
        if (obstacleDistance) {
            obstacleDistance.textContent = distanceText;
        }
        
        // Update suggested action based on distance
        const suggestedAction = document.getElementById('suggestedAction');
        const pathStatus = document.getElementById('pathStatus');
        
        if (suggestedAction && pathStatus) {
            if (minDistance < 0.5) {
                suggestedAction.textContent = 'Stop/Turn';
                pathStatus.innerHTML = '❌ Blocked';
            } else if (minDistance < 1.0) {
                suggestedAction.textContent = 'Proceed Slowly';
                pathStatus.innerHTML = '⚠️ Caution';
            } else {
                suggestedAction.textContent = 'Move Forward';
                pathStatus.innerHTML = '✅ Clear';
            }
        }
        
        // Update compass needle
        const compassNeedle = document.getElementById('compassNeedle');
        if (compassNeedle) {
            compassNeedle.style.transform = `rotate(${this.robotPosition.angle}deg)`;
        }
    }
    
    clearMap() {
        this.mapData = [];
        this.drawMap();
        this.showMessage('Map cleared', 'info');
    }
    
    centerMap() {
        this.mapOffset = { x: 200, y: 200 };
        this.drawMap();
        this.showMessage('Map centered on robot', 'info');
    }
    
    toggleAR() {
        const arView = document.getElementById('arView');
        if (arView.style.display === 'none') {
            arView.style.display = 'flex';
            this.showMessage('AR view enabled', 'info');
        } else {
            arView.style.display = 'none';
            this.showMessage('AR view disabled', 'info');
        }
    }
    
    calibrateAR() {
        // Simulate AR calibration
        this.showMessage('AR system calibrated', 'success');
        
        // Reset compass to north
        const compassNeedle = document.getElementById('compassNeedle');
        compassNeedle.style.transform = 'rotate(0deg)';
    }
    
    showMessage(message, type = 'info') {
        // Create a simple toast message
        const toast = document.createElement('div');
        toast.className = `toast toast-${type}`;
        toast.textContent = message;
        toast.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: ${type === 'success' ? '#4CAF50' : type === 'error' ? '#f44336' : '#2196F3'};
            color: white;
            padding: 15px 20px;
            border-radius: 8px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.2);
            z-index: 1000;
            animation: slideIn 0.3s ease;
        `;
        
        document.body.appendChild(toast);
        
        setTimeout(() => {
            toast.style.animation = 'slideOut 0.3s ease';
            setTimeout(() => document.body.removeChild(toast), 300);
        }, 3000);
    }
}

// Add toast animations to CSS
const style = document.createElement('style');
style.textContent = `
    @keyframes slideIn {
        from { transform: translateX(100%); opacity: 0; }
        to { transform: translateX(0); opacity: 1; }
    }
    
    @keyframes slideOut {
        from { transform: translateX(0); opacity: 1; }
        to { transform: translateX(100%); opacity: 0; }
    }
`;
document.head.appendChild(style);

// Initialize the app when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    new RobotController();
});

// Add some helpful keyboard shortcuts info
console.log(`
🤖 RoboTech Control - Keyboard Shortcuts:
   W/↑ - Move Forward
   S/↓ - Move Backward  
   A/← - Turn Left
   D/→ - Turn Right
   Space - Stop
`);