from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import json
import math
import time
import threading
import random
import os
import shutil
import subprocess
import re
from typing import Optional

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key'
# Use threading async mode to avoid eventlet/gevent requirements
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Robot state
robot_state = {
    'connected': False,
    'position': {'x': 0, 'y': 0, 'angle': 0},
    'speed': 0,
    'lidar_data': [],
    'map_data': [],
    'battery': 100,
    'status': 'idle'
}

# Simulated LiDAR robot for demo purposes
class RobotSimulator:
    def __init__(self):
        self.position = {'x': 0, 'y': 0, 'angle': 0}
        self.speed = 0
        self.running = False
        
    def start_simulation(self):
        self.running = True
        threading.Thread(target=self._simulate_lidar, daemon=True).start()
        threading.Thread(target=self._update_position, daemon=True).start()
    
    def stop_simulation(self):
        self.running = False
    
    def _simulate_lidar(self):
        """Simulate LiDAR data generation"""
        while self.running:
            lidar_points = []
            # Generate simulated LiDAR points (360 degrees)
            for angle in range(0, 360, 5):
                # Simulate obstacles at random distances
                distance = random.uniform(0.5, 5.0)
                if random.random() < 0.1:  # 10% chance of obstacle
                    distance = random.uniform(0.2, 1.0)
                
                rad = math.radians(angle)
                x = distance * math.cos(rad)
                y = distance * math.sin(rad)
                lidar_points.append({'angle': angle, 'distance': distance, 'x': x, 'y': y})
            
            robot_state['lidar_data'] = lidar_points
            socketio.emit('lidar_update', lidar_points)
            time.sleep(0.1)  # Update 10 times per second
    
    def _update_position(self):
        """Update robot position based on movement commands"""
        while self.running:
            if self.speed != 0:
                # Simple position update
                rad = math.radians(self.position['angle'])
                self.position['x'] += self.speed * math.cos(rad) * 0.1
                self.position['y'] += self.speed * math.sin(rad) * 0.1
                
                robot_state['position'] = self.position
                socketio.emit('position_update', self.position)
            
            time.sleep(0.1)
    
    def move_forward(self, speed=1.0):
        self.speed = speed
        robot_state['status'] = 'moving_forward'
    
    def move_backward(self, speed=1.0):
        self.speed = -speed
        robot_state['status'] = 'moving_backward'
    
    def turn_left(self):
        self.position['angle'] -= 5
        robot_state['status'] = 'turning_left'
    
    def turn_right(self):
        self.position['angle'] += 5
        robot_state['status'] = 'turning_right'
    
    def stop(self):
        self.speed = 0
        robot_state['status'] = 'idle'

# Initialize robot simulator
robot_sim = RobotSimulator()

# -----------------------------
# ROS2 integration (optional)
# -----------------------------

def _ros2_is_available():
    """Detect if ROS2 CLI is available in current environment or via setup.bat.
    Returns a tuple (available: bool, setup_bat: Optional[str]).
    On Windows, you can set env var ROS2_SETUP_BAT to the full path of local_setup.bat.
    """
    setup_bat = os.environ.get('ROS2_SETUP_BAT')
    if shutil.which('ros2'):
        return True, setup_bat
    # If ros2 not on PATH but setup_bat provided, we can still run via `call setup && ros2 ...`
    if setup_bat and os.path.exists(setup_bat):
        return True, setup_bat
    return False, None


def _run_ros2_command(cmd: str, setup_bat: Optional[str] = None, timeout: Optional[float] = None) -> subprocess.CompletedProcess:
    """Run a ROS2 command with optional setup.bat sourcing on Windows.
    Uses cmd /c to execute a single line with `call setup && <cmd>` if setup_bat provided.
    Returns CompletedProcess.
    """
    is_windows = os.name == 'nt'
    if is_windows:
        if setup_bat and os.path.exists(setup_bat):
            full_cmd = f'cmd.exe /c "call \"{setup_bat}\" && {cmd}"'
        else:
            full_cmd = f'cmd.exe /c "{cmd}"'
        return subprocess.run(full_cmd, shell=True, capture_output=True, text=True, timeout=timeout)
    else:
        # On POSIX, if user wants to source an install, they should have the env sourced for the service;
        # we just run the command directly.
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)


def _ros2_cmd_has_subscribers(topic: str, setup_bat: Optional[str]) -> int:
    """Return subscription count for a topic using `ros2 topic info` (or -1 on error)."""
    try:
        result = _run_ros2_command(f'ros2 topic info {topic}', setup_bat=setup_bat, timeout=5)
        if result.returncode != 0:
            return -1
        # Look for "Subscription count: N"
        m = re.search(r"Subscription count:\s*(\d+)", result.stdout)
        if not m:
            # Newer ROS2 may output Subscribers:
            m2 = re.search(r"Subscribers:\s*(\d+)", result.stdout)
            return int(m2.group(1)) if m2 else -1
        return int(m.group(1))
    except Exception:
        return -1


def _publish_cmd_vel_for_duration(linear_x: float, angular_z: float, duration_sec: float = 1.0, rate_hz: int = 5, setup_bat: Optional[str] = None) -> dict:
    """Publish Twist on /cmd_vel for duration, then send explicit stop. Runs synchronously.
    Returns a dict with 'success', 'message', 'used_ros2'.
    """
    available, _ = _ros2_is_available()
    if not available:
        return {'success': False, 'message': 'ROS2 not available', 'used_ros2': False}

    topic = '/cmd_vel'
    # Optional: wait up to ~10 seconds for subscribers
    deadline = time.time() + 10.0
    sub_count = 0
    while time.time() < deadline:
        sub_count = _ros2_cmd_has_subscribers(topic, setup_bat)
        if sub_count and sub_count > 0:
            break
        time.sleep(0.5)

    # Build Twist YAML
    twist = (
        f'"{{linear: {{x: {linear_x:.3f}, y: 0.0, z: 0.0}}, '
        f'angular: {{x: 0.0, y: 0.0, z: {angular_z:.3f}}}}}"'
    )

    # Start continuous publisher
    pub_cmd = f"ros2 topic pub {topic} geometry_msgs/msg/Twist {twist} --rate {rate_hz}"
    is_windows = os.name == 'nt'
    if is_windows:
        if setup_bat and os.path.exists(setup_bat):
            spawn_cmd = f'cmd.exe /c "call \"{setup_bat}\" && {pub_cmd}"'
        else:
            spawn_cmd = f'cmd.exe /c "{pub_cmd}"'
        proc = subprocess.Popen(spawn_cmd, shell=True)
    else:
        proc = subprocess.Popen(pub_cmd, shell=True)

    # Let it run for duration
    time.sleep(max(0.1, duration_sec))

    # Stop continuous publisher
    try:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
    except Exception:
        pass

    # Send explicit stop
    stop_twist = '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'
    stop_cmd = f"ros2 topic pub --once {topic} geometry_msgs/msg/Twist {stop_twist}"
    stop_res = _run_ros2_command(stop_cmd, setup_bat=setup_bat, timeout=5)

    msg = f"Published cmd_vel for {duration_sec:.1f}s at {rate_hz} Hz; stop sent. Subscribers: {sub_count if sub_count>=0 else 'unknown'}."
    if stop_res.returncode != 0:
        msg += f" Stop publish failed: {stop_res.stderr.strip()}"
    return {'success': True, 'message': msg, 'used_ros2': True}

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/connect', methods=['POST'])
def connect_robot():
    """Connect to the robot (simulated)"""
    try:
        robot_state['connected'] = True
        robot_state['status'] = 'connected'
        robot_sim.start_simulation()
        return jsonify({'success': True, 'message': 'Robot connected successfully'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/disconnect', methods=['POST'])
def disconnect_robot():
    """Disconnect from the robot"""
    try:
        robot_state['connected'] = False
        robot_state['status'] = 'disconnected'
        robot_sim.stop_simulation()
        return jsonify({'success': True, 'message': 'Robot disconnected'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/status')
def get_status():
    """Get current robot status"""
    return jsonify(robot_state)

@app.route('/api/control/<action>', methods=['POST'])
def control_robot(action):
    """Control robot movements"""
    if not robot_state['connected']:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        if action == 'forward':
            # Try real ROS2 move for 1 second; fallback to simulator if not available
            available, setup_bat = _ros2_is_available()
            if available:
                def _run_move():
                    res = _publish_cmd_vel_for_duration(0.1, 0.0, duration_sec=1.0, rate_hz=5, setup_bat=setup_bat)
                    # After move, reset sim state to idle (do not move sim)
                    robot_state['status'] = 'idle'
                    socketio.emit('status_update', robot_state)
                threading.Thread(target=_run_move, daemon=True).start()
                robot_state['status'] = 'moving_forward'
                return jsonify({'success': True, 'message': 'ROS2: Moving forward for 1s', 'used_ros2': True})
            else:
                robot_sim.move_forward()
                used_ros2 = False
        elif action == 'backward':
            robot_sim.move_backward()
            used_ros2 = False
        elif action == 'left':
            robot_sim.turn_left()
            used_ros2 = False
        elif action == 'right':
            robot_sim.turn_right()
            used_ros2 = False
        elif action == 'stop':
            # Also send ROS2 stop if available
            available, setup_bat = _ros2_is_available()
            if available:
                def _run_stop():
                    try:
                        _run_ros2_command(
                            'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"',
                            setup_bat=setup_bat,
                            timeout=5
                        )
                    except Exception:
                        pass
                threading.Thread(target=_run_stop, daemon=True).start()
                used_ros2 = True
            else:
                used_ros2 = False
            robot_sim.stop()
        else:
            return jsonify({'success': False, 'message': 'Unknown action'})
        
        # For the forward action with ROS2 path we early-returned; the rest use used_ros2 variable
        return jsonify({'success': True, 'message': f'Command {action} executed', 'used_ros2': locals().get('used_ros2', False)})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/shutdown', methods=['POST'])
def shutdown_robot():
    """Shutdown the robot and server"""
    try:
        robot_state['connected'] = False
        robot_state['status'] = 'shutting_down'
        robot_sim.stop_simulation()
        
        # In a real application, you might want to do a more graceful shutdown
        # For now, we'll just return success and let the client handle it
        return jsonify({'success': True, 'message': 'RoboTech system shutting down...'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('status_update', robot_state)

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('request_map_update')
def handle_map_request():
    """Send current map data to client"""
    # Generate a simple map from LiDAR data
    map_points = []
    if robot_state['lidar_data']:
        robot_pos = robot_state['position']
        for point in robot_state['lidar_data']:
            # Convert local coordinates to global coordinates
            global_x = robot_pos['x'] + point['x']
            global_y = robot_pos['y'] + point['y']
            map_points.append({'x': global_x, 'y': global_y, 'type': 'obstacle'})
    
    emit('map_update', map_points)

if __name__ == '__main__':
    socketio.run(app, debug=True, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)