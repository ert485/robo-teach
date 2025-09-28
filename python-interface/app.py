from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import json
import math
import time
import threading
import random
import os
import subprocess

app = Flask(__name__)
app.config["SECRET_KEY"] = "your-secret-key"
# Use threading async mode to avoid eventlet/gevent requirements
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# Robot state
robot_state = {
    "connected": False,
    "position": {"x": 0, "y": 0, "angle": 0},
    "speed": 0,
    "lidar_data": [],
    "map_data": [],
    "battery": 100,
    "status": "idle",
}


# Simulated LiDAR robot for demo purposes
class RobotSimulator:
    def __init__(self):
        self.position = {"x": 0, "y": 0, "angle": 0}
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
                lidar_points.append(
                    {"angle": angle, "distance": distance, "x": x, "y": y}
                )

            robot_state["lidar_data"] = lidar_points
            socketio.emit("lidar_update", lidar_points)
            time.sleep(0.1)  # Update 10 times per second

    def _update_position(self):
        """Update robot position based on movement commands"""
        while self.running:
            if self.speed != 0:
                # Simple position update
                rad = math.radians(self.position["angle"])
                self.position["x"] += self.speed * math.cos(rad) * 0.1
                self.position["y"] += self.speed * math.sin(rad) * 0.1

                robot_state["position"] = self.position
                socketio.emit("position_update", self.position)

            time.sleep(0.1)

    def move_forward(self, speed=1.0):
        self.speed = speed
        robot_state["status"] = "moving_forward"

    def move_backward(self, speed=1.0):
        self.speed = -speed
        robot_state["status"] = "moving_backward"

    def turn_left(self):
        self.position["angle"] -= 5
        robot_state["status"] = "turning_left"

    def turn_right(self):
        self.position["angle"] += 5
        robot_state["status"] = "turning_right"

    def stop(self):
        self.speed = 0
        robot_state["status"] = "idle"


# Initialize robot simulator
robot_sim = RobotSimulator()

# -----------------------------
# ROS2 integration (simplified)
# -----------------------------


def _run_move_forward_script():
    """Run the move-forward.sh script to move the robot forward.
    Returns a dict with 'success', 'message', 'used_ros2'.
    """
    script_path = os.path.join(os.path.dirname(__file__), "move-forward.sh")

    if not os.path.exists(script_path):
        return {
            "success": False,
            "message": "move-forward.sh script not found",
            "used_ros2": False,
        }

    try:
        result = subprocess.run(
            ["bash", script_path], capture_output=True, text=True, timeout=10
        )

        if result.returncode == 0:
            return {
                "success": True,
                "message": "Move forward script executed successfully",
                "used_ros2": True,
            }
        else:
            error_msg = (
                result.stderr.strip() or result.stdout.strip() or "Script failed"
            )
            return {
                "success": False,
                "message": f"Script failed: {error_msg}",
                "used_ros2": True,
            }

    except subprocess.TimeoutExpired:
        return {"success": False, "message": "Script timeout", "used_ros2": True}
    except Exception as e:
        return {
            "success": False,
            "message": f"Script error: {str(e)}",
            "used_ros2": False,
        }


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/connect", methods=["POST"])
def connect_robot():
    """Connect to the robot (simulated)"""
    try:
        robot_state["connected"] = True
        robot_state["status"] = "connected"
        robot_sim.start_simulation()
        return jsonify({"success": True, "message": "Robot connected successfully"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route("/api/disconnect", methods=["POST"])
def disconnect_robot():
    """Disconnect from the robot"""
    try:
        robot_state["connected"] = False
        robot_state["status"] = "disconnected"
        robot_sim.stop_simulation()
        return jsonify({"success": True, "message": "Robot disconnected"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route("/api/status")
def get_status():
    """Get current robot status"""
    return jsonify(robot_state)


@app.route("/api/control/<action>", methods=["POST"])
def control_robot(action):
    """Control robot movements"""
    if not robot_state["connected"]:
        return jsonify({"success": False, "message": "Robot not connected"})

    try:
        if action == "forward":
            # Try real ROS2 move using the move-forward.sh script; fallback to simulator if not available
            script_result = _run_move_forward_script()
            if script_result["success"]:

                def _run_move():
                    # After move, reset sim state to idle
                    time.sleep(1.5)  # Wait for script to complete
                    robot_state["status"] = "idle"
                    socketio.emit("status_update", robot_state)

                threading.Thread(target=_run_move, daemon=True).start()
                robot_state["status"] = "moving_forward"
                return jsonify(
                    {
                        "success": True,
                        "message": script_result["message"],
                        "used_ros2": script_result["used_ros2"],
                    }
                )
            else:
                # Fallback to simulator
                robot_sim.move_forward()
                return jsonify(
                    {
                        "success": True,
                        "message": f"Fallback to simulator: {script_result['message']}",
                        "used_ros2": False,
                    }
                )
        elif action == "backward":
            robot_sim.move_backward()
        elif action == "left":
            robot_sim.turn_left()
        elif action == "right":
            robot_sim.turn_right()
        elif action == "stop":
            robot_sim.stop()
        else:
            return jsonify({"success": False, "message": "Unknown action"})

        return jsonify(
            {
                "success": True,
                "message": f"Command {action} executed",
                "used_ros2": False,
            }
        )
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route("/api/shutdown", methods=["POST"])
def shutdown_robot():
    """Shutdown the robot and server"""
    try:
        robot_state["connected"] = False
        robot_state["status"] = "shutting_down"
        robot_sim.stop_simulation()

        # In a real application, you might want to do a more graceful shutdown
        # For now, we'll just return success and let the client handle it
        return jsonify({"success": True, "message": "RoboTech system shutting down..."})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@socketio.on("connect")
def handle_connect():
    print("Client connected")
    emit("status_update", robot_state)


@socketio.on("disconnect")
def handle_disconnect():
    print("Client disconnected")


@socketio.on("request_map_update")
def handle_map_request():
    """Send current map data to client"""
    # Generate a simple map from LiDAR data
    map_points = []
    if robot_state["lidar_data"]:
        robot_pos = robot_state["position"]
        for point in robot_state["lidar_data"]:
            # Convert local coordinates to global coordinates
            global_x = robot_pos["x"] + point["x"]
            global_y = robot_pos["y"] + point["y"]
            map_points.append({"x": global_x, "y": global_y, "type": "obstacle"})

    emit("map_update", map_points)


if __name__ == "__main__":
    socketio.run(app, debug=True, host="0.0.0.0", port=5001, allow_unsafe_werkzeug=True)
