# 🤖 RoboTech Web Controller

A simple and easy-to-use web application for controlling LiDAR robots with real-time visualization and AR features.

## Features

### 🔌 Connection Management
- Easy one-click connection to robot server
- Real-time connection status display
- Automatic reconnection handling

### 🎮 Robot Control
- Intuitive arrow-based movement controls
- Keyboard shortcuts (WASD/Arrow keys + Spacebar)
- Emergency stop button
- Real-time position and angle tracking

### 🗺️ Real-time Mapping
- Live map generation from LiDAR data
- Grid-based coordinate system
- Robot position visualization
- Map clearing and centering options

### 📡 LiDAR Visualization
- 360-degree LiDAR scan display
- Real-time point cloud visualization
- Distance measurements
- Range circles for easy distance estimation

### 🔮 AR Features
- Compass showing robot orientation
- Distance to nearest obstacle display with path status
- Field-of-View (FOV) cone overlay on map
- Safety bubble radius with adjustable slider
- Breadcrumb trail of robot path
- Waypoint mode: click map to set Goals/Waypoints
- Live navigation readouts: Goal distance, ETA, Next turn
- AR view toggle and calibration

## Installation

### Prerequisites
- Python 3.7 or higher
- Modern web browser (Chrome, Firefox, Safari, Edge)

### Setup Steps

1. **Clone or download the project**
   ```bash
   cd c:\RoboTeach\lidar-robot-webapp
   ```

2. **Install Python dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Run the application**
   ```bash
   python app.py
   ```

4. **Open your web browser**
   Navigate to: `http://localhost:5000`

## Usage Guide

### Getting Started

1. **Launch the Application**
   - Run `python app.py` in your terminal
   - Open `http://localhost:5000` in your web browser
   - You'll see the main control interface

2. **Connect to Robot**
   - Click the "Connect to Robot" button
   - The status indicator will turn green when connected
   - Robot information will appear in the connection panel

3. **Control the Robot**
   - Use the arrow buttons to move the robot
   - Or use keyboard shortcuts:
     - `W` or `↑` - Move Forward
     - `S` or `↓` - Move Backward
     - `A` or `←` - Turn Left
     - `D` or `→` - Turn Right
     - `Space` - Emergency Stop

### Interface Overview

#### Connection Panel
- Shows current connection status
- Displays robot position, angle, and status
- Connect/disconnect buttons

#### Robot Controls
- Large, easy-to-use movement buttons
- Emergency stop button for safety
- Visual feedback for button presses

#### Real-time Map
- Shows obstacles detected by LiDAR
- Robot position and orientation
- Grid system for distance reference
- Clear and center map options

#### LiDAR Scan
- Circular radar-style display
- Real-time point cloud visualization
- Shows obstacles around the robot
- Distance rings for scale reference

#### AR Features
- Compass showing robot heading
- Distance to nearest obstacle and path status
- Field-of-View cone on the map (toggle)
- Safety bubble around robot (adjustable)
- Breadcrumb trail (toggle + clear)
- Waypoint mode: click on the map to add waypoints, draws a dashed path
- Navigation readouts: goal distance, ETA, and next-turn hint
- Toggle AR view on/off

## Technical Details

### Architecture
- **Backend**: Python Flask with WebSocket support
- **Frontend**: Vanilla HTML, CSS, JavaScript
- **Real-time Communication**: Socket.IO
- **Graphics**: HTML5 Canvas for map and LiDAR visualization

### File Structure
```
lidar-robot-webapp/
├── app.py                 # Main Flask application
├── requirements.txt       # Python dependencies
├── README.md             # This documentation
├── templates/
│   └── index.html        # Main HTML interface
└── static/
    ├── css/
    │   └── style.css     # Styling and responsive design
    └── js/
        └── app.js        # Frontend logic and WebSocket handling
```

### API Endpoints
- `GET /` - Main application interface
- `POST /api/connect` - Connect to robot
- `POST /api/disconnect` - Disconnect from robot
- `GET /api/status` - Get robot status
- `POST /api/control/<action>` - Send movement commands

### WebSocket Events
- `lidar_update` - Real-time LiDAR data
- `position_update` - Robot position changes
- `map_update` - Map data updates
- `status_update` - General status updates

## Customization

### Adding Real Robot Support
To connect to a real LiDAR robot:

1. **Replace the RobotSimulator class** in `app.py` with your robot's communication protocol
2. **Modify the connection logic** to use your robot's IP address and port
3. **Adjust the data parsing** to match your robot's LiDAR data format

### Styling Customization
- Edit `static/css/style.css` to change colors, layouts, or animations
- The design is fully responsive and mobile-friendly
- Uses CSS Grid and Flexbox for modern layouts

### Adding Features
- **Camera Feed**: Add a video stream panel
- **Voice Control**: Integrate speech recognition
- **Path Planning**: Add autonomous navigation
- **Data Logging**: Save robot movements and maps

## Troubleshooting

### Common Issues

**Cannot connect to robot**
- Check if the Flask server is running
- Verify the correct port (default: 5000)
- Ensure WebSocket connections are allowed

**LiDAR visualization not working**
- Check browser console for errors
- Ensure Canvas is supported in your browser
- Verify LiDAR data format matches expected structure

**Controls not responding**
- Make sure robot is connected
- Check network connectivity
- Refresh the page and reconnect

### Browser Compatibility
- Chrome 90+ ✅
- Firefox 88+ ✅ 
- Safari 14+ ✅
- Edge 90+ ✅

## Development

### Running in Development Mode
```bash
# Enable Flask development mode
export FLASK_ENV=development  # Linux/Mac
set FLASK_ENV=development     # Windows

python app.py
```

### Code Structure
The application follows a simple MVC pattern:
- **Model**: Robot state and LiDAR data management
- **View**: HTML templates and CSS styling  
- **Controller**: Flask routes and WebSocket handlers

## License

This project is open source and available under the MIT License.

## Support

For questions, issues, or feature requests, please create an issue in the project repository.

---

**Made with ❤️ for easy robot control - Simple enough for anyone to understand!**