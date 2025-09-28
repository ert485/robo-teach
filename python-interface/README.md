# 🤖 RoboTech Web Controller

A simple and easy-to-use web application for controlling LiDAR robots with real-time visualization and AR features.

## Features

### 🔌 Connection Management

### 🎮 Robot Control

### 🗺️ Real-time Mapping

### 📡 LiDAR Visualization

### 🔮 AR Features

## Installation

### Prerequisites

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

#### Robot Controls

#### Real-time Map

#### LiDAR Scan

#### AR Features

## Technical Details

### Architecture
- **Backend**: Python Flask with WebSocket support
- **Frontend**: Vanilla HTML, CSS, JavaScript
lidar-robot-webapp/
├── app.py                 # Main Flask application
├── requirements.txt       # Python dependencies
├── README.md             # This documentation
├── templates/
│   └── index.html        # Main HTML interface
└── static/
        └── app.js        # Frontend logic and WebSocket handling
```

### API Endpoints
- `GET /` - Main application interface
- `POST /api/connect` - Connect to robot

### WebSocket Events
- `status_update` - General status updates

To connect to a real LiDAR robot:

1. **Replace the RobotSimulator class** in `app.py` with your robot's communication protocol
2. **Modify the connection logic** to use your robot's IP address and port
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

**LiDAR visualization not working**
- Check browser console for errors
- Ensure Canvas is supported in your browser
- Verify LiDAR data format matches expected structure

**Controls not responding**
- Make sure robot is connected
- Check network connectivity

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