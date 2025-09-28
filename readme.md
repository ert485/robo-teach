# RoboTeach

A proposed educational robotics platform using MakersPet / Kaia.ai robots to create interactive learning experiences for children. This project aims to make robotics accessible for educational activities that improve spatial reasoning, creativity, persistence, and problem-solving skills.

NOTE - this project is in very early stages, most of the things described are planned features, not working code

## 🤖 About the Project

This repository contains the setup, scripts, and documentation for controlling a MakersPet robot through ROS2 / Kaia.ai platform. The robot features:
- 120mm round differential drive base
- 360° 2D LiDAR sensor for mapping and navigation
- ESP32 development board
- Docker-based ROS2 environment for cross-platform compatibility

## 🎯 Educational Goals

The platform targets key learning outcomes:
- **Spatial Reasoning**: Understanding robot movement and navigation
- **Perseverance**: Debugging and problem-solving with robotics
- **Creativity**: Designing custom robot behaviors and activities  
- **Sequencing**: Programming logical command sequences

## See Also

We also have a Google Drive folder, [here](https://drive.google.com/drive/u/0/folders/1A6fvU7hGU-ppixGU_70DXUyl0i8Rt296)

## 🚀 Getting Started

To initially program a robot, you will need to follow the instructions provided by Makerspet and Kaia.ai. 

If you have a robot that is already assembled and programmed, complete the steps provided by Makerspet to connect the robot to the WiFi network that you will be using.

If you robot is already connected to a WiFi network, you can start here:

### Prerequisites
- macOS, Windows, or Linux
- Docker installed
- XQuartz (for macOS visualization)

### Setup

1. **Set your machine's IP address:**
   ```bash
   export IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
   ```

2. **Configure X11 for visualization (macOS specific instructions):**
   - Start XQuartz
   - Add your IP to the access control list:
     ```bash
     /opt/X11/bin/xhost + $IP
     ```

3. **Launch the Docker container:**
   ```bash
   docker run --platform linux/amd64 --name makerspet -it --rm -v ~/maps:/maps \
     -p $IP:8888:8888/udp \
     -p 4430:4430/tcp \
     -e DISPLAY=$IP:0 -e LIBGL_ALWAYS_INDIRECT=0 \
     kaiaai/kaiaai:iron
   ```

4. **Initialize robot communication:**
   ```bash
   ros2 launch kaiaai_bringup physical.launch.py
   ```

5. **Test basic movement:**
   ```bash
   bash move-forward.sh
   ```

### Additional Terminal Access
To run multiple terminals in the same Docker container:
```bash
docker exec -it makerspet bash
```

## 📂 Repository Structure

```
├── readme.md                    # This file
├── setup-steps.md              # Detailed setup instructions
├── challenges-and-learnings.md # Development issues and solutions
├── move-forward.sh             # Basic robot movement script
├── ideation/                   # Project planning and brainstorming
│   ├── proposal.md             # Original hackathon proposal
│   ├── imagine-the-final-product.md # Vision for educational activities
│   └── working-convo.md        # Development discussions
└── whiteboard-content/         # Planning photos and diagrams
    ├── IMG_3745.jpeg
    ├── IMG_3746.jpeg
    └── IMG_3747.jpeg
```

## 🛠 Common Commands

### Robot Control
```bash
# Manual keyboard control
ros2 run kaiaai_teleop teleop_keyboard

# Basic forward movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Stop the robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Debugging
```bash
# Check available topics
ros2 topic list

# Monitor robot communication
sudo tcpdump -i any -n port 8888

# View topic information
ros2 topic info /cmd_vel
```

## 🔧 Troubleshooting

### Known Limitations

- Move Command only works 50% of the time - see `challenges-and-learnings.md` 

### Common Issues

**ESP32 not connecting to laptop:**
- Check network connectivity with `tcpdump`

**WiFi connection issues:**
- Connect via USB cable and use Arduino Serial Monitor
- May need to remove LiDAR sensor to access USB port

**Cross-platform development:**
- Use the provided Docker image for consistent environment
- Set up proper IP forwarding for your operating system

See `challenges-and-learnings.md` for detailed troubleshooting information.

## 🎮 Educational Activities

This platform has a vision to enable various educational activities:
- **Treasure Hunt**: Robot navigates to find hidden objects using SLAM
- **Art Creator**: Children program movement patterns to create drawings
- **Maze Runner**: Problem-solving through obstacle navigation
- **Pet Follower**: Robot learns to follow and interact with children

## 🚧 Planned Architecture (In Development)

### Frontend Game Interface
A child-friendly web interface that will allow students to:
- Select and configure educational games/activities
- Monitor robot status and sensor data in real-time
- Manually control robot movement
- View SLAM mapping and navigation progress
- Track learning progress and achievements

### Robot Control Server
A middleware server that bridges the frontend interface with ROS2/Kaia.ai commands:

**Core Features (Planned):**
- **API Endpoints**: RESTful API for robot control actions
- **Real-time Communication**: WebSocket server for live robot status updates
- **Game Logic**: Server-side game state management and rules enforcement

**Technical Architecture:**
```
Frontend (React) ↔ WebSocket/HTTP ↔ Control Server ↔ ROS2/Docker Container ↔ Robot
```

**Planned API Endpoints:**
```
POST /api/robot/move          # Send movement commands
GET  /api/robot/status        # Get current robot state
GET  /api/robot/map           # Get SLAM mapping data
POST /api/game/start          # Initialize game session
GET  /api/game/progress       # Get current game state
POST /api/emergency-stop      # Immediate robot halt
```

**Current Status:**
- ✅ Basic ROS2 robot communication established
- 🚧 Server architecture in design phase
- 🚧 Frontend mockups and wireframes in progress
- ⏳ WebSocket real-time communication planned
- ⏳ Game-specific APIs to be implemented

This architecture will abstract the complexity of ROS2 commands into simple, educational game interfaces that children can easily understand and interact with.

## 🤝 Contributing

This project welcomes contributions! Areas where help is needed:
- Implementing the planned architecture (see earlier section)
- Educational activity development
- Setup instruction improvements
- Robot simulation for development
- Documentation and tutorials

## 📚 Additional Resources

- [Kaia.ai Platform](https://kaia.ai) - Robot control framework
- [ROS2 Documentation](https://docs.ros.org/en/iron/) - Robot Operating System
- [MakersPet Hardware](https://makerspet.com) - Robot kit information
