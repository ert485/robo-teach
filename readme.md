# RoboTeach

A proposed educational robotics platform using MakersPet / Kaia.ai robots to create interactive learning experiences for children. This project aims to make robotics accessible for educational activities that improve spatial reasoning, creativity, persistence, and problem-solving skills.

NOTE - this project is in very early stages, most of the things described are planned features, not working code

## 🤖 About the Project

RoboTeach transforms learning through interactive robotics, helping children develop crucial spatial reasoning skills while making STEM education fun and accessible. Research shows that spatial ability in childhood strongly predicts later achievements in science, technology, and engineering fields - even more than verbal or math skills alone.

Our platform uses a MakersPet robot with advanced sensing capabilities to create engaging learning experiences where children don't just learn coding on a screen, but bring it to life through voice commands and physical interactions.

### Robot Hardware
- 120mm round differential drive base with wheels
- 360° 2D LiDAR sensor (LDROBOT LD14P) for mapping and navigation 
- ESP32 microcontroller
- Docker-based ROS2 environment for cross-platform compatibility

## 🎯 Educational Goals

RoboTeach strengthens critical learning skills through research-backed, hands-on activities:

- **Spatial Reasoning**: Children develop laterality awareness (left vs. right, orientation, movement in space) essential for coordination, reading, and writing. As they guide the robot to "turn left" or "draw a square," they build spatial skills that boost both cognitive and academic growth.

- **Perseverance & Problem-Solving**: Multi-step challenges teach children to break big problems into smaller steps, celebrate "productive failure," and learn that mistakes are learning opportunities.

- **Creativity & Art**: Children become "conductors" directing the robot to create floor art, design dance routines, and build collaborative stories through movement and visual narratives.

- **Sequencing & Classification**: Activities like "Story Sequencer" help children understand cause-and-effect relationships, categorization, and logical thinking patterns.

### The Magic for Children
Kids don't see "robotics" or "programming" - they see a helpful, intelligent companion that makes learning feel like play. The robot's LiDAR becomes "magic eyes" that can see around corners, while its wheels become "adventure legs" for learning journeys.

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
   bash backend/move-forward.sh
   ```

### Additional Terminal Access
To run multiple terminals in the same Docker container:
```bash
docker exec -it makerspet bash
```

## How the Technology Enables Learning

### ROS2 + Kaia.ai: Making Complex Robotics Simple

ROS2 is a general-use robotics platform that handles low-level robot communication. Kaia.ai builds on top of ROS2 to implement high-level educational tasks like:

- **SLAM Mapping**: "Robot, explore this room and build a map" - the system handles all the technical complexity while children see a growing map appearing on screen
- **Autonomous Navigation**: "Go to the reading corner" becomes possible because Kaia.ai converts concepts into precise coordinates and path planning
- **Real-time Obstacle Avoidance**: The robot safely navigates around children and furniture using continuous 360° LiDAR scanning
- **Educational Abstractions**: Instead of children learning robotics, robotics becomes invisible so they can focus on math, science, and problem-solving

The LiDAR transforms abstract spatial concepts into concrete, visual experiences - children can literally see how the robot "sees" space, understand distance relationships, and experiment with geometric problem-solving in their real classroom environment.

### For Teachers

- **Curriculum Integration**: Activities align with Math, Science, and Language Arts standards
- **Assessment Tools**: The system tracks children's problem-solving approaches and collaboration skills
- **Differentiated Learning**: Activities automatically adjust difficulty based on individual student progress
- **No Programming Required**: Teachers can focus on educational outcomes while the platform handles technical complexity

## 📂 Repository Structure

```
├── readme.md                           # This file
├── setup-steps.md                      # Detailed setup instructions
├── challenges-and-learnings.md         # Development issues and solutions  
├── backend/                            # Backend server and robot control scripts
│   └── move-forward.sh                 # Basic robot movement script
├── frontend/                           # Frontend web interface (planned)
├── ideation/                           # Project planning and brainstorming
│   ├── proposal.md                     # Original hackathon proposal
│   ├── design-decisions.md             # Educational philosophy and technical approach
│   ├── imagine-the-final-product.md    # Vision for educational activities
│   ├── imagine-the-final-product-claude-output.md # AI-generated activity ideas
│   └── working-convo.md                # Development discussions
├── pitch/                              # Presentation materials
│   └── RoboTeach Co-Hack.pptx          # Hackathon presentation
└── whiteboard-content/                 # Planning photos and diagrams
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

### Spatial Reasoning Games

**"Treasure Map Navigator"**
- Children create maps by having the robot explore and scan rooms with its LiDAR
- They guide friends through obstacle courses using only verbal directions  
- The robot shows a top-down view of the space it has mapped
- Kids learn concepts like "rotate 90 degrees," "distance," and "perspective"

**"Shape Detective"**
- Robot navigates to different stations around the classroom to "collect" geometric shapes
- Children predict the shortest path and test their hypotheses
- They build 3D structures with blocks while the robot provides different viewpoint perspectives

### Creative Expression

**"Moving Murals"**
- Robot carries different colored markers and creates large floor art based on children's designs
- Kids can "conduct" the robot like an orchestra, creating flowing, dynamic patterns
- Collaborative storytelling where the robot's path creates visual narratives

### Problem-Solving Challenges

**"Mission Impossible"**
- Multi-step challenges where the robot must navigate increasingly complex mazes
- If the robot gets stuck, children problem-solve together: "What went wrong? How can we fix it?"
- Progress tracking shows improvement over time

**"Delivery Service"**
- Robot must deliver classroom supplies to different students
- Routes get blocked, requiring creative re-planning
- Children learn to break big problems into smaller steps

### Example Learning Interaction

*Teaching geometric shapes to 5-year-olds:*

**Child:** "How can I draw a square?"

**Robot:** "Good question! First, move forward 5 steps."

*(Child moves, and the robot follows)*

**Robot:** "Now, you have to turn right."

*(The Robot will complete the exercise with the kid until completing the square.)*

**Robot:** "Great job!"

The robot uses voice recognition so even very young learners can participate without typing, building confidence while developing spatial awareness.

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
Frontend ↔ HTTP/WebSocket ↔ Control Server ↔ ROS2/Docker Container ↔ Robot
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
