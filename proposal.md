I'm participating in a hackathon today. And I’m trying to figure out what my goals should be for the event. I've been some hackathons before -- most of them have been more business pitch style hackathons, but this one is a bit more "developer" technical focused.
I've been wanting to get into robotics software control, and SLAM. I bought this robot kit from makerspet:

> Learn professional robotics basics hands-on by building a little Arduino and ROS2-compatible robot. Makers Pet Mini robot consists of a 120mm round base, two N20 motors (differential drive), a 360° scanning 2D LiDAR distance sensor, a motor driver board and an ESP32 development board. The Mini robot comes with step-by-step "from-scratch" assembly, setup, bring-up, operation instruction videos and troubleshooting instructions.I chose that robot because it is the cheapest open source one I could find.  I think is a good limitation, because it will allow anything we build to be replicable and scalable. Powerful robotics is recently becoming more accessible, and I want to be on the cutting edge of that.

I assembled the robot beforehand (to make sure it actually has a good chance at working), but the robot esp32 isn't quite connecting to the laptop that it needs for localization computing.

I want to propose a specific use for the robot (incorporating a 360 camera and creating fully visual 3D reconstructions), but also just say to the team we end up forming: "let's figure out what we want to do with this robot"

Before the event starts, lets get lists for :
"Skills Needed / Things We'll Learn" split into "core skills" (ones regardless of a goal/application we choose), vs ones that could potentially be incorporated.
"Potential Goals" - this could include real life applications or things like just "make it do cool shit", or "make some part of this easier for the open source community", or "understand what the ROS2 platform is capable of”.
I want to make these lists as simple as possible.


Robotics Hackathon Project
Skills Needed / Things We'll Learn
Core Skills (regardless of application):
* ROS2 basics and node communication
* LiDAR data processing and visualization
* SLAM fundamentals (mapping + localization)
* Wireless communication troubleshooting
Potential Additional Skills:
* Computer vision and image processing
* 3D point cloud manipulation
* Real-time sensor fusion
* Web interfaces for robot control
* AI for navigation
* Multi-robot coordination
Potential Goals
Technical Learning Goals:
* Get the robot actually working and mapping spaces
* Build a web dashboard to visualize SLAM data
* Create autonomous navigation between waypoints
* Implement obstacle avoidance algorithms
Real-World Applications:
* Indoor mapping for accessibility (wheelchair routes, etc.)
* Pet/small animal monitoring system
* Educational robotics platform for schools
* Home security patrol robot
* Warehouse inventory assistant
Community/Open Source Goals:
* Create beginner-friendly ROS2 tutorials using this hardware
* Build reusable SLAM visualization tools
* Document common ESP32-ROS2 connection issues and fixes
* Run a "robot simulator" that matches the physical hardware
"Cool Shit" Goals:
* Robot that follows you around like a pet
* Collaborative mapping with multiple robots
* Voice-controlled navigation
* Robot that can "remember" and revisit interesting locations


---
We ended up deciding to do this one from the list: "Educational robotics platform for schools"
