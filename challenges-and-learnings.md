# List of Challenges and Learnings

- The room was really cold (the AC is stuck on?)
    - Erik brought a heater
- Why is the esp32 unable to do its communication to the ROS2 "launch" process?
    - debug - via `sudo tcpdump -i any -n port 8888`, to see that the esp32 is reaching the laptop (but not reaching the docker container)
    - FIX - The instructions I had for starting docker had `-p 8888:8888/udp \`, but I needed `-p $IP:8888:8888/udp \`
- Why are the `ros2 topic pub /cmd_vel` commands only 50% of the time actually moving the robot?
    - possible causes - logs say that messages are getting lost, CRC error
    - investigate/research - figure out how "nodes" and "topics" work in ROS2
    - debug - is there a way to debug "topics", to tell where it is failing? An "Explorer" UI like the one Zaid had used for mqtt topics
        - Foxglove is a robotics studio, Claude thinks it would have things like that
    - alternatives - what are `ros2 service` and `ros2 action`, and are they more reliable than `ros2 topic`?
- Misunderstandings about what ROS2 does and how Kaia fits in
    - I thought ROS2 let you do high level robot actions, but it is actually for coordinating low level things
    - I thought Kaia just enabled the setup and network connection, but it is actually the thing that allows the high level commands to be orchestrated (the commands just happen to start with `ros2`, but Kaia is doing all the implementation work)
- Rishi learned how to connect the frontend to a backend (help from Copilot)
    - python + flask
    - he has previous experience with making frontends, but Copilot made it go really fast
- Running bash commands in python is messy
    - I think there are python libraries that we can use directly, but bash was good for a hackathon so we only need to learn one SDK
- How can we have multiple developers testing out the robot, when some have Macbook, some have Windows
    - fortunately the kaia.ai platform used a docker image, but that comes with tricky setup steps and makes debugging more tedious
- How can we have multiple developers testing out the robot, while we only have one robot?
    - The open source projects (kaia.ai) have simulators, but it isn't working through docker for us yet (Zaid tried a bunch with help from Copilot)
- It randomly fails to connect to WiFi (hotspot from iPhone)
    - visible symptom - blinking white light
    - debug (both options require you have to remove the LiDAR sensor)
        - view logs - connect via cable and use Arduino serial monitor 
        - switch WiFi network - restart and press and hold "boot" button within 1 second of starting, then go to local network web UI - `192.168.4.1`
    - try turning off+on the hotspot (a.k.a. restart your router), then reboot the robot
    - the ros2 launch command can sit there waiting for a connection
    - when it works, there will be noticeable output from the launch command, and the LiDAR will start spinning
- You can't reprogram the WiFi connection without taking off the LiDAR sensor
- How can we use data that is available via ROS2 and kaiai.ai, in a AR app?
    - We could put two QR codes on the robot to help anchor the 3D scene in coordinates relative to the robot
    - AR would let the robot take different visual forms, and move its arms to point to things as part of the game/activity
- What games or activities would be best for this kind of robot?
    - Cira researched what educational outcomes we can target (spatial reasoning, perseverance, creativity, sequencing, classifying)
    - We compared that against the capabilities of what the tech can do (mapping rooms, intelligent navigation)
    - Generative AI made amazing suggestions of what sorts of games lead to each learning outcomes
        - https://claude.ai/share/3f0bd4db-ccf9-47d3-a53d-13335e2c7502


## General Learnings

- It really helps to organize your planning and assumptions into documents, and then feed it into generative AI to help brainstorm (I should have done that earlier, it was really hard to explain what I was thinking)
- It was hard to debug ROS2 actions that are not working as expected
