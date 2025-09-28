# List of Challenges and how we attempted to overcome them

- The room was really cold (the AC is stuck on?)
    - Erik brought a heater
- Why is the esp32 unable to do its communication to the ROS2 "launch" process?
    - debug - via `sudo tcpdump -i any -n port 8888`, to see that the esp32 is reaching the laptop (but not reaching the docker container)
    - FIX - The instructions I had for starting docker had `-p 8888:8888/udp \`, but I needed `-p $IP:8888:8888/udp \`
- Why are the `ros2 topic pub /cmd_vel` commands only 50% of the time actually moving the robot?
    - possible cause - Packets are reportedly getting lost
    - investigate/research - to figure out how nodes and topics work in ROS2
    - debug - is there a way to debug topics, to tell where it is failing? An "Explorer" UI like the one Zaid had used for mqtt topics
    - alternatives - what are `ros2 service` and `ros2 action`, and are they more reliable than `ros2 topic`?
- How can we use data that is available via ROS2 and kaiai.ai, in a AR app?
- How can we have multiple developers testing out the robot, when some have Macbook, some have Windows
    - fortunately the kaia.ai platform used a docker image, but that comes with tricky steps needed
- How can we have multiple developers testing out the robot, while we only have one robot?
    - The open source projects (kaia.ai) have simulators, but it isn't working through docker