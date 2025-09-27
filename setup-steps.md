## Setup steps

Set the IP address of the machine that is running the docker container
```
export IP=172.20.10.11
```

For x11 visualizations:
Make sure xQuartz is running
add your IP address to the access control list
`/opt/X11/bin/xhost + $IP`

Start the docker container:
```
docker run --platform linux/amd64 --name makerspet -it --rm -v ~/maps:/maps \
  -p $IP:8888:8888/udp \
  -p 4430:4430/tcp \
  -e DISPLAY=$IP:0 -e LIBGL_ALWAYS_INDIRECT=0 \
  kaiaai/kaiaai:iron
```

This will let you run more terminals in the same docker container:
`docker exec -it makerspet bash`

Initialize the bot communication:
`ros2 launch kaiaai_bringup physical.launch.py`

`ros2 run kaiaai_teleop teleop_keyboard`

`bash move-forward.sh`