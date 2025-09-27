#!/bin/bash
# Move the robot forward for 1 second

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "Error: ros2 command not found. Make sure ROS2 is installed and sourced."
    exit 1
fi

# Check if the cmd_vel topic exists
if ! ros2 topic list | grep -q "/cmd_vel"; then
    echo "Warning: /cmd_vel topic not found. Make sure the robot is running."
fi

echo "Moving robot forward for 1 second..."

# More robust approach: Wait for subscribers first, then publish
echo "Waiting for /cmd_vel subscribers..."

# Check for subscribers in a loop (max 10 seconds)
for i in {1..20}; do
    SUBSCRIBER_COUNT=$(ros2 topic info /cmd_vel | grep "Subscription count:" | grep -o '[0-9]*')
    if [ "$SUBSCRIBER_COUNT" -gt 0 ]; then
        echo "Found $SUBSCRIBER_COUNT subscriber(s), starting movement..."
        break
    fi
    if [ $i -eq 20 ]; then
        echo "Warning: No subscribers found after 10 seconds, trying anyway..."
    fi
    sleep 0.5
done

# Now publish with confidence that subscribers exist
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 5 &
PUB_PID=$!

# Brief delay to let first message send
sleep 0.1

# Move for 1 second
echo "Moving robot..."
sleep 1

# Stop the publisher
kill $PUB_PID 2>/dev/null

# Send explicit stop command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo "Robot stopped."