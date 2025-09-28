## How RoboTeach supports children learning?

We want to make the biggest impact by helping children strengthen spatial reasoning skills, which research shows are critical for long-term success in STEM. Studies have found that spatial ability in childhood strongly predicts later achievements in science, technology, and engineering fields, even more than verbal or math skills alone. These skills are not fixed; kids as young as 5 to 7 can improve them through play, building, and activities that involve shape recognition and movement. By using RoboTeach, children don’t just learn coding on a screen; they bring it to life through commands and physical actions, reinforcing direction, shape, and distance in a way that boosts both cognitive and academic growth.

We also want to make the biggest impact on developing laterality awareness (left vs. right, orientation, movement in space), which is essential for coordination, reading, and writing. Research shows that when kids physically execute movements or give commands to represent transformations, their spatial reasoning grows faster. RoboTeach makes this natural and fun by using voice recognition, so even very young learners can code without typing. As children guide the robot to turn left, right, or draw shapes, they build laterality skills while gaining confidence and excitement. This approach not only supports children’s learning but also makes it easier for teachers and parents to provide meaningful, developmentally aligned STEM activities at home or in school.

## Example of RoboTeach interacting with kids:

Imagine a teacher is trying to explain the concept of geometric shapes, like a square, to 5 year old kids. RoboTeach will make learning more fun with interactive activities using voice recognition.  Our product uses a robot that has wheels and a 2D 360 degree LiDAR.  ROS2 commands behind the scenes are controlling the robot, but the children can interact with it in a way that makes sense for them.  

This is an example on how it would teach kids to draw a square:

Kid: “How can I draw a square?”
Robot: “Good question!  First, move forward 5 steps.”
Kid: (The robot will follow the kid)
Robot: “Now, you have to turn right.”
Kid: (If the kid is not doing well, RoboTeach will show how to do it). 
Robot:  "Let's go this way”
Robot: “Then, move forward 5 steps”... 
The Robot will complete the exercise with the kid until completing the square. 
At the end, it will congratulate the kid for completing the task.
Robot:  “Great job!”

## What children will see and experience?

The Robot Persona: Kids can name their robot like: “Pixel”.  It will interact with children using a screen, showing gestures, and using voice when responding to commands.
Simple Interface:  Friendly interface for teachers and kids using voice recognition and intuitive interaction.

## How will teachers use it?

Curriculum Integration: Activities aligned with Math, Science, and Language Arts.  Teachers can do fun activities to teach basic concepts such as “Treasure Map” (finding a treasure by following or giving directions to the robot), “Shape Detective” (navigating to different stations around the classroom to find and count geometric shapes), “Story Sequencer” (the robot will go to different pictures in the classroom and the students will tell the story).
Assessment tool: The system allows teachers to evaluate problem-solving skills both, alone or in groups. 
Differentiated learning:  Activities can be adjusted based on the progress of children.

## The magic for children:

Kids don't see "robotics" or "programming" - they see a helpful, intelligent companion that makes learning feel like play. The robot responds to their creativity, celebrates their discoveries, and turns abstract concepts into concrete, hands-on experiences they can see and touch.

The LiDAR becomes their "magic eyes" that can see around corners, the wheels become their "adventure legs" that take them on learning journeys, and the whole experience feels less like technology and more like having a very smart pet that loves to learn alongside them.


More ideas of games/activities:
https://claude.ai/share/3f0bd4db-ccf9-47d3-a53d-13335e2c7502

## The tech behind our design

- ESP32 Microcontroller
- LDROBOT LD14P (360° triangulation LiDAR sensor)
- Kaia.ai robotics platform which utilizes ROS2
    - ROS2 is a platform for orchestrating low level robot i/o
    - Kaia uses ROS2 constructs to implement high level tasks like “make a map of this room” and “find a path to the opposite corner of the room”
    - We’re using the docker image that Kaia provides, to simulate and control the robot
- Web app (HTML and JS) that sends commands to a server running on the same docker container that is communicating with the robot
- N8N workflow that can send requests to the docker container that is communicating with the robot
- Python server that accepts HTTP requests from the Web app / N8N, that turns UI interactions into Kaia commands
