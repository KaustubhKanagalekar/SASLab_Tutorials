# Tutorials for Safe Autonomous Systems Lab (SASLAB)

In order to get familiar with ROS2, Turtlebots, and Docker containers, I had to undertake a series of tutorials.

ROS2 version: ROS Humble, Gazebo Version: 11.0.0 

## Week 1 Tutorials 
The first one was to get familiar with basic ROS2 frameworks, such as publisher/subscriber models, service/client models, and building packages. For the first exercise, I had to create a random 7-Dim state and output 6-Dim using a pub/sub model. In addition, I had to simulate a
turtlesim virtually by creating a "vacuum" like pattern (again using a pub/sub model)

## Week 2 Tutorials 
The second set of tutorials required the creation of the same vacuum simulation, however this had to be simulated in RViz. In addition, a random goal generator was also created where the simulation bot would traverse after 10 seconds. 
[![Spiral Shape using Turtlebot in RViz Sim](https://img.youtube.com/vi/MDLA3LqHRH4/0.jpg)](https://www.youtube.com/watch?v=MDLA3LqHRH4)

[![Random Goal Generator using Turtlebot in RViz Sim](https://img.youtube.com/vi/_KP4k6HO7bw/0.jpg)](https://www.youtube.com/watch?v=_KP4k6HO7bw)



## Week 3 Tutorials 
The third set of tutorials required implementing the scripts from the first two weeks onto the physical Turtlebots. The vacuum script was modified to loop only once before stopping. Additionally, I also created a script involving the Turtlebot tracing a square. 

[![Square Shape using Physical Turtlebot](https://img.youtube.com/vi/eHk-viC4IPE/0.jpg)](https://www.youtube.com/shorts/eHk-viC4IPE) 
[![Spiral Shape using Physical Turtlebot](https://img.youtube.com/vi/Ghq7X8e90cw/0.jpg)](https://www.youtube.com/shorts/Ghq7X8e90cw) 

### Commands to Reproduce Simulation and ROS2 Setup 
A Docker container was created and hosted over Chrome Remote Desktop for visualisation purposes. X-11 forwarding can also be enabled for visualisation if easier. 
If using X-11 forwarding- 
`xhost +` 
`ssh -X <name of server>` 
`bash connect_to_container.sh <name of container>` or `docker exec -it <name of container> bin.bash` 
If using Chrome Remote Desktop, then through terminal, run `cd /mounted_volume` to get into the docker container. The Chrome Remote Desktop is set based on the name of the container, hence no need to specify the name during terminal commands. 
After getting into `/mounted_volume`,
`cd ros2_ws` 
`source /opt/ros/humble/setup.bash` to activate ROS 

Creating a ROS2 package: `ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>` Add and modify code. 
Add access points and dependencies in `setup.py`, `setup.cfg`, and `package.xml`
After creating package, open a new terminal tab: go to root `/mounted_volume/ros2_ws`, then `colcon build —packages-select <package_name>` 
Go to original terminal tab: `/mounted_volume/ros2_ws/install/setup.bash` and repeat in new tab. 
In original terminal tab: `ros2 run <entry point> <package_name>`

Gazebo already came installed with the Chrome Remote Desktop creation, which is what I used for demonstration. 

####
To view and download the media behind these exercises, click here- 
👉 [Tutorials Media](https://github.com/KaustubhKanagalekar/SASLab_Tutorials/releases/latest)
