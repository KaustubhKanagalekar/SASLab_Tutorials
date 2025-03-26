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
#### Using X-11 Forwarding:
1. Run `xhost +` to allow X11 connections.
2. SSH into the server using the command:
   - `ssh -X <name of server>`
3. Connect to the Docker container:
   - Either: 
     - `bash connect_to_container.sh <name of container>`
   - Or: 
     - `docker exec -it <name of container> /bin/bash`

#### Using Chrome Remote Desktop:
1. Access the Docker container from the terminal by running:
   - `cd /mounted_volume`
2. Since the Chrome Remote Desktop is set based on the container's name, no need to specify the name during terminal commands.
3. Once in `/mounted_volume`, navigate to:
   - `cd ros2_ws`
4. Activate ROS 2 by sourcing the setup file:
   - `source /opt/ros/humble/setup.bash`

#### Creating a ROS 2 Package:
1. Create the package with the following command:
   - `ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>`
2. Modify and add code to the package as needed.
3. Add dependencies and access points in the following files:
   - `setup.py`
   - `setup.cfg`
   - `package.xml`

#### Building the Package:
1. Open a new terminal tab and navigate to the ROS workspace directory:
   - `cd /mounted_volume/ros2_ws`
2. Build the package using `colcon`:
   - `colcon build --packages-select <package_name>`
3. In the original terminal tab, source the ROS workspace setup file:
   - `source /mounted_volume/ros2_ws/install/setup.bash`
4. In the original terminal tab, run the package with:
   - `ros2 run <entry_point> <package_name>`

#### Additional Information:
- **Gazebo** is already installed with Chrome Remote Desktop for visualization purposes.
####
To view and download the media behind these exercises, click here- 
👉 [Tutorials Media](https://github.com/KaustubhKanagalekar/SASLab_Tutorials/releases/latest)
