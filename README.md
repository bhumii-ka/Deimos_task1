# Deimos Software Trials - Task 1
This repository contains all the packages that were used to do task 1 i.e., launching bookstore world in Gazebo, creating a map of the world and performing autonomous navigation of turtlebot3 robot using ROS.
## How to use this repo
### Prerequisites :
- #### Install Gazebo
  We will be launching our world in Gazebo so make sure to install it by using the command 
  ```
  curl -sSL http://get.gazebosim.org | sh
  ```
- #### Install ROS dependencies
  Used in integrating Gazebo to ROS
  ```
  sudo apt-get install ros-noetic-gazebo-ros-pkgs
  ```
  Used for describing the properties and parts of the robot model
  ```
  sudo apt-get install ros-noetic-urdf ros-noetic-xacro
  ```
  Used in mapping the world, visualising it and saving and launching the map
  ```
  sudo apt-get install ros-noetic-slam-gmapping ros-noetic-map-server ros-noetic-rviz
  ```
  Used for navigation
  ```
  sudo apt-get install ros-noetic-amcl ros-noetic-move-base ros-noetic-navigation
  ```
> [!NOTE]
> All the installation commands are for rosdep noetic change noetic with <your_distro_name>
- #### Install ROS packages
  Make a workspace and create a directory 'src' where all the turtlebot3 packages will be stored, clone this repo to get the packages and then build the catkin workspace.
  > This repo containes the packages cloned from the repos [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3.git) and [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git).
  
  ```
  $ cd ~/task1_ws/src/
  $ git clone https://github.com/bhumii-ka/Deimos_task1.git
  $ cd ~/task1_ws && catkin_make
  ```
  Source your workspace in .bashrc file by running the following command so that you don't have to source it in every terminal
  ```
  echo "source ~/task1_ws/devel/setup.bash" >> ~/.bashrc
  ```
### Launching Bookstore World in Gazebo
Inside the turtlebot3_simulations/turtlebot3_gazebo there are models, .world file and the .launch file saved of the bookstore world (from [Worlds](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps) repo) which are necessary for launching the world in Gazebo. <br>

- Execute the following command and choose a turtlebot model from burger, waffle or waffle_pi to be launched in the world.
  ```
  $ export TURTLEBOT3_MODEL=waffle_pi
  $ roslaunch turtlebot3_gazebo bookstore.launch
  ```

- The turtlebot can be tele-operated by running the following command in a new terminal
  ```
  $ export TURTLEBOT3_MODEL=waffle_pi
  $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
  ```
### Launching any custom world
  You can launch any custom world in Gazebo by following these steps
  - Save models and world files<br>
    Every world is just an arrangement of different models so for launching a custom world copy all the models used and paste in inside `turtlebot3_simulations/turtlebot3_gazebo/models` and paste the world file in `turtlebot3_simulations/turtlebot3_gazebo/worlds` directory and remove or comment the lines in world file that include robot in the world because we will be spawning the bot separately through launch file. For example
    ```
    <!-- <include>
      <pose>-2.3 6.2 0.0 0.0 0.0 -1.72</pose>
      <uri>model://turtlebot3_waffle_pi</uri>
    </include> -->
    ```
    
  - Writing a launch file<br>
    A launch file of a gazebo world should include the following things
      - Exporting any of the three turtlebots and giving the arguments for its position to be spawned wrt x,y and z axes. The position should be such that the robot doesn't not spawn into an obstacle.
        ```
        <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
        <arg name="x_pos" default="-2.3"/>
        <arg name="y_pos" default="6.2"/>
        <arg name="z_pos" default="0.0"/>
        ```
      - Launching the world file and setting necessary parameters
        ```
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/<name_of_world>.world"/>
        <arg name="paused" value="false"/>
        <arg name="use_sim_time" value="true"/>
        <arg name="gui" value="true"/>
        <arg name="headless" value="false"/>
        <arg name="debug" value="false"/>
        </include>
        ```
        
      - Calling xarco and urdf files to get the description of the exported robot and spawning it at the position specified earlier
        ```
        <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
        <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model turtlebot3 -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
        ```
      All the contents of the launch file must be enclosed within \<launch> and <\/launch\> tags and file must be saved in ```turtlebot3_simulations/turtlebot3_gazebo/launch``` directory
  - Running the world  <br>
    You can launch the custom world by the commands given [above](https://github.com/bhumii-ka/Deimos_task1?tab=readme-ov-file#launching-bookstore-world-in-gazebo)
    

### Mapping the world
The mapping is done using turtlebot3_slam package that allows Simultaneous Localisation And Mapping (SLAM) i.e., both estimating the position of the robot and creating the map in the real-time using sensor data

- Run the following command in the new terminal to start mapping
  > Gazebo and tele-op command should be running
  
  ```
  $ export TURTLEBOT3_MODEL=waffle_pi
  $ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
  ```
  This command opens Rviz that is used to visualize the map and then we can move the robot using ```tele-op``` command to create a map of the whole world
  
- Save the map by running the following command in a new terminal
  
  ```
  rosrun map_server map_saver -f ~/bookstore_map
  ```
  This command will save the map in the home directory
  > ![main_large](https://github.com/bhumii-ka/Deimos_task1/assets/151030407/b59dd978-a916-472c-b608-9eb73352b7f2)<br>
  > This is the map of bookstore world available in the repo that is made by SLAM and is used in navigation

> [!Caution]
> Don't close Gazebo or Rviz without saving Map. Closing Gazebo will terminate roscore.


### Autonomous Navigation
After saving the map you can terminate turtlebot3_slam and start navigation using that map with the help of the Rviz which allows you to set initial and target positions in terms of x,y and rotation of the robot and autonomously navigate it with the help of path planning algorithms and local and global map
<br>Make sure Gazebo is running.
- Run the following command in a new terminal to load your saved map in Rviz
  ```
  $ export TURTLEBOT3_MODEL=waffle_pi
  $ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=<path to .yaml file>/bookstore _map.yaml
  ```
  If you don't specify map_file parameter it will open the default map saved in turtlebot3_navigation/maps directory.
  > In this repo the default map contains the map of the whole bookstore world (shown above)
  
- Initial Position Estimation
  
  - The turtlebot3_navigation.launch file launches amcl file which is an adaptive file that runs multiple scans and estimates the current location of the robot (probable positions are shown as green dots).
  - Click the ```2D Pose Estimate``` in Rviz to roughly provide the position of the robot on the map.
  - We can then run tele-op the turtlebot so that there would more sensor data to get a more probable position of the robot can be determined (see the accumulation of the green dots in one place).
    
    ![image](https://github.com/bhumii-ka/Deimos_task1/assets/151030407/d141437a-d704-4c6e-a51d-b2a03eb60654) ![image](https://github.com/bhumii-ka/Deimos_task1/assets/151030407/5f23d693-ccb1-4177-bc62-8f5236424aa8)




- Target Position
  
  - Once you get a good estimation of the initial position you can then `terminate tele-op` command and set a target position for the robot to reach using ```2D Nav Goal``` button in Rviz.
  - The path planning algorithm then makes a path for the robot, to follow, to reach the goal without hitting obstacles, which also gets updated as the robot moves further.
    ![image](https://github.com/bhumii-ka/Deimos_task1/assets/151030407/5920dd67-1152-4d67-98f0-03028ec665d3)

  - Once the robot reaches the target it stops and shows 'Goal Achieved' in the terminal.
  
## Links 
[Click here](https://youtu.be/bKoAFWC1rpA) to watch the full video of Mapping and Navigation.
