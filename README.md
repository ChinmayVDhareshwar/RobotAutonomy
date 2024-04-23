# RobotAutonomy
Assignment for Robot Autonomy

To build the workspace, clone it and go to ~/RobotAutonomy folder. Then preform colcon build.

Instructions to run the simulation:
1]export ROS_DOMAIN_ID=11
2]export TURTLEBOT3_MODEL=burger
3]source install/setup.bash
4]export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix my_turtlebot)/share/my_turtlebot/models
5]export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
6]ros2 launch my_turtlebot turtlebot_simulation.launch.py

Now Gazebo and Rviz2 open. You might notice Global Status:Error on RViz, this can be fixed by using 2D Pose Estimate from the top bar.


Instructions to run localization package:
1]export ROS_DOMAIN_ID=11
2]export TURTLEBOT3_MODEL=burger
3]source install/setup.bash
4]ros2 run localization odometry_node --ros-args -p use_sim_time:=True

You can then add the topic /wheel_odom in rviz to visualize it 


How to control velocity of the robot:
1]export ROS_DOMAIN_ID=11
2]export TURTLEBOT3_MODEL=burger
3]ros2 run turtlebot3_teleop teleop_keyboard


Odometry Node uses wheel odom and works fine. Enhanced Odometry on the other hand has issues during broadcasting /tf
If you want to run Enhanced Odometry the steps are same as odometry_node but replace it with fused_odom
