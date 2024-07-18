Github link for UR Robot driver: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

Github link for UR Industrial packages: https://github.com/ros-industrial/universal_robot


To launch the Robot alongside with the central client

*********************************************************************
Terminal1:
roslaunch ur_robot_driver ur16e_bringup.launch robot_ip:=192.168.1.102 kinematics_config:=/home/host/my_robot_calibration.yaml # controller (control through remote)

Terminal2:
roslaunch ur16e_moveit_config move_group.launch # moveit configuration for UR16e

Terminal3:
roslaunch ur16e_moveit_config moveit_rviz.launch # rviz visualization

Terminal4:
roslaunch final_pkg sim_complete.launch # action server for manipulation, contains the central client too.

***********************************************************************

To Obtain calibration file if gripper/tcp/Centre of Gravity changed:
roslaunch ur_calibration calibration_correction.launch \
  robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
  
  
  To obtain Tool's position :
  The script from my_moveit_planner named Task_server.py can be used
