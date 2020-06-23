# P2_Project_Robocup
Steps to follow before controlling the arm:
1.Go to  the folder ursim-3.6.0.30.512:
	1.1Execute  sudo ./install.sh ,this will install the UR simulator
	1.2 Open starturcontrol.sh and in the second line delete yourpassword and write your computer password
	1.3 Download https://github.com/ros-industrial/universal_robot.git
	1.4 Download https://github.com/ros-industrial/ur_modern_driver.git
Steps to follow to control the arm:
	1. Go to  the folder ursim-3.6.0.30.512
	2. Execute  sudo ./starturcontrol.sh
	3. Execute  sudo ./start-ursim.sh
	4. Run roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=localhost headless_mode:=true 
	6. Run roslaunch suii_gazebo suii.launch
Inside Suii/suii_gazebo/scripts there are the following codes:
	1. Move.py: moves the robot
	2. usim_to_gaz.py: moves the arm in the simulator
	3. move1.py: send the positions of the arm in the simualtor to gazebo
	4. gripper.py: opens/closes the gripper	
	5. demo.py: moves the robot,then starts moving the arm, closes the gripper, moves the arm again and opens the gripper
	
	





