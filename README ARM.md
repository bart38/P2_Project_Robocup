# P2_Project_Robocup
Steps to follow before controlling the arm:
     1.Install the correct java version: sudo apt-get install openjdk-8-jdk
     2. Check that the version is correct: java -version
     3. If not: sudo update-alternatives --set java /usr/lib/jvm/jdk1.8.0_version/bin/java
     4.Go to  the folder ursim-3.6.0.30.512:
     5.Execute  sudo ./install.sh ,this will install the UR simulator
     6. Run sudo apt install libxmlrpc-c++8-dev:i386
     7. Unzip/ur_pcks/universal_robot
     8. Unzip /ur_pcks/ur_modern_driver
     9. Unzip /roboticsgroup/roboticsgroup_gazebo_plugins.git
     10. Run sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
     11. Execute sudo apt-get install ros-melodic-moveit
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
	
	





