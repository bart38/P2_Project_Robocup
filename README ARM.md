# P2_Project_Robocup
Steps to follow before controlling the arm:
1.Go to  the folder ursim-3.6.0.30.512:
	1.1Execute  sudo ./install.sh ,this will install the UR simulator
	1.2Open starturcontrol.sh and in the second line delete yourpassword and write your computer password
2.Create a workspace
	2.1 Follow http://wiki.ros.org/catkin/Tutorials/create_a_workspace
	2.2 Copy everything inside the catkin folder inside your workspace  src folder
	2.3 Execute catkin_make 
        
Steps to follow to control the arm:
	1.Go to  the folder ursim-3.6.0.30.512
	2.Execute  sudo ./starturcontrol.sh
	3.Execute  sudo ./start-ursim.sh
	4. In another terminal  write :source "/addres to your workspace/devel/setup.sh"(instead of addres to your workspace, add the adress to your workspace )
	5. Run roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=localhost headless_mode:=true 
	6. In another terminal do 4. and run roslaunch suii_gazebo suii.launch
	7.Run /adress to your workspace/src/Suii/suii_gazebo/scripts/usim_to_gaz.py in order to connect the UR simulator to gazebo 
	8..Run /adress to your workspace/src/Suii/suii_gazebo/scripts/move1.py if you want to send URScript command to the arm
	





