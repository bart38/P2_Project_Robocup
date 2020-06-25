# Suii Sensors
The sensors on Suii are modified to have the same specs as the real sensors.

- Lidar: Hokuyo UST-10LX
  - Detection range: 0.06 - 30m
  - Scan angle: 270°
  - Measurement steps: 1080
  - Scan speed: 25 ms (40 times/sec)

- Camera: functionality of the BlasterX Senz3D
  - 2D camera: 30 frames @ 1920x1080 - fov 77°
  - 3D Camera: 60 frames @ 640x480 - fov 85°

- Extra sensor: Sonar

### Implementation
The sensors are made and controlled in gazebo (specifically in suii_gazebo.xacro). They each use their own sensor plugins. The plugins used are as follows.

Sensor    | Sensor Type | Sensor Plugin
------    | ----------- | ------
Lidar     | gpu_ray     | libgazebo_ros_gpu_laser
Camera_2D | camera      | libgazebo_ros_camera
Camera_3D | depth       | libgazebo_ros_openni_kinect
Sonar     | ray         | libgazebo_ros_range

### Sensor Topics & Messages
The sensors send their data to their specific ros topics. The rostopics that are used with the message data types are as follows:

Sensor | Topic | Msg
------ | ----- | ---
Lidar_Front | /suii/laser/scan_front  | sensor_msgs/LaserScan
Lidar_Back  | /suii/laser/scan_back   | sensor_msgs/LaserScan
Camera_2D   | /camera_2D/image_raw    | sensor_msgs/Image
Camera_3D   | /camera_3D/depth/points | sensor_msgs/PointCloud2
Sonar       | /suii/sonar/scan        | sensor_msgs/Range

_There are more topics that are being made by the sensors but aren't discussed here. They can be found, if the simulation is running, with the folliwing command._
```
rostopic list
```

### Some scripts to test the sensors:
- Move_And_Collision.py: move straight until the lidars detect a collision, then change course and stop when another collision has been detected.
- Vision_Test.py: Shows the image of the 2D camera.
