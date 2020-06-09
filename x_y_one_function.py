import rospy
from geometry_msgs.msg._Twist import Twist # makes sure i can send data to the turtle sim
from turtlesim.msg._Pose import Pose #makes sure i can read the location data from the turtle sim
from turtlesim.srv._SetPen import SetPen
from turtlesim.srv._SetPen import SetPenRequest
from turtlesim.srv._SetPen import SetPenResponse

import time
import math

y = 0
x = 0
z = 0
theta = 0

xl = 0 
xr = 0
yu = 0
yd = 0

r = 0
g = 0
b = 0
width = 5
off = False

cw = True

coor_list = []
list_count = 0

def sim_location(location_data):
    global x
    global y
    global theta
    x = location_data.x   
    y = location_data.y         # x-coordinate from 0 to 11.08889122
    theta = location_data.theta
    
def rectangle(meters):
    global xl, xr #x_left x_right
    global yu, yd #y_up y_down
    global coor_list #coordinates list
    
    start_x = 5.544444561
    start_y = 5.544444561
    
    xr = start_x + (meters / 2)
    xl = start_x - (meters / 2)
    yu = start_y + (meters / 2)
    yd = start_y - (meters / 2)
    
    coor_list.append(xr)
    coor_list.append(yu)
    coor_list.append(xl)
    coor_list.append(yu)
    coor_list.append(xl)
    coor_list.append(yd)
    coor_list.append(xr)
    coor_list.append(yd)
    
    print(coor_list)
    
def directions():
    global x,y
    global coor_list
    global theta
    global list_count
    
    if cw == True:
        next_x = coor_list[list_count]
        next_y = coor_list[list_count + 1]
        list_count = list_count + 2
    else:
        next_x = coor_list[list_count]
        next_y = coor_list[list_count + 1]
        list_count = list_count - 2
        
    difference_x = round(next_x - x, 4)
    difference_y = round(next_y - y, 4)
    if difference_x == 0: difference_x = 0.0001
    angle = math.atan2(difference_y, difference_x)
    # print("angle {}".format(angle))
    # print("theta {}".format(theta))
    
    pythagoras_distance = math.sqrt(((difference_x ** 2) + difference_y ** 2))   
    time.sleep(2) 
    rotate(angle)
    move(pythagoras_distance)
    
    
def rotate(angle):
    global z
    global theta
    
    velocity_message = Twist()
    while not round(angle, 2) == round(theta, 2):
        velocity_message.angular.z = 1
        sim_movement_publisher.publish(velocity_message) 
    
    velocity_message.angular.z = 0
    sim_movement_publisher.publish(velocity_message)
  
def move(pythagoras):
    global x, y, z 
    global theta
    
    velocity_message = Twist()
    start_x = x
    start_y = y
    time.sleep(1)
    pythagoras_distance_moved = math.sqrt((((start_x - x) ** 2) + (start_y - y) ** 2))   
     
    while pythagoras_distance_moved <= pythagoras:
        pythagoras_distance_moved = math.sqrt((((start_x - x) ** 2) + (start_y - y) ** 2)) 
        velocity_message.linear.x = 1
        sim_movement_publisher.publish(velocity_message)    
    
    velocity_message.linear.x = 0
    sim_movement_publisher.publish(velocity_message)
    
if __name__ == '__main__':
    
    try:
        rospy.init_node('move_pose', anonymous=True)
        sim_movement_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist,  queue_size=10)
        sim_locatation_subscriber = rospy.Subscriber('/turtle1/pose', Pose, sim_location)
        sim_pen_service = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        time.sleep(1)
        
        # lenght = raw_input()
        
        sim_pen_service(255, 255, 255, 3, 0)
        lenght_int = float(input('side_lenght: '))
        cw = bool(input('True or False: '))
        if cw == True: cw = True 
        else: 
            cw = False
            list_count = 6
        count = 0
        while count < 5:
            if count == 0: 
                sim_pen_service(255, 255, 255, 3, 1)
            else: sim_pen_service(255, 255, 255, 3, 0)
            rectangle(lenght_int)
            directions()
            count = count + 1
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")        