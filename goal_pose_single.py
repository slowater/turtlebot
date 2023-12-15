#!/usr/bin/env python3

import rospy
import math
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

rospy.init_node('goal_pose')
# goal_pose = rospy.wait_for_message('/id101/aruco_single/pose', PoseStamped)



# def check_goal_reached(init_pose, goal_pose, bias):
#     if(init_pose.pose.position.x > goal_pose.pose.position.x - bias and init_pose.pose.position.x < goal_pose.pose.position.x + bias\
#         and init_pose.pose.position.y > goal_pose.pose.position.y - bias and init_pose.pose.position.y < goal_pose.pose.position.y + bias):
#         return True
#     else:
#         return False

all_positions = ["id101","id102"]
current_position_idx = 0
   
def check_goal_reached(init_pose, goal_pose, bias):
 
    if(init_pose.pose.position.x > goal_pose.pose.position.x - bias and init_pose.pose.position.x < goal_pose.pose.position.x + bias\
        and init_pose.pose.position.y > goal_pose.pose.position.y - bias and init_pose.pose.position.y < goal_pose.pose.position.y + bias):
        return True
    else:
        return False

# def goal_callback(data):
#     print("goal_callback started")
   
# init = rospy.Subscriber("/id100/aruco_single/pose", PoseStamped, goal_callback)
# init_msg = rospy.wait_for_message('/id100/aruco_single/pose', PoseStamped)

cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
init_pose = rospy.wait_for_message('/id100/aruco_single/pose', PoseStamped)
goal_pose = rospy.wait_for_message('/{}/aruco_single/pose'.format(all_positions[current_position_idx]), PoseStamped)
twist = Twist()

while current_position_idx < len(all_positions):
    print("check_goal result: " + str(check_goal_reached(init_pose, goal_pose, 0.2)))
    if check_goal_reached(init_pose, goal_pose, 0.2):
        print("Enter true goal_reached")
        current_position_idx = current_position_idx + 1
        # goal_pose = rospy.wait_for_message('/{}/aruco_single/pose'.format(all_positions[current_position_idx]), PoseStamped)

    print("INDEX: " + str(current_position_idx))
    print("CURRENT POSITION: " + str(all_positions[current_position_idx]))
    init_pose = rospy.wait_for_message('/id100/aruco_single/pose', PoseStamped)
    goal_pose = rospy.wait_for_message('/{}/aruco_single/pose'.format(all_positions[current_position_idx]), PoseStamped)
    # goal_pose = rospy.wait_for_message('/id101/aruco_single/pose', PoseStamped)
    # goal_pose = rospy.wait_for_message('/{}/aruco_single/pose'.format(all_positions[current_position_idx]), PoseStamped)
    # print("POSITION: " + str(all_positions[current_position_idx]))

    orientation_q = init_pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    Orientation = yaw
    dx = goal_pose.pose.position.x - init_pose.pose.position.x
    dy = goal_pose.pose.position.y - init_pose.pose.position.y
    distance = math.dist([init_pose.pose.position.x, init_pose.pose.position.y], [goal_pose.pose.position.x, goal_pose.pose.position.y])
    goal_direct = math.atan2(dy, dx)

    print("init_pose", [init_pose.pose.position.x, init_pose.pose.position.y])
    print("goal_pose", [goal_pose.pose.position.x, goal_pose.pose.position.y])
    print("Orientation", Orientation)

    print("goal_direct", goal_direct)
    if(Orientation < 0):
        Orientation = Orientation + 2 * math.pi
    if(goal_direct < 0):
        goal_direct = goal_direct + 2 * math.pi

    theta = goal_direct - Orientation

    if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
            theta = theta + 2 * math.pi
    elif theta > 0 and abs(theta - 2 * math.pi) < theta:
        theta = theta - 2 * math.pi
   
    print("theta:", theta)

    k2 = 2
    linear = 0.5
    angular = k2 * theta
    twist.linear.x = linear * distance * math.cos(theta)
    twist.angular.z = -angular
    cmd_pub.publish(twist)
    
## LAUNCH FILE FOR ARUCO FINDER ##
# <launch>

# <arg name="video_device" default="/dev/video0" />
# <arg name="image_width" default="640" />
# <arg name="image_height" default="480" />

# <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
# <param name="video_device" value="$(arg video_device)" />
# <param name="image_width" value="$(arg image_width)" />
# <param name="image_height" value="$(arg image_height)"/>
# <param name="pixel_format" value="mjpeg" />
# <param name="camera_frame_id" value="usb_cam" />
# <param name="io_method" value="mmap"/>
# </node>

# <!-- launch 2 auto_aruco_marker_finder with topic suffix id100 and id101  -->
# <group ns="id100">
#     <include file="$(find auto_aruco_marker_finder)/launch/aruco_marker_finder.launch">
#     <arg name="markerId" value="402"/>
#     </include>
# </group>
   
# <group ns="id101">
#     <include file="$(find auto_aruco_marker_finder)/launch/aruco_marker_finder.launch">
#     <arg name="markerId" value="403"/>
#     </include>
# </group>

# <group ns="id102">
#     <include file="$(find auto_aruco_marker_finder)/launch/aruco_marker_finder.launch">
#     <arg name="markerId" value="404"/>
#     </include>
# </group>

# </launch>