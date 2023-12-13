#!/usr/bin/env python3

import rospy
import math
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import yaml
import numpy as np
import cv2
import threading

def convert_sim_to_real_pose(x, y, matrix):
    point = np.array([x, y, 1])
    #print(f'sim point {x}')
    transformed_point = np.dot(matrix, point)
    transformed_point = transformed_point / transformed_point[2]
    #print(f'real pose {transformed_point}')
    return transformed_point[0], transformed_point[1]


def check_goal_reached(init_pose, x, y, bias):
    if(init_pose.pose.position.x > x - bias and init_pose.pose.position.x < x + bias\
        and init_pose.pose.position.y > y - bias and init_pose.pose.position.y < y + bias):
        return True
    else:
        return False

def navigation(turtlebot_name, arucoID, goal_list): 
    
    current_position_idx = 0

    cmd_pub = rospy.Publisher('/{}/cmd_vel'.format(turtlebot_name), Twist, queue_size=1)
    init_pose = rospy.wait_for_message('/{}/aruco_single/pose'.format(arucoID), PoseStamped)
    # goal_pose = rospy.wait_for_message('/{}/aruco_single/pose'.format(all_positions[current_position_idx]), PoseStamped)
    twist = Twist()


    while current_position_idx < len(goal_list):
        x, y = goal_list[current_position_idx] 

        if check_goal_reached(init_pose, x, y, 0.1):
            # print("Enter true goal_reached")
            current_position_idx = current_position_idx + 1

        init_pose = rospy.wait_for_message('/{}/aruco_single/pose'.format(arucoID), PoseStamped)

        orientation_q = init_pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        Orientation = yaw

        dx = x - init_pose.pose.position.x
        dy = y - init_pose.pose.position.y
        distance = math.dist([init_pose.pose.position.x, init_pose.pose.position.y], [x, y])
        goal_direct = math.atan2(dy, dx)
        print("init_pose", turtlebot_name, [init_pose.pose.position.x, init_pose.pose.position.y])
        print("goal_pose", turtlebot_name, [x, y])
        # print("Orientation", Orientation)

        # print("goal_direct", goal_direct)
        if(Orientation < 0):
            Orientation = Orientation + 2 * math.pi
        if(goal_direct < 0):
            goal_direct = goal_direct + 2 * math.pi

        theta = goal_direct - Orientation

        if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
                theta = theta + 2 * math.pi
        elif theta > 0 and abs(theta - 2 * math.pi) < theta:
            theta = theta - 2 * math.pi
    
        # print("theta:", theta)

        k2 = 3
        linear = 1.2
        angular = k2 * theta
        print("linear:", linear)
        print("angular:", angular)
        twist.linear.x = linear * distance * math.cos(theta)
        twist.angular.z = -angular
        cmd_pub.publish(twist)

def run(agents, all_ArucoIDs, all_coordinates):
    """
        Set up loop to publish leftwheel and rightwheel velocity for each robot to reach goal position.

        Args:

        agents: array containing the boxID for each agent

        schedule: dictionary with boxID as key and path to the goal as list for each robot.

        goals: dictionary with boxID as the key and the corresponding goal positions as values
    """

    threads = []

    for index in range(len(agents)):
        t = threading.Thread(target=navigation, args=(agents[index], all_ArucoIDs[index], all_coordinates[index]))
        threads.append(t)
        t.start()

    for t in threads:
        t.join()


### MAIN

rospy.init_node('goal_pose')

### Define your points in the simulation plane and the real-world plane
pose_tl = rospy.wait_for_message('/id500/aruco_single/pose', PoseStamped)
pose_tr = rospy.wait_for_message('/id501/aruco_single/pose', PoseStamped)
pose_br = rospy.wait_for_message('/id502/aruco_single/pose', PoseStamped)
pose_bl = rospy.wait_for_message('/id503/aruco_single/pose', PoseStamped)
# print(f'tl x={pose_tl.pose.position.x} y={pose_tl.pose.position.y}')
# print(f'tr x={pose_tr.pose.position.x} y={pose_tr.pose.position.y}')
# print(f'br x={pose_br.pose.position.x} y={pose_br.pose.position.y}')
# print(f'bl x={pose_bl.pose.position.x} y={pose_bl.pose.position.y}')

real_points = np.float32([[pose_bl.pose.position.x, pose_bl.pose.position.y],
                         [pose_br.pose.position.x, pose_br.pose.position.y],
                         [pose_tl.pose.position.x, pose_tl.pose.position.y],
                         [pose_tr.pose.position.x, pose_tr.pose.position.y]])
sim_points = np.float32([[0, 0], [10, 0], [0, 10], [10, 10]])

### Calculate the perspective transformation matrix
matrix = cv2.getPerspectiveTransform(sim_points, real_points)

coordinates_1 = []
coordinates_2 = []

def read_yaml_file(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data['schedule']  # This will be a dictionary of steps

schedule_data = read_yaml_file("./cbs_output.yaml")
#run this for all elements in matrix: convert_sim_to_real_pose(x, matrix)


for key, steps in schedule_data.items():
    # print("key: " + str(key))
    # print("steps: " + str(steps))
    for step in steps:
        # print("simulation x: " + str(step['x']))
        # print("simulation y: + " + str(step['y']))

        x,y = convert_sim_to_real_pose(step['x'], step['y'], matrix)
        # print("converted x: " + str(step))
        # print("converted y: + " + str(type(key)))

        if key == 1:
            coordinates_1.append((x,y))
        elif key == 2:
            coordinates_2.append((x,y))

all_coordinates = [coordinates_1, coordinates_2]
all_ArucoIDs = ["id402", "id89"]
all_agents = ["turtle1", "turtle2"]

run(all_agents, all_ArucoIDs, all_coordinates)