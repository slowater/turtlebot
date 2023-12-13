import pybullet as p
import time
import pybullet_data
import yaml
from cbs import cbs
import math
import threading


def create_boundaries(length, width):
    """
        create rectangular boundaries with length and width

        Args:

        length: integer

        width: integer
    """
    for i in range(length):
        p.loadURDF("./final_challenge/assets/cube.urdf", [i, -1, 0.5])
        p.loadURDF("./final_challenge/assets/cube.urdf", [i, width, 0.5])
    for i in range(width):
        p.loadURDF("./final_challenge/assets/cube.urdf", [-1, i, 0.5])
        p.loadURDF("./final_challenge/assets/cube.urdf", [length, i, 0.5])
    p.loadURDF("./final_challenge/assets/cube.urdf", [length, -1, 0.5])
    p.loadURDF("./final_challenge/assets/cube.urdf", [length, width, 0.5])
    p.loadURDF("./final_challenge/assets/cube.urdf", [-1, width, 0.5])
    p.loadURDF("./final_challenge/assets/cube.urdf", [-1, -1, 0.5])


def create_env(yaml_file):
    """
    Creates and loads assets only related to the environment such as boundaries and obstacles.
    Robots are not created in this function (check `create_turtlebot_actor`).
    """
    with open(yaml_file, 'r') as f:
        try:
            env_params = yaml.load(f, Loader=yaml.FullLoader)
        except yaml.YAMLError as e:
            print(e) 
            
    # Create env boundaries
    dimensions = env_params["map"]["dimensions"]
    create_boundaries(dimensions[0], dimensions[1])

    # Create env obstacles
    for obstacle in env_params["map"]["obstacles"]:
        p.loadURDF("./final_challenge/assets/cube.urdf", [obstacle[0], obstacle[1], 0.5])
    return env_params


def create_agents(yaml_file):
    """
    Creates and loads turtlebot agents.

    Returns list of agent IDs and dictionary of agent IDs mapped to each agent's goal.
    """
    agent_box_ids = []
    box_id_to_goal = {}
    agent_name_to_box_id = {}
    with open(yaml_file, 'r') as f:
        try:
            agent_yaml_params = yaml.load(f, Loader=yaml.FullLoader)
        except yaml.YAMLError as e:
            print(e)
        
    start_orientation = p.getQuaternionFromEuler([0,0,0])
    for agent in agent_yaml_params["agents"]:
        start_position = (agent["start"][0], agent["start"][1], 0)
        box_id = p.loadURDF("data/turtlebot.urdf", start_position, start_orientation, globalScaling=1)
        agent_box_ids.append(box_id)
        box_id_to_goal[box_id] = agent["goal"]
        agent_name_to_box_id[agent["name"]] = box_id
    return agent_box_ids, agent_name_to_box_id, box_id_to_goal, agent_yaml_params


def read_cbs_output(file):
    """
        Read file from output.yaml, store path list.

        Args:

        output_yaml_file: output file from cbs.

        Returns:

        schedule: path to goal position for each robot.
    """
    with open(file, 'r') as f:
        try:
            params = yaml.load(f, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
    return params["schedule"]


def checkPosWithBias(Pos, goal, bias):
    """
        Check if pos is at goal with bias

        Args:

        Pos: Position to be checked, [x, y]

        goal: goal position, [x, y]

        bias: bias allowed

        Returns:

        True if pos is at goal, False otherwise
    """
    if(Pos[0] < goal[0] + bias and Pos[0] > goal[0] - bias and Pos[1] < goal[1] + bias and Pos[1] > goal[1] - bias):
        return True
    else:
        return False

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

def navigation(agent, goal, schedule):
    """
        Set velocity for robots to follow the path in the schedule.

        Args:

        agents: array containing the IDs for each agent

        schedule: dictionary with agent IDs as keys and the list of waypoints to the goal as values

        index: index of the current position in the path.

        Returns:

        Leftwheel and rightwheel velocity.
    """
    basePos = p.getBasePositionAndOrientation(agent)
    index = 0
    dis_th = 0.1
    prev_x = basePos[0][0]
    prev_y = basePos[0][1]
    pid_linear = PIDController(10, 0, 0.1)  # Tune these parameters for linear velocity
    pid_angular = PIDController(20, 0, 0.05)  # Tune these parameters for angular velocity
    last_time = time.time()
    while(not checkPosWithBias(basePos[0], goal, dis_th)):
        current_time = time.time()
        delta_time = current_time - last_time
        basePos = p.getBasePositionAndOrientation(agent)
        next = [schedule[index]["x"], schedule[index]["y"]]
        if(checkPosWithBias(basePos[0], next, dis_th)):
            index = index + 1
        if(index == len(schedule)):
            p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=1)
            p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=1)
            break
        x = basePos[0][0]
        y = basePos[0][1]
        Orientation = list(p.getEulerFromQuaternion(basePos[1]))[2]
        goal_direction = math.atan2((schedule[index]["y"] - y), (schedule[index]["x"] - x))

        if(Orientation < 0):
            Orientation = Orientation + 2 * math.pi
        if(goal_direction < 0):
            goal_direction = goal_direction + 2 * math.pi
        theta = goal_direction - Orientation

        if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
            theta = theta + 2 * math.pi
        elif theta > 0 and abs(theta - 2 * math.pi) < theta:
            theta = theta - 2 * math.pi
 
        current = [x, y]
        distance = math.dist(current, next)

        print("init_pose", str(agent), str([x, y]))
        print("goal_pose", str(agent), str(goal))

        # Update PID controllers
        linear_error = distance
        angular_error = theta
        linear_velocity = pid_linear.update(linear_error, delta_time)
        angular_velocity = pid_angular.update(angular_error, delta_time)

        rightWheelVelocity = linear_velocity + angular_velocity
        leftWheelVelocity = linear_velocity - angular_velocity

        # k1, k2, A = 10, 8, 20
        # linear = k1 * distance * math.cos(theta)
        # angular = k2 * theta

        # Calculate color based on linear velocity
        max_linear_velocity = 10
        velocity_color = linear_velocity / max_linear_velocity
        # velocity_color = linear / max_linear_velocity
        color = [velocity_color, 1 - velocity_color, 0]  # RGB: Varying from red to green

        # Draw the line segment
        p.addUserDebugLine([prev_x, prev_y, 0.01], [current[0], current[1], 0.01], lineColorRGB=color, lineWidth=10)
        prev_x, prev_y = current[0], current[1]

        # rightWheelVelocity = linear + angular
        # leftWheelVelocity = linear - angular

        p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=1)
        p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=1)
        # time.sleep(0.001)
    print(agent, "here")


def run(agents, goals, schedule):
    """
        Set up loop to publish leftwheel and rightwheel velocity for each robot to reach goal position.

        Args:

        agents: array containing the boxID for each agent

        schedule: dictionary with boxID as key and path to the goal as list for each robot.

        goals: dictionary with boxID as the key and the corresponding goal positions as values
    """
    threads = []
    for agent in agents:
        t = threading.Thread(target=navigation, args=(agent, goals[agent], schedule[agent]))
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

def optimize_path(path):
    """
    Optimizes the path by skipping intermediate coordinates in a straight line.

    Args:
    path (dict): A dictionary where keys are agent IDs and values are lists of coordinates.

    Returns:
    dict: Optimized path dictionary.
    """
    def is_straight_line(p1, p2, p3):
        """
        Checks if three points are in a straight line (collinear).
        """
        # Collinearity for points (x1, y1), (x2, y2), (x3, y3) is given by:
        # (y2 - y1)*(x3 - x2) == (y3 - y2)*(x2 - x1)
        return (p2['y'] - p1['y']) * (p3['x'] - p2['x']) == (p3['y'] - p2['y']) * (p2['x'] - p1['x'])

    optimized_path = {}
    for agent, waypoints in path.items():
        optimized_waypoints = []
        i = 0
        while i < len(waypoints):
            optimized_waypoints.append(waypoints[i])
            # Look ahead to find the next 'turn' in the path
            while i + 2 < len(waypoints) and is_straight_line(waypoints[i], waypoints[i + 1], waypoints[i + 2]):
                i += 1
            i += 1
        optimized_path[agent] = optimized_waypoints

    return optimized_path


# physics_client = p.connect(p.GUI, options='--width=1920 --height=1080 --mp4=multi_3.mp4 --mp4fps=30')
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# Disable tinyrenderer, software (CPU) renderer, we don't use it here
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

plane_id = p.loadURDF("plane.urdf")

global env_loaded
env_loaded = False

# Create environment
env_params = create_env("./final_challenge/env.yaml")

# Create turtlebots
agent_box_ids, agent_name_to_box_id, box_id_to_goal, agent_yaml_params = create_agents("./final_challenge/actors.yaml")

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera(cameraDistance=5.7, cameraYaw=0, cameraPitch=-89.9,
                                     cameraTargetPosition=[4.5, 4.5, 4])


cbs.run(dimensions=env_params["map"]["dimensions"], obstacles=env_params["map"]["obstacles"], agents=agent_yaml_params["agents"], out_file="./final_challenge/cbs_output.yaml")
cbs_schedule = read_cbs_output("./final_challenge/cbs_output.yaml")
# Replace agent name with box id in cbs_schedule
box_id_to_schedule = {}
for name, value in cbs_schedule.items():
    box_id_to_schedule[agent_name_to_box_id[name]] = value
# print(box_id_to_goal)
# print(box_id_to_schedule)

optimized_path = optimize_path(box_id_to_schedule)
print(optimized_path)
sol = {71: [{'t': 0, 'x': 9, 'y': 9}, {'t': 5, 'x': 4, 'y': 9}, {'t': 7, 'x': 4, 'y': 7}, {'t': 11, 'x': 8, 'y': 7}, {'t': 13, 'x': 8, 'y': 5}, {'t': 15, 'x': 6, 'y': 5}, {'t': 17, 'x': 6, 'y': 3}, {'t': 18, 'x': 7, 'y': 3}, {'t': 20, 'x': 7, 'y': 1}, {'t': 21, 'x': 6, 'y': 1}, {'t': 22, 'x': 6, 'y': 0}], 72: [{'t': 0, 'x': 0, 'y': 9}, {'t': 3, 'x': 3, 'y': 9}, {'t': 5, 'x': 3, 'y': 7}, {'t': 6, 'x': 2, 'y': 7}, {'t': 8, 'x': 2, 'y': 5}, {'t': 11, 'x': 5, 'y': 5}, {'t': 13, 'x': 5, 'y': 3}, {'t': 15, 'x': 7, 'y': 3}, {'t': 17, 'x': 7, 'y': 1}, {'t': 21, 'x': 3, 'y': 1}, {'t': 22, 'x': 3, 'y': 0}]}

run(agent_box_ids, box_id_to_goal, optimized_path)
time.sleep(2)