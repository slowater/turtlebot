import pybullet as p
import pybullet_data
import math
import time
import matplotlib.pyplot as plt

def check_pos(Pos, goal, bias):
    """
        Check if pos is at goal with bias

        Args:

        Pos: Position to be checked, [x, y]

        goal: goal position, [x, y]

        bias: bias allowed

        Returns:

        True if pos is at goal, False otherwise
    """
    if goal[0] + bias > Pos[0] > goal[0] - bias and goal[1] + bias > Pos[1] > goal[1] - bias:
        return True
    else:
        return False


def goto(agent, targets):
    dis_th = 0.05
    basePos = p.getBasePositionAndOrientation(agent)
    prev_x, prev_y = basePos[0][0], basePos[0][1]

    # Lists to store data for plotting
    timestamps = []
    linear_velocities = []
    angular_velocities = []

    start_time = time.time()  # Record the start time


    for target in targets:
        goal_x, goal_y = target
        while True:
            basePos = p.getBasePositionAndOrientation(agent)
            current_x, current_y = basePos[0][0], basePos[0][1]

            if check_pos([current_x, current_y], [goal_x, goal_y], dis_th):
                break

            current_orientation = list(p.getEulerFromQuaternion(basePos[1]))[2]
            goal_direction = math.atan2((goal_y - current_y), (goal_x - current_x))

            if current_orientation < 0:
                current_orientation += 2 * math.pi
            if goal_direction < 0:
                goal_direction += 2 * math.pi

            theta = goal_direction - current_orientation
            if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
                theta += 2 * math.pi
            elif theta > 0 and abs(theta - 2 * math.pi) < theta:
                theta -= 2 * math.pi

            k_linear = 5
            k_angular = 5
            linear = k_linear * math.cos(theta)
            print(linear)
            angular = k_angular * theta

            # Record timestamp and linear velocity
            timestamps.append(time.time() - start_time)
            linear_velocities.append(linear)
            angular_velocities.append(angular)


            # Calculate color based on linear velocity
            max_linear_velocity = 10
            velocity_color = linear / max_linear_velocity
            color = [velocity_color, 1 - velocity_color, 0]  # RGB: Varying from red to green

            # Draw the line segment
            p.addUserDebugLine([prev_x, prev_y, 0.01], [current_x, current_y, 0.01], lineColorRGB=color, lineWidth=10)
            prev_x, prev_y = current_x, current_y

            rightWheelVelocity = linear + angular
            leftWheelVelocity = linear - angular

            p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=10)
            p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=10)
    return timestamps, linear_velocities, angular_velocities

def plot_velocity(time_data, velocity_data):
    plt.figure(figsize=(10, 6))
    plt.plot(time_data, velocity_data, label='Linear Velocity')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Linear Velocity')
    plt.title('Linear Velocity Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_angular_velocity(time_data, velocity_data):
    plt.figure(figsize=(10, 6))
    plt.plot(time_data, velocity_data, label='Angular Velocity')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Angular Velocity')
    plt.title('Angular Velocity Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)

startPosition = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

boxId = p.loadURDF("data/turtlebot.urdf", startPosition, startOrientation, globalScaling=1)
p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=-89.9,
                             cameraTargetPosition=[0, 0, 0])

# Example usage
targets = [(1,1),(-1,2)]
timestamps, linear_velocities, angular_velocities = goto(boxId, targets)
plot_angular_velocity(timestamps, angular_velocities)
plot_velocity(timestamps, linear_velocities)
# max_linear_velocity = 10  # Set this to your maximum expected linear velocity
# goto(boxId, 1, 1)
# goto(boxId, -1, 2)
