# Submission Manual
### _A brief summary over the submission details_

## Source code
The [source code](https://github.com/slowater/turtlebot.git) is a copy from the Jackson's PyBullet simulation repo but the relevant file like final_challenge.py is changed based on BigMac's implementation. In this file, we added the inmplementation path optimization and PID controller. Apart from that, additional things added here are for plotting purposes.  For single_bot_motion_control.py, I also experimented with different parameters and added code to plot the path. In order to check the implementation, you can copy the content of these file into the corresponding file in your PyBullet simulation environment.

For the real-world experiment, I put the goal_pose_single.py for the single-robot auto navigation and goal_pose_multi.py for the multi-robot auto navigation. It's worth mentioning that for the multi-robot, we copy the CBS output YAML file into the same directory as the goal_pose.py.

### PID
For testing the PID, I recommed starting with small value and see the result. Different Kp, Ki, Kd value are needed for different experiments (with path optimization vs without path optimization). The current value used in the code is the "safe" value  that you can also try. 

## Video
The [Google Drive folder](https://drive.google.com/drive/folders/1JjqtRMcOB7IXfiGPMlC5E8kEYQZFBMWo?usp=drive_link) consists of three video file, which are:
- Single-bot_Bias02.mp4: The very first successful single-robot auto navigation to two ArUco markers which we set bias = 0.2.
- Single-bot_ReportLivePlot: Another single-robot auto navigation expriment which direcly corresponds to the live robot pat plot in the report.
- Multi-robot_OldMap.mp4: This are the successful real-world experiment for the multi-robot auto navigation for two TurtleBot(s) using CBS and multi-threading. This is also the expriment that Alex has graded.


## Installation

Install pybullet:

```shell
pip3 install pybullet --upgrade --user
python3 -m pybullet_envs.examples.enjoy_TF_AntBulletEnv_v0_2017may
python3 -m pybullet_envs.examples.enjoy_TF_HumanoidFlagrunHarderBulletEnv_v1_2017jul
python3 -m pybullet_envs.deep_mimic.testrl --arg_file run_humanoid3d_backflip_args.txt
```

Install necessary dependencies to run cbs:

```shell
python3 -m pip install -r requirement.txt
```

## To run the simulation

``` shell 
$ python3 muti_robot_navigation_2.py
```
