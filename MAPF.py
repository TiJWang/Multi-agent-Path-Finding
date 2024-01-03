#!/usr/bin/env python3
import yaml
import rospy
import math
import actionlib
import threading
import cv2
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
rospy.init_node('goal_pose')
# goal_pose = rospy.wait_for_message('/id101/aruco_single/pose', PoseStamped)
agents = [1, 2]


def convert_sim_to_real_pose(point, matrix):
    point = np.array([point['x'], point['y'], 1])
    print(f'sim point {point}')
    transformed_point = np.dot(matrix, point)
    transformed_point = transformed_point / transformed_point[2]
    print(f'real pose {transformed_point}')
    return {'x': transformed_point[0], 'y': transformed_point[1]}

def read_cbs_output(file, agent):
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
    return params["schedule"][agent]


def move(goal_pose, agent):
    if agent == 1:
        init_pose = rospy.wait_for_message('/id89/aruco_single/pose', PoseStamped)
    elif agent == 2:
        init_pose = rospy.wait_for_message('/id125/aruco_single/pose', PoseStamped)
    while not check_goal_reached(init_pose, goal_pose, 0.05):
        if agent == 1:
            init_pose = rospy.wait_for_message('/id89/aruco_single/pose', PoseStamped)
        elif agent == 2:
            init_pose = rospy.wait_for_message('/id125/aruco_single/pose', PoseStamped)
        # goal_pose = rospy.wait_for_message('/id101/aruco_single/pose', PoseStamped)
        orientation_q = init_pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        Orientation = yaw
        dx = goal_pose[0] - init_pose.pose.position.x
        dy = goal_pose[1] - init_pose.pose.position.y
        distance = math.dist([init_pose.pose.position.x, init_pose.pose.position.y], [goal_pose[0], goal_pose[1]])
        goal_direct = math.atan2(dy, dx)

    print("init_pose", [init_pose.pose.position.x, init_pose.pose.position.y])
    print("goal_pose", [goal_pose[0], goal_pose[1]])
    print("Orientation", Orientation)
    print("goal_direct", goal_direct)
    if (Orientation < 0):
        Orientation = Orientation + 2 * math.pi
    if (goal_direct < 0):
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
    cmd_pub_now = cmd_pub[agent - 1]
    cmd_pub_now.publish(twist)


def check_goal_reached(init_pose, goal_pose, bias):

    if(init_pose.pose.position.x > goal_pose[0] - bias and init_pose.pose.position.x < goal_pose[0] + bias\
        and init_pose.pose.position.y > goal_pose[1] - bias and init_pose.pose.position.y < goal_pose[1] + bias):
        return True
    else:
        return False


def run(agent):
    for index in range(1, len(list_agent[agent - 1])):
        point = {"x": (list_agent[agent - 1][index]["x"]), "y": (list_agent[agent - 1][index]["y"])}
        output = convert_sim_to_real_pose(point, matrix)
        goal_pose = [output["x"], output["y"]]
        move(goal_pose, agent)
        index = index + 1

index = 0
list_Moss = read_cbs_output("./cbs_output.yaml", agents[0])
list_chef = read_cbs_output("./cbs_output.yaml", agents[1])
list_agent = [list_Moss, list_chef]
cmd_pub_Moss = rospy.Publisher('/Moss/cmd_vel', Twist, queue_size=1)
cmd_pub_chef = rospy.Publisher('/chef/cmd_vel', Twist, queue_size=1)
cmd_pub = [cmd_pub_Moss, cmd_pub_chef]
twist = Twist()

### Define your points in the simulation plane and the real-world plane

pose_tl = rospy.wait_for_message('/id500/aruco_single/pose', PoseStamped)
pose_tr = rospy.wait_for_message('/id501/aruco_single/pose', PoseStamped)
pose_br = rospy.wait_for_message('/id502/aruco_single/pose', PoseStamped)
pose_bl = rospy.wait_for_message('/id503/aruco_single/pose', PoseStamped)
print(f'tl x={pose_tl.pose.position.x} y={pose_tl.pose.position.y}')
print(f'tr x={pose_tr.pose.position.x} y={pose_tr.pose.position.y}')
print(f'br x={pose_br.pose.position.x} y={pose_br.pose.position.y}')
print(f'bl x={pose_bl.pose.position.x} y={pose_bl.pose.position.y}')
real_points = np.float32([[pose_bl.pose.position.x, pose_bl.pose.position.y],
                          [pose_br.pose.position.x, pose_br.pose.position.y],
                          [pose_tl.pose.position.x, pose_tl.pose.position.y],
                          [pose_tr.pose.position.x, pose_tr.pose.position.y]])
sim_points = np.float32([[0, 0], [10, 0], [0, 10], [10, 10]])
### Calculate the perspective transformation matrix
matrix = cv2.getPerspectiveTransform(sim_points, real_points)

threads = []
for agent in agents:
    t = threading.Thread(target=run, args=(agent,))
    threads.append(t)
    t.start()

for t in threads:
    t.join()