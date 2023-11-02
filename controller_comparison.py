from omnidirectional_robot import OmnidirectionalRobot
from draw import Draw
from controllers import PID, MPC
import cv2
from utils import *
import numpy as np


way_points = []

W, H = 1000, 1000
draw = Draw(W, H, window_name = "Canvas")

robot_mpc = OmnidirectionalRobot(50, 50)
robot_pid = OmnidirectionalRobot(50, 50)
controller_pid = PID(kp_linear = 0.5, kd_linear = 0.1, ki_linear = 0,
					 kp_angular = 3, kd_angular = 0.1, ki_angular = 0)
horizon = 5
controller_mpc = MPC(horizon = horizon)

for x in range(200, 600, 2):
	y = H/2 + 200*np.sin(2*np.pi*0.25*(x + 200)/100)
	way_points.append([x, int(y)])

lw = 0
rw = 0
current_idx_mpc = 0
current_idx_pid = 0
linear_v = 0
angular_v = 0

mpc_robot_points = []
pid_robot_points = []

while True:
	draw.clear()
	if len(way_points)>0:
		draw.draw_path(way_points, color = (255, 0, 0), thickness = 1)

	if len(mpc_robot_points)>0:
		draw.draw_path(mpc_robot_points, color = (0, 255, 0), thickness = 1, dotted = True)

	if len(pid_robot_points)>0:
		draw.draw_path(pid_robot_points, color = (0, 0, 255), thickness = 1, dotted = True)

	draw.draw(robot_mpc.get_points(), color = (0, 255, 0), thickness = 1)
	draw.draw(robot_pid.get_points(), color = (0, 0, 255), thickness = 1)

	draw.add_text("PID Controller", color = (0, 0, 255), fontScale = 0.5, thickness = 1, org = (100, 50))
	draw.add_text("MPC Controller", color = (0, 255, 0), fontScale = 0.5, thickness = 1, org = (100, 75))
	draw.add_text("Trajectory", color = (255, 0, 0), fontScale = 0.5, thickness = 1, org = (100, 100))
	

	k = draw.show()

	# MPC robot
	x, _ = robot_mpc.get_state()
	if len(way_points)>0 and current_idx_mpc != len(way_points):
		mpc_robot_points.append([int(x[0, 0]), int(x[1, 0])])
		goal_pt = way_points[current_idx_mpc]
		v_x, v_y = controller_mpc.optimize(robot = robot_mpc, goal_xy = goal_pt)
		v_w = 0
		
		dist = get_distance(x[0, 0], x[1, 0], goal_pt[0], goal_pt[1])
		if dist<10:
			current_idx_mpc+= 1
	else:
		v_x = 0
		v_y = 0
		v_w = 0
	robot_mpc.set_robot_velocity(v_x, v_y, v_w)
	robot_mpc.update(0.5)


	# PID robot
	x, _ = robot_pid.get_state()
	if len(way_points)>0 and current_idx_pid != len(way_points):
		pid_robot_points.append([int(x[0, 0]), int(x[1, 0])])
		goal_pt = way_points[current_idx_pid]
		linear_v, angular_v = controller_pid.get_control_inputs(x, goal_pt, robot_pid.get_points()[2], current_idx_pid)
		dist = get_distance(x[0, 0], x[1, 0], goal_pt[0], goal_pt[1])
		if dist<10:
			current_idx_pid+= 1
	else:
		linear_v = 0
		angular_v = 0
	robot_pid.set_robot_velocity(linear_v, angular_v, 0)
	robot_pid.update(0.5)

	if k == ord("q"):
		break