from omnidirectional_robot import OmnidirectionalRobot
from draw import Draw
from controllers import MPC
import cv2
from utils import *
from parameters import *

way_points = []

def add_waypoint(event, x, y, flags, param):
    global way_points
    if event == cv2.EVENT_LBUTTONDOWN:
        way_points.append([x, y])
    if event == cv2.EVENT_RBUTTONDOWN:
        way_points.pop()

draw = Draw(VIEW_W, VIEW_H, window_name = "Canvas", mouse_callback = add_waypoint)

robot = OmnidirectionalRobot(100, 100)

controller = MPC(horizon = MPC_HORIZON)

current_idx = 0
v_x = 0
v_y = 0
v_w = 0
robot_path_points = []
while True:
	draw.clear()
	draw.add_text("Press the right click to place a way point, press the left click to remove a way point", 
					color = (0, 0, 0), fontScale = 0.5, thickness = 1, org = (5, 20))
	if len(way_points)>0:
		draw.draw_path(way_points, color = (200, 200, 200), thickness = 1)

	if len(robot_path_points)>0:
		draw.draw_path(robot_path_points, color = (255, 0, 0), thickness = 1, dotted = True)

	draw.draw(robot.get_points(), color = (255, 0, 0), thickness = 1)
	

	k = draw.show()

	x, _ = robot.get_state()
	if len(way_points)>0 and current_idx != len(way_points):
		robot_path_points.append([int(x[0, 0]), int(x[1, 0])])
		goal_pt = way_points[current_idx]
		
		v_x, v_y = controller.optimize(robot = robot, goal_xy = goal_pt)
		v_w = 0
		
		dist = get_distance(x[0, 0], x[1, 0], goal_pt[0], goal_pt[1])
		if dist<10:
			current_idx+= 1
	else:
		v_x = 0
		v_y = 0
		v_w = 0
	robot.set_robot_velocity(v_x, v_y, v_w)
	robot.update(DELTA_T)

	if k == ord("q"):
		break