__24fps = 1. / 24.
__EPS = 0.00001
from __future__ import print_function
from numpy.linalg import norm

def __linear_interpolation(p0,p1,dist_p0_p1, val):
	return p0 + (p1 - p0) * val / dist_p0_p1

def __gen_frame_positions(com_waypoints, dt, dt_framerate=__24fps):
	total_time = (len(com_waypoints) - 1) * dt
	dt_final = total_time * dt_framerate
	num_frames_required = total_time / dt_framerate + 1
	dt_finals = [dt_final*i for i in range(int(round(num_frames_required)))]
	ids_val = []
	for i in range(len(com_waypoints) - 1):
		ids_val += [(i,val-dt*i) for val in dt_finals if (val < (i+1) *dt) and (val > i*dt)]
	return [com_waypoints[0]] + [__linear_interpolation(com_waypoints[i], com_waypoints[i+1], dt, val) for (i,val) in ids_val] + [com_waypoints[-1]]


def __find_q_t(robot, path_player, path_id, t):
	current_t = -1
	pp = path_player
	length = pp.client.problem.pathLength (path_id)
	u = min(t, length)
	q = []
	a = 0.
	b = length
	#~ print [pp.client.problem.configAtParam (path_id, i)[-1] for i in [length / 5. *j for j in range(6)]]
	while(True):
		q = pp.client.problem.configAtParam (path_id, u)
		current_t = q[-1]
		if(a >= b):
			print("ERROR, a > b, t does not exist")
		if abs(current_t - t) < __EPS:
			#print "last config q = ",q[-1]
			return q[:-1]
		elif(current_t - t) < __EPS:
			a = u
		elif (current_t - t) > __EPS:
			b = u
		u = (b+a)/2.
	return u
		
def linear_interpolate_path(robot, path_player, path_id, total_time, dt_framerate=__24fps):
	pp = path_player
	length = pp.client.problem.pathLength (path_id)
	num_frames_required = total_time / dt_framerate
	dt = float(length) / num_frames_required
	dt_finals = [dt*i for i in range(int(round(num_frames_required))-1)] + [length]
	return[pp.client.problem.configAtParam (path_id, t) for t in dt_finals]
	
def follow_trajectory_path(robot, path_player, path_id, total_time, dt_framerate=__24fps):
	pp = path_player
	print("total_times in follow path : ",total_time)
	length = pp.client.problem.pathLength (path_id)
	num_frames_required = total_time / dt_framerate
	dt = float(length) / num_frames_required
	#matches the extradof normalized
	#~ print "length ", length, "total tome ", total_time, "frame rate ", dt_framerate, "num_frames_required ", num_frames_required, "dt ", dt
	dt_finals = [dt*i / length for i in range(int(round(num_frames_required)))]
	return[__find_q_t(robot, path_player, path_id, t) for t in dt_finals]
	
def gen_trajectory_to_play(robot, path_player, path_ids, total_time_per_paths, dt_framerate=__24fps):
	print("gen_trajectory : dt = ",dt_framerate)
	config_size = len(robot.getCurrentConfig())
	res = []
	pp = path_player
	activeid = 0
	print("path_ids = ", path_ids)
	for i, path_id in enumerate(path_ids):
		config_size_path = len(path_player.client.problem.configAtParam (int(path_id), 0))
		if(config_size_path > config_size):
		#~ if(i == 1 ):
			res+= follow_trajectory_path(robot, path_player, int(path_id), total_time_per_paths[i], dt_framerate)
		else:
			res+= linear_interpolate_path(robot, path_player, int(path_id), total_time_per_paths[i], dt_framerate)
		activeid +=1
	return res
	
import time
def play_trajectory(fullBody, path_player, configs, dt_framerate=__24fps):
	for q in configs:
		start = time.time()
		path_player.publisher.robotConfig = q
		path_player.publisher.publishRobots ()
		elapsed = time.time() - start  
		if elapsed < dt_framerate :
			time.sleep(dt_framerate-elapsed)

def displayFeetTrajectory(path_player,end_effector_names,colors,path_ids):
	assert len(end_effector_names)==len(colors)
	for path_id in path_ids:
		for i in range(len(end_effector_names)):
			path_player.displayPath(int(path_id),colors[i],end_effector_names[i])

#~ import numpy as np	
#~ com_waypoints = [np.array([i,i,i]) for i in range(6)]
#~ dt = 0.2
#~ dt_framerate = __24fps
#~ 
#~ res = __gen_frame_positions(com_waypoints, dt, dt_framerate)
