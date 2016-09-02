__24fps = 1. / 24.
__EPS = 0.1
from numpy.linalg import norm

def __linear_interpolation(p0,p1,dist_p0_p1, val):
	return p0 + (p1 - p0) * val / dist_p0_p1

def __gen_frame_positions(com_waypoints, dt, dt_framerate=__24fps):
	total_time = (len(com_waypoints) - 1) * dt
	dt_final = total_time * dt_framerate
	num_frames_required = total_time / dt_framerate + 1
	dt_finals = [dt_final*i for i in range(int(num_frames_required))]
	ids_val = []
	for i in range(len(com_waypoints) - 1):
		ids_val += [(i,val-dt*i) for val in dt_finals if (val < (i+1) *dt) and (val > i*dt)]
	return [com_waypoints[0]] + [__linear_interpolation(com_waypoints[i], com_waypoints[i+1], dt, val) for (i,val) in ids_val] + [com_waypoints[-1]]


def __find_q_t(robot, path_player, path_id, t):
	u = t
	current_t = -1
	length = pp.client.problem.pathLength (path_id)
	q = []
	a = 0.
	b = length
	#~ print [pp.client.problem.configAtParam (path_id, i)[-1] for i in [length / 5. *j for j in range(6)]]
	while(abs(current_t -t )> __EPS):
		current_t = pp.client.problem.configAtParam (path_id, u)[-1]
		print "current_t",current_t
		print "t",t
		if(current_t - t) > __EPS:
			print "upb",b
			b = u
			print "upb",b
		elif (t - current_t) > __EPS:
			print "upa",a
			a = u
			print "upa",a
		if  (pp.client.problem.configAtParam (path_id, a)[-1]) > t and (pp.client.problem.configAtParam (path_id, b)[-1] > t):
			print "error, tout au dessus!!!!!!!!!!!!!!!"
			return 0
		if  (pp.client.problem.configAtParam (path_id, a)[-1]) < t and (pp.client.problem.configAtParam (path_id, b)[-1] < t):
			print "error, tout en dessous!!!!!!!!!!!!!!!"
			return 0
		#~ print "cassos"
		print "current_l", u
		u = (b-a)/2.		
		print "current_l", u
		#~ return 0
		#~ print "current_t", current_t
		#~ print "t", t
		#~ print "a", a
		#~ print "b", b
		#~ print "length", length
	print "gagne"
	return q[:-1]
		
def linear_interpolate_path(robot, path_player, path_id, total_time, dt_framerate=__24fps):
	pp = path_player
	length = pp.client.problem.pathLength (path_id)
	num_frames_required = total_time / dt_framerate
	dt = float(length) / num_frames_required
	dt_finals = [dt*i for i in range(int(num_frames_required))] + [length]
	return[pp.client.problem.configAtParam (path_id, t) for t in dt_finals]
	
def follow_trajectory_path(robot, path_player, path_id, total_time, dt_framerate=__24fps):
	pp = path_player
	length = pp.client.problem.pathLength (path_id)
	num_frames_required = total_time / dt_framerate
	dt = float(length) / num_frames_required
	#matches the extradof normalized
	dt_finals = [dt*i / length for i in range(int(num_frames_required))] + [1]
	return[__find_q_t(robot, path_player, path_id, t) for t in dt_finals]
	
#~ def playCOM(pathId):
	#~ length = pp.end*pp.client.problem.pathLength (pathId)
	#~ t = pp.start*pp.client.problem.pathLength (pathId)
	#~ while t < length :
		#~ start = time.time()
		#~ q = pp.client.problem.configAtParam (pathId, t)
		#~ fullBody.setCurrentConfig(q)
		#~ q[0:3] = fullBody.getCenterOfMass()
		#~ pp.publisher.robotConfig = q
		#~ pp.publisher.publishRobots ()
		#~ t += (pp.dt * pp.speed)
		#~ elapsed = time.time() - pp.start
		#~ if elapsed < pp.dt :
		  #~ time.sleep(pp.dt-elapsed))

import numpy as np	
com_waypoints = [np.array([i,i,i]) for i in range(6)]
dt = 0.2
dt_framerate = __24fps

res = __gen_frame_positions(com_waypoints, dt, dt_framerate)
