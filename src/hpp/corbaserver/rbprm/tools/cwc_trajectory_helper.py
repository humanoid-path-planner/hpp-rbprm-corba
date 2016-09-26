
from hpp.corbaserver.rbprm.tools.cwc_trajectory import *
from hpp.corbaserver.rbprm.tools.path_to_trajectory import *
from cwc import OptimError, cone_optimization

#global variables
res = []
trajec = []
trajec_mil = []
contacts = []
pos = []
normals = []
errorid = []

def displayComPath(pp, pathId,color=[0.,0.75,0.15,0.9]) :
	pathPos=[]
	length = pp.end*pp.client.problem.pathLength (pathId)
	t = pp.start*pp.client.problem.pathLength (pathId)
	while t < length :
		q = pp.client.problem.configAtParam (pathId, t)
		pp.publisher.robot.setCurrentConfig(q)
		q = pp.publisher.robot.getCenterOfMass()
		pathPos = pathPos + [q[:3]]
		t += pp.dt
	nameCurve = "path_"+str(pathId)+"_com"
	pp.publisher.client.gui.addCurve(nameCurve,pathPos,color)
	pp.publisher.client.gui.addToGroup(nameCurve,pp.publisher.sceneName)
	pp.publisher.client.gui.refresh()


def getContactPerPhase(fullBody, stateid, limbsCOMConstraints):
	contacts = [[],[],[]]
	for k, v in limbsCOMConstraints.iteritems():
		if(fullBody.isLimbInContact(k, stateid)):
			contacts[0]+=[v['effector']]
		if(fullBody.isLimbInContactIntermediary(k, stateid)):
			contacts[1]+=[v['effector']]
		if(fullBody.isLimbInContact(k, stateid+1)):
			contacts[2]+=[v['effector']]
	return contacts

def gencontactsPerFrame(fullBody, stateid, limbsCOMConstraints, pp, path_ids, times, dt_framerate=1./24.):
	contactsPerPhase = getContactPerPhase(fullBody, stateid, limbsCOMConstraints)
	config_size = len(fullBody.getCurrentConfig())
	interpassed = False
	res = []
	for path_id in path_ids:		
		length = pp.client.problem.pathLength (path_id)
		num_frames_required_fly = times[1] / dt_framerate
		num_frames_required_support = times[0] / dt_framerate
		dt_fly = float(length) / num_frames_required_fly
		dt_support = float(length) / num_frames_required_support
		dt_finals_fly  = [dt_fly*i for i in range(int(num_frames_required_fly))] + [1]		
		dt_finals_support  = [dt_support*i for i in range(int(num_frames_required_support))] + [1]	
		config_size_path = len(pp.client.problem.configAtParam (path_id, 0))
		if(config_size_path > config_size):
			interpassed = True
			res+= [contactsPerPhase[1] for t in dt_finals_fly]
		elif interpassed:			
			res+= [contactsPerPhase[2] for t in dt_finals_support]
		else:
			res+= [contactsPerPhase[0] for t in dt_finals_support]
	return res

def genPandNperFrame(fullBody, stateid, pp, path_ids, times, dt_framerate=1./24.):
	p, N= fullBody.computeContactPoints(stateid)
	config_size = len(fullBody.getCurrentConfig())
	interpassed = False
	pRes = []
	nRes = []
	for path_id in path_ids:		
		length = pp.client.problem.pathLength (path_id)
		num_frames_required_fly = times[1] / dt_framerate
		num_frames_required_support = times[0] / dt_framerate
		dt_fly = float(length) / num_frames_required_fly
		dt_support = float(length) / num_frames_required_support
		dt_finals_fly  = [dt_fly*i for i in range(int(num_frames_required_fly))] + [1]		
		dt_finals_support  = [dt_support*i for i in range(int(num_frames_required_support))] + [1]	
		config_size_path = len(pp.client.problem.configAtParam (path_id, 0))
		if(config_size_path > config_size):
			interpassed = True
			pRes+= [p[1] for t in dt_finals_fly]
			nRes+= [N[1] for t in dt_finals_fly]
		elif interpassed:			
			pRes+= [p[2] for t in dt_finals_support]
			nRes+= [N[2] for t in dt_finals_support]
		else:
			pRes+= [p[0] for t in dt_finals_support]
			nRes+= [N[0] for t in dt_finals_support]
	return pRes, nRes


stat_data = { 
"error_com_proj" : 0,
"error_optim_fail" : 0,
"error_unknown" : 0,
"num_errors" : 0,
"num_success" : 0,
"num_trials" : 0,
"time_cwc" : { "min" : 10000000., "avg" : 0., "max" : 0., "totaltime" : 0., "numiter" : 0 },
}

def __update_cwc_time(t):
	global stat_data
	stat_data["time_cwc"]["min"] = min(stat_data["time_cwc"]["min"], t) 
	stat_data["time_cwc"]["max"] = max(stat_data["time_cwc"]["max"], t) 
	stat_data["time_cwc"]["totaltime"] += t
	stat_data["time_cwc"]["numiter"] += 1
	

def __getTimes(fullBody, configs, i, time_scale):
	print "distance", fullBody.getEffectorDistance(i,i+1)
	trunk_distance =  np.linalg.norm(np.array(configs[i+1][0:3]) - np.array(configs[i][0:3]))
	distance = max(fullBody.getEffectorDistance(i,i+1), trunk_distance)
	dist = int(distance * time_scale)#heuristic
	while(dist %4 != 0):
		dist +=1
	total_time_flying_path = max(float(dist)/10., 0.3)
	total_time_support_path = float((int)(math.ceil(min(total_time_flying_path /2., 0.2)*10.))) / 10.
	times = [total_time_support_path, total_time_flying_path]
	if(total_time_flying_path>= 1.):
		dt = 0.1
	elif total_time_flying_path<= 0.3:
		dt = 0.05
	else:
		dt = 0.1
	return times, dt, distance
		

from hpp import Error as hpperr
import sys, time
def step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction = 0.5, optim_effectors = True, time_scale = 20., useCOMConstraints = False, use_window = 0, verbose = False, draw = False):
	global errorid
	global stat_data	
	fail = 0
	try:
	#~ if(True):
		times = [];
		dt = 1000;
		distance = __getTimes(fullBody, configs, i, time_scale)
		use_window = max(0, min(use_window,  (len(configs) - 1) - (i + 2))) # can't use preview if last state is reached
		for w in range(use_window+1):
			times2, dt2, dist2 = __getTimes(fullBody, configs, i+w, time_scale)
			times += times2
			dt = min(dt, dt2)
		print 'time per path', times
		print 'dt', dt
		if(distance > 0.0001):		
			stat_data["num_trials"] += 1
			if(useCOMConstraints):
				comC = limbsCOMConstraints
			else:
				comC = None
			if(optim_effectors):
				pid, trajectory, timeelapsed  =  solve_effector_RRT(fullBody, configs, i, True, friction, dt, times, False, optim, draw, verbose, comC, False, use_window=use_window)
			else :
				pid, trajectory, timeelapsed  =       solve_com_RRT(fullBody, configs, i, True, friction, dt, times, False, optim, draw, verbose, comC, False, use_window=use_window)
			displayComPath(pp, pid)
			#~ pp(pid)
			global res
			res = res + [pid]
			global trajec
			global trajec_mil			
			frame_rate = 1./24.
			frame_rate_andrea = 1./1000.
			#~ if(len(trajec) > 0):
				#~ frame_rate = 1./25.
				#~ frame_rate_andrea = 1./1001.
			new_traj = gen_trajectory_to_play(fullBody, pp, trajectory, times, frame_rate)
			new_traj_andrea = gen_trajectory_to_play(fullBody, pp, trajectory, times,frame_rate_andrea)
			new_contacts = gencontactsPerFrame(fullBody, i, limbsCOMConstraints, pp, trajectory, times, frame_rate_andrea)	
			Ps, Ns = genPandNperFrame(fullBody, i, pp, trajectory, times, frame_rate_andrea)
			if(len(trajec) > 0):
				new_traj = new_traj[1:]
				new_traj_andrea = new_traj_andrea[1:]
				new_contacts = new_contacts[1:]
				Ps = Ps[1:]
				Ns = Ns[1:]
			trajec = trajec + new_traj
			trajec_mil += new_traj_andrea
			global contacts
			contacts += new_contacts	
			global pos
			pos += Ps
			global normals
			normals+= Ns
			assert(len(contacts) == len(trajec_mil) and len(contacts) == len(pos) and len(normals) == len(pos))			
			stat_data["num_success"] += 1
		else:
			print "TODO, NO CONTACT VARIATION, LINEAR INTERPOLATION REQUIRED"
	except hpperr as e:
		print "hpperr failed at id " + str(i) , e.strerror
		stat_data["error_com_proj"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	except OptimError as e:
		print "OptimError failed at id " + str(i) , e.strerror
		stat_data["error_optim_fail"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	#~ except ValueError as e:
		#~ print "ValueError failed at id " + str(i) , e
		#~ stat_data["error_unknown"] += 1
		#~ stat_data["num_errors"] += 1
		#~ errorid += [i]
		#~ fail+=1
	#~ except IndexError as e:
		#~ print "IndexError failed at id " + str(i) , e
		#~ stat_data["error_unknown"] += 1
		#~ stat_data["num_errors"] += 1
		#~ errorid += [i]
		#~ fail+=1
	except Exception as e:
		stat_data["error_unknown"] += 1
		stat_data["num_errors"] += 1
		print e
		errorid += [i]
		fail+=1
		if (use_window == 0 and (len(configs) - 1) - (i + 2) > 0):
			print "could not project com, trying ti increase velocity "
			return step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction, optim_effectors, time_scale, useCOMConstraints, 1, verbose, draw)
	except:
		stat_data["error_unknown"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
		if (use_window == 0 and (len(configs) - 1) - (i + 2) > 0):
			print "could not project com, trying ti increase velocity "
			return step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction, optim_effectors, time_scale, useCOMConstraints, 1, verbose, draw)
	return fail
	
def step_profile(fullBody, configs, i, optim, limbsCOMConstraints,  friction = 0.5, optim_effectors = True, time_scale = 20., useCOMConstraints = False):
	global errorid		
	global stat_data	
	fail = 0
	try:
		trunk_distance =  np.linalg.norm(np.array(configs[i+1][0:3]) - np.array(configs[i][0:3]))
		distance = max(fullBody.getEffectorDistance(i,i+1), trunk_distance)
		dist = int(distance * time_scale)#heuristic
		while(dist %4 != 0):
			dist +=1
		total_time_flying_path = max(float(dist)/10., 0.3)
		total_time_support_path = float((int)(math.ceil(min(total_time_flying_path /2., 0.2)*10.))) / 10.
		times = [total_time_support_path, total_time_flying_path]
		if(total_time_flying_path>= 1.):
			dt = 0.1
		elif total_time_flying_path<= 0.3:
			dt = 0.05
		else:
			dt = 0.1
		if(distance > 0.0001):				
			stat_data["num_trials"] += 1
			if(useCOMConstraints):
				comC = limbsCOMConstraints
			else:
				comC = None
			if(optim_effectors):
				pid, trajectory, timeelapsed =  solve_effector_RRT(fullBody, configs, i, True, friction, dt, times, False, optim, False, False, comC, True)
			else :
				pid, trajectory, timeelapsed =       solve_com_RRT(fullBody, configs, i, True, friction, dt, times, False, optim, False, False, comC, True)			
			__update_cwc_time(timeelapsed)	
			stat_data["num_success"] += 1
		else:
			print "TODO, NO CONTACT VARIATION, LINEAR INTERPOLATION REQUIRED"
	except hpperr as e:
		print "hpperr failed at id " + str(i) , e.strerror
		stat_data["error_com_proj"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	except OptimError as e:
		print "OptimError failed at id " + str(i) , e.strerror
		stat_data["error_optim_fail"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	except ValueError as e:
		print "ValueError failed at id " + str(i) , e
		stat_data["error_unknown"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	except IndexError as e:
		print "IndexError failed at id " + str(i) , e
		stat_data["error_unknown"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	except Exception as e:
		stat_data["error_unknown"] += 1
		stat_data["num_errors"] += 1
		print e
		errorid += [i]
		fail+=1
	except:
		stat_data["error_unknown"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	return fail
	
	
def displayInSave(pp, pathId, configs):
	length = pp.end*pp.client.problem.pathLength (pathId)
	t = pp.start*pp.client.problem.pathLength (pathId)
	while t < length :
		q = pp.client.problem.configAtParam (pathId, t)
		configs.append(q)
		t += (pp.dt * pp.speed)

	

import time

from pickle import dump
def saveToPinocchio(filename):
	res = []
	for i, q_gep in enumerate(trajec_mil):
		#invert to pinocchio config:
		q = q_gep[:]
		quat_end = q[4:7]
		q[6] = q[3]
		q[3:6] = quat_end
		data = {'q':q, 'contacts': contacts[i], 'P' : pos[i], 'N' : normals[i]}
		res += [data]
	f1=open(filename, 'w+')
	dump(res, f1)
	f1.close()
		
def clean():
	global res
	global trajec
	global trajec_mil
	global contacts
	global errorid
	global pos
	global normals
	res = []
	trajec = []
	trajec_mil = []
	contacts = []
	errorid = []
	pos = []
	normals = []

import copy

def stats():	
	global stat_data	
	stat_data["error_id"] = errorid
	stat_data_copy = copy.deepcopy(stat_data)
	return stat_data_copy
	
def write_stats(filename):
	global stat_data	
	sd = copy.deepcopy(stat_data)
	f = open(filename, 'a')
	f.write("optim_error_com_proj " + str(sd["error_com_proj"]) + "\n")
	f.write("optim_error_optim_fail " + str(sd["error_optim_fail"]) + "\n")
	f.write("optim_error_unknown " + str(sd["error_unknown"]) + "\n")
	f.write("optim_num_success " + str(sd["num_success"]) + "\n")
	f.write("optim_num_trials " + str(sd["num_trials"]) + "\n")
	f.write("num_errors " + str(sd["num_errors"]) + "\n")
	f.write("error_id " + str(errorid) + "\n")
	f.write("time_cwc " + str(sd["time_cwc"]["min"]) + " " + str(sd["time_cwc"]["avg"]) + " " + str(sd["time_cwc"]["max"]) + " " + str(sd["time_cwc"]["totaltime"])  + " " + str(sd["time_cwc"]["numiter"]) + " " + "\n")
	f.close()
	return sd

def profile(fullBody, configs, i_start, i_end, limbsCOMConstraints,  friction = 0.5, optim_effectors = False, time_scale = 20., useCOMConstraints = False, filename ="log.txt"):	
	global stat_data		
	if(i_end > len(configs)-1):
		print "ERROR: i_end < len_configs ", i_end, len(configs)-1
		return # no point in trying optim, path was not fully computed
	for i in range(i_start, i_end):		
		step_profile(fullBody, configs, i, 0, limbsCOMConstraints,  friction, optim_effectors, time_scale, useCOMConstraints)
	stat_data["time_cwc"]["avg"] = stat_data["time_cwc"]["totaltime"] / float(stat_data["time_cwc"]["numiter"])
	write_stats(filename)

def saveAllData(fullBody, r, name):
	fullBody.exportAll(r, trajec, name)
	saveToPinocchio(name)

def play_traj(fullBody,pp,frame_rate):
	global trajec
	return play_trajectory(fullBody,pp,trajec,frame_rate)

#~ fullBody.exportAll(r, trajec, 'darpa_hyq_t_var_04f_andrea');
#~ saveToPinocchio('darpa_hyq_t_var_04f_andrea')
