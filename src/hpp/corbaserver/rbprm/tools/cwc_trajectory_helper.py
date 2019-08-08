from __future__ import print_function
from hpp.corbaserver.rbprm.tools.cwc_trajectory import *
from hpp.corbaserver.rbprm.tools.path_to_trajectory import *
from cwc import OptimError, cone_optimization
from hpp.corbaserver.rbprm.tools.path_to_trajectory import gen_trajectory_to_play
from numpy import append, array

#global variables
res = []
trajec = []
trajec_mil = []
#~ contacts = []
pos = []
normals = []
pEffs = []
coms = []
errorid = []
cones_saved = []

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


def genPandNperFrame(fullBody, stateid, limbsCOMConstraints, pp, path_ids, times, dt_framerate=1./24.):
	p, N= fullBody.computeContactPointsPerLimb(stateid, limbsCOMConstraints.keys(), limbsCOMConstraints)
	freeEffectors = [ [limbsCOMConstraints[limb]['effector'] for limb in limbsCOMConstraints.keys() if limbsCOMConstraints[limb]['effector'] not in p[i]] for i in range(len(p))]
	config_size = len(fullBody.getCurrentConfig())
	interpassed = False
	pRes = []
	nRes = []
	for idx, path_id in enumerate(path_ids):		
		length = pp.client.problem.pathLength (path_id)
		num_frames_required = times[idx] / dt_framerate
		#~ print "dt_framerate", dt_framerate
		#~ print "num_frames_required", times[idx], " ", num_frames_required
		dt = float(length) / num_frames_required
		dt_finals  = [dt*i for i in range(int(round(num_frames_required)))]	
		pRes+= [p[idx] for t in dt_finals]
		nRes+= [N[idx] for t in dt_finals]
	return pRes, nRes, freeEffectors


def __getPos(effector, fullBody, config):
	fullBody.setCurrentConfig (config)
	q = fullBody.getJointPosition(effector)
	quat_end = q[4:7]
	q[6] = q[3]
	q[3:6] = quat_end
	return q

def genPEffperFrame(fullBody, freeEffectorsPerPhase, qs, pp, times, dt_framerate):
	res = []
	for idx, phase in enumerate(freeEffectorsPerPhase):
		num_frames_required = int(times[idx] / dt_framerate)
		qid = len(res)
		for q in qs[qid:num_frames_required+qid]:			
			p = {}
			for effector in phase:
				p[effector] = __getPos(effector, fullBody, q)
			res.append(p)
	return res


def genComPerFrame(final_state, dt, dt_framerate = 1./1000.):
	num_frames_per_dt = int(round(dt / dt_framerate))
	inc = 1./((float)(num_frames_per_dt))
	c =   [array(final_state['x_init'][:3])] + final_state['c']
	dc =   [array(final_state['x_init'][3:])] + final_state['dc']
	ddc = final_state['ddc']
	cs = []
	for i in range(0,len(c)-1):
		for j in range(num_frames_per_dt):
			ddt = j * inc * dt
			cs.append(c[i] + ddt *dc[i] + ddt *ddt * 0.5 * ddc[i])
	return cs

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
	

"""
def __getTimes(fullBody, configs, i, time_scale):
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

"""

def __getTimes(fullBody, configs, i, time_scale,use_window=0):
		t = fullBody.getTimeAtState(i+1) - fullBody.getTimeAtState(i)
                dt = 0.02
		print("t = ",t)
		t = time_scale*t
		print("after scale, t = ",t)
		trunk_distance =  np.linalg.norm(np.array(configs[i+1][0:3]) - np.array(configs[i][0:3]))
		distance = max(fullBody.getEffectorDistance(i,i+1), trunk_distance)
		# TODO : si t = 0, hardcoded ...
                """
		if t <= dt*6.:
				print "WARNING : in getTime, t=0"
				t = dt*6.
				use_window = 2
                """
                times = [dt*5. , 0] #FIXME : hardcoded value depend on interpolation step choosen (not available here)
                """
		if t > dt*14.:
			times = [dt*4. , 0]
                """
		times[1] = t - 2*times[0]
		times[1] = float((int)(math.floor(times[1]*100.))) / 100.
		print("times : ",times)
		return times, dt, distance,use_window


from hpp import Error as hpperr
import sys, time
def step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction = 0.5, optim_effectors = True, time_scale = 20., useCOMConstraints = False, use_window = 0, verbose = False, draw = False,
trackedEffectors = [],use_velocity=False,pathId = 0):
	print("##########################################")
	global errorid
	global stat_data	
	fail = 0
	#~ try:
	print("Use window = ",use_window)
	if(True):
		times = [];
		dt = 1000;
		times, dt, distance,use_window = __getTimes(fullBody, configs, i, time_scale,use_window)
		print("Use window = ",use_window)
		if distance == 0:
				use_window = use_window+1
		use_window = max(0, min(use_window,  (len(configs) - 1) - (i + 2))) # can't use preview if last state is reached
		w = 0
		while w < (use_window+1):
			times2, dt2, dist2,use_window = __getTimes(fullBody, configs, i+w, time_scale,use_window)
			print("Use window = ",use_window)
			times += times2
			dt = min(dt, dt2)
			w = w+1
		time_per_path = [times[0]] + [times[1]] + [times [0]]
		print('time per path', times, time_per_path)
		print('dt', dt)
		if(distance > 0.0001):		
			stat_data["num_trials"] += 1
			if(useCOMConstraints):
				comC = limbsCOMConstraints
			else:
				comC = None
			if(optim_effectors):
				pid, trajectory, timeelapsed, final_state  =  solve_effector_RRT(fullBody, configs, i, True, friction, dt, times, False, optim, draw, verbose, comC, False, use_window=use_window, trackedEffectors = trackedEffectors,use_velocity=use_velocity,pathId = pathId)
			else :
				pid, trajectory, timeelapsed, final_state  =       solve_com_RRT(fullBody, configs, i, True, friction, dt, times, False, optim, draw, verbose, comC, False, use_window=use_window, trackedEffectors = trackedEffectors,use_velocity=use_velocity,pathId = pathId)
			displayComPath(pp, pid)
			#~ pp(pid)
			global res
			res = res + [pid]
			global trajec
			global trajec_mil			
			frame_rate = 0.01
			frame_rate_andrea = 1./100.
#			frame_rate_andrea = 1./1000.
			#~ if(len(trajec) > 0):
				#~ frame_rate = 1./25.
				#~ frame_rate_andrea = 1./1001.
			print("first traj :")
			new_traj = gen_trajectory_to_play(fullBody, pp, trajectory, time_per_path, frame_rate)
			print("traj Andrea : ")
			new_traj_andrea = gen_trajectory_to_play(fullBody, pp, trajectory, time_per_path,frame_rate_andrea)
			#~ new_contacts = gencontactsPerFrame(fullBody, i, limbsCOMConstraints, pp, trajectory, times, frame_rate_andrea)	
			Ps, Ns, freeEffectorsPerPhase = genPandNperFrame(fullBody, i, limbsCOMConstraints, pp, trajectory, time_per_path, frame_rate_andrea)
			NPeffs = genPEffperFrame(fullBody, freeEffectorsPerPhase, new_traj_andrea, pp, time_per_path, frame_rate_andrea)
			com = genComPerFrame(final_state, dt, frame_rate_andrea)
			#~ if(len(trajec) > 0):
				#~ new_traj = new_traj[1:]
				#~ new_traj_andrea = new_traj_andrea[1:]
				#~ Ps = Ps[1:]
				#~ Ns = Ns[1:]
				#~ com = com[1:]
				#~ NPeffs = NPeffs[1:]
			trajec = trajec + new_traj
			trajec_mil += new_traj_andrea
			#~ global contacts
			#~ contacts += new_contacts	
			global pos
			pos += Ps
			global normals
			normals+= Ns
			global pEffs
			pEffs+= NPeffs
			global coms
			coms+= com
			#print len(trajec_mil), " ",  len(pos), " ", len(normals), " ", len(coms), " ", len(pEffs)
			#assert(len(trajec_mil) == len(pos) and len(normals) == len(pos) and len(normals) == len(coms) and len(coms) == len(pEffs))
			stat_data["num_success"] += 1
		else:
			print("TODO, NO CONTACT VARIATION, LINEAR INTERPOLATION REQUIRED")
	#~ except hpperr as e:		
		#~ print "hpperr failed at id " + str(i) , e.strerror
		#~ if (use_window == 0 and (len(configs) - 1) - (i + 2) > 0):
			#~ print "could not project com, trying to increase velocity "
			#~ try:
				#~ return step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction, optim_effectors, time_scale, useCOMConstraints, 1, verbose, draw, trackedEffectors = trackedEffectors)
			#~ except: 
				#~ if ((len(configs) - 1) - (i + 3) > 0):
					#~ print "could not project com, trying to increase velocity more "
					#~ step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction, optim_effectors, time_scale, useCOMConstraints, 2, verbose, draw,  trackedEffectors = trackedEffectors)		
		#~ else:
		#~ print "In hpperr and window != 0"
		#~ print "hpperr failed at id " + str(i) , e.strerror
		#~ stat_data["error_com_proj"] += 1
		#~ stat_data["num_errors"] += 1
		#~ errorid += [i]
		#~ fail+=1
	#~ except OptimError as e:
		#~ print "OptimError failed at id " + str(i) , e
		#~ stat_data["error_optim_fail"] += 1
		#~ stat_data["num_errors"] += 1
		#~ errorid += [i]
		#~ fail+=1
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
	#~ except Exception as e:
		#~ print e
		#~ if (use_window == 0 and (len(configs) - 1) - (i + 2) > 0):
			#~ print "could not project com, trying to increase velocity "
			#~ try:
				#~ return step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction, optim_effectors, time_scale, useCOMConstraints, 1, verbose, draw,  trackedEffectors = trackedEffectors)
			#~ except: 
				#~ print "faile twice"
				#~ if ((len(configs) - 1) - (i + 3) > 0):
					#~ print "could not project com, trying to increase velocity more "
					#~ step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction, optim_effectors, time_scale, useCOMConstraints, 2, verbose, draw,  trackedEffectors = trackedEffectors)		
		#~ else:
			#~ print "In Exception and window != 0"
			#~ stat_data["error_unknown"] += 1
			#~ stat_data["num_errors"] += 1
			#~ print e
			#~ errorid += [i]
			#~ fail+=1
	#~ except:
		#~ print "unknown"
		#~ if (use_window == 0 and (len(configs) - 1) - (i + 2) > 0):
			#~ print "could not project com, trying to increase velocity "
			#~ try:
				#~ return step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction, optim_effectors, time_scale, useCOMConstraints, 1, verbose, draw,  trackedEffectors = trackedEffectors)
			#~ except: 
				#~ if ((len(configs) - 1) - (i + 3) > 0):
					#~ print "could not project com, trying to increase velocity more "
					#~ step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction, optim_effectors, time_scale, useCOMConstraints, 2, verbose, draw,  trackedEffectors = trackedEffectors)		
		#~ else:
		#~ print "In unknown and window != 0"
		#~ stat_data["error_unknown"] += 1
		#~ stat_data["num_errors"] += 1
		#~ errorid += [i]
		#~ fail+=1
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
				pid, trajectory, timeelapsed, final_state =  solve_effector_RRT(fullBody, configs, i, True, friction, dt, times, False, optim, False, False, comC, True)
			else :
				pid, trajectory, timeelapsed, final_state =       solve_com_RRT(fullBody, configs, i, True, friction, dt, times, False, optim, False, False, comC, True)			
			__update_cwc_time(timeelapsed)	
			stat_data["num_success"] += 1
		else:
			print("TODO, NO CONTACT VARIATION, LINEAR INTERPOLATION REQUIRED")
	except hpperr as e:
		print("hpperr failed at id " + str(i) , e.strerror)
		stat_data["error_com_proj"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	except OptimError as e:
		print("OptimError failed at id " + str(i) , e.strerror)
		stat_data["error_optim_fail"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	except ValueError as e:
		print("ValueError failed at id " + str(i) , e)
		stat_data["error_unknown"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	except IndexError as e:
		print("IndexError failed at id " + str(i) , e)
		stat_data["error_unknown"] += 1
		stat_data["num_errors"] += 1
		errorid += [i]
		fail+=1
	except Exception as e:
		stat_data["error_unknown"] += 1
		stat_data["num_errors"] += 1
		print(e)
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

def __isDiff(P0, P1):
	return len(set(P0.keys()) - set(P1.keys())) != 0 or len(set(P1.keys()) - set(P0.keys()))

from pickle import dump
def compressData(data_array, filename):
	qs = [data['q'][:] for data in data_array]
	C =  [data['C'][:] for data in data_array]
	a = {}
	frameswitches = []
	for i in range(0,len(pos)):
		if i == 0 or __isDiff(pos[i], pos[i-1]):
			a = {}
			for effector in pos[i].keys():
				a[effector] = {'P' : pos[i][effector], 'N' : normals[i][effector]}
			frameswitches.append([i,a])
	res = {}
	res['Q'] = [data['q'][:] for data in data_array]
	res['C'] = [data['C'][:] for data in data_array]
	res['fly'] = pEffs
	res['frameswitches'] = frameswitches
	f1=open(filename+"_compressed", 'w+')
	dump(res, f1)
	f1.close()
	return res

def saveToPinocchio(filename):
	res = []
	for i, q_gep in enumerate(trajec_mil):
		#invert to pinocchio config:
		q = q_gep[:]
		quat_end = q[4:7]
		q[6] = q[3]
		q[3:6] = quat_end
		data = {'q':q, 'P' : pos[i], 'N' : normals[i], 'C' : coms [i], 'pEffs' : pEffs[i]}
		res += [data]
	f1=open(filename, 'w+')
	dump(res, f1)
	f1.close()
	return compressData(res,filename)
		
def clean():
	global res
	global trajec
	global trajec_mil
	global contacts
	global errorid
	global pos
	global normals
	global pEffs
	global coms
	res = []
	trajec = []
	trajec_mil = []
	contacts = []
	errorid = []
	pos = []
	normals = []
	pEffs = []
	coms = []

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
		print("ERROR: i_end < len_configs ", i_end, len(configs)-1)
		return # no point in trying optim, path was not fully computed
	for i in range(i_start, i_end):		
		step_profile(fullBody, configs, i, 0, limbsCOMConstraints,  friction, optim_effectors, time_scale, useCOMConstraints)
	stat_data["time_cwc"]["avg"] = stat_data["time_cwc"]["totaltime"] / float(stat_data["time_cwc"]["numiter"])
	write_stats(filename)

def saveAllData(fullBody, r, name):
	global trajec
	fullBody.exportAll(r, trajec, name)
	return saveToPinocchio(name)

def play_traj(fullBody,pp,frame_rate):
	global trajec
	return play_trajectory(fullBody,pp,trajec,frame_rate)

#~ fullBody.exportAll(r, trajec, 'darpa_hyq_t_var_04f_andrea');
#~ saveToPinocchio('darpa_hyq_t_var_04f_andrea')
