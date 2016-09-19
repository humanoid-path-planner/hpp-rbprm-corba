
from hpp.corbaserver.rbprm.tools.cwc_trajectory import *
from hpp.corbaserver.rbprm.tools.path_to_trajectory import *

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

from hpp import Error as hpperr
import sys
numerror = 0
def step(fullBody, configs, i, optim, pp, limbsCOMConstraints,  friction = 0.5, optim_effectors = True, time_scale = 20., useCOMConstraints = False):
	global numerror
	global errorid
	fail = 0
	try:
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
		print 'time per path', times
		print 'dt', dt
		if(distance > 0.0001):
			if(useCOMConstraints):
				comC = limbsCOMConstraints
			else:
				comC = None
			if(optim_effectors):
				pid, trajectory =  solve_effector_RRT(fullBody, configs, i, True, friction, dt, times, False, optim, False, False, comC)
			else :
				pid, trajectory =       solve_com_RRT(fullBody, configs, i, True, friction, dt, times, False, optim, False, False, comC)
			displayComPath(pp, pid)
			#~ pp(pid)
			global res
			res = res + [pid]
			global trajec
			global trajec_mil
			trajec = trajec + gen_trajectory_to_play(fullBody, pp, trajectory, times)
			trajec_mil = trajec_mil + gen_trajectory_to_play(fullBody, pp, trajectory, times, 1./1000.)
			global contacts
			contacts += gencontactsPerFrame(fullBody, i, limbsCOMConstraints, pp, trajectory, times, 1./1000.)	
			Ps, Ns = genPandNperFrame(fullBody, i, pp, trajectory, times, 1./1000.)	
			global pos
			pos += Ps
			global normals
			normals+= Ns
			assert(len(contacts) == len(trajec_mil) and len(contacts) == len(pos) and len(normals) == len(pos))
		else:
			print "TODO, NO CONTACT VARIATION, LINEAR INTERPOLATION REQUIRED"
	except hpperr as e:
		print "hpperr failed at id " + str(i) , e.strerror
		numerror+=1
		errorid += [i]
		fail+=1
	except ValueError as e:
		print "ValueError failed at id " + str(i) , e
		numerror+=1
		errorid += [i]
		fail+=1
	except IndexError as e:
		print "IndexError failed at id " + str(i) , e
		numerror+=1
		errorid += [i]
		fail+=1
	except Exception as e:
		print e
		numerror+=1
		errorid += [i]
		fail+=1
	except:
		numerror+=1
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
	
def stats():
	pass

def saveAllData(fullBody, r, name):
	fullBody.exportAll(r, trajec, name)
	saveToPinocchio(name)

def play_traj(fullBody,pp,frame_rate):
	return play_trajectory(fullBody,pp,frame_rate)

#~ fullBody.exportAll(r, trajec, 'darpa_hyq_t_var_04f_andrea');
#~ saveToPinocchio('darpa_hyq_t_var_04f_andrea')
