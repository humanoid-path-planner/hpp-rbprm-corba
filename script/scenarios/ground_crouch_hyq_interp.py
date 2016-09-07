from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer

import ground_crouch_hyq_path as tp

packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hyq"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-6,5, -4, 4, 0.6, 2])

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

nbSamples = 20000

ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps)

rootName = 'base_joint_xyz'

#  Creating limbs
# cType is "_3_DOF": positional constraint, but no rotation (contacts are punctual)
cType = "_3_DOF"
# string identifying the limb
rLegId = 'rfleg'
# First joint of the limb, as in urdf file
rLeg = 'rf_haa_joint'
# Last joint of the limb, as in urdf file
rfoot = 'rf_foot_joint'
# Specifying the distance between last joint and contact surface
offset = [0.,-0.021,0.]
# Specifying the contact surface direction when the limb is in rest pose
normal = [0,1,0]
# Specifying the rectangular contact surface length
legx = 0.02; legy = 0.02
# remaining parameters are the chosen heuristic (here, manipulability), and the resolution of the octree (here, 10 cm).
fullBody.addLimb(rLegId,rLeg,rfoot,offset,normal, legx, legy, nbSamples, "forward", 0.1, cType)

lLegId = 'lhleg'
lLeg = 'lh_haa_joint'
lfoot = 'lh_foot_joint'
fullBody.addLimb(lLegId,lLeg,lfoot,offset,normal, legx, legy, nbSamples, "backward", 0.05, cType)

rarmId = 'rhleg'
rarm = 'rh_haa_joint'
rHand = 'rh_foot_joint'
fullBody.addLimb(rarmId,rarm,rHand,offset,normal, legx, legy, nbSamples, "backward", 0.05, cType)

larmId = 'lfleg'
larm = 'lf_haa_joint'
lHand = 'lf_foot_joint'
fullBody.addLimb(larmId,larm,lHand,offset,normal, legx, legy, nbSamples, "forward", 0.05, cType)

q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.q_goal[0:7]

fullBody.setCurrentConfig (q_init)
q_init = fullBody.generateContacts(q_init, [0,0,1])
q_0 = fullBody.getCurrentConfig(); 

fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])

fullBody.setStartState(q_init,[])
fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId,larmId])

r(q_init)

configs = fullBody.interpolate(0.1,1,10)

r.loadObstacleModel ('hpp-rbprm-corba', "groundcrouch", "contact")
#~ fullBody.exportAll(r, configs, 'obstacle_hyq_robust_10');
i = 0;
r (configs[i]); i=i+1; i-1


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)


from hpp.corbaserver.rbprm.tools.cwc_trajectory import *
from hpp.corbaserver.rbprm.tools.path_to_trajectory import *

def displayComPath(pathId,color=[0.,0.75,0.15,0.9]) :
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
	
limbsCOMConstraints = { rLegId : {'file': "hyq/"+rLegId+"_com.ineq", 'effector' : rfoot},  
					lLegId : {'file': "hyq/"+lLegId+"_com.ineq", 'effector' : lfoot},  
					rarmId : {'file': "hyq/"+rarmId+"_com.ineq", 'effector' : rHand},  
					larmId : {'file': "hyq/"+larmId+"_com.ineq", 'effector' : lHand} }


	

res = []
trajec = []
contacts = []
pos = []
normals = []
errorid = []

def getContactPerPhase(stateid):
	contacts = [[],[],[]]
	global limbsCOMConstraints
	for k, v in limbsCOMConstraints.iteritems():
		if(fullBody.isLimbInContact(k, stateid)):
			contacts[0]+=[v['effector']]
		if(fullBody.isLimbInContactIntermediary(k, stateid)):
			contacts[1]+=[v['effector']]
		if(fullBody.isLimbInContact(k, stateid+1)):
			contacts[2]+=[v['effector']]
	return contacts

def gencontactsPerFrame(stateid, path_ids, total_time_per_path, dt_framerate=1./24.):
	contactsPerPhase = getContactPerPhase(stateid)
	config_size = len(fullBody.getCurrentConfig())
	interpassed = False
	res = []
	for path_id in path_ids:		
		length = pp.client.problem.pathLength (path_id)
		num_frames_required = total_time_per_path / dt_framerate
		dt = float(length) / num_frames_required
		dt_finals = [dt*i for i in range(int(num_frames_required))] + [1]		
		config_size_path = len(pp.client.problem.configAtParam (path_id, 0))
		if(config_size_path > config_size):
			interpassed = True
			res+= [contactsPerPhase[1] for t in dt_finals]
		elif interpassed:			
			res+= [contactsPerPhase[2] for t in dt_finals]
		else:
			res+= [contactsPerPhase[0] for t in dt_finals]
	return res

def genPandNperFrame(stateid, path_ids, total_time_per_path, dt_framerate=1./24.):
	p, N= fullBody.computeContactPoints(stateid)
	config_size = len(fullBody.getCurrentConfig())
	interpassed = False
	pRes = []
	nRes = []
	for path_id in path_ids:		
		length = pp.client.problem.pathLength (path_id)
		num_frames_required = total_time_per_path / dt_framerate
		dt = float(length) / num_frames_required
		dt_finals = [dt*i for i in range(int(num_frames_required))] + [1]		
		config_size_path = len(pp.client.problem.configAtParam (path_id, 0))
		if(config_size_path > config_size):
			interpassed = True
			pRes+= [p[1] for t in dt_finals]
			nRes+= [N[1] for t in dt_finals]
		elif interpassed:			
			pRes+= [p[2] for t in dt_finals]
			nRes+= [N[2] for t in dt_finals]
		else:
			pRes+= [p[0] for t in dt_finals]
			nRes+= [N[0] for t in dt_finals]
	return pRes, nRes

from hpp import Error as hpperr
import sys
numerror = 0
def act(i, optim):
	global numerror
	global errorid
	fail = 0
	try:
		total_time_per_path = 1
		pid, trajectory = solve_com_RRT(fullBody, configs, i, True, 0.5, 0.2, total_time_per_path, False, optim, False, False)
		displayComPath(pid)
		#~ pp(pid)
		global res
		res = res + [pid]
		global trajec
		trajec = trajec + gen_trajectory_to_play(fullBody, pp, trajectory, total_time_per_path)
		global contacts
		contacts += gencontactsPerFrame(i, trajectory, total_time_per_path)	
		Ps, Ns = genPandNperFrame(i, trajectory, total_time_per_path)	
		global pos
		pos += Ps
		global normals
		normals+= Ns
		assert(len(contacts) == len(trajec) and len(contacts) == len(pos) and len(normals) == len(pos))
	except hpperr as e:
		print "failed at id " + str(i) , e.strerror
		numerror+=1
		errorid += [i]
		fail+=1
	except ValueError as e:
		print "failed at id " + str(i) , e
		numerror+=1
		errorid += [i]
		fail+=1
	except IndexError as e:
		print "failed at id " + str(i) , e
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
#~ play_trajectory(fullBody,pp,trajec)		

from pickle import dump
def saveToPinocchio(filename):
	res = []
	for i, q_gep in enumerate(trajec):
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
	global contacts
	global errorid
	global pos
	global normals
	res = []
	trajec = []
	contacts = []
	errorid = []
	pos = []
	normals = []

#~ saveToPinocchio('darpahyq_andrea')

