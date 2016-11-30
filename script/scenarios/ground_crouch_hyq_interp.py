import matplotlib
#~ matplotlib.use('Agg')
import matplotlib.pyplot as plt
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer

from numpy import array

from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"

import ground_crouch_hyq_path as tp
#~ import ground_crouch_hyq_path_bridge as tp

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



def addLimbDb(limbId, heuristicName, loadValues = True, disableEffectorCollision = False):
	fullBody.addLimbDatabase(str(db_dir+limbId+'.db'), limbId, heuristicName,loadValues, disableEffectorCollision)

#~ lLegId = 'lhleg'
#~ rarmId = 'rhleg'
#~ larmId = 'lfleg'
#~ 
#~ addLimbDb(rLegId, "static")
#~ addLimbDb(lLegId, "static")
#~ addLimbDb(rarmId, "static")
#~ addLimbDb(larmId, "static")

fullBody.addLimb(rLegId,rLeg,rfoot,offset,normal, legx, legy, nbSamples, "forward", 0.1, cType)

lLegId = 'lhleg'
lLeg = 'lh_haa_joint'
lfoot = 'lh_foot_joint'
fullBody.addLimb(lLegId,lLeg,lfoot,offset,normal, legx, legy, nbSamples, "backward", 0.05, cType)

rarmId = 'rhleg'
rarm = 'rh_haa_joint'
rHand = 'rh_foot_joint'
fullBody.addLimb(rarmId,rarm,rHand,offset,normal, legx, legy, nbSamples, "backward", 0.05, cType)
#~ 
larmId = 'lfleg'
larm = 'lf_haa_joint'
lHand = 'lf_foot_joint'
fullBody.addLimb(larmId,larm,lHand,offset,normal, legx, legy, nbSamples, "forward", 0.05, cType)

fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(rarmId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(larmId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)

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

configs = fullBody.interpolate(0.05,1,10, True) #hole 
#~ configs = fullBody.interpolate(0.01,1,10, True) #hole 
#~ configs = fullBody.interpolate(0.08,1,5) # bridge

r.loadObstacleModel ('hpp-rbprm-corba', "groundcrouch", "contact")
#~ fullBody.exportAll(r, configs, 'obstacle_hyq_robust_10');
i = 0;
r (configs[i]); i=i+1; i-1


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)


from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj

	
limbsCOMConstraints = { rLegId : {'file': "hyq/"+rLegId+"_com.ineq", 'effector' : rfoot},  
						lLegId : {'file': "hyq/"+lLegId+"_com.ineq", 'effector' : lfoot},  
						rarmId : {'file': "hyq/"+rarmId+"_com.ineq", 'effector' : rHand},  
						larmId : {'file': "hyq/"+larmId+"_com.ineq", 'effector' : lHand} }



def act(i, numOptim = 0, use_window = 0, friction = 0.3, optim_effectors = True, time_scale = 20,  verbose = False, draw = False, trackedEffectors = []):
	return step(fullBody, configs, i, numOptim, pp, limbsCOMConstraints, friction, optim_effectors = optim_effectors, time_scale = time_scale, useCOMConstraints = False, use_window = use_window,
	verbose = verbose, draw = draw, trackedEffectors = trackedEffectors)

def play(frame_rate = 1./24.):
	play_traj(fullBody,pp,frame_rate)
	
def saveAll(name):
	a = saveAllData(fullBody, r, name)
#~ saveAll ('hole_hyq_t_var_04f_andrea');
#~ fullBody.exportAll(r, configs, 'hole_hyq_t_var_04f_andrea_contact_planning');
#~ saveToPinocchio('obstacle_hyq_t_var_04f_andrea')



gui = r.client.gui
scene = "oddct"
r.client.gui.createScene(scene)
resolution = 0.03
i = 0
boxname = scene+"/b"+str(i)
gui.addBox(boxname,resolution,resolution,resolution, [1,1,1,1])
gui.applyConfiguration(boxname,[0,0,0,1,0,0,0])
gui.addSceneToWindow(scene,0)
gui.refresh()

import hpp.corbaserver.rbprm.tools.cwc_trajectory_helper as cwc_trajectory_helper
import time

def applycom():
	global gui
	global com
	c = fullBody.getCenterOfMass()
	gui.applyConfiguration(boxname,[c[0],c[1],0,1,0,0,0])
	gui.refresh()

def go(dt_framerate=1./24.):
	path_player = pp
	configs = cwc_trajectory_helper.trajec
	for q in configs:
		start = time.time()
		pp.publisher.robotConfig = q
		pp.publisher.publishRobots ()
		elapsed = time.time() - start  
		applycom()
		if elapsed < dt_framerate :
			time.sleep(dt_framerate-elapsed)

from numpy import min, max
for i in range(11,13):
	data, cones = act(i,verbose=True, use_window=1, numOptim=5, optim_effectors=True, draw=False);go()
#~ for j in range(8,10):
	#~ data, cones = act(j,verbose=True, use_window=0, numOptim=5, optim_effectors=True, draw=False)
	w =  array(data['w'])
	numphase = w.shape[0]/6
	print "numphase",  numphase
	for k in range(0,numphase-1):
		print "cones 1 2 3", k, (cones[0].dot(w[6*k:6*k+6])<0.0001).all() , (cones[1].dot(w[6*k:6*k+6])<0.0001).all(), (cones[2].dot(w[6*k:6*k+6])<0.0001).all(), max(cones[1].dot(w[6*k:6*k+6]))

for i in range(17,35):
	data, cones = act(i,verbose=True, use_window=1, numOptim=5, optim_effectors=True, draw=False);go()
#~ for j in range(8,10):
	#~ data, cones = act(j,verbose=True, use_window=0, numOptim=5, optim_effectors=True, draw=False)
	w =  array(data['w'])
	numphase = w.shape[0]/6
	print "numphase",  numphase
	for k in range(0,numphase-1):
		print "cones 1 2 3", k, (cones[0].dot(w[6*k:6*k+6])<0.0001).all() , (cones[1].dot(w[6*k:6*k+6])<0.0001).all(), (cones[2].dot(w[6*k:6*k+6])<0.0001).all(), max(cones[1].dot(w[6*k:6*k+6]))
	saveAll("test"+str(i))
	
#~ for j in range(19,20):
	#~ data, cones = act(j,verbose=True, use_window=0, friction = 0.3, numOptim=5, optim_effectors=True, draw=False)
	#~ w =  array(data['w'])
	#~ numphase = w.shape[0]/6
	#~ print "numphase",  numphase
	#~ for k in range(0,numphase-1):
		#~ print "cones 1 2 3", k, (cones[0].dot(w[6*k:6*k+6])<0.0001).all() , (cones[1].dot(w[6*k:6*k+6])<0.0001).all(), (cones[2].dot(w[6*k:6*k+6])<0.0001).all(), max(cones[1].dot(w[6*k:6*k+6]))
	
saveAll("test")
	
	
