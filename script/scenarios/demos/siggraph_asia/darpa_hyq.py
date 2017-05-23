#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer
#reference pose for hyq
from hyq_ref_pose import hyq_ref

#calling script darpa_hyq_path to compute root path
import darpa_hyq_path as tp

from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"


from hpp.corbaserver import Client


packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"

#  Information to retrieve urdf and srdf files.
urdfName = "hyq"
urdfSuffix = ""
srdfSuffix = ""

#  This time we load the full body model of HyQ
fullBody = FullBody () 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,6, -1, 1, 0.3, 4])

#  Setting a number of sample configurations used
nbSamples = 20000

ps = tp.ProblemSolver(fullBody)
r = tp.Viewer (ps, viewerClient=tp.r.client)

rootName = 'base_joint_xyz'

cType = "_3_DOF"
rLegId = 'rfleg'
rLeg = 'rf_haa_joint'
rfoot = 'rf_foot_joint'
offset = [0.,-0.021,0.]
normal = [0,1,0]
legx = 0.02; legy = 0.02

def addLimbDb(limbId, heuristicName, loadValues = True, disableEffectorCollision = False):
	fullBody.addLimbDatabase(str(db_dir+limbId+'.db'), limbId, heuristicName,loadValues, disableEffectorCollision)

fullBody.addLimb(rLegId,rLeg,rfoot,offset,normal, legx, legy, nbSamples, "jointlimits", 0.1, cType)

lLegId = 'lhleg'
lLeg = 'lh_haa_joint'
lfoot = 'lh_foot_joint'
fullBody.addLimb(lLegId,lLeg,lfoot,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)
#~ 
rarmId = 'rhleg'
rarm = 'rh_haa_joint'
rHand = 'rh_foot_joint'
fullBody.addLimb(rarmId,rarm,rHand,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)

larmId = 'lfleg'
larm = 'lf_haa_joint'
lHand = 'lf_foot_joint'
fullBody.addLimb(larmId,larm,lHand,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)

fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(rarmId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(larmId, "jointLimitsDistance", True)

#~ q_init = hyq_ref[:]; q_init[0:7] = tp.q_init[0:7]; 
#~ q_goal = hyq_ref[:]; q_goal[0:7] = tp.q_goal[0:7]; 
q_init = hyq_ref[:]; q_init[0:7] = tp.q_init[0:7]; q_init[2]=hyq_ref[2]+0.02
q_goal = hyq_ref[:]; q_goal[0:7] = tp.q_goal[0:7]; q_init[2]=hyq_ref[2]+0.02

# Randomly generating a contact configuration at q_init
#~ fullBody.setCurrentConfig (q_init)
#~ q_init = fullBody.generateContacts(q_init, [0,0,1])

# Randomly generating a contact configuration at q_end
#~ fullBody.setCurrentConfig (q_goal)
#~ q_goal = fullBody.generateContacts(q_goal, [0,0,1])

# specifying the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[rLegId,lLegId,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId,rarmId])
#~ fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId])


r(q_init)
configs = []


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)


from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj

	
	
#~ limbsCOMConstraints = { rLegId : {'file': "hyq/"+rLegId+"_com.ineq", 'effector' : rfoot},  
						#~ lLegId : {'file': "hyq/"+lLegId+"_com.ineq", 'effector' : lfoot},  
						#~ rarmId : {'file': "hyq/"+rarmId+"_com.ineq", 'effector' : rHand},  
						#~ larmId : {'file': "hyq/"+larmId+"_com.ineq", 'effector' : lHand} }
						
limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : rfoot},  
						lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : lfoot},
						rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand},
						larmId : {'file': "hrp2/LA_com.ineq", 'effector' : lHand} }


def act(i, numOptim = 0, use_window = 0, friction = 0.5, optim_effectors = True, verbose = False, draw = False):
	return step(fullBody, configs, i, numOptim, pp, limbsCOMConstraints, 0.4, optim_effectors = optim_effectors, time_scale = 20., useCOMConstraints = False, use_window = use_window,
	verbose = verbose, draw = draw)

def play(frame_rate = 1./24.):
	play_traj(fullBody,pp,frame_rate)
	

import time

#DEMO METHODS

def initConfig():
	r.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	r(q_init)
	
def endConfig():
	r.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	r(q_goal)
	

def rootPath():
	r.client.gui.setVisibility("hyq", "OFF")
	tp.cl.problem.selectProblem("rbprm_path")
	tp.r.client.gui.setVisibility("toto", "OFF")
	r.client.gui.setVisibility("hyq", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "ON")
	tp.pp(0)
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	r.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	
def genPlan(stepsize=0.06):
	tp.cl.problem.selectProblem("default")
	r.client.gui.setVisibility("hyq", "ON")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	global configs
	start = time.clock() 
	configs = fullBody.interpolate(stepsize, 5, 5, True)
	end = time.clock() 
	print "Contact plan generated in " + str(end-start) + "seconds"
	
def contactPlan(step = 0.5):
	r.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	for i in range(0,len(configs)):
		r(configs[i]);
		time.sleep(step)		
		
		
def a():
	print "initial configuration"
	initConfig()
		
def b():
	print "end configuration"
	endConfig()
		
def c():
	print "displaying root path"
	rootPath()
	
def d(step=0.06):
	print "computing contact plan"
	genPlan(step)
	
def e(step = 0.5):
	print "displaying contact plan"
	contactPlan(step)
	
print "Root path generated in " + str(tp.t) + " ms."

#~ d();e()
d(0.1);e(0.01)


#~ configs = d(0.005); e()
qs = configs
fb = fullBody
ttp = tp
from bezier_traj import *
init_bezier_traj(fb, r, pp, qs, limbsCOMConstraints)
#~ AFTER loading obstacles
configs = qs
fullBody = fb
tp = ttp



