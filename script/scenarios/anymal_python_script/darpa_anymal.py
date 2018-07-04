#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer

#calling script darpa_hyq_path to compute root path
import darpa_anymal_path as tp

from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"


from hpp.corbaserver import Client

packageName = "anymal_description"
meshPackageName = "anymal_description"
rootJointType = "freeflyer"

#  Information to retrieve urdf and srdf files.
urdfName = "anymal"
urdfSuffix = ""
srdfSuffix = ""

#  This time we load the full body model of HyQ
fullBody = FullBody () 
print("alive -1")
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
print("alive 0")
fullBody.setJointBounds ("base_joint_xyz", [-2.5,5, -1, 1, 0.3, 4]) #originally [-2,5, -1, 1, 0.3, 4]
print("alive 1")

#  Setting a number of sample configurations used
nbSamples = 50000 #used to be 20000 

ps = tp.ProblemSolver(fullBody)
r = tp.Viewer (ps, viewerClient=tp.r.client)

rootName = 'base_joint_xyz'

cType = "_3_DOF"
rLegId = 'rfleg'
rLeg = 'RF_HAA'
rfoot = 'RF_ADAPTER_TO_FOOT'
offset = [0.,-0.031,0.] #originally [0.,-0.031,0.]
normal = [0,1,0] #hyq needs [0,1,0], also for anymal
legx = 0.03; legy = 0.03

def addLimbDb(limbId, heuristicName, loadValues = True, disableEffectorCollision = False):
	fullBody.addLimbDatabase(str(db_dir+limbId+'.db'), limbId, heuristicName,loadValues, disableEffectorCollision)

fullBody.addLimb(rLegId,rLeg,rfoot,offset,normal, legx, legy, nbSamples, "forward", 0.1, cType)

lLegId = 'lhleg'
lLeg = 'LH_HAA'
lfoot = 'LH_ADAPTER_TO_FOOT'
fullBody.addLimb(lLegId,lLeg,lfoot,offset,normal, legx, legy, nbSamples, "random", 0.1, cType)
#~ 
rarmId = 'rhleg'
rarm = 'RH_HAA'
rHand = 'RH_ADAPTER_TO_FOOT'
fullBody.addLimb(rarmId,rarm,rHand,offset,normal, legx, legy, nbSamples, "random", 0.1, cType)

larmId = 'lfleg'
larm = 'LF_HAA'
lHand = 'LF_ADAPTER_TO_FOOT'
fullBody.addLimb(larmId,larm,lHand,offset,normal, legx, legy, nbSamples, "forward", 0.1, cType)

#~ fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(rarmId, "manipulability", True)
#~ fullBody.runLimbSampleAnalysis(larmId, "manipulability", True)

q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.q_goal[0:7]

# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
q_init = fullBody.generateContacts(q_init, [0,0,1]) #originally [0,0,1]
#q_init[7:] = [0.0, 0.0, 0.0, \
#			  0.0, 0.0, 0.0, \
#			  0.0, 0.0, 0.0, \
#			  0.0, 0.0, 0.0] 
print "q_init"
print q_init

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])

# specifying the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[rLegId,lLegId,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId])
#~ fullBody.setStartState(q_init,[rLegId,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId,larmId])


r(q_init)
#~ configs = fullBody.interpolate(0.12, 10, 10, True)
configs = []
#~ r.loadObstacleModel ('hpp-rbprm-corba', "darpa", "contact")

# calling draw with increasing i will display the sequence
i = 0;
#~ fullBody.draw(configs[i],r); i=i+1; i-1


from hpp.gepetto import PathPlayer
#pp = PathPlayer (fullBody.client.basic, r)


import time

#DEMO METHODS

def initConfig():
	r.client.gui.setVisibility("anymal", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	r(q_init)
	
def endConfig():
	r.client.gui.setVisibility("anymal", "ON")
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
	r.client.gui.setVisibility("anymal", "ON")
	tp.cl.problem.selectProblem("default")
	
def genPlan(step = 0.06, robustness = 18, cut = True):
	tp.cl.problem.selectProblem("default")
	r.client.gui.setVisibility("anymal", "ON")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	global configs
	start = time.clock()         
	configs = fullBody.interpolate(step, 10, robustness, filterStates = cut, testReachability=False, quasiStatic=False)
	#~ configs = fullBody.interpolate(step, 10, robustness, cut) #originally (0.12, 10, 10, True)
	end = time.clock() 
	print "Last element in interpolated configs"
	print configs[-1]
	print "Contact plan generated in " + str(end-start) + "seconds"
	r(configs[-1])
	
def contactPlan(t=0.5):
	r.client.gui.setVisibility("anymal", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hyq_trunk_large", "OFF")
	for i in range(1,len(configs)):
		r(configs[i]);
		time.sleep(t)	
	
		
		
def a():
	print "initial configuration"
	initConfig()
		
def b():
	print "end configuration"
	endConfig()
		
def c():
	print "displaying root path"
	rootPath()
	
def d(step = 0.06, robustness = 18, cut = True):
	print "computing contact plan"
	genPlan(step, cut)
	
def e(t = 0.2):
	print "displaying contact plan"
	contactPlan(t)
	
print "Root path generated in " + str(tp.t) + " ms."

d(0.005,20)


#print "Chcek collisions"
#print fullBody.isConfigValid(fullBody.getCurrentConfig())
#print " "

#print "Check candidate configurations"
#print fullBody.getNumSamples(rLegId)
#print fullBody.getNumSamples(lLegId)
#print fullBody.getNumSamples(larmId)
#print fullBody.getNumSamples(rarmId)

#print "Visualise Voxels"
#print "Current Configuration"
#print fullBody.getCurrentConfig()
#fullBody.createOctreeBoxes(r.client.gui,0,rLegId,fullBody.getCurrentConfig())
#fullBody.createOctreeBoxes(r.client.gui,0,lLegId,fullBody.getCurrentConfig())
#fullBody.createOctreeBoxes(r.client.gui,0,larmId,fullBody.getCurrentConfig())
#fullBody.createOctreeBoxes(r.client.gui,0,rarmId,fullBody.getCurrentConfig())
