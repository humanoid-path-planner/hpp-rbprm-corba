#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.hyq_contact6D import Robot
from hpp.corbaserver.problem_solver import ProblemSolver
from hpp.gepetto import Viewer

#calling script darpa_hyq_path to compute root path
import darpa_hyq_path as tp

from os import environ
ins_dir = environ['DEVEL_HPP_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"


from hpp.corbaserver import Client



#  This time we load the full body model of HyQ
fullBody = Robot () 
fullBody.setJointBounds ("root_joint", [-2,5, -1, 1, 0.3, 4])
#~ fullBody.client.robot.setDimensionExtraConfigSpace(6)
#~ fullBody.client.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,0,0,0,0,0,0])

#  Setting a number of sample configurations used
nbSamples = 10000

ps = tp.ProblemSolver(fullBody)
v = tp.Viewer (ps, viewerClient=tp.v.client)

rootName = 'base_joint_xyz'
cType = "_6_DOF"

def addLimbDb(limbId, heuristicName, loadValues = True, disableEffectorCollision = False):
	fullBody.addLimbDatabase(str(db_dir+limbId+'.db'), limbId, heuristicName,loadValues, disableEffectorCollision)

fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.offset,fullBody.normal, fullBody.legx, fullBody.legy, nbSamples, "forward", 0.1, cType)
fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.offset,fullBody.normal, fullBody.legx, fullBody.legy, nbSamples, "forward", 0.05, cType)
fullBody.addLimb(fullBody.rArmId,fullBody.rarm,fullBody.rhand,fullBody.offset,fullBody.normal, fullBody.legx, fullBody.legy, nbSamples, "forward", 0.05, cType)
fullBody.addLimb(fullBody.lArmId,fullBody.larm,fullBody.lhand,fullBody.offset,fullBody.normal, fullBody.legx, fullBody.legy, nbSamples, "random", 0.05, cType)

#~ fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(rarmId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(larmId, "jointLimitsDistance", True)

q_init=fullBody.referenceConfig[::] ; q_init[0:7] = tp.q_init[0:7]; 
q_goal = fullBody.referenceConfig[::]; q_goal[0:7] = tp.q_goal[0:7]; 


q_init = fullBody.generateContacts(q_init, [0,0,1])
q_goal = fullBody.generateContacts(q_goal, [0,0,1])

# specifying the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[fullBody.rLegId,fullBody.lArmId,fullBody.lLegId,fullBody.rArmId])
fullBody.setEndState(q_goal,[fullBody.rLegId,fullBody.lArmId,fullBody.lLegId,fullBody.rArmId])



v(q_init)
configs = []


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client, v)


import time

#DEMO METHODS

def initConfig():
	v.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	tp.v.client.gui.setVisibility("toto", "OFF")
	tp.v.client.gui.setVisibility("hyq_trunk_large", "OFF")
	v(q_init)
	
def endConfig():
	v.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	tp.v.client.gui.setVisibility("toto", "OFF")
	tp.v.client.gui.setVisibility("hyq_trunk_large", "OFF")
	v(q_goal)
	

def rootPath():
	v.client.gui.setVisibility("hyq", "OFF")
	tp.cl.problem.selectProblem("rbprm_path")
	tp.v.client.gui.setVisibility("toto", "OFF")
	v.client.gui.setVisibility("hyq", "OFF")
	tp.v.client.gui.setVisibility("hyq_trunk_large", "ON")
	tp.pp(0)
	tp.v.client.gui.setVisibility("hyq_trunk_large", "OFF")
	v.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	
def genPlan(stepsize=0.06):
	tp.cl.problem.selectProblem("default")
	v.client.gui.setVisibility("hyq", "ON")
	tp.v.client.gui.setVisibility("toto", "OFF")
	tp.v.client.gui.setVisibility("hyq_trunk_large", "OFF")
	global configs
	start = time.time() 
	configs = fullBody.interpolate(stepsize, 5, 0., filterStates = True,testReachability = False)
	end = time.time() 
	print("Contact plan generated in " + str(end-start) + "seconds")
	
def contactPlan(step = 0.5):
	v.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	tp.v.client.gui.setVisibility("toto", "OFF")
	tp.v.client.gui.setVisibility("hyq_trunk_large", "OFF")
	for i in range(0,len(configs)):
		v(configs[i]);
		time.sleep(step)
                		
		
print("Root path generated in " + str(tp.t) + " ms.")

genPlan(0.01);contactPlan(0.01)
