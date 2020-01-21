from hpp.corbaserver.rbprm.hrp2 import Robot
from hpp.gepetto import Viewer

from . import stair_bauzil_hrp2_path as tp
import time


fullBody = Robot ()
fullBody.setJointBounds ("root_joint", [-0.135,2, -1, 1, 0, 2.2])


ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps, viewerClient=tp.r.client)

cType = "_6_DOF"
#~ AFTER loading obstacles

fullBody.addLimb(fullBody.rLegId,fullBody.rLeg,'',fullBody.rLegOffset,fullBody.rLegNormal, fullBody.rLegx, fullBody.rLegy, 10000, "manipulability", 0.2, cType)

fullBody.addLimb(fullBody.lLegId,fullBody.lLeg,'',fullBody.lLegOffset,fullBody.rLegNormal, fullBody.lLegx, fullBody.lLegy, 10000, "manipulability", 0.2, cType)

fullBody.addLimb(fullBody.rarmId,fullBody.rarm,fullBody.rHand,fullBody.rArmOffset,fullBody.rArmNormal, fullBody.rArmx, fullBody.rArmy, 10000, "manipulability", 0.1, "_6_DOF", True)

#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, "manipulability", 0.05, "_6_DOF", True)

rKneeId = '0RKnee'
rLeg = 'RLEG_JOINT0'
rKnee = 'RLEG_JOINT3'
rLegOffset = [0.105,0.055,0.017]
rLegNormal = [-1,0,0]
rLegx = 0.05; rLegy = 0.05
#~ fullBody.addLimb(rKneeId, rLeg,rKnee,rLegOffset,rLegNormal, rLegx, rLegy, 10000, 0.01)
#~ 
lKneeId = '1LKnee'
lLeg = 'LLEG_JOINT0'
lKnee = 'LLEG_JOINT3'
lLegOffset = [0.105,0.055,0.017]
lLegNormal = [-1,0,0]
lLegx = 0.05; lLegy = 0.05
#~ fullBody.addLimb(lKneeId,lLeg,lKnee,lLegOffset,lLegNormal, lLegx, lLegy, 10000, 0.01)
 #~ 

#~ fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])

q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.q_goal[0:7]

q_init[:3] = [0.1, -0.82, 0.648702]
q_init[7:] = [ 0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # LARM       11-17
        0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # RARM       18-24
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        ];

r (q_init)

fullBody.setCurrentConfig (q_init)
#~ q_init =  [
        #~ 0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0,                         	 # Free flyer 0-6
        #~ 0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        #~ 0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # LARM       11-17
        #~ 0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # RARM       18-24
        #~ 0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        #~ 0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        #~ ]; r (q_init)

fullBody.setCurrentConfig (q_goal)
#~ r(q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1], robustnessThreshold = 0.)
r(q_goal)

fullBody.setStartState(q_init,[rLegId,lLegId]) #,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])#,rarmId,larmId])
configs = fullBody.interpolate(0.1, robustnessTreshold = 5) #TODO DEBUG
#~ i = 0;
#~ fullBody.draw(configs[i],r); i=i+1; i-1

r.loadObstacleModel ('hpp-rbprm-corba', "stair_bauzil", "contact")
#~ fullBody.exportAll(r, configs, 'stair_bauzil_hrp2_robust_2');
#~ fullBody.client.basic.robot.setJointConfig('LLEG_JOINT0',[-1])
#~ q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.draw(q_0,r);
#~ print(fullBody.client.rbprm.rbprm.getOctreeTransform(rarmId, q_0))
#~ 
#~ 
#~ fullBody.client.basic.robot.setJointConfig('LLEG_JOINT0',[1])
#~ q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.draw(q_0,r);
#~ print(fullBody.client.rbprm.rbprm.getOctreeTransform(rarmId, q_0))
#~ q_init = fullBody.generateContacts(q_init, [0,0,-1]); r (q_init)

#~ f1 = open("secondchoice","w+")
#~ f1 = open("hrp2_stair_not_robust_configs","w+")
#~ f1.write(str(configs))
#~ f1.close()

limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},  
						lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'},
						rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand} }
						#~ larmId : {'file': "hrp2/LA_com.ineq", 'effector' : lHand} }

#~ fullBody.limbRRTFromRootPath(0,len(configs)-1,0,2)
from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client, r)

def act(i, numOptim = 0, use_window = 0, friction = 0.5, optim_effectors = True, verbose = False, draw = False):
	return step(fullBody, configs, i, numOptim, pp, limbsCOMConstraints, 0.4, optim_effectors = optim_effectors, time_scale = 20., useCOMConstraints = True, use_window = use_window,
	verbose = verbose, draw = draw)

def play(frame_rate = 1./24.):
	play_traj(fullBody,pp,frame_rate)
	
def saveAll(name):
	saveAllData(fullBody, r, name)
	

def initConfig():
	r.client.gui.setVisibility("hrp2_14", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
	r(q_init)
	
def endConfig():
	r.client.gui.setVisibility("hrp2_14", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
	r(q_goal)
	

def rootPath():
	tp.cl.problem.selectProblem("rbprm_path")
	r.client.gui.setVisibility("hrp2_14", "OFF")
	tp.r.client.gui.setVisibility("toto", "OFF")
	r.client.gui.setVisibility("hyq", "OFF")
	r.client.gui.setVisibility("hrp2_trunk_flexible", "ON")
	tp.pp(0)
	r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
	r.client.gui.setVisibility("hyq", "ON")
	tp.cl.problem.selectProblem("default")
	
def genPlan():
	r.client.gui.setVisibility("hrp2_14", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
	global configs
	start = time.clock() 
	configs = configs = fullBody.interpolate(0.1, True)
	end = time.clock() 
	print("Contact plan generated in " + str(end-start) + "seconds")
	
def contactPlan():
	tp.cl.problem.selectProblem("default")
	r.client.gui.setVisibility("hrp2_14", "ON")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
	for i in range(0,len(configs)-1):
		r(configs[i]);
		time.sleep(0.5)		

def d():
	genPlan()
		
def e():
	contactPlan()

#~ fullBody.limbRRTFromRootPath(0,len(configs)-1,0,2)
from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client, r)

def act(i, numOptim = 0, use_window = 0, friction = 0.5, optim_effectors = False, verbose = False, draw = False, trackedEffectors = []):
	return step(fullBody, configs, i, numOptim, pp, limbsCOMConstraints, 0.4, optim_effectors = optim_effectors, time_scale = 20., useCOMConstraints = False, use_window = use_window,
	verbose = verbose, draw = draw, trackedEffectors = trackedEffectors)

def play(frame_rate = 1./24.):
	play_traj(fullBody,pp,frame_rate)
	
def saveAll(name):
	saveAllData(fullBody, r, name)
print("Root path generated in " + str(tp.t) + " ms.")
	
print("go") 
#~ genPlan()

from hpp.corbaserver.rbprm.rbprmstate import *

#~ com = fullBody.client.basic.robot.getCenterOfMass
s = None
def d1():
        global s
        s = State(fullBody,q = q_init, limbsIncontact = [rLegId])

        a = s.q()
        a[2]=a[2]-0.1
        s.setQ(a)
        return s.projectToCOM([0.01,0.,0.], maxNumSample = 0)
        
def d2():
        global s
        s = State(fullBody,q = q_init, limbsIncontact = [rLegId, lLegId])

        a = s.q()
        a[2]=a[2]-0.1
        #~ a[0]=a[0]+0.05
        s.setQ(a)
        return s.projectToCOM([0.01,0.,0.], maxNumSample = 0)
        
def d3():
        global s
        s = State(fullBody,q = q_init, limbsIncontact = [rarmId])

        a = s.q()
        a[2]=a[2]+0.01
        s.setQ(a)
        return s.projectToCOM([0.01,0.,0.], maxNumSample = 0)
        
from hpp.corbaserver.rbprm.state_alg  import addNewContact, isContactReachable, closestTransform, removeContact, addNewContactIfReachable, projectToFeasibleCom
from .geom import  *


def dist(q0,q1):
    #~ return norm(array(q0[7:]) - array(q1[7:]) )
    return norm(array(q0[7:]) - array(q1[7:]) )

def distq_ref(q0):
    return lambda s: dist(s.q(),q0) 

a = computeAffordanceCentroids(tp.afftool, ["Support"]) 
def computeNext(state, limb, projectToCom = False, max_num_samples = 10, mu = 0.6):
    global a
    t1 = time.clock()
    #~ candidates = [el for el in a if isContactReachable(state, limb, el[0], el[1], limbsCOMConstraints)[0] ]
    #~ print "num candidates", len(candidates)
    #~ t3 = time.clock()
    results = [addNewContactIfReachable(state, limb, el[0], el[1], limbsCOMConstraints, projectToCom, max_num_samples, mu) for el in a]
    t2 = time.clock()
    #~ t4 = time.clock()
    resultsFinal = [el[0] for el in results if el[1]]
    print("time to filter", t2 - t1)
    #~ print "time to create", t4 - t3
    print("num res", len(resultsFinal))
    #sorting
    sortedlist = sorted(resultsFinal, key=distq_ref(state.q()))
    return sortedlist
    
#~ d3()

#~ b = computeNext(s, rarmId, max_num_samples = 1)
i=0
