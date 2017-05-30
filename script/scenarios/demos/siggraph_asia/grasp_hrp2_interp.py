from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer

import grasp_hrp2_path as tp
import time

path_planner = tp

packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = "_reduced"
srdfSuffix = ""

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)


ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps, viewerClient=tp.r.client)


#~ AFTER loading obstacles
rLegId = 'hrp2_rleg_rom'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.105]
rLegNormal = [0,0,1]       
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 10000, "manipulability", 0.1)
                                                                                                
lLegId = 'hrp2_lleg_rom'                                                                                
lLeg = 'LLEG_JOINT0'                                                                     
lLegx = 0.09; lLegy = 0.05      
lLegOffset = [0,0,-0.105]
lLegNormal = [0,0,1]                                                                  
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 10000, "manipulability", 0.1)



#~ AFTER loading obstacles
larmId = 'hrp2_larm_rom'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [0,0,-0.1075]
lArmNormal = [0,0,1]
lArmx = 0.024; lArmy = 0.024
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, "manipulability", 0.1, "_6_DOF", False,grasp = True)
#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, "manipulability", 0.1, "_6_DOF", True)
#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, "manipulability", 0.1, "_6_DOF")
#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, 0.05)


rarmId = 'hrp2_rarm_rom'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [0,0,-0.1075]
rArmNormal = [0,0,1]
rArmx = 0.024; rArmy = 0.024
#disabling collision for hook
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 10000, "manipulability", 0.1, "_6_DOF", False,grasp = True)
#~ fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 10000, "manipulability", 0.1, "_6_DOF", True)
#~ fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 10000, "manipulability", 0.1, "_6_DOF")
#~ 
#~ fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(larmId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(rarmId, "jointLimitsDistance", True)

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])

q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.q_goal[0:7]


fullBody.setCurrentConfig (q_init)
q_init =  [
        -0.05, -1.12, 0.5, 1.0, 0.0 , 0.0, 0.0,                         	 # Free flyer 0-6
        0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # LARM       11-17
        0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # RARM       18-24
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        ]; r (q_init)

fullBody.setCurrentConfig (q_goal)
#~ r(q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])
q_init = fullBody.generateContacts(q_init, [0,0,1])
#~ r(q_goal)

fullBody.setStartState(q_init,[rLegId,lLegId,rarmId]) #,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId,larmId, rarmId]) #,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId]) #,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])#,rarmId,larmId])
#~ 
#~ configs = fullBody.interpolate(0.1)
#~ configs = fullBody.interpolate(0.15)
i = 0;
configs = []


limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},  
						lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'},
						rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand} ,
						larmId : {'file': "hrp2/LA_com.ineq", 'effector' : lHand} }

#~ fullBody.limbRRTFromRootPath(0,len(configs)-1,0,2)
from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

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
	
def genPlan(stepsize=0.1):
	r.client.gui.setVisibility("hrp2_14", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
	global configs
	start = time.clock() 
	print "BEFORE"
	configs = fullBody.interpolate(stepsize, 0, 0, False)
	print "AFTER"
	end = time.clock() 
	print "Contact plan generated in " + str(end-start) + "seconds"
	
def contactPlan(step = 0.5):
	r.client.gui.setVisibility("hrp2_14", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
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
	
def d(step=0.1):
	print "computing contact plan"
	genPlan(step)
	
def e(step = 0.5):
	print "displaying contact plan"
	contactPlan(step)
	
print "Root path WXXSD in " + str(tp.t) + " ms."
	
d(0.005); 

print "Root path SDDSD in " + str(tp.t) + " ms."
	
	
	
qs = configs
fb = fullBody
ttp = path_planner
from bezier_traj import *
init_bezier_traj(fb, r, pp, qs, limbsCOMConstraints)
#~ AFTER loading obstacles
configs = qs
fullBody = fb
tp = ttp

#~ test_ineq(0,{ rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'}}, 1000, [1,0,0,1])
#~ test_ineq(0,{ lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}}, 1000, [0,0,1,1])
#~ gen(0,1)

com_vel = [0.,0.,0.]
com_acc = [0.,0.,0.]

vels = []
accs = []

#~ test_ineq(0,{ rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'}}, 1000, [1,0,0,1])
#~ test_ineq(0,{ lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}}, 1000, [0,0,1,1])
#~ gen(0,1)

path = []
a_s = []
def go(sid, rg = 2, num_optim = 0, mu = 0.6, window = 2, s = None):
    global com_vel
    global com_acc
    global vels
    global accs
    global path
    global a_s
    a = []
    for l in range(sid,sid+rg):
        print "STATE ", l
        s = max(norm(array(configs[sid+1]) - array(configs[sid])), 1.) * 1
        a,com_vel,com_acc = gen_several_states_partial(l,window,mu=mu,num_optim=num_optim, s=s,init_vel=com_vel, init_acc=com_acc, path=True)
        a_s+=[a]
        vels += [com_vel[:]]
        accs += [com_acc[:]]
    print "STATE ", sid+rg
    #~ path,com_vel,com_acc = gen_several_states(sid+rg,1,mu=mu,num_optim=num_optim, s=s,init_vel=com_vel, init_acc=com_acc)
    vels += [com_vel[:]]
    accs += [com_acc[:]]
    return a
    
def go_stop(sid, rg = 2, num_optim = 0, mu = 0.6, window = 2, s = None):
	global com_vel
	global com_acc
	global vels
	global accs
	global path
	global a_s
	a = []
	for l in range(sid,sid+rg):
		print "STATE ", l		
		s = max(norm(array(configs[sid+1]) - array(configs[sid])), 1.) * 1
		a,com_vel,com_acc = gen_several_states_partial(l,window,mu=mu,num_optim=num_optim, s=s,init_vel=com_vel, init_acc=com_acc, path=True)
		a_s+=[a]
		vels += [com_vel[:]]
		accs += [com_acc[:]]
	print "STATE ", sid+rg
	s = max(norm(array(configs[sid+rg+1]) - array(configs[sid+rg])), 1.) * 1
	a,com_vel,com_acc = gen_several_states(sid+rg,1,mu=mu,num_optim=num_optim, s=s,init_vel=com_vel, init_acc=com_acc)
	a_s+=[a]
	vels += [com_vel[:]]
	accs += [com_acc[:]]
	return a
    
def go0(sid, rg, num_optim = 0, mu = 0.6, s =None):
    global com_vel
    global com_acc
    global vels
    global accs
    global path
    if s == None:
        s = max(norm(array(configs[sid+1]) - array(configs[sid])), 1.) * 1.5
        print "$$$$$$$$$$$$$$$ S $$$$$$$$ *********************444444444444444444444444444 ", s
    for i in range(rg):
        path = gen(sid+i,1,mu=mu,num_optim=num_optim, s=s)
    return path

def go2(sid, rg = 1, num_optim = 0, mu = 0.5, t =2, s =None):
    global com_vel
    global com_acc
    global vels
    global accs
    global path
    for i in range(rg):
		if s == None:
			s = max(norm(array(configs[sid+i+1]) - array(configs[sid+i])), 1.) * 0.6
			print "$$$$$$$$$$$$$$$ S $$$$$$$$ ", s
		path,com_vel,com_acc = gen_several_states(sid+i,sid+i+t,mu=mu,num_optim=num_optim, s=s,init_vel=com_vel, init_acc=com_acc)
		vels += [com_vel[:]]
		accs += [com_acc[:]]
    return path
    
#~ a = go2(0, s = 1)
#~ a = go2(0, num_optim=0, s = 1.2, mu=0.6)
#~ a = go2(2, num_optim=0, s = 1.2, mu=0.6)
#~ a = go2(4, num_optim=3, mu=0.6)

def reset():
    global com_vel
    global com_acc
    global vels
    global accs
    global a_s
    global path
    com_vel = [0.,0.,0.]
    com_acc = [0.,0.,0.]
    clean_path();
    vels = []
    accs = []
    path = []
    a_s = []
    for i, config in enumerate(configs):
		fullBody.setConfigAtState(i,config)
