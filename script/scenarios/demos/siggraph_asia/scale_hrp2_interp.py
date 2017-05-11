from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer

import scale_hrp2_path as tp
import time



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
fullBody.setJointBounds ("base_joint_xyz", [-1,3, -1, 1, 0, 2.2])


ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps, viewerClient=tp.r.client)

#~ AFTER loading obstacles
rLegId = 'hrp2_rleg_rom'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,-0.105,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 10000, "manipulability", 0.1)
                                                                                                
lLegId = 'hrp2_lleg_rom'                                                                                
lLeg = 'LLEG_JOINT0'                                                                            
lLegOffset = [0,-0.105,0]                                                                       
lLegNormal = [0,1,0]                                                                            
lLegx = 0.09; lLegy = 0.05                                                                      
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 10000, "manipulability", 0.1)



#~ AFTER loading obstacles
larmId = 'hrp2_larm_rom'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.05,-0.050,-0.050]
lArmNormal = [1,0,0]
lArmOffset = [0,0,-0.105]
lArmNormal = [0,0,1]
lArmx = 0.024; lArmy = 0.024
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, "manipulability", 0.1, "_6_DOF", True)
#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, 0.05)


rarmId = 'hrp2_rarm_rom'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [0,0,-0.105]
rArmNormal = [0,0,1]
rArmx = 0.024; rArmy = 0.024
#disabling collision for hook
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 10000, "manipulability", 0.1, "_6_DOF", True)

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
#~ 
fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(larmId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(rarmId, "jointLimitsDistance", True)

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])

q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.q_goal[0:7]


fullBody.setCurrentConfig (q_init)
q_init =  [
        -0.15, -0.82, 0.55, 1.0, 0.0 , 0.0, 0.0,                         	 # Free flyer 0-6
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
	configs = fullBody.interpolate(stepsize, 0, 2, True)
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
	
d(0.005); e()

print "Root path SDDSD in " + str(tp.t) + " ms."
	
#~ from gen_data_from_rbprm import *
#~ 
#~ for config in configs:
	#~ r(config)
	#~ print(fullBody.client.basic.robot.getComPosition())
#~ 

#~ gen_and_save(fullBody,configs, "stair_bauzil_contacts_data")
#~ main()

from gen_data_from_rbprm import *

from hpp.corbaserver.rbprm.tools.com_constraints import get_com_constraint

#computing com bounds 0 and 1
def __get_com(robot, config):
	save = robot.getCurrentConfig()
	robot.setCurrentConfig(config)
	com = robot.getCenterOfMass()
	robot.setCurrentConfig(save)
	return com

from numpy import matrix, asarray
from numpy.linalg import norm
from spline import bezier


def __curveToWps(curve):
    return asarray(curve.waypoints().transpose()).tolist()


def __Bezier(wps, init_acc = [0.,0.,0.], end_acc = [0.,0.,0.], init_vel = [0.,0.,0.], end_vel = [0.,0.,0.]):
    c = curve_constraints();
    c.init_vel = matrix(init_vel);
    c.end_vel  = matrix(end_vel);
    c.init_acc = matrix(init_acc);
    c.end_acc  = matrix(end_acc);
    matrix_bezier = matrix(wps).transpose()
    return __curveToWps(bezier(matrix_bezier, c))
    #~ return __curveToWps(bezier(matrix_bezier))

allpaths = []

def play_all_paths():
    for _, pid in enumerate(allpaths):
        pp(pid)

def play_all_paths_smooth():
    for i, pid in enumerate(allpaths):
        if i % 2 == 1 :
            pp(pid)
            
def play_all_paths_qs():
    for i, pid in enumerate(allpaths):
        if i % 2 == 0 :
            pp(pid)

def test(stateid = 1, path = False, use_rand = False, just_one_curve = False, num_optim = 0) :
    com_1 = __get_com(fullBody, configs[stateid])
    com_2 = __get_com(fullBody, configs[stateid+1])
    data = gen_sequence_data_from_state(fullBody,stateid,configs, mu = 1.)
    c_bounds_1 = get_com_constraint(fullBody, stateid, configs[stateid], limbsCOMConstraints, interm = False)
    c_bounds_mid = get_com_constraint(fullBody, stateid, configs[stateid], limbsCOMConstraints, interm = True)
    c_bounds_2 = get_com_constraint(fullBody, stateid, configs[stateid+1], limbsCOMConstraints, interm = False)
    success, c_mid_1, c_mid_2 = solve_quasi_static(data, c_bounds = [c_bounds_1, c_bounds_2, c_bounds_mid], use_rand = use_rand, mu = 0.8)
    #~ success, c_mid_1, c_mid_2 = solve_dyn(data, c_bounds = [c_bounds_1, c_bounds_2, c_bounds_mid], use_rand = use_rand)
    #~ success, c_mid_1, c_mid_2 = solve_dyn(data, c_bounds = [c_bounds_1, c_bounds_2])
    
    paths_ids = []
    if path and success:
        #~ fullBody.straightPath([c_mid_1[0].tolist(),c_mid_2[0].tolist()])
        #~ fullBody.straightPath([c_mid_2[0].tolist(),com_2])
        if just_one_curve:
            bezier_0 = __Bezier([com_1,c_mid_1[0].tolist(),c_mid_2[0].tolist(),com_2])
        
            partions = [0.,0.3,0.9,1.]
            p0 = fullBody.generateCurveTrajParts(bezier_0,partions)
            #testing intermediary configurations 
            print 'wtf', partions[1], " "
            com_interm1 = bezier_0[1]
            com_interm2 = bezier_0[2]
            success_proj1 = project_com_colfree(fullBody, stateid  , com_interm1)
            success_proj2 = project_com_colfree(fullBody, stateid+1, com_interm2)
            
            if not success_proj1:
				print "proj 1 failed"
				return False, c_mid_1, c_mid_2, paths_ids
				
            if not success_proj2:
				print "proj 2 failed"
				return False, c_mid_1, c_mid_2, paths_ids
            
            print "p0", p0
            #~ pp.displayPath(p0+1)
            #~ pp.displayPath(p0+2)
            pp.displayPath(p0)
            paths_ids = [int(el) for el in fullBody.comRRTFromPos(stateid,p0+1,p0+2,p0+3,num_optim)]
        else:
            bezier_0 = __Bezier([com_1,c_mid_1[0].tolist()]              , end_acc = c_mid_1[1].tolist() , end_vel = [0.,0.,0.])
            bezier_1 = __Bezier([c_mid_1[0].tolist(),c_mid_2[0].tolist()], end_acc = c_mid_2[1].tolist(), init_acc = c_mid_1[1].tolist(), init_vel = [0.,0.,0.], end_vel = [0.,0.,0.])
            bezier_2 = __Bezier([c_mid_2[0].tolist(),com_2]              , init_acc = c_mid_2[1].tolist(), init_vel = [0.,0.,0.])
        
            p0 = fullBody.generateCurveTraj(bezier_0)
            print "p0", p0
            fullBody.generateCurveTraj(bezier_1)
            fullBody.generateCurveTraj(bezier_2)
            pp.displayPath(p0)
            pp.displayPath(p0+1)
            pp.displayPath(p0+2)
            paths_ids = [int(el) for el in fullBody.comRRTFromPos(stateid,p0,p0+1,p0+2,num_optim)]
        #~ paths_ids = []
        global allpaths
        allpaths += paths_ids[:-1]
        #~ allpaths += [paths_ids[-1]]
        #~ pp(paths_ids[-1])
    
        #~ return success, paths_ids, c_mid_1, c_mid_2
    return success, c_mid_1, c_mid_2, paths_ids
#~ data = gen_sequence_data_from_state(fullBody,3,configs)



def prepare_whole_interp(stateid, stateid_end):
	all_points = []
	allSuc = True
	for i in range(stateid, stateid_end):
		com_1 = __get_com(fullBody, configs[stateid])
		success, c_mid_1, c_mid_2, paths_ids = test(i, False, True, False)
		allSuc = success and allSuc
		if not success:
			break
		all_points = all_points + [com_1, c_mid_1[0].tolist(), c_mid_2[0].tolist()]
	all_points = all_points + [__get_com(fullBody, configs[stateid_end])]
	if allSuc:
		bezier_0 = __Bezier(all_points)
		p0 = fullBody.generateCurveTraj(bezier_0)
		pp.displayPath(p0)
		num_paths = stateid_end - stateid
		num_sub_paths = num_paths * 3
		increment = 1. / float(num_paths)
		partitions = [0.]
		for i in range(0, num_paths):
			dec = increment * float(i)
			partitions += [dec + 0.01 * increment, dec + 0.99 * increment,dec + 1. * increment]
		print "partitions", partitions, len(partitions)
		p0 = fullBody.generateCurveTrajParts(bezier_0,partitions) +1
		paths_ids = []
		for i in range(0, num_paths):
			print "***************************3i", p0+3*i
			paths_ids += [int(el) for el in fullBody.comRRTFromPos(stateid + i,p0+3*i,p0+3*i+1,p0+3*i+2)]
        #~ paths_ids = []
			global allpaths
			allpaths += paths_ids[:-1]
			#~ pp(paths_ids[-1])
#~ success, paths_ids, c_mid_1, c_mid_2 = test(0, True, True, False)
#~ prepare_whole_interp(1, 2)

#~ pp(29),pp(9),pp(17)
from hpp.corbaserver.rbprm.tools.path_to_trajectory import *

def gen(ine_curve =False):
	#~ test(0, True, True, ine_curve)
	#~ test(0, True, True, True)
	#~ test(1, True, True, ine_curve)
	#~ test(1, True, True, True)
	test(2, True, True, ine_curve)
	#~ test(2, True, True, True)
	test(3, True, True, ine_curve)
	#~ test(3, True, True, True)
	test(4, True, True, ine_curve)
	test(5, True, True, ine_curve)
	a = gen_trajectory_to_play(fullBody, pp, allpaths, flatten([[0.3, 0.6, 0.1] for _ in range(len(allpaths) / 3)]))

#~ pp(29),pp(9),pp(17)
#~ gen(True)
