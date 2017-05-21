from gen_data_from_rbprm import *

from hpp.corbaserver.rbprm.tools.com_constraints import get_com_constraint
from hpp.gepetto import PathPlayer

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
    curve =  bezier(matrix_bezier, c)
    return __curveToWps(curve), curve
    #~ return __curveToWps(bezier(matrix_bezier))

allpaths = []

def play_all_paths():
    for _, pid in enumerate(allpaths):
        ppl(pid)

def play_all_paths_smooth():
    for i, pid in enumerate(allpaths):
        if i % 2 == 1 :
            ppl(pid)
            
def play_all_paths_qs():
    for i, pid in enumerate(allpaths):
        if i % 2 == 0 :
            ppl(pid)

def test(stateid = 1, path = False, use_rand = False, just_one_curve = False, num_optim = 0, effector = False, mu=0.5) :
    com_1 = __get_com(fullBody, configs[stateid])
    com_2 = __get_com(fullBody, configs[stateid+1])
    createPtBox(viewer.client.gui, 0, com_1, 0.01, [0,1,1,1.])
    createPtBox(viewer.client.gui, 0, com_2, 0.01, [0,1,1,1.])
    data = gen_sequence_data_from_state(fullBody,stateid,configs, mu = mu)
    c_bounds_1 = get_com_constraint(fullBody, stateid, configs[stateid], limbsCOMConstraints, interm = False)
    c_bounds_mid = get_com_constraint(fullBody, stateid, configs[stateid], limbsCOMConstraints, interm = True)
    c_bounds_2 = get_com_constraint(fullBody, stateid, configs[stateid+1], limbsCOMConstraints, interm = False)
    success, c_mid_1, c_mid_2 = solve_quasi_static(data, c_bounds = [c_bounds_1, c_bounds_2, c_bounds_mid], use_rand = use_rand, mu = mu)
    #~ success, c_mid_1, c_mid_2 = solve_dyn(data, c_bounds = [c_bounds_1, c_bounds_2, c_bounds_mid], use_rand = use_rand)
    #~ success, c_mid_1, c_mid_2 = solve_dyn(data, c_bounds = [c_bounds_1, c_bounds_2])
    
    print "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ calling effector", effector
    paths_ids = []
    if path and success:
        #~ fullBody.straightPath([c_mid_1[0].tolist(),c_mid_2[0].tolist()])
        #~ fullBody.straightPath([c_mid_2[0].tolist(),com_2])
        if just_one_curve:
            print "just one curve"
            bezier_0, curve = __Bezier([com_1,c_mid_1[0].tolist(),c_mid_2[0].tolist(),com_2])
            createPtBox(viewer.client.gui, 0, c_mid_1[0].tolist(), 0.01, [0,1,0,1.])
            createPtBox(viewer.client.gui, 0, c_mid_2[0].tolist(), 0.01, [0,1,0,1.])
        
            partions = [0.,0.2,0.8,1.]
            p0 = fullBody.generateCurveTrajParts(bezier_0,partions)
            #testing intermediary configurations 
            print 'wtf', partions[1], " "
            com_interm1 = curve(partions[1])
            com_interm2 = curve(partions[2])
            print "com_1", com_1
            success_proj1 = project_com_colfree(fullBody, stateid  , asarray((com_interm1).transpose()).tolist()[0])
            success_proj2 = project_com_colfree(fullBody, stateid+1, asarray((com_interm2).transpose()).tolist()[0])
            
            if not success_proj1:
				print "proj 1 failed"
				return False, c_mid_1, c_mid_2, paths_ids
				
            if not success_proj2:
				print "proj 2 failed"
				return False, c_mid_1, c_mid_2, paths_ids
            
            print "p0", p0
            #~ pp.displayPath(p0+1)
            #~ pp.displayPath(p0+2)
            ppl.displayPath(p0)
            #~ pp.displayPath(p0+1)
            #~ pp.displayPath(p0+2)
            #~ pp.displayPath(p0+3)
            if(effector):
				print "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ calling effector"
				paths_ids = [int(el) for el in fullBody.effectorRRT(stateid,p0+1,p0+2,p0+3,num_optim)]
            else:
				paths_ids = [int(el) for el in fullBody.comRRTFromPos(stateid,p0+1,p0+2,p0+3,num_optim)]
			
        else:
            
            success_proj1 = project_com_colfree(fullBody, stateid  , c_mid_1[0].tolist())
            success_proj2 = project_com_colfree(fullBody, stateid+1, c_mid_2[0].tolist())
            
            if not success_proj1:
				print "proj 1 failed"
				return False, c_mid_1, c_mid_2, paths_ids
				
            if not success_proj2:
				print "proj 2 failed"
				return False, c_mid_1, c_mid_2, paths_ids
            
            print "three curves"
            bezier_0, curve = __Bezier([com_1,c_mid_1[0].tolist()]              , end_acc = c_mid_1[1].tolist() , end_vel = [0.,0.,0.])
            bezier_1, curve = __Bezier([c_mid_1[0].tolist(),c_mid_2[0].tolist()], end_acc = c_mid_2[1].tolist(), init_acc = c_mid_1[1].tolist(), init_vel = [0.,0.,0.], end_vel = [0.,0.,0.])
            bezier_2, curve = __Bezier([c_mid_2[0].tolist(),com_2]              , init_acc = c_mid_2[1].tolist(), init_vel = [0.,0.,0.])
        
            p0 = fullBody.generateCurveTraj(bezier_0)
            print "p0", p0
            fullBody.generateCurveTraj(bezier_1)
            fullBody.generateCurveTraj(bezier_2)
            ppl.displayPath(p0)
            ppl.displayPath(p0+1)
            ppl.displayPath(p0+2)
            paths_ids = [int(el) for el in fullBody.comRRTFromPos(stateid,p0,p0+1,p0+2,num_optim)]
        #~ paths_ids = []
        global allpaths
        allpaths += paths_ids[:-1]
        #~ allpaths += [paths_ids[-1]]
        #~ pp(paths_ids[-1])
    
        #~ return success, paths_ids, c_mid_1, c_mid_2
    return success, c_mid_1, c_mid_2, paths_ids
#~ data = gen_sequence_data_from_state(fullBody,3,configs)

#~ pp(29),pp(9),pp(17)
from hpp.corbaserver.rbprm.tools.path_to_trajectory import *


def createPtBox(gui, winId, config, res = 0.01, color = [1,1,1,0.3]):
	resolution = res
	global scene
	global b_id
	boxname = scene+"/"+str(b_id)
	b_id += 1
	gui.addBox(boxname,resolution,resolution,resolution, color)
	gui.applyConfiguration(boxname,[config[0],config[1],config[2],1,0,0,0])
	gui.addSceneToWindow(scene,winId)
	gui.refresh()

def test_ineq(stateid, constraints, n_samples = 10, color=[1,1,1,1.]):
	Kin = get_com_constraint(fullBody, stateid, configs[stateid], constraints, interm = False)
	#~ print "kin ", Kin
	#create box around current com
	fullBody.setCurrentConfig(configs[stateid])
	com = fullBody.getCenterOfMass()
	bounds_c = flatten([[com[i]-1., com[i]+1.] for i in range(3)]) # arbitrary
	for i in range(n_samples):
		c = array([uniform(bounds_c[2*i], bounds_c[2*i+1]) for i in range(3)])
		print "c: ", c
		if(Kin[0].dot(c)<=Kin[1]).all():
			print "boundaries satisfied"
			createPtBox(viewer.client.gui, 0, c, 0.01, color)
		
#~ test_ineq(0,{ rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'}}, 1000, [1,0,0,1])
#~ test_ineq(0,{ lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}}, 1000, [0,1,0,1])
#~ test_ineq(0,{ rLegId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand}}, 1000, [0,0,1,1])
#~ test_ineq(0,{ rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'}}, 1000, [0,1,1,1])
#~ test_ineq(0, limbsCOMConstraints, 1000, [0,1,1,1])


def gen(start = 0, len_con = 1, num_optim = 0, ine_curve =True, s = 1., effector = False, mu =0.5):
    n_fail = 0;
    for i in range (start, start+len_con):
		viewer(configs[i])
		res =  test(i, True, False, ine_curve,num_optim, effector, mu)
		if(not res[0]):       
			print "lp failed"
			createPtBox(viewer.client.gui, 0, res[1][0], 0.01, [1,0,0,1.])
			createPtBox(viewer.client.gui, 0, res[2][0], 0.01, [1,0,0,1.])
			found = False
			for j in range(10):
				res = test(i, True, True, ine_curve, num_optim, effector, mu)               
				createPtBox(viewer.client.gui, 0, res[1][0], 0.01, [0,1,0,1.])
				createPtBox(viewer.client.gui, 0, res[2][0], 0.01, [0,1,0,1.])
				if res[0]:
					break
			if not res[0]:
				n_fail += 1
    print "n_fail ", n_fail
        
    a = gen_trajectory_to_play(fullBody, ppl, allpaths, flatten([[s*0.2, s* 0.6, s* 0.2] for _ in range(len(allpaths) / 3)]))
    return a
    
viewer = None
tp = None
ppl = None
fullBody = None
b_id = 0
scene = "bos"

def clean_path():
    global allpaths
    allpaths = []


def init_bezier_traj(robot, r, pplayer, qs, comConstraints):
    global viewer
    global tp
    global ppl
    global fullBody
    global viewer
    global configs
    configs = qs
    viewer = r
    ppl = pplayer
    fullBody = robot
    viewer.client.gui.createScene(scene)
    global limbsCOMConstraints
    limbsCOMConstraints = comConstraints
