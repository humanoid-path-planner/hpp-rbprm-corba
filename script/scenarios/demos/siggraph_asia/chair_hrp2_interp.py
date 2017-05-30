from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer

import chair_hrp2_path as path_planner
import hrp2_model as model
#~ import hrp2_model_grasp as model
from hrp2_model import *
import time


ps = path_planner.ProblemSolver( model.fullBody )
r = path_planner.Viewer (ps, viewerClient=path_planner.r.client)
fullBody = model.fullBody
#~ fullBody.setJointBounds ("base_joint_xyz", [-1,3, -1, 1, 0, 2.2])
fullBody.setJointBounds ("base_joint_xyz", [-1,3, -2, 2, -1, 6])
pp = PathPlayer (fullBody.client.basic, r)

from plan_execute import a, b, c, d, e, init_plan_execute
init_plan_execute(model.fullBody, r, path_planner, pp)

q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.getCurrentConfig(); q_init[0:7] = path_planner.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = path_planner.q_goal[0:7]


#~ fullBody.setCurrentConfig (q_init)
q_init =  [
        -0.05, -0.82, 0.6, 1.0, 0.0 , 0.0, 0.0,                         	 # Free flyer 0-6
        0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # LARM       11-17
        0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # RARM       18-24
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        ]; r (q_init)
        
#~ f = open("scale_1_save","r+")
#~ from pickle import load
#~ q_init= load(f)
#~ f.close()
#~ r (q_init)
 
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])
q_init = fullBody.generateContacts(q_init, [0,0,1])

fullBody.setStartState(q_init,[rLegId,lLegId]) #,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId,larmId, rarmId]) #,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId]) #,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])#,rarmId,larmId])

configs = d(0.005); e()

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
