from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer

import scale_hrp2_path as path_planner
import hrp2_model as model
#~ import hrp2_model_grasp as model
from hrp2_model import *
import time


ps = path_planner.ProblemSolver( model.fullBody )
r = path_planner.Viewer (ps, viewerClient=path_planner.r.client)
fullBody = model.fullBody
#~ fullBody.setJointBounds ("base_joint_xyz", [-1,3, -1, 1, 0, 2.2])
fullBody.setJointBounds ("base_joint_xyz", [-1,3, -1, 1, 0, 6])
pp = PathPlayer (fullBody.client.basic, r)

from plan_execute import a, b, c, d, e, init_plan_execute
init_plan_execute(model.fullBody, r, path_planner, pp)

q_0 = fullBody.getCurrentConfig();
q_init = fullBody.getCurrentConfig(); q_init[0:7] = path_planner.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = path_planner.q_goal[0:7]


fullBody.setCurrentConfig (q_init)
q_init =  [
        -0.05, -0.82, 0.55, 1.0, 0.0 , 0.0, 0.0,                         	 # Free flyer 0-6
        0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # LARM       11-17
        0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # RARM       18-24
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        ]; r (q_init)

fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])
q_init = fullBody.generateContacts(q_init, [0,0,1])

fullBody.setStartState(q_init,[rLegId,lLegId,rarmId]) #,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId,larmId, rarmId]) #,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId]) #,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])#,rarmId,larmId])

configs = d(0.06, True); e()

from bezier_traj import go0, go2, init_bezier_traj, reset
from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import play_trajectory

import time

from hpp.corbaserver.rbprm.rbprmstate import State
from hpp.corbaserver.rbprm.state_alg  import *


path = []

def sc(ec):
    pass

def pl(iid = None):
    global path
    if iid == None:
        iid = len(path) -1
    play_trajectory(fullBody,pp,path[iid])

def plc(ctx = 0, iid = None):
    sc(ctx)
    pl(iid)

def go():
    return go0(states, mu=0.6,num_optim=2, use_kin = context == 0)

def plall(first = 0):
    global path
    sc(first)
    for pId in range(len(path)):
        play_trajectory(fullBody,pp,path[pId])



from pickle import load, dump
def save(fname):
    sc(0)
    all_data=[[],[]]
    global states
    for s in states:
        all_data[0]+=[[s.q(), s.getLimbsInContact()]]
    f = open(fname, "w")
    dump(all_data,f)
    f.close()

def load_save(fname):
    f = open(fname, "r+")
    all_data = load (f)
    f.close()
    sc(0)
    global states
    states = []
    #~ for i in range(0,len(all_data[0]),2):
        #~ print "q",all_data[0][i]
        #~ print "lic",all_data[0][i+1]
        #~ states+=[State(fullBody,q=all_data[0][i], limbsIncontact = all_data[0][i+1]) ]
    for _, s in enumerate(all_data[0]):
        states+=[State(fullBody,q=s[0], limbsIncontact = s[1]) ]
	r(states[0].q())

def onepath(ol, ctxt=1, nopt=1, mu=1, effector = False):
    reset()
    sc(ctxt)
    global path
    global states
    print "ctxt", ctxt
    print "q", len(states[ol+1].q())
    s = max(norm(array(states[ol+1].q()) - array(states[ol].q())), 1.) * 0.4
    print "s",s
    if(ol > len(path) -1):
        path += [go0([states[ol],states[ol+1]], num_optim=nopt, mu=mu, use_kin = False, s=s, effector = effector)]
    else:
        path[ol]=go0([states[ol],states[ol+1]], num_optim=nopt, mu=mu, use_kin = False, s=s, effector = effector)
    all_paths[ctxt] = path

def onepath2(states_subset, ctxt=1, nopt=1, mu=1, effector = False):
    reset()
    sc(ctxt)
    global path
    global states
    #~ print "ctxt", ctxt
    #~ print "q", len(states[ol+1].q())
    #~ s = max(norm(array(states_subset[1].q()) - array(states_subset[0].q())), 1.) * 0.4
    #~ print "s",s
    #~ if(ol > len(path) -1):
    path = all_paths[ctxt][:]
    path += [go2(states_subset, num_optim=nopt, mu=mu, use_kin = False, s=None, effector = effector)]
    #~ else:
        #~ path[ol]=go2(states_subset, num_optim=nopt, mu=mu, use_kin = False, s=s, effector = effector)
    all_paths[ctxt] = path
    sac()

def save_paths(fname):
    f = open(fname, "w")
    dump(all_paths,f)
    f.close()
    #now try with latest paths
    global all_path
    global path
    sc(0)
    all_paths[0] = path[:]
    f = open(fname+"all", "w")
    dump(all_paths,f)
    f.close()

def load_paths(fname):
    f = open(fname, "r")
    global all_paths
    all_paths = load (f)
    f.close()
    sc(0)
    global path
    path = all_paths[0][:]

def sh(ctxt, i):
    sc(ctxt)
    r(states[i].q())

def lc():
    load_save("19_06_s")
    load_paths("19_06_p")
    #~ save_paths("19_06_p_save")
    save("19_06_s_save")

def sac():
    save("19_06_s")
    save_paths("19_06_p")

init_bezier_traj(fullBody, r, pp, configs, limbsCOMConstraints)

states = planToStates(fullBody,configs)

all_paths = [[],[]]
from numpy import array
