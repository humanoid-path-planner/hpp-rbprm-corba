from twister_geom import *

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer

import twister_path as path_planner
import hrp2_model as model
#~ import hrp2_model_grasp as model
from hrp2_model import *
import time




ps = path_planner.ProblemSolver( model.fullBody )
r = path_planner.Viewer (ps, viewerClient=path_planner.r.client)
pp = PathPlayer (fullBody.client.basic, r)
fullBody = model.fullBody
fullBody.setJointBounds ("base_joint_xyz", [-2,2.5, -2, 2, 0, 2.2])

from plan_execute import a, b, c, d, e, init_plan_execute
init_plan_execute(model.fullBody, r, path_planner, pp)

q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)
q_init = fullBody.getCurrentConfig(); q_init[0:7] = path_planner.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = path_planner.q_goal[0:7]


fullBody.setCurrentConfig (q_init)
q_init =  [
        0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0,                         	 # Free flyer 0-6
        0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # LARM       11-17
        0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # RARM       18-24
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        ]; r (q_init)

fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])

fullBody.setStartState(q_init,[rLegId,lLegId])#,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])#,rarmId,larmId])
i = 0;
configs = []


#~ d(0.1); e(0.01)

print "confgs ", configs

qs = configs
fb = fullBody
ttp = path_planner
from bezier_traj import *
init_bezier_traj(fb, r, pp, qs, limbsCOMConstraints)
#~ AFTER loading obstacles
configs = qs
fullBody = fb
tp = ttp

from hpp.corbaserver.rbprm.rbprmstate import State
from hpp.corbaserver.rbprm.state_alg  import addNewContact, isContactReachable, closestTransform

s1 = State(fullBody,q=q_init, limbsIncontact = [rLegId, lLegId])  

q0 = s1.q()[:]

def test(p, n):
    global s1
    t0 = time.clock()
    a, success = addNewContact(s1,rLegId,p, n)
    print "projecttion successfull ? ", success
    t1 = time.clock()
    val, com = isContactReachable(s1, rLegId,p, n,  limbsCOMConstraints)
    t2 = time.clock()
    print 'is reachable ? ', val, com
    if(val):
        com[2]+=0.1
        if(success > 0):
            print 'a > 0'
            t3 = time.clock()
            q = fullBody.projectStateToCOM(a.sId, com.tolist())
            t4=  time.clock()
            r(a.q())
        else:
            print "using s1 ", s1.sId
            t3 = time.clock()
            q = fullBody.projectStateToCOM(s1.sId, com.tolist())
            t4=  time.clock()
            a, succ = addNewContact(s1,rLegId,p, n)
            print "success projection ??", succ
            if (succ):
                r(a.q())
        print "time to addnc", t2 - t1
        print "projectcom succesfull ?", q
        print "time to check reachable", t3- t2
        print "time to project", t4- t3


p = [-0.1,-0.91499999,0.00]
n = [1,0.,0.]
test(p,n)

p = [-0.1,-0.91499999,0.00]
n = [0,0.,1.]
test(p,n)

p = [-0.1,-0.91499999,0.10]
n = [0,0.,1.]
test(p,n)


p = [-0.1,-0.91499999,0.50]
n = [0,0.,1.]
test(p,n)

p = [-0.1,-1.1, 0.30]
n = [0,0.,1.]
test(p,n)
