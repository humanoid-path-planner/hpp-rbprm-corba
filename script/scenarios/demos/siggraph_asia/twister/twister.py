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
from hpp.corbaserver.rbprm.state_alg  import addNewContact, isContactReachable, closestTransform, removeContact, addNewContactIfReachable, projectToFeasibleCom

s1 = State(fullBody,q=q_init, limbsIncontact = [rLegId, lLegId])  

q0 = s1.q()[:]

#~ def test(p, n):
    #~ global s1
    #~ t0 = time.clock()
    #~ a, success = addNewContact(s1,rLegId,p, n)
    #~ print "projecttion successfull ? ", success
    #~ t1 = time.clock()
    #~ val, com = isContactReachable(s1, rLegId,p, n,  limbsCOMConstraints)
    #~ t2 = time.clock()
    #~ print 'is reachable ? ', val, com
    #~ if(val):
        #~ com[2]+=0.1
        #~ if(success > 0):
            #~ print 'a > 0'
            #~ t3 = time.clock()
            #~ q = fullBody.projectStateToCOM(a.sId, com.tolist())
            #~ t4=  time.clock()
            #~ r(a.q())
        #~ else:
            #~ print "using s1 ", s1.sId
            #~ t3 = time.clock()
            #~ q = fullBody.projectStateToCOM(s1.sId, com.tolist())
            #~ t4=  time.clock()
            #~ a, succ = addNewContact(s1,rLegId,p, n)
            #~ if (succ):
                #~ r(a.q())
        #~ print "time to addnc", t2 - t1
        #~ print "projectcom succesfull ?", q
        #~ print "time to check reachable", t3- t2
        #~ print "time to project", t4- t3

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
    print "time to filter", t2 - t1
    #~ print "time to create", t4 - t3
    print "num res", len(resultsFinal)
    #sorting
    sortedlist = sorted(resultsFinal, key=distq_ref(state.q()))
    return sortedlist

scene = "bb"
r.client.gui.createScene(scene)
b_id = 0


def plot_feasible_Kin(state):
    com = array(state.getCenterOfMass())
    for i in range(5):
        for j in range(5):
            for k in range(10):
                c = com + array([(i - 2.5)*0.2, (j - 2.5)*0.2, (k-5)*0.2])
                active_ineq = state.getComConstraint(limbsCOMConstraints,[])
                if(active_ineq[0].dot( c )<= active_ineq[1]).all():
                    #~ print 'active'
                    createPtBox(r.client.gui, 0, c, color = [0,1,0,1])
                else:
                    if(active_ineq[0].dot( c )>= active_ineq[1]).all():
                        #~ print "inactive"
                        createPtBox(r.client.gui, 0, c, color = [1,0,0,1])
    return -1
    
def compute_w(c, ddc=array([0.,0.,0.]), dL=array([0.,0.,0.]), m = 54., g_vec=array([0.,0.,-9.81])):
	w1 = m * (ddc - g_vec)
	return array(w1.tolist() + (cross(c, w1) + dL).tolist())
    
def plot_feasible_cone(state):
    com = array(state.getCenterOfMass())
    #~ H, h = state.getContactCone(0.6)  
    ps = state.getContactPosAndNormals()
    p = ps[0][0]
    N = ps[1][0]
    H = compute_CWC(p, N, state.fullBody.client.basic.robot.getMass(), mu = 0.6, simplify_cones = False)
    #~ H = comp
    #~ H = -array(H)
    #~ h = array(h)
    #~ print "h", h
    for i in range(10):
        for j in range(10):
            for k in range(1):
                c = com + array([(i - 5)*0.1, (j - 5)*0.1, k])   
                w = compute_w(c)             
                print "w, " , w
                if(H.dot( w )<= 0).all():
                    #~ print 'active'
                    createPtBox(r.client.gui, 0, c, color = [0,1,0,1])
                else:
                    #~ if(H.dot( w )>= 0).all():
                        #~ print "inactive"
                    createPtBox(r.client.gui, 0, c, color = [1,0,0,1])
    return H

def plot_feasible(state):
    com = array(state.getCenterOfMass())
    ps = state.getContactPosAndNormals()
    p = ps[0][0]
    N = ps[1][0]
    H = compute_CWC(p, N, state.fullBody.client.basic.robot.getMass(), mu = 1, simplify_cones = False)
    for i in range(5):
        for j in range(5):
            for k in range(10):
                c = com + array([(i - 2.5)*0.2, (j - 2.5)*0.2, (k-5)*0.2])
                w = compute_w(c)           
                active_ineq = state.getComConstraint(limbsCOMConstraints,[])
                if(active_ineq[0].dot( c )<= active_ineq[1]).all() and (H.dot( w )<= 0).all():
                    #~ print 'active'
                    createPtBox(r.client.gui, 0, c, color = [0,1,0,1])
                else:
                    if(active_ineq[0].dot( c )>= active_ineq[1]).all():
                        #~ print "inactive"
                        createPtBox(r.client.gui, 0, c, color = [1,0,0,1])
    return -1
 
def plot(c):
    createPtBox(r.client.gui, 0, c, color = [0,1,0,1])

i = 0
#~ s0 = removeContact(s1,rLegId)[0]
#~ s_init =  computeNext(s0,rLegId, True,20)
#~ res = computeNext(s0,larmId, True,20)
#~ s1 = res[0]
#~ res2 = computeNext(s1,rarmId, True,20)
#~ s2 = computeNext(s1,rarmId, True,100)[0]

all_states=[]
#~ all_states=[s1,s2]
#~ s2 = removeContact(s2,rLegId)[0]
#~ s3 = computeNext(s2, larmId)[0]
#~ go0(s2.sId,1, s=1)
#~ plot_feasible(s1)
from time import sleep
def play():
    for i,el in enumerate(all_states):
        r(el.q())
        sleep(0.5)
    i = len(all_states)-1;
    for j in range(i+1):
        print "ij,sum", i, j, i-j
        r(all_states[i-j].q())
        sleep(0.5)

play()

#~ print "init valid ?", fullBody.isConfigValid(s1.q())
#~ print "end valid ?", fullBody.isConfigValid(s2.q())

r(q_init)
#~ path = go0([s2,s1], mu=0.3,num_optim=1)
res = computeNext(s1,rarmId,True,10)
s2 = res[0]; r(s2.q())
#~ s3 = computeNext(s2,rLegId,True,10)[0]; r(s3.q())
#~ res2 = computeNext(s2,larmId,True,100)
#~ s3 = res2[0]; r(s3.q())
s4 = removeContact(s2,rLegId,True,0.1)[0]; r(s4.q())
s5 = removeContact(s4,lLegId,True,0.1)[0]; r(s5.q())
#~ path = go0([s1,s2,s3,s4,s5], mu=0.6,num_optim=1)
#~ path = go0([s1,s2,s4,s5], mu=0.6,num_optim=1)

states = [s1,s2,s4,s5]

def add(lId):
    sF = states[-1]
    ns = computeNext(sF,lId,True,10)[0]
    global states
    states +=[ns]
    r(ns.q())
    
def rm(lId):
    sF = states[-1]
    ns = removeContact(sF,lId,True)[0]
    global states
    states +=[ns]
    r(ns.q())
    
def go():
    return go0(states, mu=0.6,num_optim=2)
