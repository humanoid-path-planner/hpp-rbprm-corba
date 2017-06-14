from twister_geom import *
from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer
from numpy import array, sort
from numpy.linalg import norm
from plan_execute import a, b, c, d, e, init_plan_execute
from bezier_traj import go0, init_bezier_traj, reset
from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import play_trajectory

import time

from hpp.corbaserver.rbprm.rbprmstate import State
from hpp.corbaserver.rbprm.state_alg  import addNewContact, isContactReachable, closestTransform, removeContact, addNewContactIfReachable, projectToFeasibleCom

robot_contexts = []
robot_context = None

cl = Client()

context = 0
states                   = None
tp                       = None
model                    = None
path_planner             = None
ps                       = None
r                        = None
r_parent                 = None
pp                       = None
fullBody                 = None
configs                  = None
lLegId                   = None
rLegId                   = None
larmId                   = None
rarmId                   = None
limbsCOMConstraints      = None
fullBody                 = None
configs                  = None
cl                       = None
path                     = None

def save_globals():        
    robot_context["states"] = states
    robot_context["configs"] = configs
    robot_context["path"] = path
    
def set_globals():        
    global robot_context
    
    global states
    global tp
    global model
    global path_planner
    global ps
    global r
    global pp
    global fullBody
    global configs
    global lLegId
    global rLegId
    global larmId
    global rarmId
    global limbsCOMConstraints
    global fullBody
    global configs
    global cl
    global path
    
    states     = robot_context["states"]
    tp         = robot_context["tp"]
    model    = robot_context["model"]
    ps        = robot_context["ps"]
    r        = robot_context["r"]
    pp        = robot_context["pp"]
    fullBody= robot_context["fullBody"]
    configs = robot_context["configs"]
    cl        = robot_context["cl"]
    path        = robot_context["path"]
    limbsCOMConstraints = model.limbsCOMConstraints
    rLegId = model.rLegId
    lLegId = model.lLegId
    larmId = model.larmId
    rarmId = model.rarmId
    path_planner = tp
    
    init_plan_execute(fullBody, r, path_planner, pp)
    init_bezier_traj(fullBody, r, pp, configs, limbsCOMConstraints)
    
    
    #~ states     = robot_context["states"]
    #~ tp         = robot_context["tp"]
    #~ model    = robot_context["model"]
    #~ ps        = robot_context["ps"]
    #~ r        = robot_context["r"]
    #~ pp        = robot_context["pp"]
    #~ fullBody= robot_context["fullBody"]
    #~ configs = robot_context["configs"]
    #~ limbsCOMConstraints = model.limbsCOMConstraints
    #~ lLegId = model.lLegId
    #~ lLegId = model.rLegId
    #~ larmId = model.larmId
    #~ rarmId = model.rarmId
    #~ path_planner = tp

import importlib

def init_context(path, wb, other_package ):    
    global robot_contexts
    rid = len(robot_contexts) 
    
    path_planner_1 = importlib.import_module(path)
    path_planner_1.cl.problem.selectProblem("robot" + str(rid))  
    global r_parent
    loaded = r_parent == None
    if loaded:
        r_parent = path_planner_1.r
    r  = path_planner_1.Viewer (path_planner_1.ps, viewerClient=r_parent.client)  
    path_planner_1.afftool.loadObstacleModel ('hpp-rbprm-corba', "twister", "planning", r)
    r.loadObstacleModel (*other_package)
    model_1  = importlib.import_module(wb)    
    model_1.fullBody.setJointBounds ("base_joint_xyz", [-2,2.5, -2, 2, 0, 2.2])
    ps1 =  path_planner_1.ProblemSolver( model_1.fullBody )
    r  = path_planner_1.Viewer (ps1, viewerClient=r_parent.client)  
    #~ if not loaded:
        #~ path_planner_1.afftool.loadObstacleModel ('hpp-rbprm-corba', "twister", "planning", r)
    robot_contexts += [{"model" : model_1, 
    "states" : [], "tp" :  path_planner_1,
    "ps" : ps1, 
    "fullBody" : model_1.fullBody,
    "r" : r, "fullBody" : model_1.fullBody,
    "configs" : [],
    "pp" : PathPlayer (model_1.fullBody.client.basic, r),
    "cl" : path_planner_1.cl, "path" : [] }]    
  
def publishRobot_and_switch(context_to):
    #~ r.robot.setCurrentConfig (self.robotConfig)
    saves = {}
    pos0 = None
    for j, prefix, o in r.robotBodies:
        pos = r.robot.getLinkPosition (j)
        objectName = "other/" + o+ "_0"
        #~ if pos0 == None:
            #~ pos0 = pos[0:3]
        #~ else:
            #~ pos = (array(pos[0:3]) - (array(pos[0:3]) - array(pos0))).tolist() + pos[3:]
            #~ pos = (array(pos[0:3]) - (array(pos[0:3]) )).tolist() + pos[3:]
        if o.find("_r") <0:
            #~ print  o
            saves[objectName] = pos
    switch_context(context_to)
    for objectName, pos in saves.iteritems():
        #~ r.client.gui.applyConfiguration (objectName, pos)
        try:
            r.moveObstacle (objectName, pos)
        except:
            pass
    r.client.gui.refresh ()
  
def init_contexts():
    init_context("twister_path", "hrp2_model", ['hyq_description', "hyq", "other"])
    init_context("twister_hyq_path", "hyq_model", ['hrp2_14_description', "hrp2_14_reduced", "other"])
    global robot_context
    robot_context = robot_contexts[0]
    set_globals()
    switch_context(0)
    r.client.gui.setVisibility('hyq_trunk_large', "OFF")
    sc(0)    
def switch_context(rid):
    save_globals()
    global cl 
    name = "robot" + str(rid)
    cl.problem.selectProblem(name)
    fullBody.client.rbprm.rbprm.selectFullBody(name)
    global robot_context
    robot_context = robot_contexts[rid]
    set_globals()
    global context
    context = rid
    
def sc(rid):
    publishRobot_and_switch(rid)

def dist(q0,q1):
    return norm(array(q0[7:]) - array(q1[7:]) )

def distq_ref(q0):
    return lambda s: dist(s.q(),q0) 

def computeNext(state, limb, projectToCom = False, max_num_samples = 10):
    global a
    t1 = time.clock()
    #~ candidates = [el for el in a if isContactReachable(state, limb, el[0], el[1], limbsCOMConstraints)[0] ]
    #~ print "num candidates", len(candidates)
    #~ t3 = time.clock()
    global context
    if context == 0:
        results = [addNewContactIfReachable(state, limb, el[0], el[1], limbsCOMConstraints, projectToCom, max_num_samples) for el in a]
    else:
        results = [addNewContactIfReachable(state, limb, el[0], el[1], None, projectToCom, max_num_samples) for el in a]
    t2 = time.clock()
    #~ t4 = time.clock()
    resultsFinal = [el[0] for el in results if el[1]]
    print "time to filter", t2 - t1
    #~ print "time to create", t4 - t3
    print "num res", len(resultsFinal)
    #sorting
    sortedlist = sorted(resultsFinal, key=distq_ref(state.q()))
    return sortedlist


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



init_contexts()
scene = "bb"
r.client.gui.createScene(scene)
b_id = 0

suppTargets = computeAffordanceCentroids(tp.afftool, ['Support']) 
leanTargets = computeAffordanceCentroids(tp.afftool, ["Support", 'Lean']) 

a = suppTargets

def setupHrp2():
    switch_context(0)
    q_init =  [
            0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0,                         	 # Free flyer 0-6
            0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
            0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # LARM       11-17
            0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # RARM       18-24
            0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
            0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
            ]; r (q_init)

    #~ q_init[1] = -1.05
    s1 = State(fullBody,q=q_init, limbsIncontact = [rLegId, lLegId]) 
    #~ s1 = State(fullBody,q=q_init, limbsIncontact = []) 
    q0 = s1.q()[:]
    r(q0)
    return s1

def setupSpidey():
    switch_context(1)
    q_0 = fullBody.getCurrentConfig(); r(q_0)
    #~ q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
    #~ q_init = [1.4497476605315172,
 #~ 0.586723585687863,
 #~ 0.7007986764501943,
 #~ 0.9992040068259673,
 #~ 0.009915872351221816,
 #~ -0.011224843288091112,
 #~ 0.0369733838268027,
 #~ -0.21440808741905482,
 #~ 0.5819742125714021,
 #~ -0.9540663648074178,
 #~ -0.444827228611522,
 #~ -0.2814234791010941,
 #~ 0.6601172395940408,
 #~ 0.05057907671648097,
 #~ 0.03165298930985974,
 #~ -0.8105513007687147,
 #~ 0.23456431240474232,
 #~ -0.6906832652645104,
 #~ 1.042653827624378]  

    #~ q_init = [1.403034485826222,
 #~ -0.0680461187483179,
 #~ 0.698067114148141,
 #~ 0.9994021773454228,
 #~ 0.029333888870596826,
 #~ -0.01699029633207977,
 #~ 0.006792695452013932,
 #~ -0.2670125225627798,
 #~ 0.5760223974412482,
 #~ -0.6323579390738086,
 #~ -0.3824808801598628,
 #~ 0.12703791928011066,
 #~ 0.41203223392262833,
 #~ 0.3545295626633363,
 #~ 0.4930851685230413,
 #~ -0.7792461682735826,
 #~ 0.1068537396039267,
 #~ -0.7497836830029866,
 #~ 1.1890550989396227]


    q_init = [1.7248956090657142,
 0.10332894313987395,
 0.7135677069780713,
 0.8053782375870717,
 0.009162336020200251,
 0.017984134378282467,
 -0.5924175190948179,
 -0.09343758894532768,
 -0.13260962085227604,
 -0.7334459269711523,
 -0.11305498904738602,
 -0.1883955962566395,
 0.8346448329401047,
 0.27875376841185046,
 0.654114442736956,
 -0.495495198017057,
 -0.22902533402342157,
 -0.0733460650991134,
 0.6485831393133032]


    #~ q_init[0:7] = tp.q_init[0:7]

    #~ q_init[1] += 1.05
    #~ q_0[2] = 1.1
    s1 = State(fullBody,q=q_init, limbsIncontact = [rLegId, lLegId, rarmId, larmId]) 
    #~ q_goal = fullBody.generateContacts(s1.q(), [0,0,1])
    #~ s1.setQ(q_goal)
    q0 = s1.q()[:]
    r(q0)
    return s1

s1_hp = setupHrp2()
states+=[s1_hp]
r(s1_hp.q())
#~ 
s1_sp = setupSpidey()
states+=[s1_sp]
r(s1_sp.q())

switch_context(0)

#~ res = computeNext(s1_hp,rarmId,True,10)
#~ s2 = res[0]; r(s2.q())
#~ s3 = computeNext(s2,rLegId,True,10)[0]; r(s3.q())
#~ res2 = computeNext(s2,larmId,True,100)
#~ s3 = res2[0]; r(s3.q())
#~ s4 = removeContact(s2,rLegId,True,0.1)[0]; r(s4.q())
#~ s5 = removeContact(s4,lLegId,True,0.1)[0]; r(s5.q())
#~ path = go0([s1,s2,s3,s4,s5], mu=0.6,num_optim=1)
#~ path = go0([s1,s2,s4,s5], mu=0.6,num_optim=1)

#~ states = [s1,s2,s4,s5]

def add(lId):
    sF = states[-1]
    ns = computeNext(sF,lId,True,10)[0]
    global states
    states +=[ns]
    r(ns.q())
    
def rm(lId):
    sF = states[-1]
    ns, res = removeContact(sF,lId,True)
    print "success ?", res
    #~ ns = removeContact(sF,lId,True)[0]
    global states
    if res:
        states +=[ns]
        r(ns.q())
    
def ast():
    global states
    states+=[res[i-1]]

def cpa(mu = 1):
    global path
    reset()
    try:
        path += [go0(states[-2:], num_optim=1, mu=mu, use_kin = context == 0)]
    except:
        global states
        states = states[:-1]

def sg(mu = 1, nopt = 2):
    ast()
    global path
    reset()
    try:
        path += [go0(states[-2:], num_optim=nopt, mu=mu, use_kin = context == 0)]
    except:
        global states
        states = states[:-1]
    
def pl(iid = None):
    global path
    if iid == None:
        iid = len(path) -1 
    play_trajectory(fullBody,pp,path[iid])
    
def go():
    return go0(states, mu=0.6,num_optim=2, use_kin = context == 0)
    
def plall(first = 0, second = 1):
    global path
    sc(first)
    pIds = [i for i in range(len(path))]
    cs = [item for sublist in [[[first,i],[second,i]] for i  in [j for j in range(len(path))]] for item in sublist]
    i = 0
    for ctx, pId in cs:
        sc(ctx)
        play_trajectory(fullBody,pp,path[pId])
        

from pickle import load, dump
def save(fname):
    sc(0)
    all_data=[[],[]]
    global states
    for s in states:
        all_data[0]+=[[s.q(), s.getLimbsInContact()]]
    sc(1)
    for s in states:
        all_data[1]+=[[s.q(), s.getLimbsInContact()]]
    f = open(fname, "w+")
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
    sc(1)
    global states
    #~ for i in range(0,len(all_data[1]),2):
        #~ states+=[State(fullBody,q=all_data[1][i], limbsIncontact = all_data[1][i+1]) ]
    for _, s in enumerate(all_data[1]):
        states+=[State(fullBody,q=s[0], limbsIncontact = s[1]) ]
        
def computeAllPath(nopt=1, mu=1, reverse = True):
    global states
    global path
    if(reverse):
        one = 0
        zero = 1
    else:
        one = 1
        zero = 0
    sc(zero)
    path = []
    sc(one)
    path = []
    for i in range(len(states)-1):
        sc(zero)        
        global path
        global states
        print 'path ' + str(i) + 'for' + str(zero)
        path += [go0([states[i],states[i+1]], num_optim=nopt, mu=mu, use_kin = context == 0)]
        reset()
        pl()
        sc(one)
        print 'path ' + str(i) + 'for' + str(one)
        global path
        global states
        path += [go0([states[i],states[i+1]], num_optim=nopt, mu=mu, use_kin = context == 0)]
        reset()
        pl()
    
    
r.client.gui.setVisibility("other", "OFF")
