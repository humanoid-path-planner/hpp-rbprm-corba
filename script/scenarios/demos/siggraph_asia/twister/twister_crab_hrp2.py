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

all_paths = [[],[]]
all_paths_with_pauses = [[],[]]

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
    init_context("twister_path", "hrp2_model", ['crab_description', "crab_model", "other"])
    init_context("twister_hyq_path", "crab_model", ['hrp2_14_description', "hrp2_14_reduced", "other"])
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

def computeNext(state, limb, projectToCom = False, max_num_samples = 10, mu = 0.6):
    global a
    t1 = time.clock()
    #~ candidates = [el for el in a if isContactReachable(state, limb, el[0], el[1], limbsCOMConstraints)[0] ]
    #~ print "num candidates", len(candidates)
    #~ t3 = time.clock()
    global context
    if context == 0:
        results = [addNewContactIfReachable(state, limb, el[0], el[1], limbsCOMConstraints, projectToCom, max_num_samples, mu) for el in a]
    else:
        results = [addNewContactIfReachable(state, limb, el[0], el[1], None, projectToCom, max_num_samples, mu) for el in a]
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
    q_init =  [1.465951314830228,
 -0.9176187491454572,
 0.556996332024304,
 0.8949373222781749,
 0.004991393277543945,
 0.09328595707567328,
 0.43630265344046976,
 -0.03547067445226664,
 0.6709217094864276,
 0.2612513403690041,
 0.018790826902884042,
 1.0472,
 -0.12716727062306263,
 0.014121758070711707,
 0.0349066,
 -0.005336299638361007,
 0.09555844824139348,
 -0.15100592820137693,
 1.0472,
 -0.6564341232652954,
 -0.14124478581057368,
 0.0349066,
 0.006442792051943824,
 0.0976416359755596,
 0.048557325328327426,
 0.025298798438044116,
 0.5088603837394758,
 0.09277155854142789,
 0.06056704285485918,
 -0.40397983120647746,
 -0.3442592958713712,
 -0.5709648671805531,
 -0.5813136986745363,
 -0.8406045693532259,
 0.5508411880503161,
 0.06566162355557537,
 0.5817029063967701]; r (q_init)

    #~ q_init[1] = -1.05
    s1 = State(fullBody,q=q_init, limbsIncontact = [rLegId, lLegId]) 
    #~ s1 = State(fullBody,q=q_init, limbsIncontact = []) 
    q0 = s1.q()[:]
    r(q0)
    
    fullBody.setRefConfig(q0)
    return s1

def setupSpidey():
    switch_context(1)
    q_0 = fullBody.getCurrentConfig(); r(q_0)
    
    q_init = [1.8644820749752133,
 -0.05907835662004786,
 0.36244227429714687,
 0.796622257271051,
 -0.01123057286487888,
 -0.04124810131291858,
 0.6029638858104035,
 -0.03547067445226691,
 0.6709217094864277,
 0.26125134036900377,
 0.018790826902884056,
 1.0472,
 -0.12716727062306263,
 -0.4194108294469009,
 -0.5282737768672232,
 0.6483375025026266,
 -0.36743071075802436,
 -1.3354344044074755,
 0.8731280507393882,
 -0.6564341232652954,
 -0.14124478581057368,
 0.0349066,
 1.0782285414916801,
 -1.2054219129402555,
 0.6375403038065977]




    #~ q_init[0:7] = tp.q_init[0:7]

    #~ q_init[1] += 1.05
    #~ q_0[2] = 1.1
    s1 = State(fullBody,q=q_init, limbsIncontact = ['r1', 'r3', 'l3']) 
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
    
def rm(lId, nu = 1):
    sF = states[-1]
    ns, res = removeContact(sF,lId,True,friction = nu)
    print "success ?", res
    #~ ns = removeContact(sF,lId,True)[0]
    global states
    if res:
        states +=[ns]
        r(ns.q())
        return True
    else :
        return False
    
def ast():
    global states
    states+=[res[i-1]]

def cpa(mu = 1):
    global path
    reset()
    try:
        s = max(norm(array(states[i+1].q()) - array(states[i].q())), 1.) * 1
        if(context == 0):
            s = max(norm(array(states[i+1].q()) - array(states[i].q())), 1.) * 0.6
        #~ path += [go0(states[-2:], num_optim=1, mu=mu, use_kin = context == 0)]
        path += [go0(states[-2:], num_optim=0, mu=mu, use_kin = context == 0)]
        return True
    except:
        e = sys.exc_info()[0]
        print( "Error in rm : " + str(e))
        global states
        states = states[:-1]
        return False

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
    
def plc(ctx = 0, iid = None):
    sc(ctx)
    pl(iid)

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
        
def plallp(first = 0, second = 1):
    global all_paths_with_pauses
    pIds = [i for i in range(len(all_paths_with_pauses[0]))]
    cs = [item for sublist in [[[first,i],[second,i]] for i  in [j for j in range(len(all_paths_with_pauses[0]))]] for item in sublist]
    i = 0
    print cs
    for ctx, pId in cs:
        sc(ctx)
        play_trajectory(fullBody,pp,all_paths_with_pauses[ctx][pId])
        

from pickle import load, dump
def save(fname):
    sc(0)
    all_data=[[],[]]
    global states
    for s in states:
        all_data[0]+=[[s.q(), s.getLimbsInContact()]]
    sc(1)
    global states
    for s in states:
        all_data[1]+=[[s.q(), s.getLimbsInContact()]]
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
    sc(1)
    global states
    states = []
    #~ for i in range(0,len(all_data[1]),2):
        #~ states+=[State(fullBody,q=all_data[1][i], limbsIncontact = all_data[1][i+1]) ]
    for _, s in enumerate(all_data[1]):
        states+=[State(fullBody,q=s[0], limbsIncontact = s[1]) ]
    r(states[0].q())
        
def computeAllPath(nopt=1, mu=1, reverse = False, effector = False, start = 0):
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
    for ol in range(start, len(states)-1):
        sc(zero)        
        reset()
        global path
        global states
        print 'path ' + str(ol) + 'for' + str(zero)
        s = max(norm(array(states[i+1].q()) - array(states[i].q())), 1.) * 0.4
        if(ol > len(path) -1):
            path += [go0([states[ol],states[ol+1]], num_optim=nopt, mu=mu, use_kin = context == 0, s=s, effector = effector)]
        else:
            path[ol]=go0([states[ol],states[ol+1]], num_optim=nopt, mu=mu, use_kin = ctxt == 0, s=s, effector = effector)
        all_paths[zero] = path[:]
        reset()
        pl()
        sc(one)
        reset()
        print 'path ' + str(ol) + 'for' + str(one)
        global path
        global states
        s = max(norm(array(states[i+1].q()) - array(states[i].q())), 1.) * 1
        if(ol > len(path) -1):
            path += [go0([states[ol],states[ol+1]], num_optim=nopt, mu=mu, use_kin = context == 0, s=s, effector = effector)]
        else:
            path[ol]=go0([states[ol],states[ol+1]], num_optim=nopt, mu=mu, use_kin = ctxt == 0, s=s, effector = effector)
        all_paths[one] = path[:]
        reset()
        pl()
        save_paths("test_paths")
    
def onepath(ol, ctxt, nopt=1, mu=1, effector = False):
    reset()
    sc(ctxt)
    global path
    global states
    s = max(norm(array(states[ol+1].q()) - array(states[ol].q())), 1.) * 1
    if ctxt == 0:
        print "ctxt", ctxt
        print "q", len(states[ol+1].q())
        s = max(norm(array(states[ol+1].q()) - array(states[ol].q())), 1.) * 0.6
    if(ol > len(path) -1):
        path += [go0([states[ol],states[ol+1]], num_optim=nopt, mu=mu, use_kin = ctxt == 0, s=s, effector = effector)]
    else:
        path[ol]=go0([states[ol],states[ol+1]], num_optim=nopt, mu=mu, use_kin = ctxt == 0, s=s, effector = effector)
    all_paths[ctxt] = path
    
def save_paths(fname):
    f = open(fname, "w")
    dump(all_paths,f)
    f.close()
    #now try with latest paths
    global all_path
    global path
    sc(0)
    all_paths[0] = path[:]
    sc(1)
    global all_path
    global path
    all_paths[1] = path[:]
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
    
    sc(1)
    global path
    path = all_paths[1][:]
    
def sh(ctxt, i):
    sc(ctxt)
    r(states[i].q())
    
def lc():
    load_save("crab_19_06_s")
    load_paths("crab_19_06_p")
    save_paths("crab_19_06_p_save")
    save("crab_19_06_s_save")
    
def sac():
    save("crab_19_06_s")
    save_paths("crab_19_06_p")
    
r.client.gui.setVisibility("other", "OFF")
#~ r.client.gui.setVisibility("bos", "OFF")

def addVoidWhileOtherMoves(ctx1=0, ctx2=1):    
    sc(0)
    global path
    all_paths[0] = path[:]
    sc(1)
    global path
    all_paths[1] = path[:]
    global all_paths_with_pauses
    all_paths_with_pauses = [[],[]]
    for i in range(len(all_paths[0])):
        #get the last configuration
        num_clones = len(all_paths[ctx1][i])
        if i == 0:
            ref = all_paths[ctx2][0][0][:]
        else:
            ref = all_paths[ctx2][i-1][-1][:]
        val = [ref for _ in range(num_clones)]
        all_paths_with_pauses[ctx2] += [val[:]]
        all_paths_with_pauses[ctx1] += [all_paths[ctx1][i][:]]
        
        
        num_clones = len(all_paths[ctx2][i])
        
        ref = all_paths[ctx1][i][-1][:]
        val = [ref for _ in range(num_clones)]
        all_paths_with_pauses[ctx1] += [val[:]]
        all_paths_with_pauses[ctx2] += [all_paths[ctx2][i][:]]
        
        
def breathe(ctx, state, configs, height = 0.001, time=24):
    sc(ctx)
    initQ = state.q()[:]
    initCom = state.getCenterOfMass()
    num_breaths = max(len(configs) / 24,1)
    height_step = height * num_breaths / len(configs) * 2
    switch = len(configs) / 2
    delta = [height_step * i for i in range(switch)] + [height_step * (len(configs) - i) for i in range(switch, len(configs))]
    assert(len(delta) == len(configs))
    res = []
    for i,q in enumerate(configs):
        x = initCom[:]; x[2]+=delta[i]
        print "x", x
        succ = state.fullBody.projectStateToCOM(state.sId ,x, 1)
        if not succ:
            print "failt to breahte, ", i
            state.setQ(initQ)
            return configs
        else:
            print "success to breahte, ", i
            res+=[state.q()[:]]
    state.setQ(initQ)
    return res
    
def export(ctx1=0, ctx2=1):
    addVoidWhileOtherMoves(ctx1, ctx2)
    p0 = [val for sublist in all_paths_with_pauses[0] for val in sublist]
    p1 = [val for sublist in all_paths_with_pauses[1] for val in sublist]
    sc(0)
    fullBody.exportMotion(r,p0,"hrp2_twister_path")
    sc(1)
    fullBody.exportMotion(r,p1,"hyq_twister_path")

import random
import sys

def generatePossibilities():
    global states
    s = states[-1]
    lNames = fullBody.limbNames[:]
    lix = s.getLimbsInContact()
    #first generate contact creation possibilities
    poss  = [ ["add", l] for l in lNames if(len(lix) > 1) or lix[0] != l]
    global context
    if(len(lix) > 3):    
        poss += [ ["rm", l] for l in s.getLimbsInContact()]    
    return random.sample(poss, len(poss))
    
    
from numpy.linalg import norm
def one_step(mu = 1):
    reset()
    poss = generatePossibilities()
    found = False
    global states
    s = states[-1]
    while(not found and len(poss) > 0):
        ac, lId = poss.pop(0)
        print "selected " , ac, lId , "\n nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn"
        if(ac == 'rm'):
            ok = rm(lId, mu);
            if(not ok):
                pass
            else:
                found = cpa()
                print "found PATH ?", cpa
        else: # add
            res = computeNext(s,lId,True,10, mu)
            print "CANDIDATES ", len(res)
            while(not found and len(res) >0):
                print "GOING IN"
                sc = res.pop(0)
                if (norm(array(s.getEffectorPosition(lId) [0]) - array(sc.getEffectorPosition(lId) [0])) < 0.02):
                    print "too close"
                else:
                    ol = len(path)
                    try:
                        global path
                        path += [go0([states[-1],sc], num_optim=0, mu=mu, use_kin = context == 0)]                        
                        if len(path) > 0 and len(path[-1]) == 0:
                            path = path[:-1]
                            print "empty path"
                        else:
                            print "found PATH !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \, !!!!!!!!!!!!!!!!!!!!!"
                            found = True         
                            states+=[sc]       
                    except:
                        e = sys.exc_info()[0]
                        print( "Error in go0 : " + str(e))
        if found:
            onepath(len(path)-1,context,4)
            pl()
            sac()
                
def one_step_both():
    sc(0); one_step();
    sc(1);one_step();
    
sc(1);sc(0)
