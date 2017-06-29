from twister_geom import *

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer

import overview_path as path_planner
import hrp2_model as model
#~ import hrp2_model_grasp as model
from hrp2_model import *
import time

from CWC_methods import compute_CWC, is_stable


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
q_init =  [1.1367890300824295,
 -0.9823899048024687,
 0.7831555831416924,
 0.9725869261886501,
 -0.01670417787545227,
 0.17618727482351285,
 -0.15084324856844844,
 0.7423030360258575,
 1.0358222971849176,
 0.3597890017868306,
 0.357278497891918,
 0.05914134721096142,
 -0.028345051065301965,
 0.0019434903164194522,
 -0.007238782778528547,
 -0.0006194219831704592,
 0.002561325329128686,
 0.17615022840721678,
 -0.7714501261689282,
 -0.15161471916593858,
 -1.331717012950026,
 -0.17541970156291684,
 -0.44996803992365064,
 0.42913378129510893,
 0.17250955701387216,
 0.2233648939861306,
 0.4032655444152788,
 -0.7984510391138848,
 0.6026937964534116,
 -0.5461419304557658,
 -0.23321151265395967,
 -0.6097721876414677,
 0.13669504870699725,
 -0.5914169563607768,
 1.1544223296395448,
 -0.4782876691483382,
 0.1444923564910395]; r (q_init)

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

s1 = State(fullBody,q=q_init, limbsIncontact = [rLegId, lLegId, rarmId])  

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

states=[s1]

from hpp.corbaserver.rbprm.rbprmstate import State
from hpp.corbaserver.rbprm.state_alg  import addNewContact, isContactReachable, closestTransform, removeContact, addNewContactIfReachable, projectToFeasibleCom

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
    
def onepath2(states_subset, ctxt=1, nopt=1, mu=1, effector = False, init_vel =None, init_acc = None):
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
    save_paths("19_06_p_save")
    save("19_06_s_save")
    
def sac():
    save("19_06_s")
    save_paths("19_06_p")

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
        path += [go0(states[-2:], num_optim=1, mu=0.3, use_kin = True)]
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

init_bezier_traj(fullBody, r, pp, configs, limbsCOMConstraints)

from pickle import dump
def computeSupportPolygon(state, filename = "sp"):
    com = array(state.getCenterOfMass())
    ps = state.getContactPosAndNormals()
    p = ps[0][0]
    N = ps[1][0]
    H = compute_CWC(p, N, state.fullBody.client.basic.robot.getMass(), mu = 1, simplify_cones = False)
    #~ return H
    res = []
    rg = 2000
    eq = rg /2
    scale = 0.001
    #~ rg = 20
    #~ eq = rg /2
    #~ scale = 0.1
    for i in range(rg):
        for j in range(rg):
            for k in range(1):
                c = com + array([(i - eq)*scale, (j - eq)*scale, k])
                w = compute_w(c)         
                #~ print "c", c  
                if(H.dot( w )<= 0).all():
                    print 'active'
                    res+=[c]
                    #~ createPtBox(r.client.gui, 0, c, color = [0,1,0,1])
                #~ else:
                    #~ createPtBox(r.client.gui, 0, c, color = [1,0,0,1])
                    
    f = open(filename, "w")
    dump(res,f)
    f.close()
    return res

path = []
all_paths = [[],[]]
from hpp.corbaserver.rbprm.state_alg import *
#~ d(0.07);e(0.01)
r(s1.q())
