from twister_geom import *
from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer
from numpy import array, sort
from numpy.linalg import norm
from plan_execute import a, b, c, d, e, init_plan_execute
from bezier_traj import go0, init_bezier_traj

from hpp.corbaserver.rbprm.rbprmstate import State
from hpp.corbaserver.rbprm.state_alg  import addNewContact, isContactReachable, closestTransform, removeContact, addNewContactIfReachable, projectToFeasibleCom

robot_contexts = []
robot_context = None

cl = Client()

def save_globals():		
	robot_context["states"] = states
	robot_context["configs"] = configs
	
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
	
	states 	= robot_context["states"]
	tp 		= robot_context["tp"]
	model	= robot_context["model"]
	ps		= robot_context["ps"]
	r		= robot_context["r"]
	pp		= robot_context["pp"]
	fullBody= robot_context["fullBody"]
	configs = robot_context["configs"]
	cl		= robot_context["cl"]
	limbsCOMConstraints = model.limbsCOMConstraints
	lLegId = model.lLegId
	lLegId = model.rLegId
	larmId = model.larmId
	rarmId = model.rarmId
	path_planner = tp
	
	init_plan_execute(fullBody, r, path_planner, pp)
	init_bezier_traj(fullBody, r, pp, configs, limbsCOMConstraints)
	
	
	#~ states 	= robot_context["states"]
	#~ tp 		= robot_context["tp"]
	#~ model	= robot_context["model"]
	#~ ps		= robot_context["ps"]
	#~ r		= robot_context["r"]
	#~ pp		= robot_context["pp"]
	#~ fullBody= robot_context["fullBody"]
	#~ configs = robot_context["configs"]
	#~ limbsCOMConstraints = model.limbsCOMConstraints
	#~ lLegId = model.lLegId
	#~ lLegId = model.rLegId
	#~ larmId = model.larmId
	#~ rarmId = model.rarmId
	#~ path_planner = tp

import importlib

def init_context(path, wb ):	
	global robot_contexts
	rid = len(robot_contexts) 
	
	path_planner_1 = importlib.import_module(path)
	path_planner_1.cl.problem.selectProblem("wb_robot" + str(rid))	
	model_1  = importlib.import_module(wb)	
	model_1.fullBody.setJointBounds ("base_joint_xyz", [-2,2.5, -2, 2, 0, 2.2])
	ps1 =  path_planner_1.ProblemSolver( model_1.fullBody )
	r  = path_planner_1.Viewer (ps1, viewerClient=path_planner_1.r.client)
	robot_contexts[rid] = {"model" : model_1, 
	"states" : [], "tp" :  path_planner_1,
	"ps" : ps1, 
	"fullBody" : model_1.fullBody,
	"r" : r, "fullBody" : model_1.fullBody,
	"configs" : [],
	"pp" : PathPlayer (model_1.fullBody.client.basic, r),
	"cl" : path_planner_1.cl }
	
	#~ cl.problem.selectProblem("wb_robot2")	
	#~ import twister_spidey_path as path_planner_2
	#~ import spidey_model as model_2
	#~ ps2 =  path_planner_2.ProblemSolver( model_2.fullBody )
	#~ robot2_context = {"model" : model_2, "twister_path" : path_planner_2, "states" : [], "tp" :  path_planner_1, "ps" : ps2, "fullBody" : model_2.fullBody, "r" = path_planner_2.Viewer (ps2, viewerClient=path_planner_2.r.client), "fullBody" : model_2.fullBody, "configs" : [] }
	
def init_contexts():
	init_context("twister_path", "hrp2_model")
	init_context("twister_spidey_path", "spidey_model")
	global robot_context
	robot_context = robot_contexts[0]
	set_globals()

def switch_context(rid):
	save_globals()
	global cl 
	cl.problem.selectProblem("wb_robot" + str(rid))
	robot_context = robot_contexts[rid]
	set_globals()
	
		

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
    results = [addNewContactIfReachable(state, limb, el[0], el[1], limbsCOMConstraints, projectToCom, max_num_samples) for el in a]
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

q_0 = fullBody.getCurrentConfig(); r(q_0)
q_0[2] = 1.05
s1 = State(fullBody,q=q_0, limbsIncontact = [rLegId, lLegId])  

q0 = s1.q()[:]
a = computeAffordanceCentroids(tp.afftool, ["Support"]) 
scene = "bb"
r.client.gui.createScene(scene)
b_id = 0
