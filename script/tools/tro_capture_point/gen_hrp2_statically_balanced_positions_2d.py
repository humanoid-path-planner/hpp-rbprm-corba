from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

from hpp.corbaserver.rbprm.tools.cwc_trajectory import *

from hpp import Error as hpperr
from numpy import array, matrix
import random

import sample_com_vel as scv


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
fullBody.setJointBounds ("base_joint_xyz", [-0.135,2, -1, 1, 0, 2.2])


ps = ProblemSolver( fullBody )
r = Viewer (ps)

n_samples = 10000

rLegId = '0rLeg'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.105]
rLegNormal = [0,0,1]       
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 10000, "manipulability", 0.1)
                                                                                                
lLegId = '1lLeg'                                                                                
lLeg = 'LLEG_JOINT0'                                                                     
lLegx = 0.09; lLegy = 0.05      
lLegOffset = [0,0,-0.105]
lLegNormal = [0,0,1]                                                                  
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 10000, "manipulability", 0.1)


rarmId = '3Rarm'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [0,0,-0.1]
rArmNormal = [0,0,1]
rArmOffset = [-0.045,-0.01,-0.085]
rArmNormal = [0,1,0]
rArmx = 0.015; rArmy = 0.02
#~ fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, n_samples, "manipulability", 0.05, "_6_DOF", True)

#~ AFTER loading obstacles
larmId = '4Larm'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.045,0.01,-0.085]
lArmNormal = [0,1,0]
lArmx = 0.015; lArmy = 0.02
#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, n_samples, "manipulability", 0.05, "_6_DOF", True)


rarmId = '3Rarm'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [0,0,-0.1075]
rArmNormal = [0,0,1]

rArmOffset = [-0.01,-0.045,0.085]
rArmNormal = [0,1,0]
rArmx = 0.024; rArmy = 0.024
#disabling collision for hook
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 10000, "manipulability", 0.1, "_6_DOF", True)

#~ AFTER loading obstacles
larmId = '4Larm'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [0,0,-0.1075]
lArmNormal = [0,0,1]
lArmOffset = [0.01,-0.045,0.085]
lArmNormal = [0,1,0]
lArmx = 0.024; lArmy = 0.024
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, "manipulability", 0.1, "_6_DOF", True)



limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},  
                        lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}, 
                        larmId : {'file': "hrp2/LA_com.ineq", 'effector' : lHand}, 
                        rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand} }
#~ limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},  
                        #~ lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}, 
                        #~ rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand} }

 #~ 

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])

import quaternion as quat


def _getTransform(qEffector):
    q0 = quat.Quaternion(qEffector[3:7])
    rot = q0.toRotationMatrix() #compute rotation matrix local -> world
    p = qEffector[0:3] #(0,0,0) coordinate expressed in effector fram
    rm=np.zeros((4,4))
    for k in range(0,3):
        for l in range(0,3):
            rm[k,l] = rot[k,l]
    for m in range(0,3):
        rm[m,3] = qEffector[m]
    rm[3,3] = 1
    return rm
            
from numpy.linalg import norm
def __loosely_z_aligned(limb, config):
    fullBody.setCurrentConfig(config)
    effectorName = limbsCOMConstraints[limb]['effector']
    m = _getTransform(fullBody.getJointPosition(effectorName))
    P, N = fullBody.computeContactForConfig(config, limb)
    #~ N_world = m.dot(array(N[0]+[1]))[:3]
    N_world = m[:3,:3].dot(array(N[0]))
    N_world = N_world / np.linalg.norm(N_world)
    return N_world.dot(array([0,0,1])) > 0.7

def draw_cp(cid, limb, config):
    global r
    #~ posetc = fullBody.getEffectorPosition(limb, config)
    P, N = fullBody.computeContactForConfig(config, limb)
    fullBody.setCurrentConfig(config)
    effectorName = limbsCOMConstraints[limb]['effector']
    limbId = limb
    m = _getTransform(fullBody.getJointPosition(effectorName))
    scene = "qds"+limb+str(cid)
    r.client.gui.createScene(scene)
    for i in range(4):
        #~ pos = posetc[2*i]
        print "P", array(P[i]+[1])
        print "N", array(N[i]+[1])
        print m.dot(array(P[i]+[1]))
        pos = m.dot(array(P[i]+[1]))[:3]
        print "pos", pos
        r.client.gui.addBox(scene+"/b"+str(i),0.01,0.01,0.01, [1,0,0,1])
        r.client.gui.applyConfiguration(scene+"/b"+str(i),pos.tolist()+[1,0,0,0])
        r.client.gui.refresh()  
    r.client.gui.addSceneToWindow(scene,0)
    
def draw_com(config):
    fullBody.setCurrentConfig(config)
    effectorName = limbsCOMConstraints[limb]['effector']
    limbId = limb
    m = _getTransform(fullBody.getJointPosition(effectorName))
    scene = "qds"+limb+str(cid)
    r.client.gui.createScene(scene)
    for i in range(4):
        #~ pos = posetc[2*i]
        print "P", array(P[i]+[1])
        print "N", array(N[i]+[1])
        print m.dot(array(P[i]+[1]))
        pos = m.dot(array(P[i]+[1]))[:3]
        print "pos", pos
        r.client.gui.addBox(scene+"/b"+str(i),0.01,0.01,0.01, [1,0,0,1])
        r.client.gui.applyConfiguration(scene+"/b"+str(i),pos.tolist()+[1,0,0,0])
        r.client.gui.refresh()  
    r.client.gui.addSceneToWindow(scene,0)


def fill_contact_points(limbs, fullbody, data):
    data["contact_points"] = {}
    for limb in limbs:
        print "LIMB ", limb
        effector = limbsCOMConstraints[limb]['effector']
        P, N = fullBody.computeContactForConfig(fullbody.getCurrentConfig(), limb)  
        print "\t P \n"
        for _,p in enumerate (P):
            print "\t ", p
        print "\t N \n"
        for _,p in enumerate (N):
            print "\t ", p
        data["contact_points"][effector] = {}
        data["contact_points"][effector]["P"] = P
        data["contact_points"][effector]["N"] = N
    return data
    
    
from random import randint

def pos_quat_to_pinocchio(q):
    q_res = q[:]
    quat_end = q[4:7]
    q_res[6] = q[3]
    q_res[3:6] = quat_end
    return q_res

def gen_contact_candidates_one_limb(limb, data, num_candidates = 10, projectToObstacles = False):
    effectorName = limbsCOMConstraints[limb]['effector']
    candidates = []
    config_candidates = [] #DEBUG
    if(projectToObstacles):
        candidates = fullBody.getContactSamplesProjected(limb, fullBody.getCurrentConfig(),[0,0,1],10000)
        random.shuffle(candidates)
        print "num candidates", len(candidates)
        print "num limlb", limb
        if(len(candidates) > 0):
            raw_input("enter somethin")
        for i in range(len(candidates)):
            q_contact = candidates[i]
            fullBody.setCurrentConfig(q_contact)
            m = _getTransform(fullBody.getJointPosition(effectorName))
            candidates.append(m)
            config_candidates.append(q_contact) #DEBUG
    else:
        for i in range(num_candidates):
            q_contact = fullBody.getSample(limb,randint(0,n_samples - 1))
            fullBody.setCurrentConfig(q_contact)
            m = _getTransform(fullBody.getJointPosition(effectorName))
            candidates.append(m)
            config_candidates.append(q_contact) #DEBUG
            #~ candidates.append(pos_quat_to_pinocchio(fullBody.getJointPosition(effectorName)))
    data[effectorName]["transforms"] = candidates
    return config_candidates #DEBUG
        

def removeLimb(limb, limbs):
    return [l for l in limbs if l != limb]
    
#~ 
#~ def find_limbs_broken(target_c, config, limbs):
    #~ res = []
    #~ for limb in limbs:
        #~ nLimbs = removeLimb(limb, limbs)
        #~ state_id = fullBody.createState(config, nLimbs)
        #~ if (fullBody.projectStateToCOM(state_id,target_c)):
            #~ res.append(limb)
    #~ return res
    
def predict_com_for_limb_candidate(c, limb, limbs, res, data, config_gepetto, orig_contact_points):
    effector = limbsCOMConstraints[limb]['effector']
    contact_points = {}
    maintained_limbs = [l for l in limbs if limb != l]
    for k, v in orig_contact_points.iteritems():
        if k != effector:
            contact_points[k] = v
    #~ success, dc, c_final, v0 = scv.com_pos_after_t(c, res["q"], contact_points, data["v0"])
    success, dc, c_final, v0 = scv.com_pos_after_t(c, res["q"], contact_points, data["v0"])
    print "c for limb ", c_final ,  ":", limb
    print "success ",success
    effector_data = {}
    print "maintained_limbs ", maintained_limbs 
    state_id = fullBody.createState(config_gepetto, maintained_limbs)
    if(success and fullBody.projectStateToCOM(state_id,c_final.tolist())): #all good, all contacts kinematically maintained):
        effector_data["dc"] = dc
        effector_data["c_final"] = c_final
        proj_config = fullBody.getConfigAtState(state_id)
        fullBody.setCurrentConfig(proj_config)
        effector_data["projected_config"] = proj_config  #DEBUG
        data["candidates_per_effector"][effector] = effector_data
        return True
    print "projection failed for limb ", limb
    return False
    
    
def gen_contact_candidates(limbs, config_gepetto, res, contact_points, num_candidates = 10, projectToObstacles = False):
    #first generate a com translation current configuration
    fullBody.setCurrentConfig(config_gepetto)
    c = matrix(fullBody.getCenterOfMass())
    success, v0, dc = scv.gen_com_vel(res["q"], contact_points) 
    if(success):
        data = {}
        data["v0"] = v0
        data["candidates_per_effector"] = {}
        configs_candidates = {} #DEBUG      
        data["init_config"] = config_gepetto  #DEBUG    
        for limb in limbsCOMConstraints.keys(): 
            #~ print "limb ", limb
            if (predict_com_for_limb_candidate(c, limb, limbs, res, data, config_gepetto, contact_points)):  #all good, all contacts kinematically maintained
               val  = gen_contact_candidates_one_limb(limb, data["candidates_per_effector"], num_candidates, projectToObstacles) #DEBUG 
               if len(val) > 0:
                    configs_candidates[limb] = val
        if(len(data["candidates_per_effector"].keys()) >0):
            #~ if((data["candidates_per_effector"].has_key('RARM_JOINT5') and not data["candidates_per_effector"].has_key('LARM_JOINT5')) or
                #~ (data["candidates_per_effector"].has_key('LARM_JOINT5') and not data["candidates_per_effector"].has_key('RARM_JOINT5'))):
                    #~ raise ValueError("RARM AND LARM candidates not coherent (if there is candidates for one there should be for the other)");
            data["candidates_per_effector"]["config_candidates"] = configs_candidates  #DEBUG     
            if (not res.has_key("candidates")):
                res["candidates"] = []  
            res["candidates"].append(data)
            
        
    
from numpy import cos, sin, pi
def __eulerToQuat(pitch, roll, yaw):
    t0 = cos(yaw * 0.5);
    t1 = sin(yaw * 0.5);
    t2 = cos(roll * 0.5);
    t3 = sin(roll * 0.5);
    t4 = cos(pitch * 0.5);
    t5 = sin(pitch * 0.5);
    w= t0 * t2 * t4 + t1 * t3 * t5;
    x= t0 * t3 * t4 - t1 * t2 * t5;
    y= t0 * t2 * t5 + t1 * t3 * t4;
    z= t1 * t2 * t4 - t0 * t3 * t5;
    return [w, x, y, z]
    #~ 
#~ void SampleRotation(model::DevicePtr_t so3, ConfigurationPtr_t config, JointVector_t& jv)
#~ {
    #~ std::size_t id = 1;
    #~ if(so3->rootJoint())
    #~ {
        #~ Eigen::Matrix <value_type, 3, 1> confso3;
        #~ id+=1;
        #~ model::JointPtr_t joint = so3->rootJoint();
        #~ for(int i =0; i <3; ++i)
        #~ {
            #~ joint->configuration()->uniformlySample (i, confso3);
        #~ }
        #~ Eigen::Quaterniond qt = Eigen::AngleAxisd(confso3(0), Eigen::Vector3d::UnitZ())
          #~ * Eigen::AngleAxisd(confso3(1), Eigen::Vector3d::UnitY())
          #~ * Eigen::AngleAxisd(confso3(2), Eigen::Vector3d::UnitX());
        #~ std::size_t rank = 3;
        #~ (*config)(rank+0) = qt.w();
        #~ (*config)(rank+1) = qt.x();
        #~ (*config)(rank+2) = qt.y();
        #~ (*config)(rank+3) = qt.z();
    #~ }
    #~ if(id < jv.size())
        #~ SampleRotationRec(config,jv,id);
#~ }
    
from random import uniform
def _boundSO3(q, num_limbs):
    q[:3] = [0,0,0.5]
    limb_weight = float(4 - num_limbs)
    #generate random angle 
    rot_y = uniform(-pi/(4+limb_weight), pi/(4+limb_weight))
    rot_x = uniform(-pi/8, pi/(3+limb_weight))
    
    rot_z = 0;
    q[3:7] = __eulerToQuat(rot_x, rot_y, rot_z)
    return q
    

def _genbalance(limbs, unstable):
    for i in range(10000):
        q = fullBody.client.basic.robot.shootRandomConfig()
        q = _boundSO3(q, len(limbs))
        if fullBody.isConfigValid(q)[0] and __loosely_z_aligned(limbs[0], q) and __loosely_z_aligned(limbs[1], q):
			if (unstable and not fullBody.isConfigBalanced(q, limbs, 5)) or  (not unstable and fullBody.isConfigBalanced(q, limbs, 5)): 
        #~ if fullBody.isConfigValid(q)[0] and  __loosely_z_aligned(limbs[0], q) and __loosely_z_aligned(limbs[1], q):
				return q
    print "can't generate equilibrium config"

all_qs = []
all_states = []
all_data = []
def gen(limbs, num_samples = 1000, coplanar = True, num_candidates_per_config = 0, num_contact_candidates = 10, q_entries = None, projectToObstacles = False, unstable = False):
    q_0 = fullBody.getCurrentConfig(); 
    #~ fullBody.getSampleConfig()
    qs = []; qs_gepetto = []; states = []    
    data = {}
    fill_contact_points(limbs, fullBody, data)
    if(q_entries != None):
        num_samples = len(q_entries)
        print "num_sample", num_samples, len(q_entries)
    for i in range(num_samples):
        if(q_entries == None):
            if(coplanar):
                q = fullBody.generateGroundContact(limbs)
            else:
                q = _genbalance(limbs, unstable)
        else:
            q = q_entries[i]
        q_gep = q[:]
        quat_end = q[4:7]
        q[6] = q[3]
        q[3:6] = quat_end
        res = {}
        res["q"] = q[:]
        for _ in range(num_candidates_per_config):
            gen_contact_candidates(limbs, q_gep, res, data["contact_points"], num_contact_candidates, projectToObstacles)
        if(num_candidates_per_config == 0 or res.has_key("candidates")): #contact candidates found
            states.append(res)
            qs.append(q)
            qs_gepetto.append(q_gep)
    global all_qs
    all_qs += [qs_gepetto]
    fname = ""
    for lname in limbs:
        fname += lname + "_"
    fname += "configs"
    if(coplanar):
        fname += "_coplanar"
    if(unstable):
        fname += "_unstable"
    data["samples"] = states
    all_data.append(data)
    from pickle import dump
    #~ f1=open("configs_feet_on_ground_static_eq", 'w+')
    f1=open(fname, 'w+')
    dump(data, f1)
    f1.close()
    all_states.append(states)
    return all_states

j=0

q_init =  [
        0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0,                           # Free flyer 0-6
        0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,        # LARM       11-17
        0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,        # RARM       18-24
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        ]; r (q_init)
        
#~ limbs = [[lLegId,rLegId],[lLegId,rLegId, rarmId], [lLegId,rLegId, larmId], [lLegId,rLegId, rarmId, larmId] ]
#~ limbs = [[lLegId,rLegId] ]
limbs = [[lLegId,rLegId, rarmId, larmId]]
#~ limbs = [[larmId, rarmId]]


q_init = fullBody.getCurrentConfig()

#~ gen(limbs[0], 1000)
for ls in limbs:
    gen(ls, 1, False, unstable=True)
#~ gen(limbs[0], 1000, unstable=True)
    
i = 0
a = None
b = None

#~ a = all_states[0][0]['candidates'][0]
#~ b = a ['candidates_per_effector']
def init():
    r(a['init_config'])
    
#~ b = a ['candidates_per_effector']

def rleg():
    r(b['RLEG_JOINT5']['projected_config'])
    
def lleg():
    r(b['LLEG_JOINT5']['projected_config'])
    
def larm():
    r(b['LARM_JOINT5']['projected_config'])
    
def rarm():
    r(b['RARM_JOINT5']['projected_config'])
    
def rlegi(j):                 
    r(b['config_candidates'][rLegId][j])
                             
def llegi(j):                 
    r(b['config_candidates'][lLegId][j])
                             
def larmi(j):                 
    r(b['config_candidates'][larmId][j])
                             
def rarmi(j):                 
    r(b['config_candidates'][rarmId][j])
    
    
    
def c1():
    r(b['config_candidates'][0][0])
    
def c2():
    r(b['config_candidates'][1][0])
    
def c3():
    r(b['config_candidates'][2][0])
    
def c4():
    r(b['config_candidates'][3][0])
    
    
def cij(l,m):
    r(b['config_candidates'][l][m])
    
def inc():
    global i
    global a
    global b
    i+=1
    a = all_states[0][i]['candidates'][0]
    b = a ['candidates_per_effector']
    
def a():
    return a
    
def b():
    return b

