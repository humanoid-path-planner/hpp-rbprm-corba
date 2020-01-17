from numpy.linalg import norm
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
import random
import numpy as np
from numpy.linalg import norm
from pinocchio import SE3,se3ToXYZQUATtuple, Quaternion
import sys
from math import isnan

eff_x_range = [-0.4,0.4]
eff_y_range=[-0.4,0.4]
kin_distance_max = 0.84

limb_ids = {'talos_rleg_rom':range(13,19), 'talos_lleg_rom':range(7,13)}

def projectInKinConstraints(fullBody,state):
    fullBody.setCurrentConfig(state.q())
    pL = np.matrix(fullBody.getJointPosition('leg_left_sole_fix_joint')[0:3])
    pR = np.matrix(fullBody.getJointPosition('leg_right_sole_fix_joint')[0:3])
    com_l = fullBody.getCenterOfMass()
    com = np.matrix(com_l)
    distance = max(norm(pL-com),norm(pR-com))
    successProj = True
    maxIt = 10000
    while distance > kin_distance_max and successProj and maxIt > 0:
      com_l[2] -= 0.001
      successProj = state.projectToCOM(com_l,0)
      if isnan(state.q()[0]):
        successProj = False
      else:
        fullBody.setCurrentConfig(state.q())
        pL = np.matrix(fullBody.getJointPosition('leg_left_sole_fix_joint')[0:3])
        pR = np.matrix(fullBody.getJointPosition('leg_right_sole_fix_joint')[0:3])
        com_l = fullBody.getCenterOfMass()
        com = np.matrix(com_l)
        distance = max(norm(pL-com),norm(pR-com))
      maxIt -= 1
    if maxIt == 0 :
      successProj = False
    return successProj
      

def __loosely_z_aligned(limb, config):
    fullBody.setCurrentConfig(config)
    effectorName = limbsCOMConstraints[limb]['effector']
    m = _getTransform(fullBody.getJointPosition(effectorName))
    P, N = fullBody.computeContactForConfig(config, limb)
    #~ N_world = m.dot(array(N[0]+[1]))[:3]
    N_world = m[:3,:3].dot(array(N[0]))
    N_world = N_world / np.linalg.norm(N_world)
    return N_world.dot(array([0,0,1])) > 0.7


def projectMidFeet(fullBody,s):
    fullBody.setCurrentConfig(s.q())
    com = np.zeros(3)
    num = 0.
    for limb in s.getLimbsInContact():
        com += np.array(fullBody.getJointPosition(fullBody.dict_limb_joint[limb])[0:3])
        num += 1. 
    com /= num			
    com[2] = fullBody.getCenterOfMass()[2]
    successProj = s.projectToCOM(com.tolist(),0)
    if successProj:
        return s
    else :
        return None

def sampleRotationAlongZ(placement):
    alpha = random.uniform(0.,2.*np.pi)    
    rot = placement.rotation
    rot[0,0] = np.cos(alpha)
    rot[0,1] = - np.sin(alpha)
    rot[1,0] = np.sin(alpha)
    rot[1,1] = np.cos(alpha)
    placement.rotation = rot    
    return placement

# create a state with legs config and root orientation along z axis random, the rest is equal to the referenceConfig
def createRandomState(fullBody,limbsInContact,root_at_origin = True):
    extraDof = int(fullBody.client.robot.getDimensionExtraConfigSpace())
    q0 = fullBody.referenceConfig[::]
    if extraDof > 0:
        q0 += [0]*extraDof    
    qr = fullBody.shootRandomConfig()
    if root_at_origin : 
      q0[0:2] = [0,0]
      q0[2] = qr[2]
      q0[3:7] = [0,0,0,1]
    else : 
      root = SE3.Identity()
      root.translation=np.matrix(qr[0:3]).T
      # sample random orientation along z : 
      root = sampleRotationAlongZ(root)
      q0[0:7] = se3ToXYZQUATtuple(root)
    # apply random config to legs (FIXME : ID hardcoded for Talos)
    q0[7:19] = qr[7:19]
    fullBody.setCurrentConfig(q0)
    s0 = State(fullBody,q=q0,limbsIncontact=limbsInContact)
    return s0

def sampleRandomStateFlatFloor(fullBody,limbsInContact,z):
    success = False
    it = 0
    while not success and it < 10000:
        it +=1
        s0 = createRandomState(fullBody,limbsInContact)
        # try to project feet in contact (N = [0,0,1] and p[2] = z)
        n = [0,0,1]
        for limb in limbsInContact:
            p = fullBody.getJointPosition(fullBody.dict_limb_joint[limb])[0:3]
            p[2] = z
            s0, success = StateHelper.addNewContact(s0,limb,p,n,lockOtherJoints=True)
            if not success:
                break
        if success : 
            # check stability
            success = fullBody.isStateBalanced(s0.sId,5)
        if success :
            success = projectInKinConstraints(fullBody,s0)
    if not success :
        print("Timeout for generation of static configuration with ground contact")
        sys.exit(1)
    return s0

# all limb have a contact at z choosen randomly between values in zInterval,
# exepct movingLimb which have a contact at z = z_moving
def sampleRandomStateStairs(fullBody,limbsInContact,zInterval,movingLimb,z_moving):
    success = False
    it = 0
    while not success and it < 10000:
        it +=1
        s0 = createRandomState(fullBody,limbsInContact)        
        # try to project feet in contact (N = [0,0,1] and p[2] = z)
        n = [0,0,1]
        for limb in limbsInContact:
            p = fullBody.getJointPosition(fullBody.dict_limb_joint[limb])[0:3]
            if limb == movingLimb:
                p[2] = z_moving
            else:
                p[2] = zInterval[random.randint(0,len(zInterval)-1)]
            s0, success = StateHelper.addNewContact(s0,limb,p,n,lockOtherJoints=True)
            if not success:
                break
        if success : 
            # check stability
            success = fullBody.isStateBalanced(s0.sId,5)
    if not success :
        print("Timeout for generation of static configuration with ground contact")
        sys.exit(1)
    return s0

def sampleRandomTranstionFromState(fullBody,s0,limbsInContact,movingLimb,z):
    it = 0 
    success = False
    n = [0,0,1]
    vz = np.matrix(n).T  
    while not success and it < 10000:
        it += 1 
        # sample a random position for movingLimb and try to project s0 to this position
        qr = fullBody.shootRandomConfig()
        q1 = s0.q()[::]
        q1[limb_ids[movingLimb][0]:limb_ids[movingLimb][1]] = qr[limb_ids[movingLimb][0]:limb_ids[movingLimb][1]]
        s1 = State(fullBody,q=q1,limbsIncontact=limbsInContact)
        fullBody.setCurrentConfig(s1.q())
        p = fullBody.getJointPosition(fullBody.dict_limb_joint[movingLimb])[0:3]
        p[0] =random.uniform(eff_x_range[0],eff_x_range[1])
        p[1] =random.uniform(eff_y_range[0],eff_y_range[1])
        p[2] = z
        s1,success = StateHelper.addNewContact(s1,movingLimb,p,n)
        if success :
            """
            # force root orientation : (align current z axis with vertical)
            quat_1 = Quaternion(s1.q()[6],s1.q()[3],s1.q()[4],s1.q()[5])
            v_1 = quat_1.matrix() * vz
            align = Quaternion.FromTwoVectors(v_1,vz)
            rot = align * quat_1
            q_root = s1.q()[0:7]
            q_root[3:7] = rot.coeffs().T.tolist()[0]
            """
            root =SE3.Identity()
            root.translation = np.matrix(s1.q()[0:3]).T
            root = sampleRotationAlongZ(root)
            success = s1.projectToRoot(se3ToXYZQUATtuple(root))
        # check if new state is in static equilibrium
        if success : 
            # check stability
            success = fullBody.isStateBalanced(s1.sId,3)
        if success :
            success = projectInKinConstraints(fullBody,s1)            
        # check if transition is feasible according to CROC
        if success :
            #success = fullBody.isReachableFromState(s0.sId,s1.sId) or (len(fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId, numPointsPerPhases=0)) > 0)
            success = fullBody.isReachableFromState(s0.sId,s1.sId)
    return success,s1
            
## return two states (with adjacent ID in fullBody) 
## LimbsInContact must contains the feet limbs, they are all in contact for both states 
## the only contact difference between both states is for movingLimbs
def sampleRandomTransitionFlatFloor(fullBody,limbsInContact,movingLimb, z=0):
    random.seed()    
    success = False
    it_tot = 0  
    while not success and it_tot < 1000:
        it_tot += 1
        s0 = sampleRandomStateFlatFloor(fullBody,limbsInContact,z)
        success,s1 = sampleRandomTranstionFromState(fullBody,s0,limbsInContact,movingLimb,z)
    if not success :
        print("Timeout for generation of feasible transition")
        sys.exit(1)    
    # recreate the states to assure the continuity of the index in fullBody :     
    state0 = State(fullBody,q=s0.q(),limbsIncontact=s0.getLimbsInContact())
    state1 = State(fullBody,q=s1.q(),limbsIncontact=s1.getLimbsInContact())
    return state0,state1

## return two states (with adjacent ID in fullBody) 
## LimbsInContact must contains the feet limbs, they are all in contact for both states 
## the only contact difference between both states is for movingLimbs
## the limbs have a contact z choosen randomly in zInterval,
## exept for moving limb which go from zInterval_moving[0] to zInterval_moving[1]
def sampleRandomTransitionStairs(fullBody,limbsInContact,zInterval,movingLimb,zInterval_moving):
    random.seed()    
    success = False
    it_tot = 0  
    while not success and it_tot < 1000:
        it_tot += 1
        s0 = sampleRandomStateStairs(fullBody,limbsInContact,zInterval,movingLimb,zInterval_moving[0])
        success,s1 = sampleRandomTranstionFromState(fullBody,s0,limbsInContact,movingLimb,zInterval_moving[1])
    if not success :
        print("Timeout for generation of feasible transition")
        sys.exit(1)    
    # recreate the states to assure the continuity of the index in fullBody :     
    state0 = State(fullBody,q=s0.q(),limbsIncontact=s0.getLimbsInContact())
    state1 = State(fullBody,q=s1.q(),limbsIncontact=s1.getLimbsInContact())
    return state0,state1
