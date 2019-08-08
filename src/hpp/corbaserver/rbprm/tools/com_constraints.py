from __future__ import print_function
import matplotlib
#~ matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cwc import cone_optimization
from .obj_to_constraints import ineq_from_file, rotate_inequalities
import numpy as np
import math
from numpy.linalg import norm
import time

import hpp.corbaserver.rbprm.data.com_inequalities as ine

ineqPath = ine.__path__[0] +"/"

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

def quaternion_matrix(quaternion):
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def __get_com(robot, config):
    save = robot.getCurrentConfig()
    robot.setCurrentConfig(config)
    com = robot.getCenterOfMass()
    robot.setCurrentConfig(save)
    return com

constraintsComLoaded = {}

lastspeed = np.array([0,0,0])

def get_com_constraint(fullBody, state, config, limbsCOMConstraints, interm = False, exceptList = []):
    global constraintsLoaded
    As = [];    bs = []
    fullBody.setCurrentConfig(config)
    contacts = []
    for i, v in limbsCOMConstraints.items():
        if i not in constraintsComLoaded:
            constraintsComLoaded[i] = ineq_from_file(ineqPath+v['file'])
        contact = (interm and fullBody.isLimbInContactIntermediary(i, state)) or (not interm and fullBody.isLimbInContact(i, state))
        if(contact):
            for _, el in enumerate(exceptList):
                if el == i:
                    contact = False
                    break
        if contact:
            ineq = constraintsComLoaded[i]
            qEffector = fullBody.getJointPosition(v['effector'])
            tr = quaternion_matrix(qEffector[3:7])            
            tr[:3,3] = np.array(qEffector[0:3])
            ineq_r = rotate_inequalities(ineq, tr)
            As.append(ineq_r.A); bs.append(ineq_r.b);
            contacts.append(v['effector'])
    if(len(As) > 0):
        return [np.vstack(As), np.hstack(bs)]
    else:
        print("Warning: no active contact in get_com_constraint")
        return [np.zeros([3,3]), np.zeros(3)]
    
def get_com_constraint_at_transform(pos_quat, limb, limbsCOMConstraints):
    global constraintsLoaded
    if limb not in constraintsComLoaded:
            constraintsComLoaded[limb] = ineq_from_file(ineqPath+limbsCOMConstraints[limb]['file'])
    tr = quaternion_matrix(pos_quat[3:7])            
    tr[:3,3] = np.array(pos_quat[0:3])
    ineq_r = rotate_inequalities(constraintsComLoaded[limb], tr)
    return [ineq_r.A, ineq_r.b]
        
