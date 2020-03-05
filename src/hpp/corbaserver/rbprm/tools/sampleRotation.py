import random
import numpy as np
from pinocchio import Quaternion

def Vector3FromAlpha(alpha):
    return np.matrix([np.sin(alpha), -np.cos(alpha), 0.]).T

def quatFromAlpha(alpha):
    vx = np.matrix([1,0,0]).T
    v = Vector3FromAlpha(alpha)
    return Quaternion.FromTwoVectors(vx,v)

def sampleRotationForConfig(alphaBounds,q,vPredef):
    random.seed()
    alpha = random.uniform(alphaBounds[0],alphaBounds[1])
    v = Vector3FromAlpha(alpha)
    quat = quatFromAlpha(alpha)
    q[3:7] = quat.coeffs().tolist()
    # set velocity to match this orientation :
    q[-6] = v[0,0]*vPredef 
    q[-5] = v[1,0]*vPredef  
    return q
