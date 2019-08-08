#!/usr/bin/env python
# Copyright (c) 2014 CNRS
# Author: Steve Tonneau
#
# This file is part of hpp-rbprm-corba.
# hpp-rbprm-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation-corba.  If not, see
# <http://www.gnu.org/licenses/>.

from __future__ import print_function
from hpp.corbaserver.rbprm.rbprmstate import State
from lp_find_point import find_valid_c_cwc, find_valid_c_cwc_qp, lp_ineq_4D
from hpp.corbaserver.rbprm.tools.com_constraints import *
from CWC_methods import compute_CWC, is_stable

## algorithmic methods on state
   

## Given a target location
## computes the closest transformation from the current configuration
## which is the most likely feaasible through projection
# \param state State considered
# \param limbName name of the considered limb to create contact with
# \param p 3d position of the point
# \param n 3d normal of the contact location center
# \return a 7D array (position + quaternion) in world coordinates
def closestTransform(state, limbName, p, n):
    return state.cl.computeTargetTransform(limbName, state.q(), p, n)

## Uses a lp to determine whether the com kinematic constraints
## will be satisfied at a given configuration
## if the limb is already in contact, replace the 
## previous contact. Only considers kinematic limitations.
## collision and equilibrium are NOT considered.
# \param state State considered
# \param limbName name of the considered limb to create contact with
# \param p 3d position of the point
# \param n 3d normal of the contact location center
# \param limbsCOMConstraints COM constraints per effector
# \return whether the creation was successful, as well as the new state
def isContactReachable(state, limbName, p, n, limbsCOMConstraints):
    tr = closestTransform(state, limbName, p, n)
    new_ineq = get_com_constraint_at_transform(tr, limbName, limbsCOMConstraints)
    active_ineq = state.getComConstraint(limbsCOMConstraints, [limbName])
    res_ineq = [np.vstack([new_ineq[0],active_ineq[0]]), np.hstack([new_ineq[1],active_ineq[1]])]
    success, status_ok , res = lp_ineq_4D(res_ineq[0],-res_ineq[1])
    if not success:
        print("In isContactReachable no stability, Lp failed (should not happen) ", status_ok)
        return False, [-1,-1,-1]
    return (res[3] >= 0), res[0:3]
    

## Computes the intermediary state between two states
## that is the state where the broken configuration have been remove
# \param sfrom init state
# \param sto target state
# \return whether the creation was successful, as well as the new state
def computeIntermediateState(sfrom, sto):
    sid = sfrom.cl.computeIntermediary(sfrom.sId, sto.sId)
    return State(sfrom.fullBody, sid, False)

## tries to add a new contact to the state
## if the limb is already in contact, replace the 
## previous contact. Only considers kinematic limitations.
## collision and equilibrium are NOT considered.
# \param state State considered
# \param limbName name of the considered limb to create contact with
# \param p 3d position of the point
# \param n 3d normal of the contact location center
# \return (State, success) whether the creation was successful, as well as the new state
def addNewContact(state, limbName, p, n, num_max_sample = 0):
    sId = state.cl.addNewContact(state.sId, limbName, p, n, num_max_sample)
    if(sId != -1):
        return State(state.fullBody, sId = sId), True
    return state, False

## tries to remove a new contact from a state
## if the limb is already in contact, replace the 
## previous contact. Only considers kinematic limitations.
## collision and equilibrium are NOT considered.
# \param state State considered
# \param limbName name of the considered limb to create contact with
# \return (State, success) whether the removal was successful, as well as the new state
def removeContact(state, limbName, projectToCOM = False, friction = 0.6):
    sId = state.cl.removeContact(state.sId, limbName)
    if(sId != -1):        
        s = State(state.fullBody, sId = sId)
        if projectToCOM:
            return s, projectToFeasibleCom(s, ddc =[0.,0.,0.], max_num_samples = 10, friction = friction)
        else:
            return s, True
    return state, False

## tries to add a new contact to the state
# if the limb is already in contact, replace the 
# previous contact. Only considers kinematic limitations.
# collision and equilibrium are NOT considered. Before doing the costful profjection
# first solves a reachability test to see if target is reachable
# \param state State considered
# \param limbName name of the considered limb to create contact with
# \param p 3d position of the point
# \param n 3d normal of the contact location center
# \param max_num_samples max number of sampling in case projection ends up in collision
# \return (State, success) whether the creation was successful, as well as the new state
def addNewContactIfReachable(state, limbName, p, n, limbsCOMConstraints, projectToCom = False, max_num_samples = 0, friction = 0.6):
    if limbsCOMConstraints == None:
        ok = True
    else:
        ok, res  = isContactReachable(state, limbName, p, n, limbsCOMConstraints)
    if(ok):
        s, success = addNewContact(state, limbName, p, n, max_num_samples)
        if success and projectToCom:
            success = projectToFeasibleCom(s, ddc =[0.,0.,0.], max_num_samples = max_num_samples, friction = friction)
        return s, success
    else:
        return state, False
        
## Project a state to a static equilibrium location
# \param state State considered
# \param ddc name considered acceleeration (null by default)
# \param max_num_samples max number of sampling in case projection ends up in collision
# \param friction considered friction coefficient
# \return whether the projection was successful
def projectToFeasibleCom(state,  ddc =[0.,0.,0.], max_num_samples = 10, friction = 0.6):
    #~ H, h = state.getContactCone(friction)
    ps = state.getContactPosAndNormals()
    p = ps[0][0]
    N = ps[1][0]
    print("p", p)
    print("N", N)
    #~ try:
    H = compute_CWC(p, N, state.fullBody.client.basic.robot.getMass(), mu = friction, simplify_cones = False)
    c_ref = state.getCenterOfMass()
        #~ Kin = state.getComConstraint(limbsCOMConstraints, [])
        #~ res = find_valid_c_cwc_qp(H, c_ref, Kin, ddc, state.fullBody.getMass())
    success, p_solved , x = find_valid_c_cwc_qp(H, c_ref,None, ddc, state.fullBody.getMass())
    #~ except:
        #~ success = False
    if success:
        x = x.tolist()
        #~ if x[2] < 0.9:
        x[2] += 0.35
        for i in range(10):
            if state.fullBody.projectStateToCOM(state.sId ,x, max_num_samples):
                print("success after " + str(i) + " trials")
                return True
            else:
                x[2]-=0.05
    else:
        print("qp failed")
    return False;
    
def isContactCreated(s1, s2):
    s15 = computeIntermediateState(s1,s2)
    lcS15 = s15.getLimbsInContact()
    lcS1 = s1.getLimbsInContact()
    lcS2 = s2.getLimbsInContact()
    if(len(lcS15) == len(lcS1)): #no contact breaks
        return True
    else:
        return not (len(lcS15) == len(lcS2))
        
def planToStates(fullBody, configs):
    states = []
    for i, _ in enumerate(configs):
        states += [State(fullBody,i) ]
    return states
        
