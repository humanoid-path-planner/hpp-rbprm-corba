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

from hpp.corbaserver.rbprm.rbprmstate import State
from lp_find_point import find_valid_c_cwc, lp_ineq_4D
from hpp.corbaserver.rbprm.tools.com_constraints import *

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
        print "In isContactReachable no stability, Lp failed (should not happen) ", status_ok
        return False, [-1,-1,-1]
    return (res[3] >= 0), res[0:3]
    

## tries to add a new contact to the state
## if the limb is already in contact, replace the 
## previous contact. Only considers kinematic limitations.
## collision and equilibrium are NOT considered.
# \param state State considered
# \param limbName name of the considered limb to create contact with
# \param p 3d position of the point
# \param n 3d normal of the contact location center
# \return (State, success) whether the creation was successful, as well as the new state
def addNewContact(state, limbName, p, n):
    sId = state.cl.addNewContact(state.sId, limbName, p, n)
    if(sId != -1):
        return State(state.fullBody, sId = state.cl.addNewContact(sId, limbName, p, n)), True
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
# \return (State, success) whether the creation was successful, as well as the new state
def addNewContactIfReachable(state, limbName, p, n, limbsCOMConstraints):
    ok, res  = isContactReachable(state, limbName, p, n, limbsCOMConstraints)
    if(ok):
        return addNewContact(state, limbName, p, n)
    else:
        return state, False
