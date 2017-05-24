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

## algorithmic methods on state

## tries to add a new contact to the state
## if the limb is already in contact, replace the 
## previous contact. Only considers kinematic limitations.
## collision and equilibrium are NOT considered.
# \param state State considered
# \param limbName name of the considered limb to create contact with
# \param p 3d position of the point
# \param n 3d normal of the contact location center
# \return (success,NState) whether the creation was successful, as well as the new state
def addNewContact(state, limbName, p, n):
    return State(state.fullBody, state.cl.addNewContact(state.sId, limbName, p, n))
    

## Given a target location
## computes the closest transformation from the current configuration
## which is the most likely feaasible through projection
# \param state State considered
# \param limbName name of the considered limb to create contact with
# \param p 3d position of the point
# \param n 3d normal of the contact location center
# \return (success,NState) whether the creation was successful, as well as the new state
def predictTransform(state, limbName, p, n):

## Uses a lp to determine whether the com kinematic constraints
## will be satisfied at a given configuration
## if the limb is already in contact, replace the 
## previous contact. Only considers kinematic limitations.
## collision and equilibrium are NOT considered.
# \param state State considered
# \param limbName name of the considered limb to create contact with
# \param p 3d position of the point
# \param n 3d normal of the contact location center
# \return (success,NState) whether the creation was successful, as well as the new state
def isContactReachable(state, limbName, p, n):
    
