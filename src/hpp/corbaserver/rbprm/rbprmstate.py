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

from hpp.corbaserver.rbprm import Client as RbprmClient
from hpp.corbaserver import Client as BasicClient
#from hpp.corbaserver.rbprm.tools.com_constraints import *
from numpy import array


## Creates a state given an Id pointing to an existing c++ state
#
#  A RbprmDevice robot is a set of two robots. One for the 
#  trunk of the robot, one for the range of motion
class State (object):
    ## Constructor
    def __init__ (self, fullBody, sId=-1, isIntermediate = False, q = None, limbsIncontact = []):
        assert ((sId > -1 and len(limbsIncontact) == 0) or sId == -1), "state created from existing id can't specify limbs in contact"
        self.cl = fullBody.clientRbprm.rbprm
        if(sId == -1):
            self.sId = self.cl.createState(q, limbsIncontact)
            self.isIntermediate = False    
        else:
            self.sId = sId
            self.isIntermediate = isIntermediate    
        self.fullBody = fullBody
        self.H_h = None
        self.__last_used_friction = None
            
    ## assert for case where functions can't be used with intermediate state
    def _cni(self):
        assert not self.isIntermediate, "method can't be called with intermediate state"
            
    ## Get the state configuration
    def q(self):
        self._cni()
        return self.cl.getConfigAtState(self.sId)
        
    ## Set the state configuration
    # \param q configuration of the robot
    # \return whether or not the configuration was successfully set
    def setQ(self, q):
        self._cni()
        return self.cl.setConfigAtState(self.sId, q)    > 0
        
    # \param state1 current state considered
    # \param limb name of the limb for which the request aplies
    # \return whether the limb is in contact at this state
    def isLimbInContact(self, limbname):
        if(self.isIntermediate):
            return self.cl.isLimbInContactIntermediary(limbname, self.sId) >0
        else:            
            return self.cl.isLimbInContact(limbname, self.sId) >0
                
    # 
    # \param state1 current state considered
    # \param limb name of the limb for which the request aplies
    # \return all limbs in contact at this state
    def getLimbsInContact(self):
        return [limbName for limbName in self.fullBody.limbNames if self.isLimbInContact(limbName)]
        
    ## Get the end effector position for a given configuration, assuming z normal
    # \param limbName name of the limb from which to retrieve a sample
    # \param configuration configuration of the robot
    # \return world position of the limb end effector given the current robot configuration.
    # array of size 4, where each entry is the position of a corner of the contact surface
    def getEffectorPosition(self, limbName):
        self._cni()
        return self.cl.getEffectorPosition(limbName,self.q())
     
    ## Get the end effector position for a given configuration, assuming z normal
    # \param limbName name of the limb from which to retrieve a sample
    # \param configuration configuration of the robot
    # \return world position of the limb end effector given the current robot configuration.
    # array of size 4, where each entry is the position of a corner of the contact surface
    def getContactPosAndNormals(self):
        if(self.isIntermediate):  
            rawdata = self.cl.computeContactPointsAtState(self.sId, 1)
        else:            
            rawdata = self.cl.computeContactPointsAtState(self.sId, 0) 
        return [[b[i:i+3] for i in range(0, len(b), 6)] for b in rawdata], [[b[i+3:i+6] for i in range(0, len(b), 6)] for b in rawdata]
     
    ## Get the end effector position for a given configuration, assuming z normal
    # \param limbName name of the limb from which to retrieve a sample
    # \param configuration configuration of the robot
    # \return world position of the limb end effector given the current robot configuration.
    # array of size 4, where each entry is the position of a corner of the contact surface
    def getContactPosAndNormalsForLimb(self, limbName):
        assert self.isLimbInContact(limbName), "in getContactPosAndNormals: limb " + limbName +  "is not in contact at  state" + str(stateId) 
        if(self.isIntermediate):  
            rawdata = self.cl.computeContactPointsAtStateForLimb(self.sId,1, limbName)
        else:            
            rawdata = self.cl.computeContactPointsAtStateForLimb(self.sId,0, limbName) 
        return [[b[i:i+3] for i in range(0, len(b), 6)] for b in rawdata], [[b[i+3:i+6] for i in range(0, len(b), 6)] for b in rawdata]
        
    def getCenterOfContactForLimb(self,limbName):
        assert self.isLimbInContact(limbName), "in getContactPosAndNormals: limb " + limbName +  "is not in contact at  state" + str(stateId)
        return self.cl.computeCenterOfContactAtStateForLimb(self.sId,self.isIntermediate, limbName)
    
    ## Get position of center of mass
    def getCenterOfMass (self):
        q0 = self.fullBody.client.robot.getCurrentConfig()
        self.fullBody.client.robot.setCurrentConfig(self.q())
        c = self.fullBody.client.robot.getCenterOfMass()
        self.fullBody.client.robot.setCurrentConfig(q0)
        return c
    
    
    ## Get the end effector position for a given configuration, assuming z normal
    # \param limbName name of the limb from which to retrieve a sample
    # \param configuration configuration of the robot
    # \return world position of the limb end effector given the current robot configuration.
    # array of size 4, where each entry is the position of a corner of the contact surface
    def getContactCone(self, friction):
        if (self.H_h == None or self.__last_used_friction != friction):
            self.__last_used_friction = friction
            if(self.isIntermediate):  
                rawdata = self.cl.getContactIntermediateCone(self.sId,friction)
            else:            
                rawdata = self.cl.getContactCone(self.sId,friction) 
            self.H_h =  array(rawdata)
        return self.H_h[:,:-1], self.H_h[:, -1]
        
    def projectToCOM(self, targetCom, maxNumSample = 0, toNewState=False):
        if toNewState:
            return self.fullBody.projectToCom(self.sId, targetCom)
        else:
            return self.fullBody.projectStateToCOM(self.sId, targetCom,maxNumSample)     > 0

    def projectToRoot(self,targetRoot):
        return self.fullBody.projectStateToRoot(self.sId,targetRoot) > 0
        
    def getComConstraint(self, limbsCOMConstraints, exceptList = []):
        return get_com_constraint(self.fullBody, self.sId, self.cl.getConfigAtState(self.sId), limbsCOMConstraints, interm = self.isIntermediate, exceptList = exceptList)

    def contactsVariations(self,state):
        return self.fullBody.getContactsVariations(self.sId,state.sId)


    def isValid(self):
        return self.fullBody.isConfigValid(self.cl.getConfigAtState(self.sId))

    def isBalanced(self,robustness = 0):
        return self.fullBody.isStateBalanced(self.sId,robustness)

    def robustness(self):
        return self.fullBody.clientRbprm.rbprm.isStateBalanced(self.sId)


class StateHelper(object):

    ## tries to add a new contact to the state
    ## if the limb is already in contact, replace the
    ## previous contact. Only considers kinematic limitations.
    ## collision and equilibrium are NOT considered.
    # \param state State considered
    # \param limbName name of the considered limb to create contact with
    # \param p 3d position of the point
    # \param n 3d normal of the contact location center
    # \param max_num_sample number of times it will try to randomly sample a configuration before failing
    # \param lockOtherJoints : if True, the values of all the joints exepct the ones of 'limbName' are constrained to be constant.
    #                This only affect the first projection, if max_num_sample > 0 and the first projection was unsuccessfull, the parameter is ignored
    # \return (State, success) whether the creation was successful, as well as the new state
    @staticmethod
    def addNewContact(state, limbName, p, n, num_max_sample = 0, lockOtherJoints= False):
        sId = state.cl.addNewContact(state.sId, limbName, p, n, num_max_sample,lockOtherJoints)
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
    @staticmethod
    def removeContact(state, limbName):
        sId = state.cl.removeContact(state.sId, limbName)
        if(sId != -1):
            s = State(state.fullBody, sId = sId)
            return s, True
        return state, False

    @staticmethod
    def generateStateInContact(fullBody,q,direction,acceleration):
        sId = fullBody.generateStateInContact(q,direction,acceleration)
        s = State(fullBody,sId=sId)
        return s
        
    @staticmethod
    def moveAndContact(state,rootOffset,limbName):
        s, success = StateHelper.removeContact(state, limbName)
        assert success
        success = s.projectToRoot((array(s.q()[:3]) + array(rootOffset)).tolist()+s.q()[3:7])
        if not success:
            return state, False
        sId = s.fullBody.clientRbprm.rbprm.generateContactState(s.sId, limbName,rootOffset)
        if sId < 0:
            return state, False
        s = State(s.fullBody,sId=sId)
        return s, True
