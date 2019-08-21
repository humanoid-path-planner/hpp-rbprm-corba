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

from hpp.corbaserver.rbprm import rbprmstate
from hpp.corbaserver.rbprm.rbprmstate import State

def _interpolateState(ps, fullBody, stepsize, pathId, robustnessTreshold = 0, filterStates = False, testReachability = True, quasiStatic = False, erasePreviousStates = True):
  if(filterStates):
        filt = 1
  else:
        filt = 0
        
  #discretize path
  totalLength = ps.pathLength(pathId)
  configsPlan = []; step = 0.
  configSize = fullBody.getConfigSize() - len(ps.configAtParam(pathId,0.))
  z = [0. for _ in range(configSize) ]
  while step < totalLength:
    configsPlan += [ps.configAtParam(pathId,step) + z[:]]
    step += stepsize
  configsPlan += [ps.configAtParam(pathId,totalLength)+ z[:]]
    
  configs = fullBody.clientRbprm.rbprm.interpolateConfigs(configsPlan, robustnessTreshold, filt, testReachability, quasiStatic, erasePreviousStates)
  firstStateId = fullBody.clientRbprm.rbprm.getNumStates() - len(configs)
  return [ State(fullBody, i) for i in range(firstStateId, firstStateId + len(configs)) ], configs
          

def _guidePath(problemSolver, fromPos, toPos):
  ps = problemSolver
  ps.setInitialConfig (fromPos)
  ps.resetGoalConfigs()
  ps.addGoalConfig(toPos)
  ps.solve ()
  return ps.numberPaths() - 1
  

class FewStepPlanner(object):
  def __init__ (self, client, problemSolver, rbprmBuilder, fullBody, planContext="rbprm_path", fullBodyContext="default", pathPlayer = None, viewer = None ):
    self.fullBody =  fullBody
    self.rbprmBuilder =  rbprmBuilder
    self.client   =  client
    self.planContext   =  planContext
    self.fullBodyContext   =  fullBodyContext
    self.problemSolver   =  problemSolver
    self.pathPlayer   =  pathPlayer
    self.viewer   =  viewer
    
  def setPlanningContext(self):
    self.client.problem.selectProblem(self.planContext) 
    
  def setFullBodyContext(self):
    self.client.problem.selectProblem(self.fullBodyContext ) 
    
  def setCurrentContext(self,context):
    return self.client.problem.selectProblem(context) 
    
  def currentContext(self):
    return self.client.problem.getSelected("problem")[0]
    
  def _actInContext(self, context,f,*args):
    oldContext = self.currentContext()
    self.setCurrentContext(context)
    res = f(*args)
    self.setCurrentContext(oldContext)
    return res
    
  def guidePath(self, fromPos, toPos, displayPath = False):
    pId =  self._actInContext(self.planContext,_guidePath,self.problemSolver, fromPos, toPos)
    self.setPlanningContext()
    names =  self.rbprmBuilder.getAllJointNames()[1:]
    if displayPath:
      if self.pathPlayer is None:
        print "can't display path, no path player given"
      else:
        self.pathPlayer(pId)        
    self.client.problem.movePathToProblem(pId,self.fullBodyContext, names)
    self.setFullBodyContext()
    return pId
    
  def interpolateStates(self, stepsize, pathId = 1, robustnessTreshold = 0, filterStates = False, testReachability = True, quasiStatic = False, erasePreviousStates = True):
    return _interpolateState(self.problemSolver, self.fullBody, stepsize, pathId, robustnessTreshold, filterStates, testReachability, quasiStatic, erasePreviousStates)
    
  def goToQuasiStatic(self, currentState, toPos, stepsize = 0.002, goalLimbsInContact = None, goalNormals = None, displayGuidePath = False):
    pId = self.guidePath(currentState.q()[:7], toPos, displayPath = displayGuidePath)    
    self.fullBody.setStartStateId(currentState.sId)
    targetState = currentState.q()[:]; targetState[:7] = toPos
    if goalLimbsInContact is None:
      goalLimbsInContact = currentState.getLimbsInContact()
    if goalNormals is None:
      goalNormals = [[0.,0.,1.] for _ in range(len(goalLimbsInContact))]
    self.fullBody.setEndState(targetState, goalLimbsInContact,goalNormals)
    return self.interpolateStates(stepsize,pathId=pId,robustnessTreshold = 1, filterStates = True,quasiStatic=True, erasePreviousStates = False)
    
       
              
