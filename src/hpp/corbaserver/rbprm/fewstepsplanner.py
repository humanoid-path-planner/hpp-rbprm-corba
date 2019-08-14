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

def interpolateState(fullBody, stepsize, pathId = 1, robustnessTreshold = 0, filterStates = False, testReachability = True, quasiStatic = False, erasePreviousStates = False):
  if(filterStates):
        filt = 1
  else:
        filt = 0
  configs = fullBody.clientRbprm.rbprm.interpolate(stepsize, pathId, robustnessTreshold, filt, testReachability, quasiStatic, erasePreviousStates)
  firstStateId = fullBody.clientRbprm.rbprm.getNumStates() - len(configs)
  return [ State(fullBody, i) for i in range(firstStateId, firstStateId + len(configs)) ]
          

def guidePath(problemSolver, fromPos, toPos):
  ps = problemSolver
  ps.setInitialConfig (fromPos)
  ps.addGoalConfig(toPos)
  ps.solve ()
  return ps.numberPaths() - 1
  

class FewStepPlanner(object):
  def __init__ (self, client, problemSolver, rbprmBuilder, fullBody, planContext="rbprm_path", fullBodyContext="default", pathPlayer = None ):
    self.fullBody =  fullBody
    self.rbprmBuilder =  rbprmBuilder
    self.client   =  client
    self.planContext   =  planContext
    self.fullBodyContext   =  fullBodyContext
    self.problemSolver   =  problemSolver
    self.pathPlayer   =  pathPlayer
    
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
    
  def guidePath(self, fromPos, toPos):
    pId =  self._actInContext(self.planContext,guidePath,self.problemSolver, fromPos, toPos)
    self.setPlanningContext()
    names =  self.rbprmBuilder.getAllJointNames()[1:]
    self.pathPlayer(pId)
    self.client.problem.movePathToProblem(pId,self.fullBodyContext, names)
    self.setFullBodyContext()
    return pId
       
              
