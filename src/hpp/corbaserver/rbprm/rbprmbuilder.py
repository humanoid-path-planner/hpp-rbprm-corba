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
from hpp.corbaserver.robot import Robot
import hpp.gepetto.blender.exportmotion as em

## Load and handle a RbprmDevice robot for rbprm planning
#
#  A RbprmDevice robot is a dual representation of a robots. One robot describes the 
#  trunk of the robot, and a set of robots describe the range of motion of each limb of the robot.
class Builder (Robot):
  ## Constructor
  def __init__ (self, load = True):
      self.tf_root = "base_link"
      self.clientRbprm = RbprmClient ()
      self.load = load

  ## Virtual function to load the robot model.
  #
  # \param urdfName urdf description of the robot trunk,
  # \param urdfNameroms either a string, or an array of strings, indicating the urdf of the different roms to add.
  # \param rootJointType type of root joint among ("freeflyer", "planar",
  #        "anchor"),
  # \param meshPackageName name of the meshpackage from where the robot mesh will be loaded
  # \param packageName name of the package from where the robot will be loaded
  # \param urdfSuffix optional suffix for the urdf of the robot package
  # \param srdfSuffix optional suffix for the srdf of the robot package
  def loadModel (self, urdfName, urdfNameroms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix):
    Robot.__init__(self,urdfName,rootJointType, False)
    if(isinstance(urdfNameroms, list)):
      for urdfNamerom in urdfNameroms:
        self.clientRbprm.rbprm.loadRobotRomModel(urdfNamerom, rootJointType, packageName, urdfNamerom, urdfSuffix, srdfSuffix)
    else:
      self.clientRbprm.rbprm.loadRobotRomModel(urdfNameroms, rootJointType, packageName, urdfNameroms, urdfSuffix, srdfSuffix)
    self.clientRbprm.rbprm.initNewProblemSolver()
    self.clientRbprm.rbprm.loadRobotCompleteModel(urdfName, rootJointType, packageName, urdfName, urdfSuffix, srdfSuffix)
    self.client.robot.meshPackageName = meshPackageName
    self.meshPackageName = meshPackageName
    self.packageName = packageName
    self.urdfName = urdfName
    self.urdfSuffix = urdfSuffix
    self.srdfSuffix = srdfSuffix



  ## Init RbprmShooter
  #
  def initshooter (self):
    return self.clientRbprm.rbprm.initshooter ()

  ## Sets limits on robot orientation, described according to Euler's ZYX rotation order
  #
  # \param bounds 6D vector with the lower and upperBound for each rotation axis in sequence
  def boundSO3 (self,  bounds):
    return self.clientRbprm.rbprm.boundSO3 (bounds)

  ## Specifies a preferred affordance for a given rom.
  # This constrains the planner to accept a rom configuration only if
  # it collides with a surface the normal of which has these properties.
  #
  # \param rom name of the rome,
  # \param affordances list of affordance names
  def setAffordanceFilter (self, rom, affordances):
    return self.clientRbprm.rbprm.setAffordanceFilter (rom, affordances)

  ## Specifies a rom constraint for the planner.
  # A configuration will be valid if and only if the considered rom collides
  # with the environment.
  #
  # \param romFilter array of roms indicated by name, which determine the constraint.
  def setFilter (self, romFilter):
    return self.clientRbprm.rbprm.setFilter (romFilter)

  ## Export a computed path for blender
    #
  # \param problem the problem associated with the path computed for the robot
  # \param stepsize increment along the path
  # \param pathId if of the considered path
  # \param filename name of the output file where to save the output
  def exportPath (self, viewer, problem, pathId, stepsize, filename):
    em.exportPath(viewer, self.client.robot, problem, pathId, stepsize, filename)

  ## set a reference position of the end effector for the given ROM
  # This reference will be used in the heuristic that choose the "best" contact surface
  # and approximate the contact points in the kinodynamic planner
  # \param romName the name of the rom
  # \param ref the 3D reference position of the end effector, expressed in the root frame
  def setReferenceEndEffector(self,romName,ref):
    return self.clientRbprm.rbprm.setReferenceEndEffector(romName,ref)
