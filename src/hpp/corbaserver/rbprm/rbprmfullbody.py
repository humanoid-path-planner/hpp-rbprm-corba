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
from numpy import array, matrix
from hpp_spline import bezier

## Load and handle a RbprmFullbody robot for rbprm planning
#
#  A RbprmDevice robot is a set of two robots. One for the 
#  trunk of the robot, one for the range of motion
class FullBody (Robot):
     ## Constructor
     def __init__ (self, load = True):
          self.tf_root = "base_link"
          self.clientRbprm = RbprmClient ()
          self.load = load
          self.limbNames = []
     
     ## Virtual function to load the fullBody robot model.
     #
     # \param urdfName urdf description of the fullBody robot
     # \param rootJointType type of root joint among ("freeflyer", "planar",
     #          "anchor"), WARNING. Currently RB-PRM only considerds freeflyer roots
     # \param meshPackageName name of the meshpackage from where the robot mesh will be loaded
     # \param packageName name of the package from where the robot will be loaded
     # \param urdfSuffix optional suffix for the urdf of the robot package
     # \param srdfSuffix optional suffix for the srdf of the robot package
     def loadFullBodyModel (self, urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix):
          Robot.__init__(self,urdfName,rootJointType, False)
          self.clientRbprm.rbprm.loadFullBodyRobot(urdfName, rootJointType, packageName, urdfName, urdfSuffix, srdfSuffix, self.client.problem.getSelected("problem")[0])
          self.client.robot.meshPackageName = meshPackageName
          self.meshPackageName = meshPackageName
          self.packageName = packageName
          self.urdfName = urdfName
          self.urdfSuffix = urdfSuffix
          self.srdfSuffix = srdfSuffix
          self.octrees={}
     
     # Virtual function to load the fullBody robot model.
     #
     def loadFullBodyModelFromActiveRobot (self, urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix):
          Robot.__init__(self,urdfName,rootJointType, False)
          self.clientRbprm.rbprm.loadFullBodyRobotFromExistingRobot()
          self.client.robot.meshPackageName = meshPackageName
          self.meshPackageName = meshPackageName
          self.packageName = packageName
          self.urdfName = urdfName
          self.urdfSuffix = urdfSuffix
          self.srdfSuffix = srdfSuffix
          self.octrees={}


     ## Add a limb to the model
     #
     # \param id: user defined id for the limb. Must be unique.
     #  The id is used if several contact points are defined for the same limb (ex: the knee and the foot)
     # \param name: name of the joint corresponding to the root of the limb.
     # \param effectorName name of the joint to be considered as the effector of the limb
     # \param offset position of the effector in joint coordinates relatively to the effector joint
     # \param unit normal vector of the contact point, expressed in the effector joint coordinates
     # \param x half width of the default support polygon of the effector
     # \param y half height of the default support polygon of the effector
     # \param collisionObjects objects to be considered for collisions with the limb. TODO remove
     # \param nbSamples number of samples to generate for the limb
     # \param resolution, resolution of the octree voxels. The samples generated are stored in an octree data
     # \param resolution, resolution of the octree voxels. The samples generated are stored in an octree data
     # structure to perform efficient proximity requests. The resulution of the octree, in meters, specifies the size
     # of the unit voxel of the octree. The larger they are, the more samples will be considered as candidates for contact.
     # This can be problematic in terms of performance. The default value is 3 cm.
     def addLimb(self, limbId, name, effectorname, offset, normal, x, y, samples):
          self.clientRbprm.rbprm.addLimb(limbId, name, effectorname, offset, normal, x, y, samples, "EFORT", 0.03)
          self.limbNames += [limbId]

     ## Add a limb no used for contact generation to the model
     #
     # \param id: user defined id for the limb. Must be unique.
     #  The id is used if several contact points are defined for the same limb (ex: the knee and the foot)
     # \param name: name of the joint corresponding to the root of the limb.
     # \param effectorName name of the joint to be considered as the effector of the limb
     # \param collisionObjects objects to be considered for collisions with the limb. TODO remove
     # \param nbSamples number of samples to generate for the limb
     def addNonContactingLimb(self, limbId, name, effectorname, samples):
          self.clientRbprm.rbprm.addNonContactingLimb(limbId, name, effectorname, samples)
          self.limbNames += [limbId]

     ## Add a limb to the model
     #
     # \param databasepath: path to the database describing the robot
     # \param limbId: user defined id for the limb. Must be unique.
     #  The id is used if several contact points are defined for the same limb (ex: the knee and the foot)
     # \param heuristicName: name of the selected heuristic for configuration evaluation
     # \param loadValues: whether values computed, other than the static ones, should be loaded in memory
     # \param disableEffectorCollision: whether collision detection should be disabled for end effector bones
     def addLimbDatabase(self, databasepath, limbId, heuristicName, loadValues = True, disableEffectorCollision = False, grasp =False):
          boolVal = 0.
          boolValEff = 0.
          graspv = 0.
          if(loadValues):
                boolVal = 1.
          if(disableEffectorCollision):
                boolValEff = 1.
          if(grasp):
                graspv = 1.
          self.clientRbprm.rbprm.addLimbDatabase(databasepath, limbId, heuristicName, boolVal,boolValEff,graspv)
          self.limbNames += [limbId]     

     ## Add a limb to the model
     #
     # \param id: user defined id for the limb. Must be unique.
     #  The id is used if several contact points are defined for the same limb (ex: the knee and the foot)
     # \param name: name of the joint corresponding to the root of the limb.
     # \param effectorName name of the joint to be considered as the effector of the limb
     # \param offset position of the effector in joint coordinates relatively to the effector joint
     # \param unit normal vector of the contact point, expressed in the effector joint coordinates
     # \param x width of the default support polygon of the effector
     # \param y height of the default support polygon of the effector
     # \param collisionObjects objects to be considered for collisions with the limb. TODO remove
     # \param nbSamples number of samples to generate for the limb
     # \param heuristicName: name of the selected heuristic for configuration evaluation
     # \param resolution, resolution of the octree voxels. The samples generated are stored in an octree data
     # structure to perform efficient proximity requests. The resulution of the octree, in meters, specifies the size
     # of the unit voxel of the octree. The larger they are, the more samples will be considered as candidates for contact.
     # This can be problematic in terms of performance. The default value is 3 cm.
     # \param contactType whether the contact is punctual ("_3_DOF") or surfacic ("_6_DOF")
     # \param disableEffectorCollision: whether collision detection should be disabled for end effector bones
     # \param limbOffset the offset between the limb joint and it's link
     # \param kinematicConstraintsPath : path that point to the .obj file containing the kinematic constraints of the limb,
     # if not set the default is "package://hpp-rbprm-corba/com_inequalities/"+name+"_com_constraints.obj"
     # \param kinematicConstraintsMin : add an additionnal inequalities on the kinematicConstraints, of normal (0,0,1) and origin (0,0,kinematicConstraintsMin)
     def addLimb(self, limbId, name, effectorname, offset, normal, x, y, samples, heuristicName, resolution, contactType="_6_DOF",disableEffectorCollision = False, grasp =False,limbOffset=[0,0,0],kinematicConstraintsPath = "", kinematicConstraintsMin = 0.):
          boolValEff = 0.
          if(disableEffectorCollision):
                boolValEff = 1.
          graspv = 0.
          if(grasp):
                graspv = 1.
          self.clientRbprm.rbprm.addLimb(limbId, name, effectorname, offset, normal, x, y, samples, heuristicName, resolution,contactType, boolValEff,graspv,limbOffset,kinematicConstraintsPath,kinematicConstraintsMin)
          self.limbNames += [limbId]

     ## Returns the configuration of a limb described by a sample
     #
     # \param name name of the limb considered
     # \param idsample identifiant of the sample considered
     def getSample(self, name, idsample):
          return self.clientRbprm.rbprm.getSampleConfig(name,idsample)
          
     ## Returns the end effector position of en effector,
     # given the current root configuration of the robot and a limb sample
     #
     # \param name name of the limb considered
     # \param idsample identifiant of the sample considered
     def getSamplePosition(self, name, idsample):
          return self.clientRbprm.rbprm.getSamplePosition(name,idsample)
          
     ## Get the end effector position for a given configuration, assuming z normal
     # \param limbName name of the limb from which to retrieve a sample
     # \param configuration configuration of the robot
     # \return world position of the limb end effector given the current robot configuration.
     # array of size 4, where each entry is the position of a corner of the contact surface
     def getEffectorPosition(self, limbName, configuration):
          return self.clientRbprm.rbprm.getEffectorPosition(limbName,configuration)
          
     ##compute the distance between all effectors replaced between two states
     # does not account for contact not present in both states
     # \param state1 from state
     # \param state2 destination state
     # \return the value computed for the given sample and analytics
     def getEffectorDistance(self, state1, state2):
          return self.clientRbprm.rbprm.getEffectorDistance(state1,state2)

     ## Generates a balanced contact configuration, considering the
     #  given current configuration of the robot, and a direction of motion.
     #  Typically used to generate a start and / or goal configuration automatically for a planning problem.
     #
     # \param configuration the initial robot configuration
     # \param direction a 3d vector specifying the desired direction of motion
     # \return the configuration in contact
     def generateContacts(self, configuration, direction,acceleration = [0,0,0], robustnessThreshold = 0):
          return self.clientRbprm.rbprm.generateContacts(configuration, direction, acceleration, robustnessThreshold)

     ## Generates a balanced contact configuration, considering the
     #  given current configuration of the robot, and a direction of motion.
     #  Typically used to generate a start and / or goal configuration automatically for a planning problem.
     #
     # \param configuration the initial robot configuration
     # \param direction a 3d vector specifying the desired direction of motion
     # \return the Id of the new state
     def generateStateInContact(self, configuration, direction,acceleration = [0,0,0], robustnessThreshold = 0):
          return self.clientRbprm.rbprm.generateStateInContact(configuration, direction, acceleration, robustnessThreshold)


     ## Generate an autocollision free configuration with limbs in contact with the ground
     # \param contactLimbs name of the limbs to project in contact
     # \return a sampled contact configuration
     def generateGroundContact(self, contactLimbs):
          return self.clientRbprm.rbprm.generateGroundContact(contactLimbs)
          
     ## Create a state and push it to the state array
     # \param configuration configuration
     # \param contacts list of effectors in contact
     # \return stateId
     def createState(self, configuration, contacts, normals = None):
          if normals is None:
               return self.clientRbprm.rbprm.createState(configuration, contacts)
          cl = self.clientRbprm.rbprm
          sId = cl.createState(configuration, contacts)
          num_max_sample = 1
          for (limbName, normal) in  zip(contacts, normals):
               p = cl.getEffectorPosition(limbName,configuration)[0]
               cl.addNewContact(sId, limbName, p, normal, num_max_sample, False)
          return sId
          
     ## Retrieves the contact candidates configurations given a configuration and a limb
     #
     # \param name id of the limb considered
     # \param configuration the considered robot configuration
     # \param direction a 3d vector specifying the desired direction of motion
     def getContactSamplesIds(self, name, configuration, direction):
          return self.clientRbprm.rbprm.getContactSamplesIds(name, configuration, direction)
          
     ## Retrieves the contact candidates configurations given a configuration and a limb
     #
     # \param name id of the limb considered
     # \param configuration the considered robot configuration
     # \param direction a 3d vector specifying the desired direction of motion
     def getContactSamplesProjected(self, name, configuration, direction, numSamples = 10):
          return self.clientRbprm.rbprm.getContactSamplesProjected(name, configuration, direction, numSamples)
          
     ## Retrieves the samples IDs In a given octree cell
     #
     # \param name id of the limb considered
     # \param configuration the considered robot configuration
     # \param direction a 3d vector specifying the desired direction of motion
     def getSamplesIdsInOctreeNode(self, limbName, octreeNodeId):
          return self.clientRbprm.rbprm.getSamplesIdsInOctreeNode(limbName, octreeNodeId)
          
     ## Get the number of samples generated for a limb
     #
     # \param limbName name of the limb from which to retrieve a sample
     def getNumSamples(self, limbName):
          return self.clientRbprm.rbprm.getNumSamples(limbName)
          
     ## Get the number of existing states in the client 
     #
     def getNumStates(self, limbName):
          return self.clientRbprm.rbprm.getNumStates()
          
     ## Get the number of octreeNodes
     #
     # \param limbName name of the limb from which to retrieve a sample
     def getOctreeNodeIds(self, limbName):
          return self.clientRbprm.rbprm.getOctreeNodeIds(limbName)
          
     ## Get the sample value for a given analysis
     #
     # \param limbName name of the limb from which to retrieve a sample
     # \param valueName name of the analytic measure desired
     # \param sampleId id of the considered sample
     def getSampleValue(self, limbName, valueName, sampleId):
          return self.clientRbprm.rbprm.getSampleValue(limbName, valueName, sampleId)
                    
     ## Initialize the first configuration of the path interpolation
     # with a balanced configuration for the interpolation problem;
     #
     # \param configuration the desired start configuration
     # \param contacts the array of limbs in contact
     # \param normals  the array describing one normal direction for each limb in contact
     def setStartState(self, configuration, contacts, normals = None):
          if normals is None:
               return self.clientRbprm.rbprm.setStartState(configuration, contacts)
          cl = self.clientRbprm.rbprm
          sId = cl.createState(configuration, contacts)
          num_max_sample = 1
          for (limbName, normal) in  zip(contacts, normals):
               p = cl.getEffectorPosition(limbName,configuration)[0]
               cl.addNewContact(sId, limbName, p, normal, num_max_sample, True)
          return cl.setStartStateId(sId)
          
          
     ## Initialize the goal configuration of the path interpolation
     # with a balanced configuration for the interpolation problem;
     #
     # \param configuration the desired goal configuration
     # \param contacts the array of limbs in contact
     # \param normals  the array describing one normal direction for each limb in contact
     def setEndState(self, configuration, contacts, normals = None):
          if normals is None:
               return self.clientRbprm.rbprm.setEndState(configuration, contacts)
          cl = self.clientRbprm.rbprm
          sId = cl.createState(configuration, contacts)
          num_max_sample = 1
          for (limbName, normal) in  zip(contacts, normals):
               p = cl.getEffectorPosition(limbName,configuration)[0]
               cl.addNewContact(sId, limbName, p, normal, num_max_sample, True)
          return cl.setEndStateId(sId)

     ## Initialize the first state of the path interpolation
     # \param stateId the Id of the desired start state in fullBody
     def setStartStateId(self,stateId):
          return self.clientRbprm.rbprm.setStartStateId(stateId)

     ## Initialize the goal state of the path interpolation
     # \param stateId the Id of the desired start state in fullBody
     def setEndStateId(self,stateId):
          return self.clientRbprm.rbprm.setEndStateId(stateId)


     ## Create a state given a configuration and contacts
     #
     # \param configuration the desired start configuration
     # \param contacts the array of limbs in contact
     # \return id of created state
     def createState(self, configuration, contacts):
          return self.clientRbprm.rbprm.createState(configuration, contacts)
                     
     ## Saves a computed contact sequence in a given filename
     #
     # \param The file where the configuration must be saved
     def saveComputedStates(self, filename):
          return self.clientRbprm.rbprm.saveComputedStates(filename)
     
     ## Saves a limb database
     #
     # \param limb name of the limb for which the DB must be saved
     # \param The file where the configuration must be saved
     def saveLimbDatabase(self, limbName, filename):
          return self.clientRbprm.rbprm.saveLimbDatabase(limbName, filename)
     
     ## Discretizes a path return by a motion planner into a discrete
     # sequence of balanced, contact configurations and returns
     # the sequence as an array of configurations
     #
     # \param stepSize discretization step
     # \param pathId Id of the path to compute from
     # \param robustnessTreshold minimum value of the static equilibrium robustness criterion required to accept the configuration (0 by default).
     # \param filterStates If different than 0, the resulting state list will be filtered to remove unnecessary states
     # \param testReachability : if true, check each contact transition with our reachability criterion
     # \param quasiStatic : if True, use our reachability criterion with the quasiStatic constraint
     # \param erasePreviousStates : if True the list of previously computed states is erased
     def interpolate(self, stepsize, pathId = 1, robustnessTreshold = 0, filterStates = False,testReachability = True, quasiStatic = False, erasePreviousStates = True):
          if(filterStates):
                filt = 1
          else:
                filt = 0
          return self.clientRbprm.rbprm.interpolate(stepsize, pathId, robustnessTreshold, filt,testReachability, quasiStatic, erasePreviousStates)
     
     ## Provided a discrete contact sequence has already been computed, computes
     # all the contact positions and normals for a given state, the next one, and the intermediate between them.
     #
     # \param stateId normalized step for generation along the path (ie the path has a length of 1).
     # \return list of 2 or 3 lists of 6d vectors [pox, poy, posz, nprmalx, normaly, normalz]
     def computeContactPoints(self, stateId):
          rawdata = self.clientRbprm.rbprm.computeContactPoints(stateId)
          return [[b[i:i+3] for i in range(0, len(b), 6)] for b in rawdata], [[b[i+3:i+6] for i in range(0, len(b), 6)] for b in rawdata]
     
     ### Provided a discrete contact sequence has already been computed, computes
     # all the contact positions and normals for a given state
     # \param stateId iD of the considered state
     # \param isIntermediate whether the intermediate state should be considerred rather than this one
     # \return list of 6d vectors [pox, poy, posz, nprmalx, normaly, normalz]
     def computeContactPointsAtState(self, stateId,isIntermediate=False):
          rawdata =  self.clientRbprm.rbprm.computeContactPointsAtState(stateId,isIntermediate)
          return [[b[i:i+6] for i in range(0, len(b), 6)] for b in rawdata][0]

     ## Provided a discrete contact sequence has already been computed, computes
     # all the contact positions and normals for a given state, the next one, and the intermediate between them.
     #
     # \param stateId normalized step for generation along the path (ie the path has a length of 1).
     # \return list of 2 or 3 lists of 6d vectors [pox, poy, posz, nprmalx, normaly, normalz]
     def computeContactPointsForLimb(self, stateId, limb):
          rawdata = self.clientRbprm.rbprm.computeContactPointsForLimb(stateId, limb)
          return [[b[i:i+3] for i in range(0, len(b), 6)] for b in rawdata], [[b[i+3:i+6] for i in range(0, len(b), 6)] for b in rawdata]
     
     ## Compute effector contact points and normals for a given configuration
     # in local coordinates
     #
     # \param q configuration of the robot
     # \param limbName ids of the limb in contact
     # \return list 6d vectors [pox, poy, posz, nprmalx, normaly, normalz]
     def computeContactForConfig(self, q, limbName):
          rawdata = self.clientRbprm.rbprm.computeContactForConfig(q, limbName)
          return [rawdata[i:i+3] for i in range(0, len(rawdata), 6)], [rawdata[i+3:i+6] for i in range(0, len(rawdata), 6)]

     ## Provided a discrete contact sequence has already been computed, computes
     # all the contact positions and normals for a given state, the next one, and the intermediate between them.
     #
     # \param stateId normalized step for generation along the path (ie the path has a length of 1).
     # \return list of 2 or 3 lists of 6d vectors [pox, poy, posz, nprmalx, normaly, normalz]
     def computeContactPointsPerLimb(self, stateId, limbs, dicEffector = {}):
          Ps = []; Ns = []
          for limb in limbs:
                P, N = self.computeContactPointsForLimb(stateId, limb)
                for i in range(len(P)):
                     if i > len(Ps) - 1 :
                          Ps.append({})
                          Ns.append({})
                     if(len(P[i]) > 0):
                          targetName = limb
                          if(limb in dicEffector):
                                targetName = dicEffector[limb]['effector']
                          Ps[i][targetName] = P[i]
                          Ns[i][targetName] = N[i]
          return Ps, Ns
          
     ## Given start and goal states
     #  generate a contact sequence over a list of configurations
     #
     # \param stepSize discretization step
     # \param pathId Id of the path to compute from
     # \param robustnessTreshold minimum value of the static equilibrium robustness criterion required to accept the configuration (0 by default).
     # \param filterStates If different than 0, the resulting state list will be filtered to remove unnecessary states
     # \param testReachability : if true, check each contact transition with our reachability criterion
     # \param quasiStatic : if True, use our reachability criterion with the quasiStatic constraint
     def interpolateConfigs(self, configs, robustnessTreshold = 0, filterStates = False, testReachability = True, quasiStatic = False):
          if(filterStates):
                filt = 1
          else:
                filt = 0
          return self.clientRbprm.rbprm.interpolateConfigs(configs, robustnessTreshold, filt,testReachability, quasiStatic)
          
     ##
     # 
     # \param state1 current state considered
     # \param limb name of the limb for which the request aplies
     # \return whether the limb is in contact at this state
     def isLimbInContact(self, limbname, state1):
          return self.clientRbprm.rbprm.isLimbInContact(limbname, state1) >0
          
     ##
     # 
     # \param state1 current state considered
     # \param limb name of the limb for which the request aplies
     # \return whether the limb is in contact at this state
     def isLimbInContactIntermediary(self, limbname, state1):
          return self.clientRbprm.rbprm.isLimbInContactIntermediary(limbname, state1) >0
          
     ##
     # 
     # \param state1 current state considered
     # \param limb name of the limb for which the request aplies
     # \return all limbs in contact at this state
     def getLimbsInContact(self, limbNames, state1):
          return [limbName for limbName in limbNames if self.isLimbInContact(limbName, state1)]
          
     ##
     # 
     # \param state1 current state considered
     # \param limb name of the limb for which the request aplies
     # \return all limbs in contact at this state
     def getLimbsInContactIntermediary(self, limbNames, state1):
          return [limbName for limbName in limbNames if self.isLimbInContactIntermediary(limbName, state1)]
     
     ## returns the CWC of the robot at a given state
     #
     # \param stateId The considered state
     # \return H matrix and h column, such that H w <= h
     def getContactCone(self, stateId, friction = 0.5):
          H_h =  array(self.clientRbprm.rbprm.getContactCone(stateId, friction))
          #~ print "H_h", H_h.shape 
          #~ print "norm h", ( H_h[:, -1] != 0).any()
          # now decompose cone 
          return H_h[:,:-1], H_h[:, -1]
          
     ## returns the CWC of the robot  between two states
     #
     # \param stateId The first considered state
     # \return H matrix and h column, such that H w <= h
     def getContactIntermediateCone(self, stateId, friction = 0.5):
          H_h =  array(self.clientRbprm.rbprm.getContactIntermediateCone(stateId, friction))
          #~ print "H_h", H_h.shape 
          #~ print "norm h", ( H_h[:, -1] != 0).any()
          # now decompose cone 
          return H_h[:,:-1], H_h[:, -1]
          
     ## Create a path for the root given
     #  a set of 3d key points
     #  The path is composed of n+1 linear interpolations
     #  between the n positions.
     #  The rotation is linearly interpolated as well,
     #  between a start and a goal rotation. The resulting path
     #  is added to the problem solver
     #  \param positions array of positions
     #  \param quat_1 quaternion of 1st rotation
     #  \param quat_2 quaternion of 2nd rotation
     def generateRootPath(self, positions, quat_1, quat_2):
          return self.clientRbprm.rbprm.generateRootPath(positions, quat_1, quat_2)
     
     ## Create a com trajectory given list of positions, velocities and accelerations
     # accelerations list contains one less element because it does not have an initial state.
     # a set of 3d key points
     # The path is composed of n+1 integrations
     # between the n positions.
     # The resulting path
     # is added to the problem solver
     # \param positions array of positions
     # \return id of the root path computed
     def straightPath(self, positions):
          return self.clientRbprm.rbprm.straightPath(positions)
          
     ## Create a com trajectory given a polynomial trajectory.
     # The path is composed of n+1 integrations
     # between the n waypoints.
     # The resulting path
     # is added to the problem solver
     # \param positions array of positions
     # \return id of the root path computed
     def generateCurveTraj(self, positions):
          return self.clientRbprm.rbprm.generateCurveTraj(positions)
          
     ## Create a com trajectory given a polynomial trajectory.
     # The path is composed of n+1 integrations
     # between the n waypoints. Then splits the curve into several sub curves
     # The resulting paths
     # are added to the problem solver
     # \param positions array of positions
     # \param partition array of times [0,1.] that describe the beginning or end of each phase
     # \return id of the complete path computed
     def generateCurveTrajParts(self, positions, partition):
          return self.clientRbprm.rbprm.generateCurveTrajParts(positions, partition)
                
     ## Create a com trajectory given list of positions, velocities and accelerations
     # accelerations list contains one less element because it does not have an initial state.
     # a set of 3d key points
     # The path is composed of n+1 integrations
     # between the n positions.
     # The resulting path
     # is added to the problem solver
     # \param positions array of positions
     # \param velocities array of velocities
     # \param accelerations array of accelerations
     # \param dt time between two points
     # \return id of the root path computed
     def generateComTraj(self, positions, velocities, accelerations, dt):
          return self.clientRbprm.rbprm.generateComTraj(positions, velocities, accelerations, dt)
          
     ## Create a path for the root given
     #  a set of 3d key points
     #  The path is composed of n+1 linear interpolations
     #  between the n positions.
     #  The rotation is linearly interpolated as well,
     #  between a start and a goal configuration. Assuming the robot is a
     #  free-flyer, rotations are extracted automatically. The resulting path
     #  is added to the problem solver
     #  \param positions array of positions
     #  \param configState1 configuration of 1st rotation
     #  \param configState2 configuration of 2nd rotation
     #  \return id of the resulting path
     def generateRootPathStates(self, positions, configState1, configState2):
          return self.clientRbprm.rbprm.generateRootPath(positions, configState1[3:7], configState2[3:7])
          
     ## Given start and goal states
     #  Provided a path has already been computed and interpolated, generate a continuous path
     #  between two indicated states. Will fail if the index of the states do not exist
     #
     # \param index of first state.
     # \param index of second state.
     # \param numOptim Number of iterations of the shortcut algorithm to apply between each states
     #  \return id of the resulting path
     def limbRRT(self, state1, state2, numOptim = 10):
          return self.clientRbprm.rbprm.limbRRT(state1, state2, numOptim)
          
     ## Provided a path has already been computed and interpolated, generate a continuous path                                
     # between two indicated states. The states do not need to be consecutive, but increasing in Id.                      
     # Will fail if the index of the states do not exist                                                                                 
     # The path of the root and limbs not considered by the contact transitions between                                        
     # two states is assumed to be already computed, and registered in the solver under the id specified by the user.
     # It must be valid in the sense of the active PathValidation.                                                                    
     # \param state1 index of first state.                                                                                                    
     # \param state2 index of second state.                                                                                                  
     # \param path index of the path considered for the generation                                                                    
     # \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states 
     #  \return id of the resulting path
     def limbRRTFromRootPath(self, state1, state2, path, numOptim = 10):
          return self.clientRbprm.rbprm.limbRRTFromRootPath(state1, state2, path, numOptim)
          
          
     ## Provided a center of mass trajectory has already been computed and interpolated, generate a continuous full body path                                
     # between two indicated states. The states do not need to be consecutive, but increasing in Id.                      
     # Will fail if the index of the states do not exist                                                                                 
     # The path of the COM  between                                        
     # two states is assumed to be already computed, and registered in the solver under the id specified by the user.
     # It must be valid in the sense of the active PathValidation.                                                                    
     # \param state1 index of first state.                                                                                                    
     # \param state2 index of second state.                                                                                                  
     # \param path index of the com path considered for the generation                                                                    
     # \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states 
     #  \return id of the resulting path
     def comRRT(self, state1, state2, path, numOptim = 10):
          return self.clientRbprm.rbprm.comRRT(state1, state2, path, numOptim)
          
                
     ## Provided a path has already been computed and interpolated, generate a continuous path
     # between two indicated states. The states do not need to be consecutive, but increasing in Id.
     # Will fail if the index of the states do not exist
     # The path of the COM of thr robot and limbs not considered by the contact transitions between
     # two states is assumed to be already computed, and registered in the solver under the id specified by the user.
     # It must be valid in the sense of the active PathValidation.
     # \param state1 index of first state.
     # \param rootPositions1 com positions to track for 1st phase
     # \param rootPositions1 com positions to track for 2nd phase
     # \param rootPositions1 com positions to track for 3rd phase
     # \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states
     # \return id of the root path computed
     def comRRTFromPos(self, state1, comPos1, comPos2, comPos3, numOptim = 10):
          return self.clientRbprm.rbprm.comRRTFromPos(state1, comPos1, comPos2, comPos3, numOptim)
          
     ## Provided a path has already been computed and interpolated, generate a continuous path
     # between two indicated states. The states do not need to be consecutive, but increasing in Id.
     # Will fail if the index of the states do not exist
     # The path of the COM of thr robot and limbs not considered by the contact transitions between
     # two states is assumed to be already computed, and registered in the solver under the id specified by the user.
     # It must be valid in the sense of the active PathValidation.
     # \param state1 index of first state.
     # \param rootPositions1 com positions to track for 1st phase
     # \param rootPositions1 com positions to track for 2nd phase
     # \param rootPositions1 com positions to track for 3rd phase
     # \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states
     # \return id of the root path computed
     def effectorRRTFromPosBetweenState(self, state1, state2, comPos1, comPos2, comPos3, numOptim = 10):
          return self.clientRbprm.rbprm.effectorRRTFromPosBetweenState(state1, state2, comPos1, comPos2, comPos3, numOptim)
          
     def comRRTFromPosBetweenState(self, state1, state2, comPos1, comPos2, comPos3, numOptim = 10):
          return self.clientRbprm.rbprm.comRRTFromPosBetweenState(state1, state2, comPos1, comPos2, comPos3, numOptim)
          
                
     ## Provided a path has already been computed and interpolated, generate a continuous path
     # between two indicated states. The states do not need to be consecutive, but increasing in Id.
     # Will fail if the index of the states do not exist
     # The path of the COM of thr robot and limbs not considered by the contact transitions between
     # two states is assumed to be already computed, and registered in the solver under the id specified by the user.
     # It must be valid in the sense of the active PathValidation.
     # \param state1 index of first state.
     # \param rootPositions1 com positions to track for 1st phase
     # \param rootPositions1 com positions to track for 2nd phase
     # \param rootPositions1 com positions to track for 3rd phase
     # \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states
     # \return id of the root path computed
     def effectorRRT(self, state1, comPos1, comPos2, comPos3, numOptim = 10):
          return self.clientRbprm.rbprm.effectorRRT(state1, comPos1, comPos2, comPos3, numOptim)
          
                          
     ## Provided a path has already been computed and interpolated, generate a continuous path
     # between two indicated states. The states do not need to be consecutive, but increasing in Id.
     # Will fail if the index of the states do not exist
     # The path of the COM of thr robot and limbs not considered by the contact transitions between
     # two states is assumed to be already computed, and registered in the solver under the id specified by the user.
     # It must be valid in the sense of the active PathValidation.
     # \param state1 index of first state.
     # \param rootPositions1 com positions to track for 1st phase
     # \param rootPositions1 com positions to track for 2nd phase
     # \param rootPositions1 com positions to track for 3rd phase
     # \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states
     # \return id of the root path computed
     def effectorRRTFromPath(self, state1, refPathId, path_start, path_to, comPos1, comPos2, comPos3, numOptim = 10, trackedEffectors = []):
          return self.clientRbprm.rbprm.effectorRRTFromPath(state1, refPathId, path_start, path_to, comPos1, comPos2, comPos3, numOptim, trackedEffectors)


    ## Provided a path has already been computed and interpolated, generate a continuous path
    # between two indicated states. The states need to be consecutive with no contact variation between them
    # (the free limbs can move, but there should be no contact creation/break)
    # \param state1 index of the first state
    # \param state2 index of the second state
    # \param comPos com position to track
    # \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states
     def effectorRRTOnePhase(self,state1,state2,comPos,numOptim=10):
         return self.clientRbprm.rbprm.effectorRRTOnePhase(state1,state2,comPos,numOptim)

    ## Provided a path has already been computed and interpolated, generate an array of bezier curves,
    ## with varying weightRRT (see effector-rrt.cc::fitBezier)
    # (the free limbs can move, but there should be no contact creation/break)
    # \param state1 index of the first state
    # \param state2 index of the second state
    # \param comPos com position to track
    # \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states
    # \return array of pathIds : first index is the trajectorie, second index is the curve inside this trajectory
    # (there should be 3 curves per trajectories : takeoff / mid / landing)
     def generateEffectorBezierArray(self,state1,state2,comPos,numOptim=10):
         return self.clientRbprm.rbprm.generateEffectorBezierArray(state1,state2,comPos,numOptim)



    ## Provided a path has already been computed and interpolated, generate a continuous path
    # between two indicated states. The states need to be consecutive with no contact variation between them
    # (the free limbs can move, but there should be no contact creation/break)
    # \param state1 index of the first state
    # \param state2 index of the second state
    # \param comPos com position to track
    # \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states
     def comRRTOnePhase(self,state1,state2,comPos,numOptim=10):
         return self.clientRbprm.rbprm.comRRTOnePhase(state1,state2,comPos,numOptim)


    # compute and add a trajectory for the end effector between the 2 states
    # represented as a bezier curve.
    # Do not check the kinematic feasability of this trajectory
    # \param state1 index of first state.
    # \param state2 index of end state.
    # \param rootPositions com positions to track
     def generateEndEffectorBezier(self, state1, state2, comPos):
          return self.clientRbprm.rbprm.generateEndEffectorBezier(state1, state2, comPos)

          
     ## Project a given state into a given COM position
     # between two indicated states. The states do not need to be consecutive, but increasing in Id.
     # Will fail if the index of the state does not exist.
     # \param state index of first state.
     # \param targetCom 3D vector for the com position
     # \return projected configuration
     def projectToCom(self, state, targetCom, maxNumSample = 0):
          return self.clientRbprm.rbprm.projectToCom(state, targetCom, maxNumSample)
          
     ## Returns the configuration at a given state
     # Will fail if the index of the state does not exist.
     # \param state index of state.
     # \return state configuration
     def getConfigAtState(self, state):
          return self.clientRbprm.rbprm.getConfigAtState(state)
          
     ## Project a given state into a given COM position
     # and update the state configuration.
     # \param state index of first state.
     # \param targetCom 3D vector for the com position
     # \return whether the projection was successful
     def projectStateToCOM(self, state, targetCom, maxNumSample = 0):
          return self.clientRbprm.rbprm.projectStateToCOM(state, targetCom, maxNumSample)     > 0

     ## Project a given state into a given root position
     # and update the state configuration.
     # \param state index of first state.
     # \param root : root configuration (size 7)
     # \return whether the projection was successful
     def projectStateToRoot(self, state, root):
          return self.clientRbprm.rbprm.projectStateToRoot(state, root)     > 0
          
     ## Project a given state into a given COM position
     # and update the state configuration.
     # \param state index of first state.
     # \param targetCom 3D vector for the com position
     # \return whether the projection was successful
     def setConfigAtState(self, state, targetCom):
          return self.clientRbprm.rbprm.setConfigAtState(state, targetCom)     > 0

     ## Given start and goal states
     #  generate a contact sequence over a list of configurations
     #
     # \param stepSize discretization step
     # \param pathId Id of the path to compute from
     def isConfigBalanced(self, config, names, robustness = 0,CoM = [0,0,0]):
          if (self.clientRbprm.rbprm.isConfigBalanced(config, names, robustness,CoM) == 1):
                return True
          else:
                return False

     ## Check if the state at the given index is balanced for a given robustness
     #
     def isStateBalanced(self, stateId, robustnessThreshold = 0, robustnessFound = 0):
          robustnessFound = self.clientRbprm.rbprm.isStateBalanced(stateId)
          return robustnessFound > robustnessThreshold

     ## Updates limb databases with a user chosen computation
     #
     # \param analysis name of computation
     # \param isstatic whether the computation should be used to sort samples by default
     def runSampleAnalysis(self, analysis, isstatic):
          isStatic = 0.
          if(isstatic):
                isStatic = 1.
          self.clientRbprm.rbprm.runSampleAnalysis(analysis,isStatic)
          
     ## Updates a limb database with a user chosen computation
     #
     # \param limbname name of the limb chosen for computation
     # \param analysis name of computation
     # \param isstatic whether the computation should be used to sort samples by default
     def runLimbSampleAnalysis(self, limbname, analysis, isstatic):
          isStatic = 0.
          if(isstatic):
                isStatic = 1.
          return self.clientRbprm.rbprm.runLimbSampleAnalysis(limbname, analysis,isStatic)
          
     ## if the preprocessor variable PROFILE is active
     # dump the profiling data into a logFile
     # \param logFile name of the file where to dump the profiling data
     def dumpProfile(self, logFile="log.txt"):
          return self.clientRbprm.rbprm.dumpProfile(logFile)

     ## Create octree nodes representation for a given limb
     #
     # \param gui gepetoo viewer instance discretization step
     # \param winId window to draw to step
     # \param limbName name of the limb considered
     # \param config initial configuration of the robot, used when created octree
     # \param color of the octree boxes
     def createOctreeBoxes(self, gui, winId, limbName, config, color = [1,1,1,0.3]):
          boxes = self.clientRbprm.rbprm.getOctreeBoxes(limbName, config)
          scene = "oct"+limbName
          gui.createScene(scene)
          resolution = boxes[0][3]
          i = 0
          for box in boxes:
                boxname = scene+"/b"+str(i)
                gui.addBox(boxname,resolution,resolution,resolution, color)
                gui.applyConfiguration(boxname,[box[0],box[1],box[2],1,0,0,0])
                i = i+1
          self.octrees[limbName] = scene
          gui.addSceneToWindow(scene,winId)
          gui.refresh()
          
     ## Create octree nodes representation for a given limb
     #
     # \param limbName name of the limb considered
     # \param ocId of the octree box
     def getOctreeBox(self, limbName, ocId):
          return self.clientRbprm.rbprm.getOctreeBox(limbName, ocId)
     
     ## Draws robot configuration, along with octrees associated
     #
     # \param viewer gepetto viewer instance
     def draw(self, configuration, viewer):
          viewer(configuration)
          for limb, groupid in self.octrees.items():
                transform = self.clientRbprm.rbprm.getOctreeTransform(limb, configuration)
                viewer.client.gui.applyConfiguration(groupid,transform)
          viewer.client.gui.refresh()
          
          
    ## Computes the closest projection matrix that will bring a limb's effector
    # from its current configuration to a specified location
    # \param limbName name of the considered limb
    # \param configuration considered configuration of the robot
    # \param p target position
    # \param n target normal
    # \return 7D Transform of effector in contact (position + quaternion)
     def computeTargetTransform(self, limbName, configuration, p, n):         
          return self.clientRbprm.rbprm.computeTargetTransform(limbName, configuration, p, n)

     ## Export motion to a format readable by the blender
     # importFromGepetto script
     # \param viewer gepetto viewer instance
     # \param configurations list of configurations to save
     # \param filename outputfile where to export the motion
     def exportMotion(self, viewer, configurations, filename):
          import hpp.gepetto.blender.exportmotion as em
          em.exportStates(viewer, self.client.robot, configurations, filename)
          
     ## Export motion to a format readable by the blender
     # importFromGepetto script
     # \param viewer gepetto viewer instance
     # \param configurations list of configurations to save
     # \param filename outputfile where to export the motion
     def exportAll(self, viewer, configurations, testname):
          self.exportMotion(viewer, configurations, testname+"_motion.txt")
          self.saveComputedStates(testname+"_states.txt")
          f1 = open(testname+"_configs.txt","w+")
          f1.write(str(configurations))
          f1.close()

      ## set a boolean in rbprmFullBody
      # if true, the acceleration doesn't account for the stability check
      #
        # \param staticStability boolean
     def setStaticStability(self,staticStability):
          return self.clientRbprm.rbprm.setStaticStability(staticStability)

     ## set a reference configuration in FullBody
     # \param referenceConfig dofArray
     def setReferenceConfig(self,referenceConfig):
          return self.clientRbprm.rbprm.setReferenceConfig(referenceConfig)

     ## set the weights used when computing a postural task
     # \param postureWeights dofArray, must be of same size as device->numberDof
     def setPostureWeights(self,postureWeights):
          return self.clientRbprm.rbprm.setPostureWeights(postureWeights)

     ## If true, optimize the orientation of all the newly created contact using a postural task
     # \param use bool
     def usePosturalTaskContactCreation(self,use):
          return self.clientRbprm.rbprm.usePosturalTaskContactCreation(use)

      ## return the time at the given state index (in the path computed during the first phase)
      # \param stateId : index of the state
     def getTimeAtState(self,stateId):
          return self.clientRbprm.rbprm.getTimeAtState(stateId)

      ## return the duration for the given state index
      # note : it depend on the implementation of interpolate : which state do we add to the list : the first one or the last or one in the middle
      # current implementation : the last one
      # \param stateId : index of the state
     def getDurationForState(self,stateId):
          if stateId == 0:
               return self.clientRbprm.rbprm.getTimeAtState(stateId)
          else:
               return (self.clientRbprm.rbprm.getTimeAtState(stateId) - self.clientRbprm.rbprm.getTimeAtState(stateId-1))


     ## Return the names of all the effector for which a trajectory have been computed for a given path index.
     #  \param pathId : index of the path, the same index as the wholeBody path stored in problem-solver
     #  \return the list of all the end-effector (joint names) for which a trajectory have been defined
     def getEffectorsTrajectoriesNames(self, pathId):
         return self.clientRbprm.rbprm.getEffectorsTrajectoriesNames(pathId)

     ## Return the waypoints of the bezier curve for a given pathIndex and effector name
     #  \param pathId : index of the path, the same index as the wholeBody path stored in problem-solver
     #  \param effectorName : the name of the desired effector (Joint name)
     #  \return the waypoints of the bezier curve followed by this end effector
     #  Throw an error if there is no trajectory computed for the given id/name
     def getEffectorTrajectoryWaypoints(self,pathId,effectorName):
         return self.clientRbprm.rbprm.getEffectorTrajectoryWaypoints(pathId,effectorName)


     ## Return the bezier curve for a given pathIndex and effector name
     #  \param pathId : index of the path, the same index as the wholeBody path stored in problem-solver
     #  \param effectorName : the name of the desired effector (Joint name)
     #  \return a list of bezier curve (from spline library) followed by the end effector.
     #  Throw an error if there is no trajectory computed for the given id/name
     def getEffectorTrajectory(self,pathId,effectorName):
         curvesWp =  self.clientRbprm.rbprm.getEffectorTrajectoryWaypoints(pathId,effectorName)
         res=[]
         for cid in range(len(curvesWp)):
             wp=curvesWp[cid]
             if len(wp[0]) == 1: # length is provided :
                waypoints = matrix(wp[1:]).transpose()
                curve = bezier(waypoints,wp[0][0])
             else :
                waypoints=matrix(wp).transpose()
                curve = bezier(waypoints)
             res +=[curve]
         return res

    ## Return the bezier curve corresponding to a given path index
     def getPathAsBezier(self,pathId):
         l = self.clientRbprm.rbprm.getPathAsBezier(pathId)
         t = l[0][0]
         wps = matrix(l[1:]).transpose()
         curve = bezier(wps,t)
         return curve


      ## return the contacts variation between two states
      # \param stateIdFrom : index of the first state
      # \param stateIdTo : index of the second state
     def getContactsVariations(self,stateIdFrom,stateIdTo):
          return self.clientRbprm.rbprm.getContactsVariations(stateIdFrom,stateIdTo)

      ## return a list of all the limbs names
     def getAllLimbsNames(self):
          return self.clientRbprm.rbprm.getAllLimbsNames()

      ## return a list of all the limbs in contact
      # \param stateId : index of the state
     def getAllLimbsInContact(self,stateId):
          return self.getLimbsInContact(self.getAllLimbsNames(),stateId)

     ## check the kinematics constraints for the given point, assuming all contact are established
     # \param point : test if the point is inside the kinematics constraints for the current configuration
     def areKinematicsConstraintsVerified(self,point):
          return self.clientRbprm.rbprm.areKinematicsConstraintsVerified(point)

     def areKinematicsConstraintsVerifiedForState(self,stateFrom, point):
         return self.clientRbprm.rbprm.areKinematicsConstraintsVerifiedForState(stateFrom,point)

     def isReachableFromState(self,stateFrom,stateTo,computePoint=False,useIntermediateState=True):
          raw =  self.clientRbprm.rbprm.isReachableFromState(stateFrom,stateTo,useIntermediateState)
          if computePoint :
                res = []
                res += [raw[0]>0.]
                res += [[raw[1],raw[2],raw[3]]]
                if len(raw) > 4:
                    res += [[raw[4],raw[5],raw[6]]]
                return res
          else :
                return raw[0] > 0.

     def isDynamicallyReachableFromState(self,stateFrom,stateTo,addPathPerPhase = False,timings=[],numPointsPerPhases=5):
          return self.clientRbprm.rbprm.isDynamicallyReachableFromState(stateFrom,stateTo,addPathPerPhase,timings,numPointsPerPhases)

