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
import hpp.gepetto.blender.exportmotion as em
from numpy import array

## Corba clients to the various servers
#
class CorbaClient:
     """
     Container for corba clients to various interfaces.
     """
     def __init__ (self):
          self.basic = BasicClient ()
          self.rbprm = RbprmClient ()

## Load and handle a RbprmFullbody robot for rbprm planning
#
#  A RbprmDevice robot is a set of two robots. One for the 
#  trunk of the robot, one for the range of motion
class FullBody (object):
     ## Constructor
     def __init__ (self, load = True):
          self.tf_root = "base_link"
          self.rootJointType = dict()
          self.client = CorbaClient ()
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
          self.client.rbprm.rbprm.loadFullBodyRobot(urdfName, rootJointType, packageName, urdfName, urdfSuffix, srdfSuffix, self.client.basic.problem.getSelected("problem")[0])
          self.name = urdfName
          self.displayName = urdfName
          self.tf_root = "base_link"
          self.rootJointType = rootJointType
          self.jointNames = self.client.basic.robot.getJointNames ()
          self.allJointNames = self.client.basic.robot.getAllJointNames ()
          self.client.basic.robot.meshPackageName = meshPackageName
          self.meshPackageName = meshPackageName
          self.rankInConfiguration = dict ()
          self.rankInVelocity = dict ()
          self.packageName = packageName
          self.urdfName = urdfName
          self.urdfSuffix = urdfSuffix
          self.srdfSuffix = srdfSuffix
          self.octrees={}
          rankInConfiguration = rankInVelocity = 0
          for j in self.jointNames:
                self.rankInConfiguration [j] = rankInConfiguration
                rankInConfiguration += self.client.basic.robot.getJointConfigSize (j)
                self.rankInVelocity [j] = rankInVelocity
                rankInVelocity += self.client.basic.robot.getJointNumberDof (j)
     
     # Virtual function to load the fullBody robot model.
     #
     def loadFullBodyModelFromActiveRobot (self, urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix):
          self.client.rbprm.rbprm.loadFullBodyRobotFromExistingRobot()
          self.name = urdfName
          self.displayName = urdfName
          self.tf_root = "base_link"
          self.rootJointType = rootJointType
          self.jointNames = self.client.basic.robot.getJointNames ()
          self.allJointNames = self.client.basic.robot.getAllJointNames ()
          self.client.basic.robot.meshPackageName = meshPackageName
          self.meshPackageName = meshPackageName
          self.rankInConfiguration = dict ()
          self.rankInVelocity = dict ()
          self.packageName = packageName
          self.urdfName = urdfName
          self.urdfSuffix = urdfSuffix
          self.srdfSuffix = srdfSuffix
          self.octrees={}
          rankInConfiguration = rankInVelocity = 0
          for j in self.jointNames:
                self.rankInConfiguration [j] = rankInConfiguration
                rankInConfiguration += self.client.basic.robot.getJointConfigSize (j)
                self.rankInVelocity [j] = rankInVelocity
                rankInVelocity += self.client.basic.robot.getJointNumberDof (j)

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
     # \param resolution, resolution of the octree voxels. The samples generated are stored in an octree data
     # \param resolution, resolution of the octree voxels. The samples generated are stored in an octree data
     # structure to perform efficient proximity requests. The resulution of the octree, in meters, specifies the size
     # of the unit voxel of the octree. The larger they are, the more samples will be considered as candidates for contact.
     # This can be problematic in terms of performance. The default value is 3 cm.
     def addLimb(self, limbId, name, effectorname, offset, normal, x, y, samples):
          self.client.rbprm.rbprm.addLimb(limbId, name, effectorname, offset, normal, x, y, samples, "EFORT", 0.03)
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
          self.client.rbprm.rbprm.addLimbDatabase(databasepath, limbId, heuristicName, boolVal,boolValEff,graspv)     
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
     def addLimb(self, limbId, name, effectorname, offset, normal, x, y, samples, heuristicName, resolution, contactType="_6_DOF",disableEffectorCollision = False, grasp =False):
          boolValEff = 0.
          if(disableEffectorCollision):
                boolValEff = 1.
          graspv = 0.
          if(grasp):
                graspv = 1.
          self.client.rbprm.rbprm.addLimb(limbId, name, effectorname, offset, normal, x, y, samples, heuristicName, resolution,contactType, boolValEff,graspv)
          self.limbNames += [limbId]

     ## Returns the configuration of a limb described by a sample
     #
     # \param name name of the limb considered
     # \param idsample identifiant of the sample considered
     def getSample(self, name, idsample):
          return self.client.rbprm.rbprm.getSampleConfig(name,idsample)
          
     ## Returns the end effector position of en effector,
     # given the current root configuration of the robot and a limb sample
     #
     # \param name name of the limb considered
     # \param idsample identifiant of the sample considered
     def getSamplePosition(self, name, idsample):
          return self.client.rbprm.rbprm.getSamplePosition(name,idsample)
          
     ## Get the end effector position for a given configuration, assuming z normal
     # \param limbName name of the limb from which to retrieve a sample
     # \param configuration configuration of the robot
     # \return world position of the limb end effector given the current robot configuration.
     # array of size 4, where each entry is the position of a corner of the contact surface
     def getEffectorPosition(self, limbName, configuration):
          return self.client.rbprm.rbprm.getEffectorPosition(limbName,configuration)
          
     ##compute the distance between all effectors replaced between two states
     # does not account for contact not present in both states
     # \param state1 from state
     # \param state2 destination state
     # \return the value computed for the given sample and analytics
     def getEffectorDistance(self, state1, state2):
          return self.client.rbprm.rbprm.getEffectorDistance(state1,state2)          

     ## Generates a balanced contact configuration, considering the
     #  given current configuration of the robot, and a direction of motion.
     #  Typically used to generate a start and / or goal configuration automatically for a planning problem.
     #
     # \param configuration the initial robot configuration
     # \param direction a 3d vector specifying the desired direction of motion
     def generateContacts(self, configuration, direction):
          return self.client.rbprm.rbprm.generateContacts(configuration, direction)
          
     ## Generate an autocollision free configuration with limbs in contact with the ground
     # \param contactLimbs name of the limbs to project in contact
     # \return a sampled contact configuration
     def generateGroundContact(self, contactLimbs):
          return self.client.rbprm.rbprm.generateGroundContact(contactLimbs)
          
     ## Create a state and push it to the state array
     # \param q configuration
     # \param names list of effectors in contact
     # \return stateId
     def createState(self, q, contactLimbs):
          return self.client.rbprm.rbprm.createState(q, contactLimbs)
          
     ## Retrieves the contact candidates configurations given a configuration and a limb
     #
     # \param name id of the limb considered
     # \param configuration the considered robot configuration
     # \param direction a 3d vector specifying the desired direction of motion
     def getContactSamplesIds(self, name, configuration, direction):
          return self.client.rbprm.rbprm.getContactSamplesIds(name, configuration, direction)
          
     ## Retrieves the contact candidates configurations given a configuration and a limb
     #
     # \param name id of the limb considered
     # \param configuration the considered robot configuration
     # \param direction a 3d vector specifying the desired direction of motion
     def getContactSamplesProjected(self, name, configuration, direction, numSamples = 10):
          return self.client.rbprm.rbprm.getContactSamplesProjected(name, configuration, direction, numSamples)
          
     ## Retrieves the samples IDs In a given octree cell
     #
     # \param name id of the limb considered
     # \param configuration the considered robot configuration
     # \param direction a 3d vector specifying the desired direction of motion
     def getSamplesIdsInOctreeNode(self, limbName, octreeNodeId):
          return self.client.rbprm.rbprm.getSamplesIdsInOctreeNode(limbName, octreeNodeId)
          
     ## Get the number of samples generated for a limb
     #
     # \param limbName name of the limb from which to retrieve a sample
     def getNumSamples(self, limbName):
          return self.client.rbprm.rbprm.getNumSamples(limbName)
          
     ## Get the number of octreeNodes
     #
     # \param limbName name of the limb from which to retrieve a sample
     def getOctreeNodeIds(self, limbName):
          return self.client.rbprm.rbprm.getOctreeNodeIds(limbName)
          
     ## Get the sample value for a given analysis
     #
     # \param limbName name of the limb from which to retrieve a sample
     # \param valueName name of the analytic measure desired
     # \param sampleId id of the considered sample
     def getSampleValue(self, limbName, valueName, sampleId):
          return self.client.rbprm.rbprm.getSampleValue(limbName, valueName, sampleId)
          
     ## Initialize the first configuration of the path discretization 
     # with a balanced configuration for the interpolation problem;
     #
     # \param configuration the desired start configuration
     # \param contacts the array of limbs in contact
     def setStartState(self, configuration, contacts):
          return self.client.rbprm.rbprm.setStartState(configuration, contacts)
          
     ## Create a state given a configuration and contacts
     #
     # \param configuration the desired start configuration
     # \param contacts the array of limbs in contact
     # \return id of created state
     def createState(self, configuration, contacts):
          return self.client.rbprm.rbprm.createState(configuration, contacts)
                
     ## Initialize the last configuration of the path discretization 
     # with a balanced configuration for the interpolation problem;
     #
     # \param configuration the desired end configuration
     # \param contacts the array of limbs in contact          
     def setEndState(self, configuration, contacts):
          return self.client.rbprm.rbprm.setEndState(configuration, contacts)
     
     ## Saves a computed contact sequence in a given filename
     #
     # \param The file where the configuration must be saved
     def saveComputedStates(self, filename):
          return self.client.rbprm.rbprm.saveComputedStates(filename)
     
     ## Saves a limb database
     #
     # \param limb name of the limb for which the DB must be saved
     # \param The file where the configuration must be saved
     def saveLimbDatabase(self, limbName, filename):
          return self.client.rbprm.rbprm.saveLimbDatabase(limbName, filename)
     
     ## Discretizes a path return by a motion planner into a discrete
     # sequence of balanced, contact configurations and returns
     # the sequence as an array of configurations
     #
     # \param stepSize discretization step
     # \param pathId Id of the path to compute from
     # \param robustnessTreshold minimum value of the static equilibrium robustness criterion required to accept the configuration (0 by default).
     # \param filterStates If different than 0, the resulting state list will be filtered to remove unnecessary states
     def interpolate(self, stepsize, pathId = 1, robustnessTreshold = 0, filterStates = False):
          if(filterStates):
                filt = 1
          else:
                filt = 0
          return self.client.rbprm.rbprm.interpolate(stepsize, pathId, robustnessTreshold, filt)
     
     ## Provided a discrete contact sequence has already been computed, computes
     # all the contact positions and normals for a given state, the next one, and the intermediate between them.
     #
     # \param stateId normalized step for generation along the path (ie the path has a length of 1).
     # \return list of 2 or 3 lists of 6d vectors [pox, poy, posz, nprmalx, normaly, normalz]
     def computeContactPoints(self, stateId):
          rawdata = self.client.rbprm.rbprm.computeContactPoints(stateId)
          return [[b[i:i+3] for i in range(0, len(b), 6)] for b in rawdata], [[b[i+3:i+6] for i in range(0, len(b), 6)] for b in rawdata]
     
     ## Provided a discrete contact sequence has already been computed, computes
     # all the contact positions and normals for a given state, the next one, and the intermediate between them.
     #
     # \param stateId normalized step for generation along the path (ie the path has a length of 1).
     # \return list of 2 or 3 lists of 6d vectors [pox, poy, posz, nprmalx, normaly, normalz]
     def computeContactPointsForLimb(self, stateId, limb):
          rawdata = self.client.rbprm.rbprm.computeContactPointsForLimb(stateId, limb)
          return [[b[i:i+3] for i in range(0, len(b), 6)] for b in rawdata], [[b[i+3:i+6] for i in range(0, len(b), 6)] for b in rawdata]
     
     ## Compute effector contact points and normals for a given configuration
     # in local coordinates
     #
     # \param q configuration of the robot
     # \param limbName ids of the limb in contact
     # \return list 6d vectors [pox, poy, posz, nprmalx, normaly, normalz]
     def computeContactForConfig(self, q, limbName):
          rawdata = self.client.rbprm.rbprm.computeContactForConfig(q, limbName)
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
                          if(dicEffector.has_key(limb)):
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
     def interpolateConfigs(self, configs, robustnessTreshold = 0, filterStates = False):
          if(filterStates):
                filt = 1
          else:
                filt = 0
          return self.client.rbprm.rbprm.interpolateConfigs(configs, robustnessTreshold, filt)
          
     ##
     # 
     # \param state1 current state considered
     # \param limb name of the limb for which the request aplies
     # \return whether the limb is in contact at this state
     def isLimbInContact(self, limbname, state1):
          return self.client.rbprm.rbprm.isLimbInContact(limbname, state1) >0
          
     ##
     # 
     # \param state1 current state considered
     # \param limb name of the limb for which the request aplies
     # \return whether the limb is in contact at this state
     def isLimbInContactIntermediary(self, limbname, state1):
          return self.client.rbprm.rbprm.isLimbInContactIntermediary(limbname, state1) >0
          
     ##
     # 
     # \param state1 current state considered
     # \param limb name of the limb for which the request aplies
     # \return all limbs in contact at this state
     def getLimbsInContact(self, limbNames, state1):
          return [limbName for limbName in limbNames if self.isLimbInContact(limbname, state1)]
          
     ##
     # 
     # \param state1 current state considered
     # \param limb name of the limb for which the request aplies
     # \return all limbs in contact at this state
     def getLimbsInContactIntermediary(self, limbNames, state1):
          return [limbName for limbName in limbNames if self.isLimbInContactIntermediary(limbname, state1)]
     
     ## returns the CWC of the robot at a given state
     #
     # \param stateId The considered state
     # \return H matrix and h column, such that H w <= h
     def getContactCone(self, stateId, friction = 0.5):
          H_h =  array(self.client.rbprm.rbprm.getContactCone(stateId, friction))
          #~ print "H_h", H_h.shape 
          #~ print "norm h", ( H_h[:, -1] != 0).any()
          # now decompose cone 
          return H_h[:,:-1], H_h[:, -1]
          
     ## returns the CWC of the robot  between two states
     #
     # \param stateId The first considered state
     # \return H matrix and h column, such that H w <= h
     def getContactIntermediateCone(self, stateId, friction = 0.5):
          H_h =  array(self.client.rbprm.rbprm.getContactIntermediateCone(stateId, friction))
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
          return self.client.rbprm.rbprm.generateRootPath(positions, quat_1, quat_2)
     
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
          return self.client.rbprm.rbprm.straightPath(positions)
          
     ## Create a com trajectory given a polynomial trajectory.
     # The path is composed of n+1 integrations
     # between the n waypoints.
     # The resulting path
     # is added to the problem solver
     # \param positions array of positions
     # \return id of the root path computed
     def generateCurveTraj(self, positions):
          return self.client.rbprm.rbprm.generateCurveTraj(positions)
          
     ## Create a com trajectory given a polynomial trajectory.
     # The path is composed of n+1 integrations
     # between the n waypoints. Then splits the curve into several sub curves
     # The resulting paths
     # are added to the problem solver
     # \param positions array of positions
     # \param partition array of times [0,1.] that describe the beginning or end of each phase
     # \return id of the complete path computed
     def generateCurveTrajParts(self, positions, partition):
          return self.client.rbprm.rbprm.generateCurveTrajParts(positions, partition)
                
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
          return self.client.rbprm.rbprm.generateComTraj(positions, velocities, accelerations, dt)
          
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
          return self.client.rbprm.rbprm.generateRootPath(positions, configState1[3:7], configState2[3:7])
          
     ## Given start and goal states
     #  Provided a path has already been computed and interpolated, generate a continuous path
     #  between two indicated states. Will fail if the index of the states do not exist
     #
     # \param index of first state.
     # \param index of second state.
     # \param numOptim Number of iterations of the shortcut algorithm to apply between each states
     #  \return id of the resulting path
     def limbRRT(self, state1, state2, numOptim = 10):
          return self.client.rbprm.rbprm.limbRRT(state1, state2, numOptim)
          
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
          return self.client.rbprm.rbprm.limbRRTFromRootPath(state1, state2, path, numOptim)          
          
          
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
          return self.client.rbprm.rbprm.comRRT(state1, state2, path, numOptim)          
          
                
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
          return self.client.rbprm.rbprm.comRRTFromPos(state1, comPos1, comPos2, comPos3, numOptim)
          
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
          return self.client.rbprm.rbprm.effectorRRTFromPosBetweenState(state1, state2, comPos1, comPos2, comPos3, numOptim)
          
     def comRRTFromPosBetweenState(self, state1, state2, comPos1, comPos2, comPos3, numOptim = 10):
          return self.client.rbprm.rbprm.comRRTFromPosBetweenState(state1, state2, comPos1, comPos2, comPos3, numOptim)
          
                
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
          return self.client.rbprm.rbprm.effectorRRT(state1, comPos1, comPos2, comPos3, numOptim)
          
                          
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
          return self.client.rbprm.rbprm.effectorRRTFromPath(state1, refPathId, path_start, path_to, comPos1, comPos2, comPos3, numOptim, trackedEffectors)
          
     ## Project a given state into a given COM position
     # between two indicated states. The states do not need to be consecutive, but increasing in Id.
     # Will fail if the index of the state does not exist.
     # \param state index of first state.
     # \param targetCom 3D vector for the com position
     # \return projected configuration
     def projectToCom(self, state, targetCom, maxNumSample = 0):
          return self.client.rbprm.rbprm.projectToCom(state, targetCom, maxNumSample)     
          
     ## Returns the configuration at a given state
     # Will fail if the index of the state does not exist.
     # \param state index of state.
     # \return state configuration
     def getConfigAtState(self, state):
          return self.client.rbprm.rbprm.getConfigAtState(state)          
          
     ## Project a given state into a given COM position
     # and update the state configuration.
     # \param state index of first state.
     # \param targetCom 3D vector for the com position
     # \return whether the projection was successful
     def projectStateToCOM(self, state, targetCom, maxNumSample = 0):
          return self.client.rbprm.rbprm.projectStateToCOM(state, targetCom, maxNumSample)     > 0
          
     ## Project a given state into a given COM position
     # and update the state configuration.
     # \param state index of first state.
     # \param targetCom 3D vector for the com position
     # \return whether the projection was successful
     def setConfigAtState(self, state, targetCom):
          return self.client.rbprm.rbprm.setConfigAtState(state, targetCom)     > 0
          
     def setRefConfig(self, targetCom):
          return self.client.rbprm.rbprm.setRefConfig(targetCom)     > 0
          
     ## Given start and goal states
     #  generate a contact sequence over a list of configurations
     #
     # \param stepSize discretization step
     # \param pathId Id of the path to compute from
     def isConfigBalanced(self, config, names, robustness = 0):
          if (self.client.rbprm.rbprm.isConfigBalanced(config, names, robustness) == 1):
                return True
          else:
                return False
          
     ## Given start and goal states
     #  generate a contact sequence over a list of configurations
     #
     # \param stepSize discretization step
     # \param pathId Id of the path to compute from
     def isStateBalanced(self, stateId, robustness = 0):
          return self.client.rbprm.rbprm.isStateBalanced(stateId) > robustness
                          
     ## Updates limb databases with a user chosen computation
     #
     # \param analysis name of computation
     # \param isstatic whether the computation should be used to sort samples by default
     def runSampleAnalysis(self, analysis, isstatic):
          isStatic = 0.
          if(isstatic):
                isStatic = 1.
          self.client.rbprm.rbprm.runSampleAnalysis(analysis,isStatic)
          
     ## Updates a limb database with a user chosen computation
     #
     # \param limbname name of the limb chosen for computation
     # \param analysis name of computation
     # \param isstatic whether the computation should be used to sort samples by default
     def runLimbSampleAnalysis(self, limbname, analysis, isstatic):
          isStatic = 0.
          if(isstatic):
                isStatic = 1.
          return self.client.rbprm.rbprm.runLimbSampleAnalysis(limbname, analysis,isStatic)
          
     ## if the preprocessor variable PROFILE is active
     # dump the profiling data into a logFile
     # \param logFile name of the file where to dump the profiling data
     def dumpProfile(self, logFile="log.txt"):
          return self.client.rbprm.rbprm.dumpProfile(logFile)

     ## Create octree nodes representation for a given limb
     #
     # \param gui gepetoo viewer instance discretization step
     # \param winId window to draw to step
     # \param limbName name of the limb considered
     # \param config initial configuration of the robot, used when created octree
     # \param color of the octree boxes
     def createOctreeBoxes(self, gui, winId, limbName, config, color = [1,1,1,0.3]):
          boxes = self.client.rbprm.rbprm.getOctreeBoxes(limbName, config)
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
          return self.client.rbprm.rbprm.getOctreeBox(limbName, ocId)
     
     ## Draws robot configuration, along with octrees associated
     #
     # \param viewer gepetto viewer instance
     def draw(self, configuration, viewer):
          viewer(configuration)
          for limb, groupid in self.octrees.iteritems():
                transform = self.client.rbprm.rbprm.getOctreeTransform(limb, configuration)
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
          return self.client.rbprm.rbprm.computeTargetTransform(limbName, configuration, p, n)

     ## Export motion to a format readable by the blender
     # importFromGepetto script
     # \param viewer gepetto viewer instance
     # \param configurations list of configurations to save
     # \param filename outputfile where to export the motion
     def exportMotion(self, viewer, configurations, filename):
          em.exportStates(viewer, self.client.basic.robot, configurations, filename)
          
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

    ## \name Degrees of freedom
     #  \{

     ## Get size of configuration
     # \return size of configuration
     def getConfigSize (self):
          return self.client.basic.robot.getConfigSize ()

     # Get size of velocity
     # \return size of velocity
     def getNumberDof (self):
          return self.client.basic.robot.getNumberDof ()
     ## \}

     ## \name Joints
     #\{

     ## Get joint names in the same order as in the configuration.
     def getJointNames (self):
          return self.client.basic.robot.getJointNames ()

     ## Get joint names in the same order as in the configuration.
     def getAllJointNames (self):
          return self.client.basic.robot.getAllJointNames ()

     ## Get joint position.
     def getJointPosition (self, jointName):
          return self.client.basic.robot.getJointPosition (jointName)

     ## Set static position of joint in its parent frame
     def setJointPosition (self, jointName, position):
          return self.client.basic.robot.setJointPosition (jointName, position)

     ## Get joint number degrees of freedom.
     def getJointNumberDof (self, jointName):
          return self.client.basic.robot.getJointNumberDof (jointName)

     ## Get joint number config size.
     def getJointConfigSize (self, jointName):
          return self.client.basic.robot.getJointConfigSize (jointName)

     ## set bounds for the joint
     def setJointBounds (self, jointName, inJointBound):
          return self.client.basic.robot.setJointBounds (jointName, inJointBound)

     ## Set bounds on the translation part of the freeflyer joint.
     #
     #  Valid only if the robot has a freeflyer joint.
     def setTranslationBounds (self, xmin, xmax, ymin, ymax, zmin, zmax):
          self.client.basic.robot.setJointBounds \
                (self.displayName + "base_joint_x", [xmin, xmax])
          self.client.basic.robot.setJointBounds \
                (self.displayName + "base_joint_y", [ymin, ymax])
          self.client.basic.robot.setJointBounds \
                (self.displayName + "base_joint_z", [zmin, zmax])

     ## Get link position in joint frame
     #
     # Joints are oriented in a different way as in urdf standard since
     # rotation and uni-dimensional translation joints act around or along
     # their x-axis. This method returns the position of the urdf link in
     # world frame.
     #
     # \param jointName name of the joint
     # \return position of the link in world frame.
     def getLinkPosition (self, jointName):
          return self.client.basic.robot.getLinkPosition (jointName)

     ## Get link name
     #
     # \param jointName name of the joint,
     # \return name of the link.
     def getLinkName (self, jointName):
          return self.client.basic.robot.getLinkName (jointName)
     ## \}

     ## \name Access to current configuration
     #\{

     ## Set current configuration of composite robot
     #
     #  \param q configuration of the composite robot
     def setCurrentConfig (self, q):
          self.client.basic.robot.setCurrentConfig (q)

     ## Get current configuration of composite robot
     #
     #  \return configuration of the composite robot
     def getCurrentConfig (self):
          return self.client.basic.robot.getCurrentConfig ()

     ## Shoot random configuration
     #  \return dofArray Array of degrees of freedom.
     def shootRandomConfig(self):
          return self.client.basic.robot.shootRandomConfig ()

     ## \}

     ## \name Bodies
     #  \{

     ##  Get the list of objects attached to a joint.
     #  \param inJointName name of the joint.
     #  \return list of names of CollisionObject attached to the body.
     def getJointInnerObjects (self, jointName):
          return self.client.basic.robot.getJointInnerObjects (jointName)


     ##  Get list of collision objects tested with the body attached to a joint
     #  \param inJointName name of the joint.
     #  \return list of names of CollisionObject
     def getJointOuterObjects (self, jointName):
          return self.client.basic.robot.getJointOuterObjects (jointName)

     ## Get position of robot object
     #  \param objectName name of the object.
     #  \return transformation as a hpp.Transform object
     def getObjectPosition (self, objectName):
          return Transform (self.client.basic.robot.getObjectPosition
                                  (objectName))

     ## \brief Remove an obstacle from outer objects of a joint body
     #
     #  \param objectName name of the object to remove,
     #  \param jointName name of the joint owning the body,
     #  \param collision whether collision with object should be computed,
     #  \param distance whether distance to object should be computed.
     def removeObstacleFromJoint (self, objectName, jointName, collision,
                                            distance):
          return self.client.basic.obstacle.removeObstacleFromJoint \
                (objectName, jointName, collision, distance)


     ## \}

     ## \name Collision checking and distance computation
     # \{

     ## Test collision with obstacles and auto-collision.
     #
     # Check whether current configuration of robot is valid by calling
     # CkwsDevice::collisionTest ().
     # \return whether configuration is valid
     # \note Deprecated. Use isConfigValid instead.
     def collisionTest (self):
          print "Deprecated. Use isConfigValid instead"
          return self.client.basic.robot.collisionTest ()

     ## Check the validity of a configuration.
     #
     # Check whether a configuration of robot is valid.
     # \param cfg a configuration
     # \return whether configuration is valid
     def isConfigValid (self, cfg):
          return self.client.basic.robot.isConfigValid (cfg)

     ## Compute distances between bodies and obstacles
     #
     # \return list of distances,
     # \return names of the objects belonging to a body
     # \return names of the objects tested with inner objects,
     # \return  closest points on the body,
     # \return  closest points on the obstacles
     # \note outer objects for a body can also be inner objects of another
     # body.
     def distancesToCollision (self):
          return self.client.basic.robot.distancesToCollision ()
     ## \}

     ## \}
     ## \name Mass and inertia
     # \{

     ## Get mass of robot
     def getMass (self):
          return self.client.basic.robot.getMass ()

     ## Get position of center of mass
     def getCenterOfMass (self):
          return self.client.basic.robot.getCenterOfMass ()
     ## Get Jacobian of the center of mass
     def getJacobianCenterOfMass (self):
          return self.client.basic.robot.getJacobianCenterOfMass ()
     ##\}
