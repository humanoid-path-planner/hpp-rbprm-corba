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
	
	## Virtual function to load the fullBody robot model.
	#
    # \param urdfName urdf description of the fullBody robot
    # \param rootJointType type of root joint among ("freeflyer", "planar",
    #        "anchor"), WARNING. Currently RB-PRM only considerds freeflyer roots
    # \param meshPackageName name of the meshpackage from where the robot mesh will be loaded
    # \param packageName name of the package from where the robot will be loaded
    # \param urdfSuffix optional suffix for the urdf of the robot package
    # \param srdfSuffix optional suffix for the srdf of the robot package
    def loadFullBodyModel (self, urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix):
		self.client.rbprm.rbprm.loadFullBodyRobot(urdfName, rootJointType, packageName, urdfName, urdfSuffix, srdfSuffix)
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
    # structure to perform efficient proximity requests. The resulution of the octree, in meters, specifies the size
    # of the unit voxel of the octree. The larger they are, the more samples will be considered as candidates for contact.
    # This can be problematic in terms of performance. The default value is 3 cm.
    def addLimb(self, limbId, name, effectorname, offset, normal, x, y, samples):
		self.client.rbprm.rbprm.addLimb(limbId, name, effectorname, offset, normal, x, y, samples, "EFORT", 0.03)

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
    # structure to perform efficient proximity requests. The resulution of the octree, in meters, specifies the size
    # of the unit voxel of the octree. The larger they are, the more samples will be considered as candidates for contact.
    # This can be problematic in terms of performance. The default value is 3 cm.
    def addLimb(self, limbId, name, effectorname, offset, normal, x, y, samples, heuristicName, resolution):
		self.client.rbprm.rbprm.addLimb(limbId, name, effectorname, offset, normal, x, y, samples, heuristicName, resolution)

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

	## Generates a balanced contact configuration, considering the
    #  given current configuration of the robot, and a direction of motion.
    #  Typically used to generate a start and / or goal configuration automatically for a planning problem.
	#
    # \param configuration the initial robot configuration
    # \param direction a 3d vector specifying the desired direction of motion
    def generateContacts(self, configuration, direction):
		return self.client.rbprm.rbprm.generateContacts(configuration, direction)
		
	## Retrieves the contact candidates configurations given a configuration and a limb
	#
    # \param name id of the limb considered
    # \param configuration the considered robot configuration
    # \param direction a 3d vector specifying the desired direction of motion
    def getContactSamplesIds(self, name, configuration, direction):
		return self.client.rbprm.rbprm.getContactSamplesIds(name, configuration, direction)
	
	## Initialize the first configuration of the path discretization 
	# with a balanced configuration for the interpolation problem;
	#
    # \param configuration the desired start configuration
    # \param contacts the array of limbs in contact
    def setStartState(self, configuration, contacts):
		return self.client.rbprm.rbprm.setStartState(configuration, contacts)
		
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
	
	## Discretizes a path return by a motion planner into a discrete
	# sequence of balanced, contact configurations and returns
	# the sequence as an array of configurations
	#
    # \param stepSize discretization step
    # \param pathId Id of the path to compute from
    def interpolate(self, stepsize, pathId = 1):
		return self.client.rbprm.rbprm.interpolate(stepsize, pathId)
		
	## Given start and goal states
	#  generate a contact sequence over a list of configurations
	#
    # \param stepSize discretization step
    # \param pathId Id of the path to compute from
    def interpolateConfigs(self, configs):
		return self.client.rbprm.rbprm.interpolateConfigs(configs)
		
	## Create octree nodes representation for a given limb
	#
    # \param stepSize discretization step
    # \param gui gepetoo viewer instance discretization step
    # \param winId window to draw to step
    # \param limbName name of the limb considered
    # \param config initial configuration of the robot, used when created octree
    # \param color of the octree boxes
    def createOctreeBoxes(self, gui, winId, limbName, config, color = [1,1,1,0.3]):
		boxes = self.client.rbprm.rbprm.GetOctreeBoxes(limbName, config)
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
	
	## Draws robot configuration, along with octrees associated
	#
    # \param viewer gepetto viewer instance
    def draw(self, configuration, viewer):
	viewer(configuration)
	for limb, groupid in self.octrees.iteritems():
		transform = self.client.rbprm.rbprm.getOctreeTransform(limb, configuration)
		viewer.client.gui.applyConfiguration(groupid,transform)
	viewer.client.gui.refresh()


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
