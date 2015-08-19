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

## Corba clients to the various servers
#
class CorbaClient:
    """
    Container for corba clients to various interfaces.
    """
    def __init__ (self):
        self.basic = BasicClient ()
        self.rbprm = RbprmClient ()

## Load and handle a RbprmDevice robot for rbprm planning
#
#  A RbprmDevice robot is a set of two robots. One for the 
#  trunk of the robot, one for the range of motion
class Builder (object):
    ## Constructor
    # \param trunkName name of the first robot that is loaded now,
    # \param romName name of the first robot that is loaded now,
    # \param rootJointType type of root joint among ("freeflyer", "planar",
    #        "anchor"),
    # \param load whether to actually load urdf files. Set to no if you only
    #        want to initialize a corba client to an already initialized
    #        problem.
    def __init__ (self, load = True):
        self.tf_root = "base_link"
        self.rootJointType = dict()
        self.client = CorbaClient ()
        self.load = load
        
    ## Virtual function to load the robot model
    def loadModel (self, urdfName, urdfNameroms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix):
		if(isinstance(urdfNameroms, list)):		
			for urdfNamerom in urdfNameroms:
				self.client.rbprm.rbprm.loadRobotRomModel(urdfNamerom, rootJointType, packageName, urdfNamerom, urdfSuffix, srdfSuffix)
		else:
			self.client.rbprm.rbprm.loadRobotRomModel(urdfNameroms, rootJointType, packageName, urdfNameroms, urdfSuffix, srdfSuffix)
		self.client.rbprm.rbprm.loadRobotCompleteModel(urdfName, rootJointType, packageName, urdfName, urdfSuffix, srdfSuffix)		
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
		rankInConfiguration = rankInVelocity = 0
		for j in self.jointNames:
			self.rankInConfiguration [j] = rankInConfiguration
			rankInConfiguration += self.client.basic.robot.getJointConfigSize (j)
			self.rankInVelocity [j] = rankInVelocity
			rankInVelocity += self.client.basic.robot.getJointNumberDof (j)



	## Init RbprmShooter
    #
    # \param jointName name of the joint,
    # \return name of the link.
    def initshooter (self):
        return self.client.rbprm.rbprm.initshooter ()
    ## \}

	## Init RbprmShooter
    #
    # \param jointName name of the joint,
    # \return name of the link.
    def setFilter (self, romFilter):
        return self.client.rbprm.rbprm.setFilter (romFilter)
    ## \}

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
