#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import quaternion as quat

packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"

#  Information to retrieve urdf and srdf files.
urdfName = "hyq"
urdfSuffix = "_6D"
srdfSuffix = ""

#  This time we load the full body model of HyQ
fullBody = FullBody () 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-20,20, -20, 20, -20, 20])

#  Setting a number of sample configurations used
nbSamples = 100000

rootName = 'base_joint_xyz'

#  Creating limbs
# cType is "_3_DOF": positional constraint, but no rotation (contacts are punctual)
cType = "_6_DOF"
# string identifying the limb
rLegId = 'rfleg'
# First joint of the limb, as in urdf file
rLeg = 'rf_haa_joint'
# Last joint of the limb, as in urdf file
rfoot = 'rf_foot_Z'
# Specifying the distance between last joint and contact surface
offset = [0.,-0.021,0.]
# Specifying the contact surface direction when the limb is in rest pose
normal = [0,1,0]
# Specifying the rectangular contact surface length
legx = 0.02; legy = 0.02
# remaining parameters are the chosen heuristic (here, manipulability), and the resolution of the octree (here, 10 cm).
fullBody.addLimb(rLegId,rLeg,rfoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

lLegId = 'lhleg'
lLeg = 'lh_haa_joint'
lfoot = 'lh_foot_Z'
fullBody.addLimb(lLegId,lLeg,lfoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

rarmId = 'rhleg'
rarm = 'rh_haa_joint'
rHand = 'rh_foot_Z'
fullBody.addLimb(rarmId,rarm,rHand,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

larmId = 'lfleg'
larm = 'lf_haa_joint'
lHand = 'lf_foot_Z'
fullBody.addLimb(larmId,larm,lHand,offset,normal, legx, legy, nbSamples, "forward", 0.05, cType)

#make sure this is 0
q_0 = fullBody.getCurrentConfig ()
zeroConf = [0,0,0, 1, 0, 0, 0]
q_0[0:7] = zeroConf
fullBody.setCurrentConfig (q_0)

effectors = [rfoot, lfoot, lHand, rHand]
limbIds = [rLegId, lLegId, larmId, rarmId ]

import numpy as np
#~ effectorName = rfoot
#~ limbId = rLegId
#~ q = fullBody.getSample(limbId, 1)
#~ fullBody.setCurrentConfig(q) #setConfiguration matching sample
#~ qEffector = fullBody.getJointPosition(effectorName)
#~ q0 = quat.Quaternion(qEffector[3:7])
#~ rot = q0.toRotationMatrix() #compute rotation matrix world -> local
#~ p = qEffector[0:3] #(0,0,0) coordinate expressed in effector fram
#~ rm=np.zeros((4,4))
#~ for i in range(0,3):
	#~ for j in range(0,3):
		#~ rm[i,j] = rot[i,j]
#~ for i in range(0,3):
	#~ rm[i,3] = qEffector[i]
#~ rm[3,3] = 1
#~ invrm = np.linalg.inv(rm)
#~ p = invrm.dot([0,0,0,1])

points = [[],[],[],[]]

def printComPosition(nbConfigs):
	num_invalid = 0
	for i in range(0,nbConfigs):
		q = fullBody.shootRandomConfig()
		q[0:7] = zeroConf
		fullBody.setCurrentConfig(q) #setConfiguration matching sample
		com = fullBody.getCenterOfMass()
		for x in range(0,3):
			q[x] = -com[x]
		fullBody.setCurrentConfig(q)
		#~ print ("final com" + str(com))
		#~ print ("final com" + str(fullBody.getCenterOfMass()))
		if(fullBody.isConfigValid(q)[0]):
			for j in range(0,len(effectors)):
				effectorName = effectors[j]
				limbId = limbIds[j]
				qEffector = fullBody.getJointPosition(effectorName)
				q0 = quat.Quaternion(qEffector[3:7])
				rot = q0.toRotationMatrix() #compute rotation matrix world -> local
				p = qEffector[0:3] #(0,0,0) coordinate expressed in effector fram
				rm=np.zeros((4,4))
				for k in range(0,3):
					for l in range(0,3):
						rm[k,l] = rot[k,l]
				for m in range(0,3):
					rm[m,3] = qEffector[m]
				rm[3,3] = 1
				invrm = np.linalg.inv(rm)
				p = invrm.dot([0,0,0,1])
				points[j].append(p)
				#~ print (points[j])
		else:
			num_invalid +=1
	for j in range(0,len(limbIds)):
		f1=open('./'+str(limbIds[j])+'_com.erom', 'w+')
		for p in points[j]:
			f1.write(str(p[0]) + "," + str(p[1]) + "," + str(p[2]) + "\n")
		f1.close()
	print "%invalid ", (float)(num_invalid) / (float)(nbConfigs) * 100, "%"

#~ printRootPosition(rLegId, rfoot, nbSamples)
#~ printRootPosition(lLegId, lfoot, nbSamples)
#~ printRootPosition(rarmId, rHand, nbSamples)
#~ printRootPosition(larmId, lHand, nbSamples) 
printComPosition(100000)
