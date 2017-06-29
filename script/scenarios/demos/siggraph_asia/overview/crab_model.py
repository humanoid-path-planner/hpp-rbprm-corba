from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp import Error as hpperr
from numpy import array, matrix


packageName = "crab_description"
meshPackageName = "crab_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "crab"
urdfSuffix = "_model"
srdfSuffix = ""

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-10,20, -10, 10, -10, 12.2])


from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer

#  Setting a number of sample configurations used
nbSamples = 20000

rootName = 'base_joint_xyz'



cType = "_3_DOF"
rLegId = 'r1' 
rLeg =   'leg_center_joint_r1' 
rfoot =  'tibia_foot_joint_r1'
offset = [0.,0.,-0.005]
normal = [0,0,1]
offsetr = [0.,0.,0.005]
normalr = [0,0,-1]
legx = 0.02; legy = 0.02

fullBody.addLimb(rLegId,rLeg,rfoot,offset,normal, legx, legy, nbSamples, "jointlimits", 0.1, cType)

lLegId = 'l1'
lLeg = 'leg_center_joint_l1'
lfoot = 'tibia_foot_joint_l1' 
fullBody.addLimb(lLegId,lLeg,lfoot,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)
#~ 
rarmId = 'r3'
rarm = 'leg_center_joint_r3'
rHand = 'tibia_foot_joint_r3'
fullBody.addLimb(rarmId,rarm,rHand,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)

larmId = 'l3'
larm = 'leg_center_joint_l3'
lHand = 'tibia_foot_joint_l3' 
fullBody.addLimb(larmId,larm,lHand,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)

rarmId2 = 'r2'
rarm2 = 'leg_center_joint_r2'
rHand2 = 'tibia_foot_joint_r2'
fullBody.addLimb(rarmId2,rarm2,rHand2,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)

larmId2 = 'l2'
larm2 = 'leg_center_joint_l2'
lHand2 = 'tibia_foot_joint_l2' 
fullBody.addLimb(larmId2,larm2,lHand2,offset,normal, legx, legy, nbSamples, "jointlimits", 0.05, cType)

fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(rarmId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(larmId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(larmId2, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(rarmId2, "jointLimitsDistance", True)

	
	
limbsCOMConstraints = { rLegId :  {'file': "hrp2/RL_com.ineq", 'effector' : rfoot},  
						lLegId :  {'file': "hrp2/LL_com.ineq", 'effector' : lfoot},  
						rarmId :  {'file': "hrp2/RA_com.ineq", 'effector' : rHand},  
						rarmId2 : {'file': "hrp2/LA_com.ineq", 'effector' : rHand},  
						larmId2 : {'file': "hrp2/RA_com.ineq", 'effector' : rHand2},  
						larmId :  {'file': "hrp2/LA_com.ineq", 'effector' : lHand2} }
