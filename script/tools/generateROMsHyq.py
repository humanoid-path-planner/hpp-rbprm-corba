from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer


import quaternion as quat

packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hyq"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

nbSamples = 50000

ps = ProblemSolver( fullBody )

rootName = 'base_joint_xyz'

#~ AFTER loading obstacles
rLegId = 'rfleg'
rLeg = 'rf_haa_joint'
rfoot = 'rf_foot_joint'
rLegOffset = [0,0,0]
rLegNormal = [0,1,0]
rLegx = 0.02; rLegy = 0.02
fullBody.addLimb(rLegId,rLeg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "EFORT", 0.01)

lLegId = 'lhleg'
lLeg = 'lh_haa_joint'
lfoot = 'lh_foot_joint'
lLegOffset = [0,0,0]
lLegNormal = [0,1,0]
lLegx = 0.02; lLegy = 0.02
fullBody.addLimb(lLegId,lLeg,lfoot,lLegOffset,rLegNormal, lLegx, lLegy, nbSamples, "EFORT", 0.01)

rarmId = 'rhleg'
rarm = 'rh_haa_joint'
rHand = 'rh_foot_joint'
rArmOffset = [0,0,0]
rArmNormal = [1,0,0]
rArmx = 0.02; rArmy = 0.02
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, "EFORT", 0.01)

larmId = 'lfleg'
larm = 'lf_haa_joint'
lHand = 'lf_foot_joint'
lArmOffset = [0,0,0]
lArmNormal = [1,0,0]
lArmx = 0.02; lArmy = 0.02
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, "EFORT", 0.01)


#make sure this is 0
q_0 = fullBody.getCurrentConfig ()

def printEffPosition(limbId, nbSamples):
	limit = nbSamples-1;
	f1=open('./data/roms/hyq/'+limbId+'.erom', 'w+')
	for i in range(0,limit):
		q = fullBody.getSamplePosition(limbId,i)
		f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
	f1.close()

printEffPosition(rarmId, nbSamples)
printEffPosition(rLegId, nbSamples)
printEffPosition(larmId, nbSamples)
printEffPosition(lLegId, nbSamples)
