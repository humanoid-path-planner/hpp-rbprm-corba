from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer


import quaternion as quat

packageName = "phantomx_description"
meshPackageName = "phantomx_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "phantomx"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

nbSamples = 50000

ps = ProblemSolver( fullBody )

rootName = 'base_joint_xyz'

#~ AFTER loading obstacles
rmId = 'rm'
rm = 'j_c1_rm'
rme = 'j_tibia_rm'
rmOffset = [0,0,0]
rmNormal = [0,1,0]
rmx = 0.02; rmy = 0.02
fullBody.addLimb(rmId,rm,rme,rmOffset,rmNormal, rmx, rmy, nbSamples, "EFORT", 0.01)

rrId = 'rr'
rr = 'j_c1_rr'
rre = 'j_tibia_rr'
rrOffset = [0,0,0]
rrNormal = [0,1,0]
rrx = 0.02; rry = 0.02
fullBody.addLimb(rrId,rr,rre,rrOffset,rrNormal, rrx, rry, nbSamples, "EFORT", 0.01)

rfId = 'rf'
rf = 'j_c1_rf'
rfe = 'j_tibia_rf'
rfOffset = [0,0,0]
rfNormal = [0,1,0]
rfx = 0.02; rfy = 0.02
fullBody.addLimb(rfId,rf,rfe,rfOffset,rfNormal, rfx, rfy, nbSamples, "EFORT", 0.01)


lmId = 'lm'
lm = 'j_c1_lm'
lme = 'j_tibia_lm'
lmOffset = [0,0,0]
lmNormal = [0,1,0]
lmx = 0.02; lmy = 0.02
fullBody.addLimb(lmId,lm,lme,lmOffset,lmNormal, lmx, lmy, nbSamples, "EFORT", 0.01)

lrId = 'lr'
lr = 'j_c1_lr'
lre = 'j_tibia_lr'
lrOffset = [0,0,0]
lrNormal = [0,1,0]
lrx = 0.02; lry = 0.02
fullBody.addLimb(lrId,lr,lre,lrOffset,lrNormal, lrx, lry, nbSamples, "EFORT", 0.01)

lfId = 'lf'
lf = 'j_c1_lf'
lfe = 'j_tibia_lf'
lfOffset = [0,0,0]
lfNormal = [0,1,0]
lfx = 0.02; lfy = 0.02
fullBody.addLimb(lfId,lf,lfe,lfOffset,lfNormal, lfx, lfy, nbSamples, "EFORT", 0.01)

#make sure this is 0
q_0 = fullBody.getCurrentConfig ()

def printEffPosition(limbId, nbSamples):
	limit = nbSamples-1;
	f1=open('./data/roms/phantom/'+limbId+'.erom', 'w+')
	for i in range(0,limit):
		q = fullBody.getSamplePosition(limbId,i)
		f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
	f1.close()

printEffPosition(rmId, nbSamples)
printEffPosition(rrId, nbSamples)
printEffPosition(rfId, nbSamples)
printEffPosition(lmId, nbSamples)
printEffPosition(lrId, nbSamples)
printEffPosition(lfId, nbSamples)
