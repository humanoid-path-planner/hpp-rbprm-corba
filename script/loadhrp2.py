from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer


packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-1, 2, -2, 0, -1, 1.5])



from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( fullBody )
r = Viewer (ps)
r.loadObstacleModel ('hpp-rbprm-corba', "scene", "car")
#~ 
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.105]
fullBody.addLimb(rLeg,rLegOffset, 5000, 0.001)

lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,-0.105]
fullBody.addLimb(lLeg,lLegOffset, 5000, 0.001)

q_init = fullBody.getCurrentConfig ()
q_init [0:3] = [0, -0.5, 0.7]
r (q_init)
fullBody.setCurrentConfig (q_init)
#~ 
#~ 
q_init = fullBody.generateContacts(q_init, [0,0,1])
r (q_init)
#~ 
fullBody.getContactSamplesIds(rLeg, q_init, [0,0,1])

#~ q_init = fullBody.getSample(rLeg, 1)
#~ r (q_init)

ids = fullBody.getContactSamplesIds(rLeg, q_init, [0,0,1])
i =-1
#~ i=i+1; q_init = fullBody.getSample(rLeg, int(ids[i])); r(q_init); ids[i]

