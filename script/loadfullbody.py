from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer


packageName = "hpp_tutorial"
meshPackageName = "pr2_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "pr2"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0, 2, -2, 0, -1, 1.5])
fullBody.addLimb("r_shoulder_pan_joint", 10000, 0.001)

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( fullBody )
r = Viewer (ps)
r.loadObstacleModel ('hpp-rbprm-corba', "scene", "car")

q_init = fullBody.getCurrentConfig ()
q_init [0:3] = [-0.6, -0.5, -0.4]
r (q_init)
fullBody.setCurrentConfig (q_init)


q_init = fullBody.generateContacts(q_init, [0,1,0])
r (q_init)

fullBody.getContactSamplesIds("r_shoulder_pan_joint", q_init, [0,1,0])

#~ q_init = fullBody.getSample('r_shoulder_pan_joint', 1)
#~ r (q_init)
q_init [0:3] = [-0.6, 0, -0.4]
