#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer

#calling script darpa_hyq_path to compute root path
import darpa_hyq_path as tp

packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"

#  Information to retrieve urdf and srdf files.
urdfName = "hyq"
urdfSuffix = ""
srdfSuffix = ""

#  This time we load the full body model of HyQ
fullBody = FullBody () 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)

#  Setting a number of sample configurations used
nbSamples = 20000

ps = tp.ProblemSolver(fullBody)
r = tp.Viewer (ps)

rootName = 'base_joint_xyz'

#  Creating limbs
# cType is "_3_DOF": positional constraint, but no rotation (contacts are punctual)
cType = "_3_DOF"
# string identifying the limb
rLegId = 'rfleg'
# First joint of the limb, as in urdf file
rLeg = 'rf_haa_joint'
# Last joint of the limb, as in urdf file
rfoot = 'rf_foot_joint'
# Specifying the distance between last joint and contact surface
offset = [0.,-0.021,0.]
offsetl = [0.,0.021,0.]
# Specifying the contact surface direction when the limb is in rest pose
normal = [0,1,0]
normall = [0,-1,0]
# Specifying the rectangular contact surface length
legx = 0.02; legy = 0.02
# remaining parameters are the chosen heuristic (here, manipulability), and the resolution of the octree (here, 10 cm).
fullBody.addLimb(rLegId,rLeg,rfoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.1, cType)

lLegId = 'lhleg'
lLeg = 'lh_haa_joint'
lfoot = 'lh_foot_joint'
fullBody.addLimb(lLegId,lLeg,lfoot,offsetl,normall, legx, legy, nbSamples, "manipulability", 0.05, cType)

rarmId = 'rhleg'
rarm = 'rh_haa_joint'
rHand = 'rh_foot_joint'
fullBody.addLimb(rarmId,rarm,rHand,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

larmId = 'lfleg'
larm = 'lf_haa_joint'
lHand = 'lf_foot_joint'
fullBody.addLimb(larmId,larm,lHand,offsetl,normall, legx, legy, nbSamples, "forward", 0.05, cType)

q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.q_goal[0:7]

# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
q_init = fullBody.generateContacts(q_init, [0,0,1])

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])

# specifying the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[])
fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId,larmId])


r(q_init)
# computing the contact sequence
configs = fullBody.interpolate(0.1, 1, 0)

r.loadObstacleModel ('hpp-rbprm-corba', "darpa", "contact")

# calling draw with increasing i will display the sequence
i = 0;
fullBody.draw(configs[i],r); i=i+1; i-1

from hpp.gepetto import PathPlayer
pp = PathPlayer (ps.robot.client.basic, r)


rbprmBuilder = tp.rbprmBuilder

i = 19 # choose a state to be shown and visualise
fullBody.draw(configs[i],r)
q1 = fullBody.getCurrentConfig()
rbprmBuilder.setCurrentConfig(q1)
tp.r(rbprmBuilder.getCurrentConfig()) # ROM model
r(q1) # full-body model
# debug function to visualise contact points
q = rbprmBuilder.client.rbprm.rbprm.getDebugContactPoints("hyq_rfleg_rom",i)
r.client.gui.addCurve('contact' + str(i), q, [1, 0, 0, 1])
r.client.gui.addToGroup('contact' + str(i), 'Support')

# find contact area. returns radii and transformation. Use this info to get points on elliptic curve
# and visualise
t = rbprmBuilder.client.rbprm.rbprm.getReachableContactArea("hyq_rfleg_rom", True,i)
pts = rbprmBuilder.client.rbprm.rbprm.getPointsOnCurve(t[0],t[1])
r.client.gui.addCurve('reachable' + str(i), pts, [0, 0, 1, 1])
r.client.gui.addToGroup('reachable' + str(i), 'Support')


