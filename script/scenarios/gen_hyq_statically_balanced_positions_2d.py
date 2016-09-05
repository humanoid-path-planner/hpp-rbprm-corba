from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

from hpp.corbaserver.rbprm.tools.cwc_trajectory import *

from hpp import Error as hpperr


packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"

#  Information to retrieve urdf and srdf files.
urdfName = "hyq"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-0.135,2, -1, 1, 0, 2.2])


#  This time we load the full body model of HyQ
fullBody = FullBody () 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-2,5, -1, 1, 0.3, 4])

#  Setting a number of sample configurations used
nbSamples = 20000

ps = ProblemSolver( fullBody )
r = Viewer (ps)

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
# Specifying the contact surface direction when the limb is in rest pose
normal = [0,1,0]
# Specifying the rectangular contact surface length
legx = 0.02; legy = 0.02
# remaining parameters are the chosen heuristic (here, manipulability), and the resolution of the octree (here, 10 cm).
fullBody.addLimb(rLegId,rLeg,rfoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.1, cType)

lLegId = 'lhleg'
lLeg = 'lh_haa_joint'
lfoot = 'lh_foot_joint'
fullBody.addLimb(lLegId,lLeg,lfoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

rarmId = 'rhleg'
rarm = 'rh_haa_joint'
rHand = 'rh_foot_joint'
fullBody.addLimb(rarmId,rarm,rHand,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

larmId = 'lfleg'
larm = 'lf_haa_joint'
lHand = 'lf_foot_joint'
fullBody.addLimb(larmId,larm,lHand,offset,normal, legx, legy, nbSamples, "forward", 0.05, cType)

q_0 = fullBody.getCurrentConfig(); 


r(q_0)
def draw_cp(cid, limb, config):
	global r
	posetc = fullBody.getEffectorPosition(limb, config)
	print "pos ", posetc[0]
	scene = "qds"+limb+str(cid)
	r.client.gui.createScene(scene)
	for i in range(4):
		pos = posetc[2*i]
		r.client.gui.addBox(scene+"/b"+str(i),0.01,0.01,0.01, [1,0,0,1])
		r.client.gui.applyConfiguration(scene+"/b"+str(i),pos+[1,0,0,0])
		r.client.gui.refresh()	
	r.client.gui.addSceneToWindow(scene,0)

def fill_contact_points(limbs, config, config_pinocchio):
	res = {}
	res["q"] = config_pinocchio[:]
	res["contact_points"] = {}
	res["P"] = []
	res["N"] = []
	for limb in limbs:
		posetc = fullBody.getEffectorPosition(limb, config)
		res["contact_points"][limb] = {}
		res["contact_points"][limb]["P"] = [p for i, p in enumerate (posetc) if (i%2 == 0)]
		res["P"] += [p for i, p in enumerate (posetc) if (i%2 == 0)]
		res["contact_points"][limb]["N"] = [n for i, n in enumerate (posetc) if (i%2 == 1)]
		res["N"] += [n for i, n in enumerate (posetc) if (i%2 == 1)]
	return res

q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.getSampleConfig()
qs = []; qs_gepetto = []; states = []
limbs = [lLegId,rLegId] 
for i in range(2):
	q = fullBody.generateGroundContact(limbs)
	q_gep = q[:]
	quat_end = q[4:7]
	q[6] = q[3]
	q[3:6] = quat_end
	qs.append(q)
	qs_gepetto.append(q_gep)
	states.append(fill_contact_points(limbs,q_gep,q))
	
from pickle import dump
f1=open("configs_feet_on_ground_static_eq", 'w+')
dump(qs, f1)
f1.close()
