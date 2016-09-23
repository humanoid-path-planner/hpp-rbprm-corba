from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer

from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"

import ground_crouch_hyq_path as tp

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
fullBody.setJointBounds ("base_joint_xyz", [-6,5, -4, 4, 0.6, 2])

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

nbSamples = 20000

ps = tp.ProblemSolver( fullBody )
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
# Specifying the contact surface direction when the limb is in rest pose
normal = [0,1,0]
# Specifying the rectangular contact surface length
legx = 0.02; legy = 0.02
# remaining parameters are the chosen heuristic (here, manipulability), and the resolution of the octree (here, 10 cm).



def addLimbDb(limbId, heuristicName, loadValues = True, disableEffectorCollision = False):
	fullBody.addLimbDatabase(str(db_dir+limbId+'.db'), limbId, heuristicName,loadValues, disableEffectorCollision)

lLegId = 'lhleg'
rarmId = 'rhleg'
larmId = 'lfleg'

addLimbDb(rLegId, "static")
addLimbDb(lLegId, "static")
addLimbDb(rarmId, "static")
addLimbDb(larmId, "static")

lfoot = 'lh_foot_joint'
rHand = 'rh_foot_joint'
lHand = 'lf_foot_joint'

q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.q_goal[0:7]

fullBody.setCurrentConfig (q_init)
q_init = fullBody.generateContacts(q_init, [0,0,1])
q_0 = fullBody.getCurrentConfig(); 

fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])

fullBody.setStartState(q_init,[])
fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId,larmId])

r(q_init)

configs = fullBody.interpolate(0.08,1,3) #hole 
#~ configs = fullBody.interpolate(0.08,1,5) # bridge

#~ r.loadObstacleModel ('hpp-rbprm-corba', "groundcrouch", "contact")
#~ fullBody.exportAll(r, configs, 'obstacle_hyq_robust_10');
i = 0;
r (configs[i]); i=i+1; i-1


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)


from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj, profile

	
limbsCOMConstraints = { rLegId : {'file': "hyq/"+rLegId+"_com.ineq", 'effector' : rfoot},  
						lLegId : {'file': "hyq/"+lLegId+"_com.ineq", 'effector' : lfoot},  
						rarmId : {'file': "hyq/"+rarmId+"_com.ineq", 'effector' : rHand},  
						larmId : {'file': "hyq/"+larmId+"_com.ineq", 'effector' : lHand} }



def act(i, numOptim = 0):
	return step(fullBody, configs, i, numOptim, pp, limbsCOMConstraints, 0.5, optim_effectors = False, time_scale = 20., useCOMConstraints = False)

def play(frame_rate = 1./24.):
	play_traj(fullBody,pp,frame_rate)
	
def saveAll(name):
	saveAllData(fullBody, r, name)
#~ fullBody.exportAll(r, trajec, 'obstacle_hyq_t_var_04f_andrea');
#~ saveToPinocchio('obstacle_hyq_t_var_04f_andrea')

#~ profile(fullBody, configs, 4, 10, limbsCOMConstraints)	
profile(fullBody, configs, 4, len(configs)-4, limbsCOMConstraints)	
import time
try:
	time.sleep(2)
	fullBody.dumpProfile()
except Exception as e:
	pass
	
    



