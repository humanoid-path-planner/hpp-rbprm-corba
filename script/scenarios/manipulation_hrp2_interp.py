from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver import Client
from hpp.gepetto import ViewerFactory
from hpp.gepetto import Viewer
import sys

from hpp.corbaserver.manipulation.romeo import Robot
from hpp.corbaserver.manipulation import ProblemSolver, Rule
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer, PathPlayerGui
from math import sqrt


from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"

# Create a new manipulation problem
cl = Client()
cl.problem.selectProblem("rbprm")

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
fullBody.setJointBounds ("base_joint_xyz", [-4,4,-4,4,-4,4])

def manipToRbPrm(qManip):
  q = fullBody.getCurrentConfig()
  for jn, rk in fullBody.rankInConfiguration.iteritems():
    l = fullBody.getJointConfigSize(jn)
    if manipulation.robot.rankInConfiguration.has_key("hrp2/" + jn):
      rk2 = manipulation.robot.rankInConfiguration["hrp2/" + jn]
      q[rk:rk + l] = qManip[rk2:rk2 + l]
    else:
      print "Joint not set", jn
  return q

ps = ProblemSolver( fullBody )
#~ vf = ViewerFactory (ps)
r = Viewer (ps)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.loadObstacleModel ("hpp_environments", "hrp2/floor_as_mesh", "floor", r)
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])
fullBody.client.rbprm.rbprm.setAffordanceFilter('0rLeg', ['Support',])
fullBody.client.rbprm.rbprm.setAffordanceFilter('1lLeg', ['Support'])
#~ vf.loadObstacleModel("hpp_environments", "hrp2/floor_as_mesh", "floor")

cl.problem.selectProblem("manipulationProblem")
ps.client.problem.movePathToProblem(manipulation.ps.numberPaths()-1, "rbprm", ["hrp2/" + jn for jn in fullBody.jointNames])
cl.problem.selectProblem("rbprm")

#~ AFTER loading obstacles
rLegId = '0rLeg'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
# fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 10000, "manipulability", 0.1)
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 10000, "static", 0.1)

lLegId = '1lLeg'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,0]
lLegNormal = [0,1,0]
lLegx = 0.09; lLegy = 0.05
# fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 10000, "manipulability", 0.1)
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 10000, "static", 0.1)

#~ AFTER loading obstacles
larmId = '4Larm'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.05,-0.050,-0.050]
lArmNormal = [1,0,0]
lArmx = 0.024; lArmy = 0.024
#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, 0.05)

rKneeId = '0RKnee'
rLeg = 'RLEG_JOINT0'
rKnee = 'RLEG_JOINT3'
rLegOffset = [0.105,0.055,0.017]
rLegNormal = [-1,0,0]
rLegx = 0.05; rLegy = 0.05
#~ fullBody.addLimb(rKneeId, rLeg,rKnee,rLegOffset,rLegNormal, rLegx, rLegy, 10000, 0.01)
#~ 
lKneeId = '1LKnee'
lLeg = 'LLEG_JOINT0'
lKnee = 'LLEG_JOINT3'
lLegOffset = [0.105,0.055,0.017]
lLegNormal = [-1,0,0]
lLegx = 0.05; lLegy = 0.05
#~ fullBody.addLimb(lKneeId,lLeg,lKnee,lLegOffset,lLegNormal, lLegx, lLegy, 10000, 0.01)
 #~ 

q_init = manipToRbPrm(manipulation.q_init)
q_goal = manipToRbPrm(manipulation.q_goal)

fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId])

fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)

configs = fullBody.interpolate(0.15, 0, 10, True)

limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},  
						lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}, }
						#~ rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand},
						#~ larmId : {'file': "hrp2/LA_com.ineq", 'effector' : lHand} }

#~ fullBody.limbRRTFromRootPath(0,len(configs)-1,0,2)
from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

def act(i, numOptim = 0, use_window = 0, friction = 0.5, optim_effectors = True, verbose = False, draw = False, trackedEffectors = []):
	return step(fullBody, configs, i, numOptim, pp, limbsCOMConstraints, 0.4, optim_effectors = optim_effectors, time_scale = 20., useCOMConstraints = True, use_window = use_window,
	verbose = verbose, draw = draw, trackedEffectors = trackedEffectors)

def play(frame_rate = 1./24.):
	play_traj(fullBody,pp,frame_rate)
	
def saveAll(name):
	saveAllData(fullBody, r, name)

def playPaths(rs = None):
    import time
    ps.client.problem.selectProblem("rbprm")
    ls = [  ps.pathLength(i) for i in range(ps.numberPaths()) ]
    if rs is None:
        rs = [ vf.createViewer() ]
        ps.client.problem.selectProblem("manipulationProblem")
        rs.append( manipulation.vf.createViewer() )
    for i in range(1000):
        ps.client.problem.selectProblem("rbprm")
        rs[0] (ps.configAtParam(1,i * ls[1] / 1000.))
        ps.client.problem.selectProblem("manipulationProblem")
        rs[1] (manipulation.ps.configAtParam(0, i * ls[0] / 1000.))
        time.sleep(0.5)
    return rs
    
#~ for i in range(1,5):
    #~ act(i,60, use_window = 0, optim_effectors = True, draw = False, verbose = True)
    
trackedEffectors = [0, 0, 0.15, ['LARM_JOINT5']]

#~ for i in range(0,1):
	#~ trackedEffectors = [0, i * 0.15, (i+1) * 0.15, ['LARM_JOINT5']];
	#~ act(i,60, use_window = 0, optim_effectors = True, draw = False, verbose = False, trackedEffectors = trackedEffectors)


