from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer

import wall_spiderman_path as path_planner
#~ import hrp2_model as model
import time
tp = path_planner



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""
#~ V0list = tp.V0list
#~ Vimplist = tp.Vimplist
base_joint_xyz_limits = tp.base_joint_xyz_limits

fullBody = FullBody ()
robot = fullBody.client.basic.robot
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)

ps = path_planner.ProblemSolver( fullBody )
r = path_planner.Viewer (ps, viewerClient=path_planner.r.client)
rr = r
pp = PathPlayer (fullBody.client.basic, rr); pp.speed = 0.6
q_0 = fullBody.getCurrentConfig(); rr(q_0)


rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
rfoot = 'SpidermanRFootSphere'
lfoot = 'SpidermanLFootSphere'
lHand = 'SpidermanLHandSphere'
rHand = 'SpidermanRHandSphere'
nbSamples = 50000; x = 0.03; y = 0.08
fullBody.addLimb(rLegId,'RThigh_rx','SpidermanRFootSphere',[0,0,0],[0,0,1], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(lLegId,'LThigh_rx','SpidermanLFootSphere',[0,0,0],[0,0,1], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(rarmId,'RHumerus_rx','SpidermanRHandSphere',[0,0,0],[0,-1,0], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF", grasp = True)
fullBody.addLimb(larmId,'LHumerus_rx','SpidermanLHandSphere',[0,0,0],[0,1,0], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF", grasp = True)


limbsCOMConstraints = { rLegId : {'file': "spiderman/RL_com.ineq", 'effector' : rfoot},  
						lLegId : {'file': "spiderman/LL_com.ineq", 'effector' : rHand},
						rarmId : {'file': "spiderman/RA_com.ineq", 'effector' : rHand},
						larmId : {'file': "spiderman/LA_com.ineq", 'effector' : lHand} }
						
						
ps = path_planner.ProblemSolver( fullBody )
r = path_planner.Viewer (ps, viewerClient=path_planner.r.client)
#~ fullBody.setJointBounds ("base_joint_xyz", [-1,3, -1, 1, 0, 2.2])
fullBody.setJointBounds ("base_joint_xyz", [-1,3, -1, 1, 0, 6])
pp = PathPlayer (fullBody.client.basic, r)

from plan_execute import a, b, c, d, e, init_plan_execute
init_plan_execute(fullBody, r, path_planner, pp)

q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.getCurrentConfig(); q_init[0:7] = path_planner.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = path_planner.q_goal[0:7]

fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])
q_init = fullBody.generateContacts(q_init, [0,0,1])

fullBody.setStartState(q_init,[rLegId,lLegId]) #,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])#,rarmId,larmId])

configs = d(0.005); e()

from bezier_traj import *
init_bezier_traj(fullBody, r, pp, configs, limbsCOMConstraints)
