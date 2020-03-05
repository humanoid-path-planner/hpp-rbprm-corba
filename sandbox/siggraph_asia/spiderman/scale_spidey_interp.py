from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer

import scale_spidey_path as path_planner
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

#psf = ProblemSolver(fullBody); rr = Viewer (psf); gui = rr.client.gui
ps = path_planner.ProblemSolver( fullBody )
r = path_planner.Viewer (ps, viewerClient=path_planner.r.client)
rr = r
#~ psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
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
fullBody.addLimb(rarmId,'RHumerus_rx','SpidermanRHandSphere',[0,0,0],[0,-1,0], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(larmId,'LHumerus_rx','SpidermanLHandSphere',[0,0,0],[0,1,0], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
#~ 
#~ fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)



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


#~ fullBody.setCurrentConfig (q_init)
#~ q_init =  [
        #~ -0.05, -0.82, 0.55, 1.0, 0.0 , 0.0, 0.0,                         	 # Free flyer 0-6
        #~ 0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        #~ 0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # LARM       11-17
        #~ 0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 		 # RARM       18-24
        #~ 0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        #~ 0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        #~ ]; r (q_init)

fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])
q_init = fullBody.generateContacts(q_init, [0,0,1])

#~ fullBody.setStartState(q_init,[rLegId,lLegId,rarmId]) #,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId,larmId, rarmId]) #,rarmId,larmId])
fullBody.setStartState(q_init,[rLegId,lLegId]) #,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])#,rarmId,larmId])

configs = d(0.005); e()

from bezier_traj import *
init_bezier_traj(fullBody, r, pp, configs, limbsCOMConstraints)
#~ AFTER loading obstacles


#~ test_ineq(0,{ rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'}}, 1000, [1,0,0,1])
#~ test_ineq(0,{ lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}}, 1000, [0,0,1,1])
#~ gen(0,1)
