#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a skeleton-robot and a groundcrouch environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 4 end-effectors

from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'spiderman_trunk'
urdfNameRoms = ['SpidermanLFootSphere','SpidermanRFootSphere','SpidermanLHandSphere','SpidermanRHandSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
base_joint_xyz_limits = [-10, 10, -10, 15, 0, 10]

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
rbprmBuilder.boundSO3([-0.2,0.2,-3.14,3.14,-0.3,0.3])
rbprmBuilder.setFilter(urdfNameRoms)
affordanceType = ['Support']
rbprmBuilder.setAffordanceFilter('SpidermanLFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('SpidermanRFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('SpidermanLHandSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('SpidermanRHandSphere', affordanceType)
rbprmBuilder.setContactSize (0.03,0.08)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)


q_init = rbprmBuilder.getCurrentConfig ();
q_init [3:7] = [ 0.98877108,  0.        ,  0.14943813,  0.        ]
q_init [0:3] = [-0.05, -0.82, 0.50]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
#~ q_init [0:3] = [0.1, -0.82, 0.648702]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
#~ q_init [0:3] = [0, -0.63, 0.6]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
#~ q_init [3:7] = [ 0.98877108,  0.        ,  0.14943813,  0.        ]

q_goal = q_init [::]
q_goal [3:7] = [ 0.98877108,  0.        ,  0.14943813,  0.        ]
q_goal [0:3] = [0.6 	, -0.82, 1.5]; r (q_goal)
#~ q_goal [0:3] = [3, -0.82, 6]; r(q_goal)
#~ q_goal [0:3] = [1.2, -0.65, 1.1]; r (q_goal)

#~ ps.addPathOptimizer("GradientBased")
ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.setAffordanceConfig('Lean', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel (packageName, "scale", "planning", r)
#~ afftool.analyseAll()
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])
afftool.visualiseAffordances('Lean', r, [0, 0, 0.9])

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
#~ ps.solve ()
t = ps.solve ()

print t;
if isinstance(t, list):
	t = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("path computation " + str(t) + "\n")
f.close()


from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
#~ pp.fromFile("/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")
#~ 
#~ pp (2)
#~ pp (0)

#~ pp (1)
#~ pp.toFile(1, "/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")
#~ rbprmBuilder.exportPath (r, ps.client.problem, 1, 0.01, "stair_bauzil_hrp2_path.txt")

cl = Client()
cl.problem.selectProblem("rbprm_path")
rbprmBuilder2 = Robot ("toto")
ps2 = ProblemSolver( rbprmBuilder2 )
cl.problem.selectProblem("default")
cl.problem.movePathToProblem(1,"rbprm_path",rbprmBuilder.getAllJointNames())
r2 = Viewer (ps2, viewerClient=r.client)
r.client.gui.setVisibility("toto", "OFF")
#~ r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
#~ r2(q_far)
