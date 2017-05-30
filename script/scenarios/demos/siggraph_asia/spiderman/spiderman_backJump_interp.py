#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *

import spiderman_backJump_path as tp


packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""
V0list = tp.V0list
Vimplist = tp.Vimplist
base_joint_xyz_limits = tp.base_joint_xyz_limits

fullBody = FullBody ()
robot = fullBody.client.basic.robot
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)

#psf = ProblemSolver(fullBody); rr = Viewer (psf); gui = rr.client.gui
r = tp.r; ps = tp.ps
psf = tp.ProblemSolver( fullBody ); rr = tp.Viewer (psf); gui = rr.client.gui
pp = PathPlayer (fullBody.client.basic, rr); pp.speed = 0.6
q_0 = fullBody.getCurrentConfig(); rr(q_0)


flexion  = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0, 0.2, 0, -0.0, -0.3, 0, 0.2, 0.9, 0, -0.6, 0, 0, 0, -0.2, 0.9, 0, -0.6, 0, 0, 0, -1.1, -1.8, -1, 2.2, -0.9, 0, 0.0, 1.1, -1.8, 1, 2.2, -0.9, 0, 0.0]
q_contact_takeoff = [0, 0, 0, 1, 0, 0, 0, 0, 0.0, 0, -0.0, 0.0, 0.0, 2.2, 0.1, 0.3, -1.5, 0.8, 0, 0, -2.2, 0.1, -0.3, -1.5, -0.8, 0, 0, 0.3, -1.1, 0.2, 2, -0.8, 0, 0.0, -0.3, -1.1, -0.2, 2, -0.8, 0, 0.0]
extending = [0, 0, 0, 1, 0, 0, 0, -0.0, 0.8, 0, -0.0, -0.6, 0, 1.5, 0.5, 1, 0, 0, 0, 0, -1.5, 0.5, -1, 0, 0, 0, 0, 1.4, -1.2, 1.6, 2.1, 0.4, 0, 0.0, -1.4, -1.2, -1.6, 2.1, 0.4, 0.0, 0.0]

fullBody.setPose (extending, "extending")
fullBody.setPose (flexion, "flexion")
fullBody.setPose (q_contact_takeoff, "takeoffContact")

rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
nbSamples = 50000; x = 0.03; y = 0.08
fullBody.addLimb(rLegId,'RThigh_rx','SpidermanRFootSphere',[0,0,0],[0,0,1], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(lLegId,'LThigh_rx','SpidermanLFootSphere',[0,0,0],[0,0,1], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(rarmId,'RHumerus_rx','SpidermanRHandSphere',[0,0,0],[0,-1,0], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(larmId,'LHumerus_rx','SpidermanLHandSphere',[0,0,0],[0,1,0], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
print("Limbs added to fullbody")


confsize = len(tp.q11)

### TEST OF CONTACT CREATION FOR INTERMEDIATE CONFIG, NOT USED FOR INTERPOLATION
q_tmp = q_contact_takeoff [::]; q_tmp[0:confsize-tp.ecsSize] = tp.q_cube[0:confsize-tp.ecsSize]; rr(q_tmp)
fullBody.setCurrentConfig (q_tmp)
q_tmp_test = fullBody.generateContacts(q_tmp, [0,-1,0], True); rr (q_tmp_test)
fullBody.isConfigValid(q_tmp_test)

# result:
# q_tmp_test = [-1.2, -2.8, 3.6, 0.707107, 0.0, 0.0, 0.707107, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9324800803082636, -0.9184709614284768, 0.6886200849241174, -0.6066622060535428, 0.47649495495305294, 1.0976823065116303, -0.538404483799899, -1.0681738092891575, -1.1021076588270258, 1.1498838725595328, -0.6156809000975677, 0.5815958533218022, -1.4659758542959642, -0.3603605133380307, 0.36116581520970376, -1.048638878548546, 0.24059108997189355, 1.23953255675232, -0.7519812787252685, -0.1402404928640359, -1.0, 0.023118656707620415, -0.6298340889273957, -0.15800407650545129, 0.4963824557225752, -0.4989080182494368, 0.2774303858753873, -0.9974339561414656]

