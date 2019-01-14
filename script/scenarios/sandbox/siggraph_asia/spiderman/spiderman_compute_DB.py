#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder

from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.tools.plot_analytics  import plotOctreeValues
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import numpy as np
from viewer_library import *



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""


fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])

"""
from hpp.gepetto import Viewer
psf = ProblemSolver( fullBody ); rr = Viewer (psf)
q_0 = fullBody.getCurrentConfig(); rr(q_0); fullBody.isConfigValid(q_0)
"""


q_flexion  = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0, 0.2, 0, -0.0, -0.3, 0, 0.2, 0.9, 0, -0.6, 0, 0, 0, -0.2, 0.9, 0, -0.6, 0, 0, 0, -1.1, -1.8, -1, 2.2, -0.9, 0, 0.0, 1.1, -1.8, 1, 2.2, -0.9, 0, 0.0]
q_contact = [0, 0, 0, 1, 0, 0, 0, 0, 0.0, 0, -0.0, 0.0, 0.0, 2.2, 0.1, 0.3, -1.5, 0.8, 0, 0, -2.2, 0.1, -0.3, -1.5, -0.8, 0, 0, 0.3, -1.1, 0.2, 2, -0.8, 0, 0.0, -0.3, -1.1, -0.2, 2, -0.8, 0, 0.0]

fullBody.isConfigValid(q_flexion)
fullBody.isConfigValid(q_contact)



q_lfeet = q_flexion[fullBody.rankInConfiguration ['LThigh_rx']:fullBody.rankInConfiguration ['LFootToe_ry']+1]
q_rfeet = q_flexion[fullBody.rankInConfiguration ['RThigh_rx']:fullBody.rankInConfiguration ['RFootToe_ry']+1]
q_larm = q_contact[fullBody.rankInConfiguration ['LHumerus_rx']:fullBody.rankInConfiguration ['LHand_ry']+1]
q_rarm = q_contact[fullBody.rankInConfiguration ['RHumerus_rx']:fullBody.rankInConfiguration ['RHand_ry']+1]

fullBody.addRefConfigAnalysisWeight(q_lfeet,"RefPoseLFeet",[1.,1.,1.,5.,1.,1.,1.])
fullBody.addRefConfigAnalysisWeight(q_rfeet,"RefPoseRFeet",[1.,1.,1.,5.,1.,1.,1.])
fullBody.addRefConfigAnalysis(q_larm,"RefPoseLArm")
fullBody.addRefConfigAnalysis(q_rarm,"RefPoseRArm")


nbSamples = 50000
x = 0.03 # contact surface width
y = 0.08 # contact surface length

rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'

fullBody.addLimb(rLegId,'RThigh_rx','SpidermanRFootSphere',[0,0,0],[0,0,1], x, y, nbSamples, "EFORT_Normal", 0.01,"_3_DOF")
fullBody.addLimb(lLegId,'LThigh_rx','SpidermanLFootSphere',[0,0,0],[0,0,1], x, y, nbSamples, "EFORT_Normal", 0.01,"_3_DOF")

fullBody.addLimb(rarmId,'RHumerus_rx','SpidermanRHandSphere',[0,0,0],[0,-1,0], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(larmId,'LHumerus_rx','SpidermanLHandSphere',[0,0,0],[0,1,0], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")

print("Limbs added to fullbody")

def runallLLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseLFeet", True)
    fullBody.saveLimbDatabase(lid, dbName)

def runallRLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseRFeet", True)
    fullBody.saveLimbDatabase(lid, dbName)

def runallLArm(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseLArm", True)
    fullBody.saveLimbDatabase(lid, dbName)

def runallRArm(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseRArm", True)
    fullBody.saveLimbDatabase(lid, dbName)



print("Run all legs : ")
runallLLeg(lLegId, './Spiderman_lleg_flexion_3DOF_EN.db')
runallRLeg(rLegId, './Spiderman_rleg_flexion_3DOF_EN.db')
print("Run all arms : ")
#runallLArm(larmId, './Spiderman_larm_contact_6DOF_EN.db')
#runallRArm(rarmId, './Spiderman_rarm_contact_6DOF_EN.db')

