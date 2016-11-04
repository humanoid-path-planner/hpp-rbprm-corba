'''
To do.
'''
import sys
import os

cwd = os.getcwd()[:-7]; # current path without "python"
if cwd+'/data/config' not in sys.path:
    sys.path += [cwd+'/data/config',];

import conf_hpp as conf
from utils import compute_initial_joint_velocities_multi_contact

import pinocchio as se3
from pinocchio.utils import zero as mat_zeros
from pinocchio.utils import cross as mat_cross

from pinocchio_inv_dyn.robot_wrapper import RobotWrapper
from pinocchio_inv_dyn.inv_dyn_formulation_util import InvDynFormulation
from pinocchio_inv_dyn.tasks import SE3Task
from pinocchio_inv_dyn.trajectories import ConstantSE3Trajectory

from pinocchio_inv_dyn.simulator import Simulator

import pickle
from pickle import load
import numpy as np
from numpy.linalg import norm

EPS = 1e-4;

def createInvDynFormUtil(q, v):
    invDynForm = InvDynFormulation('inv_dyn', q, v, conf.dt, conf.model_path, conf.urdfFileName, conf.freeFlyer);
    invDynForm.enableCapturePointLimits(conf.ENABLE_CAPTURE_POINT_LIMITS);
    invDynForm.enableTorqueLimits(conf.ENABLE_TORQUE_LIMITS);
    invDynForm.enableForceLimits(conf.ENABLE_FORCE_LIMITS);
    invDynForm.enableJointLimits(conf.ENABLE_JOINT_LIMITS, conf.IMPOSE_POSITION_BOUNDS, conf.IMPOSE_VELOCITY_BOUNDS, 
                                 conf.IMPOSE_VIABILITY_BOUNDS, conf.IMPOSE_ACCELERATION_BOUNDS);
    invDynForm.JOINT_POS_PREVIEW            = conf.JOINT_POS_PREVIEW;
    invDynForm.JOINT_VEL_PREVIEW            = conf.JOINT_VEL_PREVIEW;
    invDynForm.MAX_JOINT_ACC                = conf.MAX_JOINT_ACC;
    invDynForm.MAX_MIN_JOINT_ACC            = conf.MAX_MIN_JOINT_ACC;
    invDynForm.USE_JOINT_VELOCITY_ESTIMATOR = conf.USE_JOINT_VELOCITY_ESTIMATOR;
    invDynForm.ACCOUNT_FOR_ROTOR_INERTIAS   = conf.ACCOUNT_FOR_ROTOR_INERTIAS;    
    return invDynForm;
    
def updateConstraints(t, q, v, invDynForm, contacts):
    contact_changed = False;
    
    for (name, PN) in contacts.iteritems():
        if(invDynForm.existUnilateralContactConstraint(name)):
            continue;
        
        contact_changed =True;
        invDynForm.r.forwardKinematics(q, v, 0 * v);
        invDynForm.r.framesKinematics(q);
        
        fid = invDynForm.getFrameId(name);
        oMi = invDynForm.r.framePosition(fid);
        ref_traj = ConstantSE3Trajectory(name, oMi);
        constr = SE3Task(invDynForm.r, fid, ref_traj, name);
        constr.kp = conf.kp_constr;
        constr.kv = conf.kd_constr;
        constr.mask(conf.constraint_mask);
        
        Pi = np.matrix(PN['P']).T;
        Ni = np.matrix(PN['N']).T;
        for j in range(Pi.shape[1]):
            print "    contact point %d in world frame:"%j, oMi.act(Pi[:,j]).T, (oMi.rotation * Ni[:,j]).T;
        invDynForm.addUnilateralContactConstraint(constr, Pi, Ni, conf.fMin, conf.mu);

    return contact_changed;
    

def createSimulator(q0, v0):
    simulator  = Simulator('hrp2_sim', 
                                   q0, v0, conf.fMin, conf.mu, conf.dt, conf.model_path, conf.urdfFileName);
    simulator.viewer.CAMERA_FOLLOW_ROBOT = False;
    simulator.USE_LCP_SOLVER = conf.USE_LCP_SOLVER;
    simulator.ENABLE_TORQUE_LIMITS = conf.FORCE_TORQUE_LIMITS;
    simulator.ENABLE_FORCE_LIMITS = conf.ENABLE_FORCE_LIMITS;
    simulator.ENABLE_JOINT_LIMITS = conf.FORCE_JOINT_LIMITS;
    simulator.ACCOUNT_FOR_ROTOR_INERTIAS = conf.ACCOUNT_FOR_ROTOR_INERTIAS;
    simulator.VIEWER_DT = conf.DT_VIEWER;
    simulator.CONTACT_FORCE_ARROW_SCALE = 2e-3;
    simulator.verb=0;
    return simulator;

def draw_q(q0):	
	p = np.matrix.copy(invDynForm.contact_points);
	simulator.viewer.updateRobotConfig(q0);
	simulator.updateComPositionInViewer(np.matrix([invDynForm.x_com[0,0], invDynForm.x_com[1,0], 0.]).T);
	q0[2] -= 0.005
	simulator.viewer.updateRobotConfig(q0);
	p[2,:] -= 0.005;
	for j in range(p.shape[1]):
		simulator.viewer.addSphere("contact_point"+str(j), 0.005, p[:,j], (0.,0.,0.), (1, 1, 1, 1));
		simulator.viewer.updateObjectConfigRpy("contact_point"+str(j), p[:,j]);
	
	f = open(conf.INITIAL_CONFIG_FILENAME, 'rb');
	res = pickle.load(f);
	f.close();
	p_steve = np.matrix(res[conf.INITIAL_CONFIG_ID]['P']);
	p_steve[:,2] -= 0.005;
	for j in range(p_steve.shape[0]):
		simulator.viewer.addSphere("contact_point_steve"+str(j), 0.005, p_steve[j,:].T, color=(1, 0, 0, 1));
		simulator.viewer.updateObjectConfigRpy("contact_point_steve"+str(j), p_steve[j,:].T);  

def q_pin(q_hpp):
	return np.matrix(q_hpp).T

def gen_com_vel(q0, contacts):
    init(q0);
    v0 = mat_zeros(nv);
    invDynForm.setNewSensorData(0, q0, v0);
    updateConstraints(0, q0, v0, invDynForm, contacts);
    #~ draw_q(q0);

    print 'Gonna compute initial joint velocities that satisfy contact constraints';
    print 'conf.MAX_INITIAL_COM_VEL', conf.MAX_INITIAL_COM_VEL
    (success, v)= compute_initial_joint_velocities_multi_contact(q0, invDynForm, conf.mu[0], 
												conf.ZERO_INITIAL_ANGULAR_MOMENTUM, 
												conf.ZERO_INITIAL_VERTICAL_COM_VEL,
												False, #conf.ENSURE_STABILITY, 
                                                                  True, #conf.ENSURE_UNSTABILITY, 
                                                                  conf.MAX_INITIAL_COM_VEL, 100);
    if success:						
        print "Initial velocities found"
        return (success, invDynForm.J_com * v);
    print "Could not find initial velocities"
    return (success, v[:]);
						
								
def comTranslationAfter07s(q_hpp, contacts):
	(success, dc0)= gen_com_vel(q_pin(q_hpp), contacts)
	return success, dc0 * 0.7
	

np.set_printoptions(precision=2, suppress=True);

if(conf.freeFlyer):
	robot = RobotWrapper(conf.urdfFileName, conf.model_path, root_joint=se3.JointModelFreeFlyer());
else:
	robot = RobotWrapper(conf.urdfFileName, conf.model_path, None);
nq = robot.nq;
nv = robot.nv;
v0 = mat_zeros(nv);
invDynForm = None;
robot = None;
na = None;    # number of joints
simulator =  None;

def init(q0):
	''' CREATE CONTROLLER AND SIMULATOR '''
	global invDynForm
	global robot
	global na
	global simulator
	print "reset invdyn"
	invDynForm = createInvDynFormUtil(q0, v0);
	robot = invDynForm.r;
	na = invDynForm.na;    # number of joints
	simulator = createSimulator(q0, v0);
	#~ gen_com_vel(q0, config_test['contact_points'])
