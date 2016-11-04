# -*- coding: utf-8 -*-
"""
Created on Thu Oct  6 14:50:16 2016

@author: adelpret
"""

import numpy as np
from numpy.linalg import norm
from math import sqrt
from pinocchio.utils import zero as mat_zeros
from pinocchio.utils import rand as mat_rand
from pinocchio_inv_dyn.acc_bounds_util import computeVelLimits
from pinocchio_inv_dyn.sot_utils import solveLeastSquare
from pinocchio_inv_dyn.multi_contact.utils import can_I_stop
from pinocchio_inv_dyn.multi_contact.stability_criterion import StabilityCriterion


''' Solve a QP to compute initial joint velocities that satisfy contact constraints and optionally other
    specified constraints, such as having the capture point inside the convex hull of the contact points.
    @param q Joint configuration
    @param invDynForm An instance of invDynFormUtil
    @param ZERO_INITIAL_ANGULAR_MOMENTUM True if initial state must have zero initial angular momentum
    @param ZERO_INITIAL_VERTICAL_COM_VEL True if initial state must have zero initial vertical com velocity
    @param ENSURE_INITIAL_CAPTURE_POINT_OUT True if initial capture point must be outside the support polygon
    @param ENSURE_INITIAL_CAPTURE_POINT_IN True if initial capture point must be inside the support polygon
    @param MAX_INITIAL_COM_VEL The maximum norm of the initial com velocity
    @param MAX_ITER Maximum number of iterations
    @return (success, v), where success is a boolean flag and v contains the joint velocities.
'''    
def compute_initial_joint_velocities(q, invDynForm, ZERO_INITIAL_ANGULAR_MOMENTUM, ZERO_INITIAL_VERTICAL_COM_VEL,
                                     ENSURE_INITIAL_CAPTURE_POINT_OUT, ENSURE_INITIAL_CAPTURE_POINT_IN, 
                                     MAX_INITIAL_COM_VEL, MAX_ITER):
    nv = invDynForm.nv;
    na = invDynForm.na;
    (B_ch, b_ch) = invDynForm.getSupportPolygon();
    
    k = invDynForm.Jc.shape[0];
    n_in = k+3 if ZERO_INITIAL_ANGULAR_MOMENTUM else k;
    n_in +=  1 if ZERO_INITIAL_VERTICAL_COM_VEL else 0;
    A = np.matrix.copy(invDynForm.J_com);
    A_in = mat_zeros((n_in,nv));
    lb_in = mat_zeros(n_in);
    ub_in = mat_zeros(n_in);
    A_in[:k,:] = invDynForm.Jc;
    if(ZERO_INITIAL_ANGULAR_MOMENTUM):
        A_in[k:k+3,:] = invDynForm.M[3:6,:];
    if(ZERO_INITIAL_VERTICAL_COM_VEL):
        A_in[-1,:] = invDynForm.J_com[2,:];
    # compute joint velocity bounds for viability
    v_lb = mat_zeros(nv)-1e10;
    v_ub = mat_zeros(nv)+1e10;
    for j in range(na):
        (v_lb[6+j], v_ub[6+j]) = computeVelLimits(q[7+j], invDynForm.qMin[7+j], invDynForm.qMax[7+j], invDynForm.dqMax[6+j], invDynForm.MAX_MIN_JOINT_ACC);
    # sample desired capture point position
    p = np.matlib.copy(invDynForm.contact_points);
    cp_ub = np.matrix([np.max(p[:,0]), np.max(p[:,1])]).T;
    cp_lb = np.matrix([np.min(p[:,0]), np.min(p[:,1])]).T;
    dx_com_des = mat_zeros(3);
    
    initial_state_generated = False;
    counter = 0;
    v0 = mat_zeros(3);
    while(not initial_state_generated):
        counter += 1;        
        if(counter > MAX_ITER):
            return (False, v0);
        
        cp_des = cp_lb + np.multiply(mat_rand(2), (cp_ub-cp_lb));
        cp_ineq = np.min(np.dot(B_ch, cp_des) + b_ch);
        if((ENSURE_INITIAL_CAPTURE_POINT_OUT and cp_ineq>=0.0) or (ENSURE_INITIAL_CAPTURE_POINT_IN and cp_ineq<0.0)):
            continue;
        
        # compute com vel corresponding to desired capture point        
        dx_com_des[:2] = sqrt(9.81/invDynForm.x_com[2]) * (cp_des - invDynForm.x_com[:2]);
        if(norm(dx_com_des)>=MAX_INITIAL_COM_VEL):
            continue;

        # solve QP to find joint velocities
        (imode,v0) = solveLeastSquare(A.A, dx_com_des.A, v_lb.A, v_ub.A, A_in.A, lb_in.A, ub_in.A, regularization=1e-4);
        if(imode==0):
            dx_com_0 = np.dot(invDynForm.J_com, v0);
            cp_0     = invDynForm.x_com[:2] + dx_com_0[:2] / sqrt(9.81/invDynForm.x_com[2,0]);
            cp_ineq = np.min(np.dot(B_ch, cp_0) + b_ch);
            
            if((ENSURE_INITIAL_CAPTURE_POINT_OUT and cp_ineq<0.0) or
               (ENSURE_INITIAL_CAPTURE_POINT_IN and cp_ineq>=0.0) or
               (ENSURE_INITIAL_CAPTURE_POINT_OUT==False and ENSURE_INITIAL_CAPTURE_POINT_IN==False)):
                initial_state_generated = True;
        
    return (True, v0);

''' Solve a QP to compute initial joint velocities that satisfy contact constraints and optionally other
    specified constraints, such as multi-contact stability/unstability. Stability is computed using the 
    multi-contact stability criterion, as implemented in the function can_I_stop.
    @param q Joint configuration
    @param invDynForm An instance of invDynFormUtil
    @param mu Contact friction coefficient
    @param ZERO_INITIAL_ANGULAR_MOMENTUM True if initial state must have zero initial angular momentum
    @param ZERO_INITIAL_VERTICAL_COM_VEL True if initial state must have zero initial vertical com velocity
    @param ENSURE_STABILITY True if initial state must be stable
    @param ENSURE_UNSTABILITY True if initial state must be unstable
    @param MAX_INITIAL_COM_VEL The maximum norm of the initial com velocity
    @param MAX_ITER Maximum number of iterations
    @return (success, v), where success is a boolean flag and v contains the joint velocities.
    @note It assumes that the initial com position allows for static equilibrium. If this is not
          the case it returns False.
'''       
def compute_initial_joint_velocities_multi_contact(q, invDynForm, mu, ZERO_INITIAL_ANGULAR_MOMENTUM, 
                                                   ZERO_INITIAL_VERTICAL_COM_VEL,
                                                   ENSURE_STABILITY, ENSURE_UNSTABILITY, 
                                                   MAX_INITIAL_COM_VEL, MAX_ITER):
    nv = invDynForm.nv;
    na = invDynForm.na;
    
    k = invDynForm.Jc.shape[0];
    n_in = k+3 if ZERO_INITIAL_ANGULAR_MOMENTUM else k;
    n_in +=  1 if ZERO_INITIAL_VERTICAL_COM_VEL else 0;
    A = np.matrix.copy(invDynForm.J_com);
    A_in = mat_zeros((n_in,nv));
    lb_in = mat_zeros(n_in);
    ub_in = mat_zeros(n_in);
    A_in[:k,:] = invDynForm.Jc;
    if(ZERO_INITIAL_ANGULAR_MOMENTUM):
        A_in[k:k+3,:] = invDynForm.M[3:6,:];
    if(ZERO_INITIAL_VERTICAL_COM_VEL):
        A_in[-1,:] = invDynForm.J_com[2,:];
    # compute joint velocity bounds for viability
    v_lb = mat_zeros(nv)-1e10;
    v_ub = mat_zeros(nv)+1e10;
    for j in range(na):
        (v_lb[6+j], v_ub[6+j]) = computeVelLimits(q[7+j], invDynForm.qMin[7+j], invDynForm.qMax[7+j], invDynForm.dqMax[6+j], invDynForm.MAX_MIN_JOINT_ACC);
    # sample desired capture point position
    P = np.matlib.copy(invDynForm.contact_points);
    N = np.matlib.copy(invDynForm.contact_normals);
    dx_com_des = mat_zeros(3);
    
    initial_state_generated = False;
    counter = 0;
    v0 = mat_zeros(3);

    stab_criterion = StabilityCriterion("default", invDynForm.x_com, dx_com_des, P.T, N.T, mu, np.array([0,0,-9.81]), invDynForm.M[0,0])    
    try:
        # check initial state is in static equilibrium
        res = stab_criterion.can_I_stop();
        if(not res.is_stable):
            print "ERROR: initial configuration is not in static equilibrium", res.c, res.dc;
            return (False, v0);
    except Exception as e:
        print "[compute_initial_joint_velocities_multi_contact] Error while testing static equilibrium. %s"%str(e);
        
    while(not initial_state_generated):
        counter += 1;        
        if(counter > MAX_ITER):
            return (False, v0);
        
        dx_com_des = MAX_INITIAL_COM_VEL * mat_rand(3) / sqrt(3.0);
        if(ENSURE_STABILITY or ENSURE_UNSTABILITY):
            try:
                res = stab_criterion.can_I_stop(invDynForm.x_com, dx_com_des);
                if((ENSURE_STABILITY and not res.is_stable) or (ENSURE_UNSTABILITY and res.is_stable)):
                    continue;
            except Exception as e:
                print "[compute_initial_joint_velocities_multi_contact] Error while checking stability of desired com velocity. %s"%str(e);
                continue;
        
        # solve QP to find joint velocities
        (imode,v0) = solveLeastSquare(A.A, dx_com_des.A, v_lb.A, v_ub.A, A_in.A, lb_in.A, ub_in.A, regularization=1e-4);
        if(imode==0):
            dx_com_0 = np.dot(invDynForm.J_com, v0);
            try:
                res = stab_criterion.can_I_stop(invDynForm.x_com, dx_com_0);
                if((ENSURE_UNSTABILITY and not res.is_stable) or (ENSURE_STABILITY and res.is_stable) or
                   (not ENSURE_STABILITY and not ENSURE_UNSTABILITY)):
                    initial_state_generated = True;
            except Exception as e:
                print "[compute_initial_joint_velocities_multi_contact] Error while checking stability of com velocity. %s"%str(e);
        
    return (True, v0);
