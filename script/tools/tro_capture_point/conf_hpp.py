from math import sqrt
import numpy as np

''' *********************** USER-PARAMETERS *********************** '''
SOLVER_ID       = 0;    # classic TSID formulation
SOLVER_CP       = 1;
SOLVER_TO_INTEGRATE         = [SOLVER_ID];
DATA_FILE_NAME              = 'data';
TEXT_FILE_NAME              = 'results.txt';
SAVE_DATA                   = True;

''' INITIAL STATE PARAMETERS '''
MAX_TEST_DURATION           = 8000;
dt                          = 1e-3;
model_path                  = ["/home_local/dev/hpp/install/share"];
urdfFileName                = model_path[0] + "/hrp2_14_description/urdf/hrp2_14_reduced.urdf";
freeFlyer                   = True;


''' CONTROLLER CONFIGURATION '''
ENABLE_CAPTURE_POINT_LIMITS     = False;
ENABLE_TORQUE_LIMITS            = True;
ENABLE_FORCE_LIMITS             = False;
ENABLE_JOINT_LIMITS             = True;
IMPOSE_POSITION_BOUNDS          = True;
IMPOSE_VELOCITY_BOUNDS          = True;
IMPOSE_VIABILITY_BOUNDS         = True;
IMPOSE_ACCELERATION_BOUNDS      = True;
JOINT_POS_PREVIEW               = 1.5; # preview window to convert joint pos limits into joint acc limits
JOINT_VEL_PREVIEW               = 1;   # preview window to convert joint vel limits into joint acc limits
MAX_JOINT_ACC                   = 30.0;
MAX_MIN_JOINT_ACC               = 10.0;
USE_JOINT_VELOCITY_ESTIMATOR    = False;
ACCOUNT_FOR_ROTOR_INERTIAS      = True;

# CONTROLLER GAINS
kp_posture  = 30; #1.0;   # proportional gain of postural task
kd_posture  = 2*sqrt(kp_posture);
kp_constr   = 100.0;   # constraint proportional feedback gain
kd_constr   = 2*sqrt(kp_constr);   # constraint derivative feedback gain
kp_com      = 0;
kd_com      = 1.0/dt;
kp_ee       = 100.0;
kd_ee       = 2*sqrt(kp_ee);
constraint_mask = np.array([True, True, True, True, True, True]).T;
ee_mask         = np.array([True, True, True, True, True, True]).T;

# CONTROLLER WEIGTHS
w_com           = 1;
w_posture       = 1e-3;  #1e-5;  # weight of postural task

# QP SOLVER PARAMETERS
maxIter = 300;      # max number of iterations
maxTime = 0.8;      # max computation time for the solver in seconds
verb=0;             # verbosity level (0, 1, or 2)

# CONTACT PARAMETERS
mu  = np.array([0.3, 0.1]);          # force and moment friction coefficient
fMin = 1e-3;					     # minimum normal force

''' SIMULATOR PARAMETERS '''
FORCE_TORQUE_LIMITS            = ENABLE_TORQUE_LIMITS;
FORCE_JOINT_LIMITS             = ENABLE_JOINT_LIMITS and IMPOSE_POSITION_BOUNDS;
USE_LCP_SOLVER                 = False

''' STOPPING CRITERIA THRESHOLDS '''
ZERO_JOINT_VEL_THR          = 1e-2;
ZERO_COM_VEL_THR            = 1e-3;
ZERO_ANG_MOM_THR            = 1e-2;
MAX_COM_VELOCITY            = 5;
MAX_CONSTRAINT_ERROR        = 0.1;

''' INITIAL STATE PARAMETERS '''
INITIAL_CONFIG_ID                   = 1;
ENSURE_INITIAL_CAPTURE_POINT_OUT    = False;
ENSURE_INITIAL_CAPTURE_POINT_IN     = False;
ENSURE_STABILITY                    = False;
ENSURE_UNSTABILITY                  = True;
ZERO_INITIAL_VERTICAL_COM_VEL       = False;
ZERO_INITIAL_ANGULAR_MOMENTUM       = True;
MAX_INITIAL_COM_VEL                 = 1.0;
INITIAL_CONFIG_FILENAME             = '../data/1lLeg_0rLeg_configs';
# 1lLeg_0rLeg_configs  1lLeg_0rLeg_4Larm_configs  1lLeg_0rLeg_3Rarm_configs  1lLeg_0rLeg_3Rarm_4Larm_configs



''' VIEWER PARAMETERS '''
ENABLE_VIEWER               = False;
PLAY_MOTION_WHILE_COMPUTING = False;
PLAY_MOTION_AT_THE_END      = False;
DT_VIEWER                   = 10*dt;   # timestep used to display motion with viewer
SHOW_VIEWER_FLOOR           = 'coplanar' in INITIAL_CONFIG_FILENAME;

''' FIGURE PARAMETERS '''
SAVE_FIGURES     = False;
SHOW_FIGURES     = False;
SHOW_LEGENDS     = True;
LINE_ALPHA       = 0.7;
BUTTON_PRESS_TIMEOUT        = 100.0;
