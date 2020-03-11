
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import numpy as np
#from hpp.corbaserver.rbprm.tools.cwc_trajectory import *
import time
from hpp import Error as hpperr
from numpy import array, matrix
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
import pickle as pickle
from pathlib2 import Path
import os
import generate_muscod_problem as mp
from . import muscodSSH as ssh
from config import *
from .check_qp import check_traj_valid,check_muscod_traj
from . import disp_bezier
from .constraint_to_dae import *
from hpp.corbaserver.rbprm.tools.display_tools import *


from .gen_hrp2_statically_balanced_positions_2d_state import *
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)
from generate_contact_sequence import *

class Struct(object):
    def __init__(self):
        self.active = False
        self.placement = SE3()


def exportPickle(fullBody,s0,s1):
    configs = [s0.q() , s1.q()]
    cs = generateContactSequence(fullBody,configs,s0.sId, s1.sId)
    #cs = generateContactSequence(fullBody,configs,s0.sId, s1.sId)
    cs_dic = {}
    i = 0
    for phase in cs.contact_phases :
        lf_patch = Struct()
        rf_patch = Struct()
        lh_patch = Struct()
        rh_patch = Struct()
        if phase.LF_patch.active :
            lf_patch.active = True
            lf_patch.placement = phase.LF_patch.placement.copy()
        if phase.RF_patch.active :
            rf_patch.active = True
            rf_patch.placement = phase.RF_patch.placement.copy()
        if phase.LH_patch.active :
            lh_patch.active = True
            lh_patch.placement = phase.LH_patch.placement.copy()
        if phase.RH_patch.active :
            rh_patch.active = True
            rh_patch.placement = phase.RH_patch.placement.copy()
        phase_dic = {"LF":lf_patch, "RF":rf_patch, "RH":rh_patch,"LH":lh_patch}
        cs_dic.update({i:phase_dic})
        i+=1

    pickle.dump(cs_dic, open('/local/fernbac/bench_iros18/export_pickle/cs0', 'wb'))



def callMuscodBetweenTwoState(fullBody,s0,s1,c_qp = [], t_qp = []):
    configs = [s0.q() , s1.q()]
    cs = generateContactSequence(fullBody,configs,s0.sId, s1.sId,curves_initGuess=c_qp,timings_initGuess=t_qp)
    #cs = generateContactSequence(fullBody,configs,s0.sId, s1.sId)
    filename_xml = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
    cs.saveAsXML(filename_xml, "ContactSequence")
    mp.generate_muscod_problem(filename_xml,True)
    success,ssh_ok = ssh.call_muscod()
    time.sleep(5.) # wait for sync of the ~/home (worst case, usually 0.1 is enough ... )
    converged = success and Path(CONTACT_SEQUENCE_WHOLEBODY_FILE).is_file()
    if converged :
        cs.loadFromXML(CONTACT_SEQUENCE_WHOLEBODY_FILE,CONTACT_SEQUENCE_XML_TAG)
        kin_valid,stab_valid,stab_valid_discretized = check_muscod_traj(fullBody,cs,s0,s1)
        os.remove(CONTACT_SEQUENCE_WHOLEBODY_FILE)
    else :
        kin_valid = False
        stab_valid = False
        stab_valid_discretized = False
    return ssh_ok,converged,kin_valid,stab_valid,stab_valid_discretized



if Path(CONTACT_SEQUENCE_WHOLEBODY_FILE).is_file():
    os.remove(CONTACT_SEQUENCE_WHOLEBODY_FILE)


states = genStateWithOneStep(fullBody,limbs[0], 500,False)
name = "/local/fernbac/bench_iros18/muscod_qp/one_step"
file_exist = True
i = 0
while file_exist:
    filename = name +"_"+str(i)+".log"
    file_exist = Path(filename).is_file()
    i+= 1

f = open(filename,"w")
print("write results in file : "+filename)
i=0
cs_out = ContactSequenceHumanoid(0)

"""
_quasiStatic
_qp3
_qp7
_qp15
_qpC
_muscod
_muscodWarmStart
"""

false_positive = 0.
valid_false_positive = 0.

total_success_quasiStatic = 0.
total_success_qp = 0.
total_success_qp3 = 0.
total_success_qp7 = 0.
total_success_qp15 = 0.
total_success_qpC = 0.
total_success_muscod = 0.
total_success_muscodWarmStart = 0.
total_success_muscod_either = 0.
# valid = stab_valid
total_valid_success_qp = 0.
total_valid_success_qp3 = 0.
total_valid_success_qp7 = 0.
total_valid_success_qp15 = 0.
total_valid_success_qpC = 0.
total_valid_success_muscod = 0.
total_valid_success_muscodWarmStart = 0.
kin_invalidated_qp3 = 0.
kin_invalidated_qp7 = 0.
kin_invalidated_qp15 = 0.
kin_invalidated_qpC = 0.
kin_invalidated_muscod = 0.
kin_invalidated_muscodWarmStart = 0.
stab_invalidated_qp3 = 0.
stab_invalidated_qp7 = 0.
stab_invalidated_qp15 = 0.
stab_invalidated_qpC = 0.
stab_invalidated_muscod = 0.
stab_invalidated_muscodWarmStart = 0.
stab_invalidated_discretized_muscod = 0.
stab_invalidated_discretized_muscodWarmStart = 0.
total_feasible = 0.
total_valid_feasible = 0.
total_discret_notContinuous = 0.
total_discret_notContinuous_valid = 0.
total_continuous_notDiscret = 0.
muscod_only_warmStart = 0.
muscod_wrong_warmStart = 0.
GT_success_qp3 = 0.
GT_success_qp7 = 0.
GT_success_qp15 = 0.
GT_success_qpC = 0.
GT_success_muscod = 0.
GT_success_muscodWarmStart = 0.
GT_valid_success_qp3 = 0.
GT_valid_success_qp7 = 0.
GT_valid_success_qp15 = 0.
GT_valid_success_qpC = 0.
GT_valid_success_muscod = 0.
GT_valid_success_muscodWarmStart = 0.
GT_valid_success_valid_qp3 = 0.
GT_valid_success_valid_qp7 = 0.
GT_valid_success_valid_qp15 = 0.
GT_valid_success_valid_qpC = 0.
GT_valid_success_valid_muscod = 0.
GT_valid_success_valid_muscodWarmStart = 0.
GT_valid_stab_invalid_qp3 = 0.
GT_valid_stab_invalid_qp7 = 0.
GT_valid_stab_invalid_qp15 = 0.
GT_valid_stab_invalid_qpC = 0.
GT_valid_stab_invalid_muscod = 0.
GT_valid_stab_invalid_muscodWarmStart = 0.

for [s0,s1] in states:
    print("#################################")
    print("#######  iter : "+str(i)+" / "+str(len(states))+"  ########")
    print("#################################")

    f.write("Try for pair "+str(i)+" ------- \n")
    f.write("q0 = "+str(s0.q())+"\n")
    f.write("q1 = "+str(s1.q())+"\n")
    r(s0.q())
    success_quasiStatic = False
    success_qp3= False
    success_qp7= False
    success_qp15= False
    success_qpC= False
    success_muscod= False
    success_muscodWarmStart  = False
    stab_valid_qp3= False
    stab_valid_qp7= False
    stab_valid_qp15= False
    stab_valid_qpC= False
    stab_valid_muscod = False
    stab_valid_muscodWarmStart = False
    stab_valid_discretized_muscod = False
    stab_valid_discretized_muscodWarmStart = False

    kin_valid_qp3= False
    kin_valid_qp7= False
    kin_valid_qp15= False
    kin_valid_qpC= False
    kin_valid_muscod = False
    kin_valid_muscodWarmStart = False
    success_qp_discret= False
    success_qp= False
    success_qp_valid= False
    success_qp_discret_valid= False

    success_quasiStatic = fullBody.isReachableFromState(s0.sId,s1.sId)

    best_pid = []
    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True,numPointsPerPhases=3)
    success_qp3 = (len(pid) > 0)
    if success_qp3:
        print("qp3 converged.")
        best_pid = pid
        kin_valid_qp3,stab_valid_qp3 = check_traj_valid(ps,fullBody,s0,s1,pid)


    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True,numPointsPerPhases=7)
    success_qp7 = (len(pid) > 0)
    if success_qp7:
        print("qp7 converged.")
        best_pid = pid
        kin_valid_qp7,stab_valid_qp7 = check_traj_valid(ps,fullBody,s0,s1,pid)

    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True,numPointsPerPhases=15)
    success_qp15 = (len(pid) > 0)
    if success_qp15:
        print("qp15 converged.")
        best_pid = pid
        kin_valid_qp15,stab_valid_qp15 = check_traj_valid(ps,fullBody,s0,s1,pid)

    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True,numPointsPerPhases=0)
    success_qpC = (len(pid) > 0)
    if success_qpC:
        print("qp continuous converged.")
        best_pid = pid
        kin_valid_qpC,stab_valid_qpC = check_traj_valid(ps,fullBody,s0,s1,pid)

    success_qp = success_qpC or success_qp3 or success_qp7 or success_qp15
    success_qp_valid = (success_qpC and stab_valid_qpC) or (success_qp3 and stab_valid_qp3) or (success_qp7 and stab_valid_qp7) or (success_qp15  and stab_valid_qp15)
    success_qp_discret = success_qp3 or success_qp7 or success_qp15
    success_qp_discret_valid = (success_qp3 and stab_valid_qp3) or (success_qp7 and stab_valid_qp7) or (success_qp15  and stab_valid_qp15)


    ssh_ok,success_muscod,kin_valid_muscod,stab_valid_muscod,stab_valid_discretized_muscod = callMuscodBetweenTwoState(fullBody,s0,s1)


    if success_qp :
        #compute init guess curve for muscod :
        c_qp = fullBody.getPathAsBezier(int(best_pid[0]))
        t_qp = [ps.pathLength(int(best_pid[1])), ps.pathLength(int(best_pid[2])), ps.pathLength(int(best_pid[3]))]
        ssh_ok,success_muscodWarmStart,kin_valid_muscodWarmStart,stab_valid_muscodWarmStart,stab_valid_discretized_muscodWarmStart = callMuscodBetweenTwoState(fullBody,s0,s1,[c_qp],[t_qp])

    if not ssh_ok :
        f.write("Error in ssh connection to muscod server ... \n")


    ### compile the results and update the sums :

    if success_quasiStatic :
        total_success_quasiStatic += 1.

    if success_qp:
        total_success_qp += 1.
    if success_qp_valid:
        total_valid_success_qp += 1.

    if success_muscod or success_muscodWarmStart:
        total_success_muscod_either += 1.

    if success_qp3:
        total_success_qp3 += 1.
        f.write( "qp 3 converged\n")
        if stab_valid_qp3 :
            total_valid_success_qp3 += 1.
            f.write( "qp 3 stab valid\n")
        else :
            stab_invalidated_qp3 += 1.
            f.write( "qp 3 stab invalid\n")
        if not kin_valid_qp3 :
            kin_invalidated_qp3 += 1.
            f.write( "qp 3 kin invalid \n")

    if success_qp7:
        total_success_qp7 += 1.
        f.write( "qp 7 converged\n")
        if stab_valid_qp7 :
            total_valid_success_qp7 += 1.
            f.write( "qp 7 stab valid\n")
        else :
            stab_invalidated_qp7 += 1.
            f.write( "qp 7 stab invalid\n")
        if not kin_valid_qp7 :
            kin_invalidated_qp7 += 1.
            f.write( "qp 7 kin invalid \n")

    if success_qp15:
        total_success_qp15 += 1.
        f.write( "qp 15 converged\n")
        if stab_valid_qp15 :
            total_valid_success_qp15 += 1.
            f.write( "qp 15 stab valid\n")
        else :
            stab_invalidated_qp15 += 1.
            f.write( "qp 15 stab invalid\n")
        if not kin_valid_qp15 :
            kin_invalidated_qp15 += 1.
            f.write( "qp 15 kin invalid \n")

    if success_qpC:
        total_success_qpC += 1.
        f.write( "qp C converged\n")
        if stab_valid_qpC :
            total_valid_success_qpC += 1.
            f.write( "qp C stab valid\n")
        else :
            stab_invalidated_qpC += 1.
            f.write( "qp C stab invalid\n")
        if not kin_valid_qpC :
            kin_invalidated_qpC += 1.
            f.write( "qp C kin invalid \n")

    if success_muscod:
        total_success_muscod += 1.
        f.write( "muscod converged\n")
        if stab_valid_muscod :
            total_valid_success_muscod += 1.
            f.write( "muscod stab valid\n")
        else :
            stab_invalidated_muscod += 1.
            f.write( "muscod stab invalid\n")
        if not stab_valid_discretized_muscod:
            stab_invalidated_discretized_muscod +=1.
        if not kin_valid_muscod :
            kin_invalidated_muscod += 1.
            f.write( "muscod kin invalid \n")

    if success_muscodWarmStart:
        total_success_muscodWarmStart += 1.
        f.write( "muscodWarmStart converged\n")
        if stab_valid_muscodWarmStart :
            total_valid_success_muscodWarmStart += 1.
            f.write( "muscodWarmStart stab valid\n")
        else :
            stab_invalidated_muscodWarmStart += 1.
            f.write( "muscodWarmStart stab invalid\n")
        if not stab_valid_discretized_muscodWarmStart:
            stab_invalidated_discretized_muscodWarmStart +=1.
        if not kin_valid_muscodWarmStart :
            kin_invalidated_muscodWarmStart += 1.
            f.write( "muscodWarmStart kin invalid \n")

    if success_muscodWarmStart and not success_muscod :
        muscod_only_warmStart += 1.
    if success_muscod and success_qp and not success_muscodWarmStart :
        muscod_wrong_warmStart += 1.

    if success_qpC and not success_qp_discret:
        total_continuous_notDiscret += 1.
    if success_qp_discret and not success_qpC:
        total_discret_notContinuous += 1.
    if success_qp_discret_valid and not success_qpC :
        total_discret_notContinuous_valid += 1.

    if success_qp and not (success_muscod or success_muscodWarmStart):
        false_positive += 1.
    if success_qp_valid and not (success_muscod or success_muscodWarmStart):
        valid_false_positive += 1.

    if not success_muscodWarmStart and not success_qp :
        # success muscod warm start : when no warm start was available, take same result as muscod
        success_muscodWarmStart = success_muscod
        stab_valid_muscodWarmStart = stab_valid_muscod

    if success_muscodWarmStart :
        # assume MUSCOD is Ground truth :
        total_feasible += 1.
        GT_success_muscodWarmStart += 1.
        if success_muscod:
            GT_success_muscod += 1.
        if success_qp3:
            GT_success_qp3 += 1.
        if success_qp7:
            GT_success_qp7 += 1.
        if success_qp15:
            GT_success_qp15 += 1.
        if success_qpC:
            GT_success_qpC += 1.

        if stab_valid_muscodWarmStart :
            total_valid_feasible += 1.
            GT_valid_success_valid_muscodWarmStart += 1.
            GT_valid_success_muscodWarmStart +=1.
            if success_muscod :
                GT_valid_success_muscod += 1.
                if stab_valid_muscod:
                    GT_valid_success_valid_muscod += 1.
                else :
                    GT_valid_stab_invalid_muscod += 1.
            if success_qp3 :
                GT_valid_success_qp3 += 1.
                if stab_valid_qp3:
                    GT_valid_success_valid_qp3 += 1.
                else :
                    GT_valid_stab_invalid_qp3 += 1.
            if success_qp7 :
                GT_valid_success_qp7 += 1.
                if stab_valid_qp7:
                    GT_valid_success_valid_qp7 += 1.
                else :
                    GT_valid_stab_invalid_qp7 += 1.
            if success_qp15 :
                GT_valid_success_qp15 += 1.
                if stab_valid_qp15:
                    GT_valid_success_valid_qp15 += 1.
                else :
                    GT_valid_stab_invalid_qp15 += 1.
            if success_qpC :
                GT_valid_success_qpC += 1.
                if stab_valid_qpC:
                    GT_valid_success_valid_qpC += 1.
                else :
                    GT_valid_stab_invalid_qpC += 1.


    f.flush()
    i += 1

f.close()


num_states = float(len(states))

f = open(filename+"C","w")
f.write( "for : "+str(num_states)+" states.\n")
f.write( "Assuming muscod is ground truth, we get : \n")
if total_feasible > 0 :
    f.write(str(float(GT_success_muscodWarmStart/total_feasible)*100.)+" % of success with muscod with warm start\n")
    f.write(str(float(GT_success_muscod/total_feasible)*100.)+" % of success with muscod \n")
    f.write(str(float(GT_success_qp3/total_feasible)*100.)+" % of success with qp discret and 3 points per phases \n")
    f.write(str(float(GT_success_qp7/total_feasible)*100.)+" % of success with qp discret and 7 points per phases\n")
    f.write(str(float(GT_success_qp15/total_feasible)*100.)+" % of success with qp discret and 15 points per phases\n")
    f.write(str(float(GT_success_qpC/total_feasible)*100.)+" % of success with qp continuous\n")
else :
    f.write( "No feasible pair found ... \n")
f.write( "Considering only the results where muscod was valid, we get : \n")
if total_valid_feasible > 0:
    f.write(str(float(GT_valid_success_muscodWarmStart/total_valid_feasible)*100.)+" % of success with muscod with warm start\n")
    f.write(str(float(GT_valid_success_muscod/total_valid_feasible)*100.)+" % of success with muscod \n")
    f.write(str(float(GT_valid_success_qp3/total_valid_feasible)*100.)+" % of success with qp discret and 3 points per phases \n")
    f.write(str(float(GT_valid_success_qp7/total_valid_feasible)*100.)+" % of success with qp discret and 7 points per phases\n")
    f.write(str(float(GT_valid_success_qp15/total_valid_feasible)*100.)+" % of success with qp discret and 15 points per phases\n")
    f.write(str(float(GT_valid_success_qpC/total_valid_feasible)*100.)+" % of success with qp continuous\n")
    f.write( "Considering only the valid results, we get : \n")
    f.write(str(float(GT_valid_success_valid_muscodWarmStart/total_valid_feasible)*100.)+" % of success with muscod with warm start\n")
    f.write(str(float(GT_valid_success_valid_muscod/total_valid_feasible)*100.)+" % of success with muscod \n")
    f.write(str(float(GT_valid_success_valid_qp3/total_valid_feasible)*100.)+" % of success with qp discret and 3 points per phases \n")
    f.write(str(float(GT_valid_success_valid_qp7/total_valid_feasible)*100.)+" % of success with qp discret and 7 points per phases\n")
    f.write(str(float(GT_valid_success_valid_qp15/total_valid_feasible)*100.)+" % of success with qp discret and 15 points per phases\n")
    f.write(str(float(GT_valid_success_valid_qpC/total_valid_feasible)*100.)+" % of success with qp continuous\n")
else:
    f.write( "No feasible valid pair found ... \n")

f.write( "Analysis of dynamic validity of the trajectories  : \n")
if total_success_muscodWarmStart > 0:
    f.write(str(float(stab_invalidated_muscodWarmStart/total_success_muscodWarmStart)*100.)+" % of trajs were dynamically invalid for muscod with warm start with the same discretization as in the solver\n")
if total_success_muscod > 0:
    f.write(str(float(stab_invalidated_muscod/total_success_muscod)*100.)+" % of trajs were dynamically invalid for muscod with the same discretization as in the solver\n")
if total_success_muscodWarmStart > 0:
    f.write(str(float(stab_invalidated_discretized_muscodWarmStart/total_success_muscodWarmStart)*100.)+" % of trajs were dynamically invalid for muscod with warm start\n")
if total_success_muscod > 0:
    f.write(str(float(stab_invalidated_discretized_muscod/total_success_muscod)*100.)+" % of trajs were dynamically invalid for muscod \n")
if total_success_qp3 > 0:
    f.write(str(float(stab_invalidated_qp3/total_success_qp3)*100.)+" % of trajs were dynamically invalid for qp3\n")
if total_success_qp7 > 0:
    f.write(str(float(stab_invalidated_qp7/total_success_qp7)*100)+" % of trajs were dynamically invalid for qp7\n")
if total_success_qp15 > 0:
    f.write(str(float(stab_invalidated_qp15/total_success_qp15)*100.)+" % of trajs were dynamically invalid for qp15\n")
if total_success_qpC > 0:
    f.write(str(float(stab_invalidated_qpC/total_success_qpC)*100.)+" % of trajs were dynamically invalid for qp continuous\n")
f.write( "Analysis of dynamic validity of the trajectories only when muscod was valid  : \n")
if GT_valid_success_muscodWarmStart > 0:
    f.write(str(float(GT_valid_stab_invalid_muscodWarmStart/GT_valid_success_muscodWarmStart)*100.)+" % of trajs were dynamically invalid for muscod with warm start\n")
if GT_valid_success_muscod > 0:
    f.write(str(float(GT_valid_stab_invalid_muscod/GT_valid_success_muscod)*100.)+" % of trajs were dynamically invalid for muscod \n")
if GT_valid_success_qp3 > 0:
    f.write(str(float(GT_valid_stab_invalid_qp3/GT_valid_success_qp3)*100.)+" % of trajs were dynamically invalid for qp3\n")
if GT_valid_success_qp7 > 0:
    f.write(str(float(GT_valid_stab_invalid_qp7/GT_valid_success_qp7)*100.)+" % of trajs were dynamically invalid for qp7\n")
if GT_valid_success_qp15 > 0:
    f.write(str(float(GT_valid_stab_invalid_qp15/GT_valid_success_qp15)*100.)+" % of trajs were dynamically invalid for qp15\n")
if GT_valid_success_qpC > 0:
    f.write(str(float(GT_valid_stab_invalid_qpC/GT_valid_success_qpC)*100.)+" % of trajs were dynamically invalid for qp continuous\n")
f.write( "Analysis of validity of the kinematics constraints  : \n")
if total_success_muscodWarmStart > 0:
    f.write(str(float(kin_invalidated_muscodWarmStart/total_success_muscodWarmStart)*100.)+" % of trajs were kinematically invalid for muscod with warm start\n")
if total_success_muscod > 0:
    f.write(str(float(kin_invalidated_muscod/total_success_muscod)*100.)+" % of trajs were kinematically invalid for muscod \n")
if total_success_qp3 > 0:
    f.write(str(float(kin_invalidated_qp3/total_success_qp3)*100.)+" % of trajs were kinematically invalid for qp3\n")
if total_success_qp7 > 0:
    f.write(str(float(kin_invalidated_qp7/total_success_qp7)*100.)+" % of trajs were kinematically invalid for qp7\n")
if total_success_qp15 > 0:
    f.write(str(float(kin_invalidated_qp15/total_success_qp15)*100.)+" % of trajs were kinematically invalid for qp15\n")
if total_success_qpC > 0:
    f.write(str(float(kin_invalidated_qpC/total_success_qpC)*100.)+" % of trajs were kinematically invalid for qp continuous\n")
f.write( "Misc values to check   : \n")
f.write(str(float(false_positive/num_states)*100.)+" % of false positive\n")
f.write(str(float(valid_false_positive/num_states)*100.)+" % of false positive after trajectory validation\n")
if total_success_qp > 0:
    f.write(str(float(total_continuous_notDiscret/total_success_qp)*100.)+" % of times one qp converged, the continuous QP did not converge\n")
    f.write(str(float(total_discret_notContinuous_valid/total_valid_success_qp)*100.)+" % of times one qp converged with a valid solution, the continuous QP did not converge\n")
if total_success_muscod_either > 0:
    f.write(str(float(muscod_only_warmStart/total_success_muscod_either)*100.)+" % of times where MUSCOD converged, it only converged with the warm start\n")
    f.write(str(float(muscod_wrong_warmStart/total_success_muscod_either)*100.)+" % of times where MUSCOD converged, the init guess forbid muscod to converge.\n")


f.write( "################# total without percentage    : \n")

f.write( "Assuming muscod is ground truth, we get : \n")
f.write(str(float(total_feasible))+" total feasible\n")
f.write(str(float(GT_success_muscodWarmStart))+" % of success with muscod with warm start\n")
f.write(str(float(GT_success_muscod))+"  success with muscod \n")
f.write(str(float(GT_success_qp3))+"  success with qp discret and 3 points per phases \n")
f.write(str(float(GT_success_qp7))+"  success with qp discret and 7 points per phases\n")
f.write(str(float(GT_success_qp15))+" success with qp discret and 15 points per phases\n")
f.write(str(float(GT_success_qpC))+" success with qp continuous\n")
f.write( "Considering only the results where muscod was valid, we get : \n")
f.write(str(float(total_valid_feasible))+"total valid feasible\n")
f.write(str(float(GT_valid_success_muscodWarmStart))+" % of success with muscod with warm start\n")
f.write(str(float(GT_valid_success_muscod))+"  success with muscod \n")
f.write(str(float(GT_valid_success_qp3))+"  success with qp discret and 3 points per phases \n")
f.write(str(float(GT_valid_success_qp7))+"  success with qp discret and 7 points per phases\n")
f.write(str(float(GT_valid_success_qp15))+"  success with qp discret and 15 points per phases\n")
f.write(str(float(GT_valid_success_qpC))+"  success with qp continuous\n")
f.write( "Considering only the valid results, we get : \n")
f.write(str(float(GT_valid_success_valid_muscodWarmStart))+"  success with muscod with warm start\n")
f.write(str(float(GT_valid_success_valid_muscod))+"  success with muscod \n")
f.write(str(float(GT_valid_success_valid_qp3))+"  success with qp discret and 3 points per phases \n")
f.write(str(float(GT_valid_success_valid_qp7))+"  success with qp discret and 7 points per phases\n")
f.write(str(float(GT_valid_success_valid_qp15))+"  success with qp discret and 15 points per phases\n")
f.write(str(float(GT_valid_success_valid_qpC))+" success with qp continuous\n")

f.close()



"""

s0 = State(fullBody,q=q0,limbsIncontact=[rLegId,lLegId])
s1 = State(fullBody,q=q1,limbsIncontact=[rLegId,lLegId])


pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True
disp_bezier.showPath(r,pp,pid)
sm, success = StateHelper.removeContact(s0,'hrp2_rleg_rom')
fullBody.isReachableFromState(sm.sId,sm.sId)
displayOneStepConstraints(r)

"""
