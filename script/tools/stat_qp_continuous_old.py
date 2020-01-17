
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import numpy as np
#from hpp.corbaserver.rbprm.tools.cwc_trajectory import *

from hpp import Error as hpperr
from numpy import array, matrix
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
import pickle as pickle
from pathlib2 import Path

import generate_muscod_problem as mp
import muscodSSH as ssh
from config import *
from check_qp import check_traj_valid 

from gen_hrp2_statically_balanced_positions_2d_state import *
from generate_contact_sequence import *


def callMuscodBetweenTwoState(fullBody,s0,s1,c_qp = [], t_qp = []):
    configs = [s0.q() , s1.q()]
    cs = generateContactSequence(fullBody,configs,s0.sId, s1.sId,curves_initGuess=c_qp,timings_initGuess=t_qp)
    #cs = generateContactSequence(fullBody,configs,s0.sId, s1.sId,curves_initGuess=c_qp,timings_initGuess=t_qp)
    filename_xml = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
    cs.saveAsXML(filename_xml, "ContactSequence")   
    mp.generate_muscod_problem(filename_xml,True)
    return ssh.call_muscod()
    




states = genStateWithOneStep(fullBody,limbs[0], 20,False)

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
same_positif = 0
same_negatif = 0
false_positif = 0
true_false_positif = 0
true_false_negative = 0
true_same_negative = 0
false_positive_quasiStatic = 0
false_negatif = 0
muscod_converged = 0
muscod_converged_init_guess = 0
need_init_guess =0
wrong_init_guess = 0
total_success_qp = 0
total_success_qpC = 0
total_success_qp3 = 0
total_success_qp7 = 0
total_success_qp15 = 0
total_valid_qpC = 0
total_valid_qp3 = 0
total_valid_qp7 = 0
total_valid_qp15 = 0
total_discret_notContinuous = 0
total_notDiscret_Continuous = 0
total_discret_notContinuous_valid = 0
total_notDiscret_Continuous_valid = 0
total_invalid_continous = 0
total_invalid_discret = 0
total_success_qp_discret = 0
for [s0,s1] in states:
    
    f.write("Try for pair "+str(i)+" ------- \n")
    f.write("q0 = "+str(s0.q())+"\n")
    f.write("q1 = "+str(s1.q())+"\n")
    
    
    success_quasiStatic = False
    success_qp = False 
    success_qp_discret = False
    success_qp3 = False
    success_qp7 = False
    success_qp15 = False
    success_qC = False
    qp3_valid = False
    qp7_valid = False
    qp15_valid = False
    qC_valid = False
    success_muscod = False
    success_qp_valid = False
    success_qp_discret_valid = False
    
    success_quasiStatic = fullBody.isReachableFromState(s0.sId,s1.sId)
    best_pid = []
    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True,numPointsPerPhases=3)
    success_qp3 = (len(pid) > 0)
    if success_qp3:
        best_pid = pid        
        total_success_qp3 += 1
        f.write( "qp 3 converged\n")        
        qp3_valid = check_traj_valid(ps,fullBody,s0,s1,pid)
        if qp3_valid : 
            total_valid_qp3 +=1
            f.write( "qp 3 was valid\n")        
        else:
            f.write("qp 3 was invalid\n") 
    else :
        f.write( "qp 3 did not converge\n")        
        
    
    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True,numPointsPerPhases=7)
    success_qp7 = (len(pid) > 0)
    if success_qp7:       
        best_pid = pid        
        total_success_qp7 += 1  
        f.write( "qp 7 converged\n")                
        qp7_valid = check_traj_valid(ps,fullBody,s0,s1,pid)
        if qp7_valid : 
            total_valid_qp7 +=1 
            f.write( "qp 7 was valid\n")
        else:
            f.write("qp 7 was invalid\n")
    else :
        f.write( "qp 7 did not converge\n")            
    
    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True,numPointsPerPhases=15)
    success_qp15 = (len(pid) > 0)
    if success_qp15:
        best_pid = pid                
        total_success_qp15 += 1
        f.write( "qp 15 converged\n")        
        qp15_valid = check_traj_valid(ps,fullBody,s0,s1,pid)
        if qp15_valid : 
            total_valid_qp15 +=1 
            f.write( "qp 15 was valid\n")        
        else:
            f.write("qp 15 was invalid\n")            
    else :
        f.write( "qp 15 did not converge\n")
        
    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True,numPointsPerPhases=0)
    success_qC = (len(pid) > 0)
    if success_qC:
        best_pid = pid                
        total_success_qpC += 1
        f.write( "qp continuous converged\n")        
        qC_valid = check_traj_valid(ps,fullBody,s0,s1,pid)
        if qC_valid : 
            total_valid_qpC +=1  
            f.write( "qp continuous was valid\n")        
        else:
            f.write("qp continuous was invalid\n")            
    else :
        f.write( "qp continuous did not converge\n")
        
    success_qp = success_qC or success_qp3 or success_qp7 or success_qp15   
    success_qp_valid = (success_qC and qC_valid) or (success_qp3 and qp3_valid) or (success_qp7 and qp7_valid) or (success_qp15  and qp15_valid) 
    success_qp_discret = success_qp3 or success_qp7 or success_qp15 
    success_qp_discret_valid = (success_qp3 and qp3_valid) or (success_qp7 and qp7_valid) or (success_qp15  and qp15_valid) 
    
    if success_qp_discret:
        total_success_qp_discret +=1
    if success_qp_discret and not success_qp_discret_valid : 
        total_invalid_discret +=1
    if success_qC and not qC_valid : 
        total_invalid_continous +=1
        
    if success_qp_discret and not success_qC : 
        total_discret_notContinuous +=1
        f.write("qp continuous did not converge but discret converged\n")
    if success_qp_discret_valid and not success_qC : 
        total_discret_notContinuous_valid +=1
        f.write("qp continuous did not converge but discret converged and was valid \n")
        
    if success_qC and not success_qp_discret :
        total_notDiscret_Continuous +=1
        f.write("qp discret did not converge but continuous converged\n")
    if success_qC and qC_valid and not success_qp_discret_valid :
        total_notDiscret_Continuous_valid +=1
        f.write("qp discret did not converge but continuous converged\n")
        
    
        

    if success_qp :
        #compute init guess curve for muscod :
        total_success_qp +=1
        c_qp = fullBody.getPathAsBezier(int(best_pid[0]))
        t_qp = [ps.pathLength(int(best_pid[1])), ps.pathLength(int(best_pid[2])), ps.pathLength(int(best_pid[3]))]
        success_muscod_initGuess, ssh_ok = callMuscodBetweenTwoState(fullBody,s0,s1,[c_qp],[t_qp])
        success_muscod, ssh_ok = callMuscodBetweenTwoState(fullBody,s0,s1)
        if success_muscod_initGuess and not success_muscod:
            f.write("muscod converged only with initial guess\n")
            need_init_guess +=1
        if not success_muscod_initGuess and success_muscod:
            f.write("muscod did not converge with initial guess\n")
            wrong_init_guess += 1
        success_muscod = success_muscod or success_muscod_initGuess
        if success_muscod :
            muscod_converged_init_guess +=1
    else :
        success_muscod, ssh_ok = callMuscodBetweenTwoState(fullBody,s0,s1) 

            
    if not ssh_ok :
        f.write("Error in ssh connection to muscod server ... \n")
    else :
        f.write("quasiStatic = "+str(success_quasiStatic)+"\n")        
        f.write("qp = "+str(success_qp)+"\n")
        f.write("muscod = "+str(success_muscod)+"\n")
        
        # print timings : 
        if success_qp :
            f.write("time_qp "+str(t_qp[0])+" "+str(t_qp[1])+" "+str(t_qp[2])+"\n")
        else :
            f.write("time_qp\n")
        if success_muscod:
            muscod_converged += 1            
            cs_out.loadFromXML(CONTACT_SEQUENCE_WHOLEBODY_FILE,CONTACT_SEQUENCE_XML_TAG)
            t_muscod = []
            for k in range(3):
                t_muscod +=[cs_out.contact_phases[k].time_trajectory[-1] - cs_out.contact_phases[k].time_trajectory[0]]
            f.write("time_muscod "+str(t_muscod[0])+" "+str(t_muscod[1])+" "+str(t_muscod[2])+"\n")
        else :
            f.write("time_muscod\n")
            
        if not success_muscod and not success_qp:
            f.write("same results, unreachable \n")
            same_negatif += 1
        if not success_muscod and not success_qp_valid:
            f.write("same results, unreachable or qp was invalid \n")
            true_same_negative += 1
                
        if success_muscod and success_qp:
            f.write("same results, reachable \n")
            same_positif += 1
        if success_qp and not success_muscod:
            false_positif += 1
            if success_quasiStatic:
                f.write("false positive but with quasi-static solution \n")
                false_positive_quasiStatic += 1
            if not success_qp_valid : 
                f.write("false positive but check path was not valid \n")
            else :
                f.write("false positive \n")
                true_false_positif +=1
                    
        if not success_qp and success_muscod:
            f.write("false negative \n")
            false_negatif +=1
        if not success_qp_valid and success_muscod:
            f.write("false negative with only valid qp\n")
            true_false_negative +=1            
    f.flush()
    i +=1

f.close()


num_states = float(len(states))
print("for : "+str(num_states)+" states.")
print("same result, unreachable : "+str(same_negatif) + "   ; "+str((float(same_negatif)/num_states)*100.)+" % ")
print("same result, reachable : "+str(same_positif) + "   ; "+str((float(same_positif)/num_states)*100.)+" % ")
print("false negative : "+str(false_negatif) + "   ; "+str((float(false_negatif)/num_states)*100.)+" %")
print("false positive : "+str(false_positif) + "   ; "+str((float(false_positif)/num_states)*100.)+" %")
print("considering only valid results of QP : ")
print("True false negative : "+str(true_false_negative) + "   ; "+str((float(true_false_negative)/num_states)*100.)+" %")
print("True false positive : "+str(true_false_positif) + "   ; "+str((float(true_false_positif)/num_states)*100.)+" %")

f = open(filename+"C","w")
f.write( "for : "+str(num_states)+" states.\n")
f.write(  "same result, unreachable : "+str(same_negatif) + "   ; "+str((float(same_negatif)/num_states)*100.)+" % \n")
f.write(  "same result, reachable : "+str(same_positif) + "   ; "+str((float(same_positif)/num_states)*100.)+" % \n")
f.write(  "false positive : "+str(false_positif) + "   ; "+str((float(false_positif)/num_states)*100.)+" %\n")
f.write(  "false negative : "+str(false_negatif) + "   ; "+str((float(false_negatif)/num_states)*100.)+" %\n\n")
f.write("##### Excluding unreachable states : \n")
num_states -= true_same_negatif
f.write( "for : "+str(num_states)+" states.\n")
f.write(  "same result, reachable : "+str(same_positif) + "   ; "+str((float(same_positif)/num_states)*100.)+" % \n")
f.write(  "false negative : "+str(false_negatif) + "   ; "+str((float(false_negatif)/num_states)*100.)+" %\n")
f.write(  "false positive : "+str(false_positif) + "   ; "+str((float(false_positif)/num_states)*100.)+" %\n")
f.write(  "but in  : "+str(false_positive_quasiStatic) + " cases there was a quasi-static solution \n")
f.write(  "false positive with no quasi-static solution : "+str(false_positif - false_positive_quasiStatic)+"   ; "+str((float((false_positif - false_positive_quasiStatic))/num_states)*100.)+" %\n\n")
f.write("##### Excluding invalid QP solutions : \n")
f.write(  "True false negative : "+str(true_false_negatif) + "   ; "+str((float(true_false_negatif)/num_states)*100.)+" %\n")
f.write(  "True false positive : "+str(true_false_positif) + "   ; "+str((float(true_false_positif)/num_states)*100.)+" %\n")

f.write("##### Continuous discret comparison  : \n")
total_discret_notContinuous = 0
total_notDiscret_Continuous = 0
total_discret_notContinuous_valid = 0
total_notDiscret_Continuous_valid = 0
f.write(" In "+str(float(total_discret_notContinuous/num_states)) +" % of the time a solution was found, it was found with the discret formulation but not with the continuous one. \n")
f.write(" In "+str(float(total_notDiscret_Continuous/num_states)) +" % of the time a solution was found, it was found with the continuous formulation but not with the discret one. \n")
f.write("continuous solution was invalid in "+str(float(total_invalid_continous/total_success_qpC)) +" % of the time \n")
f.write("discret    solution was invalid in "+str(float(total_invalid_discret/total_success_qp_discret)) +" % of the time \n")

f.write(" Considering only the solution that were valid : \n")
f.write(" In "+str(float(total_discret_notContinuous_valid/num_states)) +" % of the time a solution was found, it was found with the discret formulation but not with the continuous one. \n")
f.write(" In "+str(float(total_notDiscret_Continuous_valid/num_states)) +" % of the time a solution was found, it was found with the continuous formulation but not with the discret one. \n")

f.write("##### Muscod convergence  : \n")
f.write(" Muscod converged a total of "+str(muscod_converged)+" times \n")
f.write(" Muscod converged a total of "+str(muscod_converged_init_guess)+" times for a problem where an initial guess was available\n")
f.write(" This mean that "+str((float(muscod_converged_init_guess)/float(muscod_converged))*100.)+" % of the time where muscod converged, an initial guess was available\n")
f.write(" In "+str((float(wrong_init_guess)/float(total_success_qp))*100.)+" % the init guess prevented muscod to converge \n")
f.write(" In "+str((float(need_init_guess)/float(muscod_converged_init_guess))*100.)+" % of the case were an initial guess was provided, muscod only converged with the initial guess  \n")
f.write(" In "+str((float(need_init_guess)/float(muscod_converged))*100.)+" % of all the cases muscod only converged with an initial guess  \n")

f.close()
