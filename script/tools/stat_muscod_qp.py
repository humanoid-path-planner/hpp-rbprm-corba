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

import planning.generate_muscod_problem as mp
import muscodSSH as ssh
from planning.config import *


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





states = genStateWithOneStep(fullBody,limbs[0], 100,False)

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
false_positive_quasiStatic = 0
false_negatif = 0
muscod_converged = 0
muscod_converged_init_guess = 0
need_init_guess =0
wrong_init_guess = 0
total_success_qp = 0
for [s0,s1] in states:

    f.write("Try for pair "+str(i)+" ------- \n")
    f.write("q0 = "+str(s0.q())+"\n")
    f.write("q1 = "+str(s1.q())+"\n")


    success_quasiStatic = False
    success_qp = False
    success_muscod = False

    success_quasiStatic = fullBody.isReachableFromState(s0.sId,s1.sId)
    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True)
    success_qp = (len(pid) > 0)

    if success_qp :
        #compute init guess curve for muscod :
        total_success_qp +=1
        c_qp = fullBody.getPathAsBezier(int(pid[0]))
        t_qp = [ps.pathLength(int(pid[1])), ps.pathLength(int(pid[2])), ps.pathLength(int(pid[3]))]
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
        if success_muscod and success_qp:
            f.write("same results, reachable \n")
            same_positif += 1
        if success_qp and not success_muscod:
            false_positif += 1
            if success_quasiStatic:
                f.write("false positive but with quasi-static solution \n")
                false_positive_quasiStatic += 1
            else :
                f.write("false positive \n")

        if not success_qp and success_muscod:
            f.write("false negative \n")
            false_negatif +=1
    f.flush()
    i +=1

f.close()


num_states = float(len(states))
print("for : "+str(num_states)+" states.")
print("same result, unreachable : "+str(same_negatif) + "   ; "+str((float(same_negatif)/num_states)*100.)+" % ")
print("same result, reachable : "+str(same_positif) + "   ; "+str((float(same_positif)/num_states)*100.)+" % ")
print("false negative : "+str(false_negatif) + "   ; "+str((float(false_negatif)/num_states)*100.)+" %")
print("false positive : "+str(false_positif) + "   ; "+str((float(false_positif)/num_states)*100.)+" %")

f = open(filename+"C","w")
f.write( "for : "+str(num_states)+" states.\n")
f.write(  "same result, unreachable : "+str(same_negatif) + "   ; "+str((float(same_negatif)/num_states)*100.)+" % \n")
f.write(  "same result, reachable : "+str(same_positif) + "   ; "+str((float(same_positif)/num_states)*100.)+" % \n")
f.write(  "false positive : "+str(false_positif) + "   ; "+str((float(false_positif)/num_states)*100.)+" %\n")
f.write(  "false negative : "+str(false_negatif) + "   ; "+str((float(false_negatif)/num_states)*100.)+" %\n\n")
f.write("##### Excluding unreachable states : \n")
num_states -= same_negatif
f.write( "for : "+str(num_states)+" states.\n")
f.write(  "same result, reachable : "+str(same_positif) + "   ; "+str((float(same_positif)/num_states)*100.)+" % \n")
f.write(  "false negative : "+str(false_negatif) + "   ; "+str((float(false_negatif)/num_states)*100.)+" %\n")
f.write(  "false positive : "+str(false_positif) + "   ; "+str((float(false_positif)/num_states)*100.)+" %\n")
f.write(  "but in  : "+str(false_positive_quasiStatic) + " cases there was a quasi-static solution \n")
f.write(  "false positive with no quasi-static solution : "+str(false_positif - false_positive_quasiStatic)+"   ; "+str((float((false_positif - false_positive_quasiStatic))/num_states)*100.)+" %\n\n")


f.write("##### Muscod convergence  : \n")
f.write(" Muscod converged a total of "+str(muscod_converged)+" times \n")
f.write(" Muscod converged a total of "+str(muscod_converged_init_guess)+" times for a problem where an initial guess was available\n")
f.write(" This mean that "+str((float(muscod_converged_init_guess)/float(muscod_converged))*100.)+" % of the time where muscod converged, an initial guess was available\n")
f.write(" In "+str((float(wrong_init_guess)/float(total_success_qp))*100.)+" % the init guess prevented muscod to converge \n")
f.write(" In "+str((float(need_init_guess)/float(muscod_converged_init_guess))*100.)+" % of the case were an initial guess was provided, muscod only converged with the initial guess  \n")
f.write(" In "+str((float(need_init_guess)/float(muscod_converged))*100.)+" % of all the cases muscod only converged with an initial guess  \n")

f.close()

"""
from constraint_to_dae import *
from hpp.corbaserver.rbprm.tools.display_tools import *
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)


def showPath(pid):
    if len(pid)==1:
        pp.displayPath(int(pid[0]),color=r.color.blue)
        r.client.gui.setVisibility('path_'+str(int(pid[0]))+'_root','ALWAYS_ON_TOP')
    elif len(pid)==4:
        pp.displayPath(int(pid[1]),color=r.color.green)
        r.client.gui.setVisibility('path_'+str(int(pid[1]))+'_root','ALWAYS_ON_TOP')
        pp.displayPath(int(pid[2]),color=r.color.blue)
        r.client.gui.setVisibility('path_'+str(int(pid[2]))+'_root','ALWAYS_ON_TOP')
        pp.displayPath(int(pid[3]),color=r.color.yellow)
        r.client.gui.setVisibility('path_'+str(int(pid[3]))+'_root','ALWAYS_ON_TOP')
    elif len(pid) == 3:
        print "only two phases, not implemented yet."
    else:
        print "no path, test failed."


[s0,s2] = states[8]
r(s0.q())
r(s2.q())

leg = rLegId
# leg = lLegId
s1,success = StateHelper.removeContact(s0,leg)
success

pid = fullBody.isDynamicallyReachableFromState(s0.sId,s2.sId,True)
showPath(pid)


c_qp = fullBody.getPathAsBezier(int(pid[0]))
t_qp = [ps.pathLength(int(pid[1])), ps.pathLength(int(pid[2])), ps.pathLength(int(pid[3]))]
#cs = generateContactSequence(fullBody,[s0.q(),s2.q()],s0.sId, s2.sId,curves_initGuess=[c_qp],timings_initGuess=[t_qp])
callMuscodBetweenTwoState(fullBody,s0,s2,c_qp=[c_qp],t_qp=[t_qp])

q0 = [-0.17608794810800854, 0.2910028141264251, 0.6331113268445426, 0.9762318767074574, -0.03329845286589081, 0.053793887560739634, 0.20728905807609832, 0.0, 0.0, 0.0, 0.0, 0.261799388, 0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 0.4250133737512345, -0.11156388170456344, -1.242181321203497, 1.4133331985634723, -0.13083383642865257, 0.349066, -0.3703957634008732, -0.060915884410948806, -0.6488106973944403, 1.3115039520511262, -0.6125539084426079, -0.349066, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q1 = [-0.29494066983225786, 0.09282560478561819, 0.5712900482217269, 0.988259554853585, 0.0, 0.0, 0.15278433244477005, 0.0, 0.0, 0.0, 0.0, 0.261799388, 0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 0.5287973121214414, 0.026253043235683338, -1.2852663946582035, 0.8948519765377176, 0.556791558819234, 0.220496212563994, -0.2549772760822048, 0.27430957553464475, -0.6637644091040604, 1.7965782911949775, -1.055964350187135, -0.2978706786815243, 0.0, 0.0, 0.0, 0., 0.,0.]

s0 = State(fullBody,q= q0,limbsIncontact=limbs[0])
s2 = State(fullBody,q= q1,limbsIncontact=limbs[0])

s1,success = StateHelper.removeContact(s0,lLegId)

"""
