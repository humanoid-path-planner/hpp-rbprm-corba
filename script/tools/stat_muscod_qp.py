from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import numpy as np
#from hpp.corbaserver.rbprm.tools.cwc_trajectory import *

from hpp import Error as hpperr
from numpy import array, matrix
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
import cPickle as pickle
from pathlib2 import Path

import planning.generate_muscod_problem as mp
import muscodSSH as ssh
from planning.config import *


from gen_hrp2_statically_balanced_positions_2d_state import *
from generate_contact_sequence import *


def callMuscodBetweenTwoState(fullBody,s0,s1,c_qp = [], t_qp = []):
    configs = [s0.q() , s1.q()]
    cs = generateContactSequence(fullBody,configs,s0.sId, s1.sId)
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
print "write results in file : "+filename
i=0
cs_out = ContactSequenceHumanoid(0)
same_positif = 0
same_negatif = 0
false_positif = 0
false_negatif = 0
for [s0,s1] in states:
    success_quasiStatic = False    
    success_qp = False
    success_muscod = False
    
    success_quasiStatic = fullBody.isReachableFromState(s0.sId,s1.sId)
    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True)
    success_qp = (len(pid) > 0)

    if success_qp :
        #compute init guess curve for muscod : 
        c_qp = fullBody.getPathAsBezier(int(pid[0]))
        t_qp = [ps.pathLength(int(pid[1])), ps.pathLength(int(pid[2])), ps.pathLength(int(pid[3]))]
        success_muscod, ssh_ok = callMuscodBetweenTwoState(fullBody,s0,s1,[c_qp],[t_qp])
    success_muscod, ssh_ok = callMuscodBetweenTwoState(fullBody,s0,s1)
    f.write("Try for pair "+str(i)+"\n")
    f.write("q0 = "+str(s0.q())+"\n")
    f.write("q1 = "+str(s1.q())+"\n")
            
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
            f.write("false positive \n")
            false_positif += 1
        if not success_qp and success_muscod:
            f.write("false negative \n")
            false_negatif +=1
    f.flush()
    i +=1

f.close()


num_states = float(len(states))
print "for : "+str(num_states)+" states."
print "same result, unreachable : "+str(same_negatif) + "   ; "+str((float(same_negatif)/num_states)*100.)+" % "
print "same result, reachable : "+str(same_positif) + "   ; "+str((float(same_positif)/num_states)*100.)+" % "
print "false positive : "+str(false_positif) + "   ; "+str((float(false_positif)/num_states)*100.)+" %"
print "false negative : "+str(false_negatif) + "   ; "+str((float(false_negatif)/num_states)*100.)+" %"
f = open(filename+"C","w")
f.close()
f.write( "for : "+str(num_states)+" states.")
f.write(  "same result, unreachable : "+str(same_negatif) + "   ; "+str((float(same_negatif)/num_states)*100.)+" % ")
f.write(  "same result, reachable : "+str(same_positif) + "   ; "+str((float(same_positif)/num_states)*100.)+" % ")
f.write(  "false positive : "+str(false_positif) + "   ; "+str((float(false_positif)/num_states)*100.)+" %")
f.write(  "false negative : "+str(false_negatif) + "   ; "+str((float(false_negatif)/num_states)*100.)+" %")


from constraint_to_dae import *
from display_tools import *
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)


def showPath(pid):
    if len(pid)==1:
        pp.displayPath(int(pid[0]),color=r.color.blue)
        r.client.gui.setVisibility('path_'+str(int(pid[0]))+'_root','ALWAYS_ON_TOP')
    elif len(pid)==3:
        pp.displayPath(int(pid[0]),color=r.color.green)
        r.client.gui.setVisibility('path_'+str(int(pid[0]))+'_root','ALWAYS_ON_TOP')  
        pp.displayPath(int(pid[1]),color=r.color.blue)
        r.client.gui.setVisibility('path_'+str(int(pid[1]))+'_root','ALWAYS_ON_TOP')  
        pp.displayPath(int(pid[2]),color=r.color.yellow)
        r.client.gui.setVisibility('path_'+str(int(pid[2]))+'_root','ALWAYS_ON_TOP')
    elif len(pid) == 2:
        print "only two phases, not implemented yet."
    else:
        print "no path, test failed."

  
[s0,s2] = states[25]
r(s0.q())
r(s2.q())

leg = rLegId
# leg = lLegId
s1,success = StateHelper.removeContact(s0,leg)
success

pid = fullBody.isDynamicallyReachableFromState(s0.sId,s2.sId,True)
showPath(pid)
