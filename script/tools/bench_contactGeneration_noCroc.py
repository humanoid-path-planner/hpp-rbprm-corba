from stair_bauzil_hrp2_interp import *
import generate_muscod_problem as mp
from pathlib2 import Path
import muscodSSH as ssh
tryWarmStart = False

print("run bench without feasibility criterion")

com = fullBody.getCenterOfMass()
if com[0] > 1.3:
    success = True
else :
    success = False

#success = False
numConf = len(configs)

if success :
    # muscod without warm start :
    if Path(CONTACT_SEQUENCE_WHOLEBODY_FILE).is_file():
        os.remove(CONTACT_SEQUENCE_WHOLEBODY_FILE)
    filename_xml = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
    mp.generate_muscod_problem(filename_xml,True)
    successMuscod,ssh_ok = ssh.call_muscod()
    time.sleep(5.) # wait for sync of the ~/home (worst case, usually 0.1 is enough ... )
    muscodConverged = successMuscod and Path(CONTACT_SEQUENCE_WHOLEBODY_FILE).is_file()
    
    crocConverged = True
    for id_state in range(beginState,endState):
        pid = fullBody.isDynamicallyReachableFromState(id_state,id_state+1,numPointsPerPhases=0)
        if len(pid) == 0:
            print("Croc did not converge for state "+str(id_state))
            crocConverged = False
        
else : 
    crocConverged = False    
    muscodConverged = False
    tInterpolateConfigs = 0.
    numConf = 0.
    

name = "/local/fernbac/bench_iros18/bench_contactGeneration/stairsMC_noCroc.log"
f = open(name,"a")
f.write("new\n")
f.write("success "+str(success)+"\n")
f.write("muscodNoWarmStart "+str(muscodConverged)+"\n")
f.write("crocConverged "+str(crocConverged)+"\n")
f.write("time "+str(tInterpolateConfigs)+"\n")
f.write("configs "+str(numConf)+"\n")
f.close()

