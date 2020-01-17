from stair_bauzil_hrp2_interp import *
import generate_muscod_problem as mp
from pathlib2 import Path
import muscodSSH as ssh
tryWarmStart = True

print("run bench with feasibility criterion")

com = fullBody.getCenterOfMass()
if com[0] > 1.25 :
    success = True
else :
    success = False

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
  
    if tryWarmStart :
        if not success : 
            # generate warm start from planning : 
            if Path(CONTACT_SEQUENCE_WHOLEBODY_FILE).is_file():
                os.remove(CONTACT_SEQUENCE_WHOLEBODY_FILE)
            filename_xml = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE 
            cs = generateContactSequenceWithInitGuess(ps,fullBody,configs,beginState,endState)
            cs.saveAsXML(filename, "ContactSequence")
            mp.generate_muscod_problem(filename_xml,True)
            successMuscod,ssh_ok = ssh.call_muscod()
            time.sleep(5.) # wait for sync of the ~/home (worst case, usually 0.1 is enough ... )
            muscodWarmStartConverged = successMuscod and Path(CONTACT_SEQUENCE_WHOLEBODY_FILE).is_file()
        else : 
            muscodWarmStartConverged= True
    else :
        muscodWarmStartConverged = False
else : 
    muscodConverged = False
    muscodWarmStartConverged = False
    tInterpolateConfigs = 0.
    numConf = 0.

name = "/local/fernbac/bench_iros18/bench_contactGeneration/stairsMC_croc.log"
f = open(name,"a")
f.write("new\n")
f.write("success "+str(success)+"\n")
f.write("muscodNoWarmStart "+str(muscodConverged)+"\n")
f.write("muscodWarmStart "+str(muscodWarmStartConverged)+"\n")
f.write("time "+str(tInterpolateConfigs)+"\n")
f.write("configs "+str(numConf)+"\n")
f.close()

