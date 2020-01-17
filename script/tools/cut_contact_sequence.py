import math
import pinocchio as se3
from pinocchio import SE3, Quaternion
from pinocchio.utils import *

import locomote
from locomote import WrenchCone,SOC6,ControlType,IntegratorType,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid
import generate_muscod_problem as mp
import muscodSSH as ssh
from config import *

statesPerStep=5 # number of double support configs from the planning per call to muscod
stepSize=statesPerStep*2 - 1 # contact_sequence contain double support AND simple support states

def solveMuscodProblem(configsFull,cs):
    begin=0 *2 # expressed in cs index (and not configs index!)
    end=(len(configsFull)) *2 -1
    
    numStep = int( math.ceil((end-begin)/(stepSize-2)))
    #numStep=2
    P3_Num_step = 100
    
    sequences= []
    outputs=[]
    isInit=True
    for i in range(0,numStep):
        if i == (numStep-1) : 
            # last call, include all the phases left (it may be more than stepSize)
            currentSteps = (stepSize-3)*(numStep-1) # id of the next phase to compute            
            sequences += [ContactSequenceHumanoid(cs.size()-currentSteps)]  
            print("currentSteps :",currentSteps)
            print("last size = ",(cs.size()-currentSteps))            
            for k in range (cs.size() - currentSteps):
                sequences[i].contact_phases[k] = cs.contact_phases[currentSteps + k] # last phases of sequence i is the second phase of sequence i+1, not the first        
        else : 
            sequences += [ContactSequenceHumanoid(stepSize)]
            for k in range(0,stepSize):
                sequences[i].contact_phases[k] = cs.contact_phases[(stepSize-3)*i + k] # last phases of sequence i is the second phase of sequence i+1, not the first
        if i > 0:
            sequences[i].contact_phases[0].init_state = outputs[i-1].contact_phases[stepSize-3].state_trajectory[P3_Num_step]
            sequences[i].contact_phases[0].reference_configurations[0] = outputs[i-1].contact_phases[stepSize-3].reference_configurations[0]
        filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE[:-4] + "_" + str(i) + ".seq"   
        sequences[i].saveAsXML(filename, "ContactSequence")
        print("save contact sequence : ",filename)
    
        mp.generate_muscod_problem(filename,isInit)
        isInit=False
        success = ssh.call_muscod()
        assert success,"Error in muscod call"
        outputs += [ContactSequenceHumanoid(0)]
        outputs[i].loadFromXML(CONTACT_SEQUENCE_WHOLEBODY_FILE,CONTACT_SEQUENCE_XML_TAG)
        print("cut contact sequence, call "+str(i)+" size = "+str(outputs[i].size()))
    
    
    
    ## merge sequences outputs together : 
    init_time_at_step = 0.    
    finalSeq = ContactSequenceHumanoid(cs.size()) 
    for id_steps in range(0,numStep):
        if id_steps>0 : 
            # merging phase, we should merge state_trajectory; the first half (up to P3_num_step) is in the first half of mid_phase0
            # the second half is the full trajectory in mid_phase1, it should be reduced and the time step should be adjusted
            mid_phase1 = outputs[id_steps].contact_phases[0]
            mid_phase0 = outputs[id_steps-1].contact_phases[stepSize-3]
            old_state_traj=[]
            old_time_traj=[]
            for j in range (0,len(mid_phase1.state_trajectory)): # save the traj from mid_phase1 before overwriting them
                old_state_traj.append(mid_phase1.state_trajectory[j])
                old_time_traj.append(mid_phase1.time_trajectory[j])
            for j in range (0,P3_Num_step): # first half, take mid_phase0
                mid_phase1.state_trajectory[j] = mid_phase0.state_trajectory[j]
                mid_phase1.time_trajectory[j]=mid_phase0.time_trajectory[j] + init_time_at_step
            init_time_at_step += mid_phase0.time_trajectory[P3_Num_step] # need to offset all the times of the next phases by this value
            for j in range (P3_Num_step,len(mid_phase1.state_trajectory)): # second half, take and expand mid_phase1
                mid_phase1.state_trajectory[j]=old_state_traj[(j-P3_Num_step)*2] 
                mid_phase1.time_trajectory[j] = old_time_traj[(j-P3_Num_step)*2] +init_time_at_step
            finalSeq.contact_phases[(stepSize-3)*id_steps]=mid_phase1 
        else:
            finalSeq.contact_phases[0]=outputs[0].contact_phases[0] 
        for id_phase in range(1,stepSize-3):
            phase=outputs[id_steps].contact_phases[id_phase]
            for i in range(0,len(phase.time_trajectory)):
                phase.time_trajectory[i] += init_time_at_step # offset all the times by the last value of the last step   
            finalSeq.contact_phases[(stepSize-3)*id_steps+id_phase]=phase
    
            
            
    for id_phase in range(stepSize-3,outputs[id_steps].size()):
        phase=outputs[id_steps].contact_phases[id_phase]
        for i in range(0,len(phase.time_trajectory)):
            phase.time_trajectory[i] += init_time_at_step # offset all the times by the last value of the last step       
        finalSeq.contact_phases[(stepSize-3)*id_steps+id_phase]=phase
    print("cut_contact_sequence, write file : ",CONTACT_SEQUENCE_WHOLEBODY_FILE)
    print("contact_sequence size = ",finalSeq.size())
    finalSeq.saveAsXML(CONTACT_SEQUENCE_WHOLEBODY_FILE,CONTACT_SEQUENCE_XML_TAG)

    
