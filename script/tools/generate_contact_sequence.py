import pinocchio as se3
from pinocchio import SE3, Quaternion
from pinocchio.utils import *

import locomote
from locomote import WrenchCone,SOC6,ControlType,IntegratorType,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid

DISPLAY_CONTACTS = True


# FIXME : retrive value from somewhere ? (it's actually in generate_straight_walk_muscod)
MRsole_offset = se3.SE3.Identity()
MRsole_offset.translation = np.matrix([0.0146,  -0.01, -0.105])
MLsole_offset = se3.SE3.Identity()
MLsole_offset.translation = np.matrix([0.0146,  0.01, -0.105])


def pinnochioQuaternion(q):
    assert len(q)>6, "config vector size must be superior to 7"
    w = q[3]
    q[3:6] = q[4:7]
    q[6] = w
    return q

def generateContactSequence(fb,configs,viewer=None):
    print "generate contact sequence from planning : "
    #print "MR offset",MRsole_offset
    #print "ML offset",MLsole_offset
    i_sphere = 0    
    n_double_support = len(configs)
    # config only contains the double support stance
    n_steps = n_double_support*2 -1 
    
    cs = ContactSequenceHumanoid(n_steps)
    unusedPatch = cs.contact_phases[0].LF_patch.copy()
    unusedPatch.placement = SE3.Identity()
    unusedPatch.active= False
    
    # for each double support phase we must create 2 contact_stance (exept for the final one)
    for k in range(0,n_double_support-1):
        # double support : 
        phase_d = cs.contact_phases[k*2]
        fb.setCurrentConfig(configs[k])
        # compute MRF and MLF : the position of the contacts
        q_r = fb.getJointPosition('RLEG_JOINT5')
        q_l = fb.getJointPosition('LLEG_JOINT5')
        MRF = SE3()
        MLF = SE3()
        MRF.translation = np.matrix(q_r[0:3])
        MLF.translation = np.matrix(q_l[0:3])
        rot_r = Quaternion(q_r[3],q_r[4],q_r[5],q_r[6])
        rot_l = Quaternion(q_l[3],q_l[4],q_l[5],q_l[6])
        MRF.rotation = rot_r.matrix()
        MLF.rotation = rot_l.matrix()
        
        # apply the transform ankle -> center of contact
        MRF *= MRsole_offset
        MLF *= MLsole_offset
        
        if DISPLAY_CONTACTS and viewer:
            gui = viewer.client.gui
            name = 's'+str(i_sphere)
            i_sphere += 1
            gui.addSphere(name,0.01,[1,0,0,1])
            gui.setVisibility(name,"ALWAYS_ON_TOP")
            gui.addToGroup(name,viewer.sceneName)
            p = MRF.translation.transpose().tolist()[0]
            rot = Quaternion(MRF.rotation)
            p +=  [rot.w]
            p += rot.coeffs().transpose().tolist()[0][0:3]
            gui.applyConfiguration(name,p)
            gui.addLandmark(name,0.1)
            name = 's'+str(i_sphere)
            i_sphere += 1
            gui.addSphere(name,0.01,[1,0,0,1])
            gui.setVisibility(name,"ALWAYS_ON_TOP")
            gui.addToGroup(name,viewer.sceneName)
            p = MLF.translation.transpose().tolist()[0]
            rot = Quaternion(MLF.rotation)
            p +=  [rot.w]
            p += rot.coeffs().transpose().tolist()[0][0:3]           
            gui.applyConfiguration(name,p)  
            gui.addLandmark(name,0.1)            
            gui.refresh()            
        
        if k==0: #init stance
            phase_d.RF_patch.placement = MRF
            phase_d.RF_patch.active = True
        
            phase_d.LF_patch.placement = MLF
            phase_d.LF_patch.active = True   
        else: # we need to copy the unchanged patch from the last simple support phase (and not create a new one with the same placement)
            variations = fb.getContactsVariations(k-1,k)
            assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"
            for var in variations:            
                if var == 'hrp2_lleg_rom':
                    phase_d.RF_patch = phase_s.RF_patch
                    phase_d.LF_patch.placement = MLF
                    phase_d.LF_patch.active = True                       
                if var == 'hrp2_rleg_rom':
                    phase_d.LF_patch = phase_s.LF_patch                    
                    phase_d.RF_patch.placement = MRF
                    phase_d.RF_patch.active = True
        #FIXME : retrieve list of active contact in planning
        phase_d.LH_patch=unusedPatch.copy()
        phase_d.RH_patch=unusedPatch.copy()
        # retrieve the COM position for init and final state (equal for double support phases)
        init_state = phase_d.init_state
        init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
        init_state[3:9] = np.matrix(configs[k][-6:]).transpose()
        phase_d.init_state=init_state
        phase_d.final_state=init_state
        phase_d.reference_configurations.append(np.matrix(pinnochioQuaternion(configs[k][:-6])))
        phase_d.time_trajectory.append(0.)
        
        # simple support : 
        phase_s = cs.contact_phases[k*2 + 1]
        # copy previous placement :
        phase_s.RF_patch = phase_d.RF_patch
        phase_s.LF_patch = phase_d.LF_patch
        phase_s.LH_patch=unusedPatch.copy()
        phase_s.RH_patch=unusedPatch.copy()  
        # find the contact to break : 
        variations = fb.getContactsVariations(k,k+1)
        for var in variations:
            if var == 'hrp2_lleg_rom': # FIXME : retrieve the names from somewhere
                phase_s.LF_patch.active=False
                phase_s.RF_patch.active=True
            if var == 'hrp2_rleg_rom':
                phase_s.RF_patch.active=False 
                phase_s.LF_patch.active=True
        # retrieve the COM position for init and final state 
        phase_s.init_state=init_state
        final_state = phase_d.final_state
        final_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
        final_state[3:9] = np.matrix(configs[k+1][-6:]).transpose()        
        phase_s.final_state=final_state
        phase_s.reference_configurations.append(np.matrix(pinnochioQuaternion(configs[k][:-6])))
        phase_s.time_trajectory.append(fb.getTimeAtState(k+1) - fb.getTimeAtState(k))
        
        
    # add the final double support stance : 
    phase_d = cs.contact_phases[n_steps-1]
    fb.setCurrentConfig(configs[n_double_support - 1])
    # compute MRF and MLF : the position of the contacts
    q_r = fb.getJointPosition('RLEG_JOINT5')
    q_l = fb.getJointPosition('LLEG_JOINT5')
    MRF = SE3()
    MLF = SE3()
    MRF.translation = np.matrix(q_r[0:3])
    MLF.translation = np.matrix(q_l[0:3])
    rot_r = Quaternion(q_r[3],q_r[4],q_r[5],q_r[6])
    rot_l = Quaternion(q_l[3],q_l[4],q_l[5],q_l[6])
    MRF.rotation = rot_r.matrix()
    MLF.rotation = rot_l.matrix()

    # apply the transform ankle -> center of contact
    MRF *= MRsole_offset
    MLF *= MLsole_offset   
    
    # we need to copy the unchanged patch from the last simple support phase (and not create a new one with the same placement)
    variations = fb.getContactsVariations(n_double_support - 2,n_double_support - 1)
    assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"
    for var in variations:            
        if var == 'hrp2_lleg_rom':
            phase_d.RF_patch = phase_s.RF_patch
            phase_d.LF_patch.placement = MLF
            phase_d.LF_patch.active = True                       
        if var == 'hrp2_rleg_rom':
            phase_d.LF_patch = phase_s.LF_patch                    
            phase_d.RF_patch.placement = MRF
            phase_d.RF_patch.active = True
    phase_d.LH_patch=unusedPatch.copy()
    phase_d.RH_patch=unusedPatch.copy()            
    # retrieve the COM position for init and final state (equal for double support phases)
    init_state = phase_d.init_state
    init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
    init_state[3:9] = np.matrix(configs[-1][-6:]).transpose()        
    phase_d.init_state=init_state
    phase_d.final_state=init_state
    phase_d.reference_configurations.append(np.matrix(pinnochioQuaternion(configs[-1][:-6])))  
    phase_d.time_trajectory.append(0.)
    
    return cs


