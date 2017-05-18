import pinocchio as se3
from pinocchio import SE3, Quaternion
from pinocchio.utils import *

import locomote
from locomote import WrenchCone,SOC6,ControlType,IntegratorType,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid

DURATION_n_CONTACTS = 0.2 # percentage of time allocated to the movement of the com without moving the contacts
global i_sphere 
DISPLAY_CONTACTS = True
rleg_id = "RLEG_JOINT5"
lleg_id = "LLEG_JOINT5"
rhand_id = "RARM_JOINT5"
lhand_id = "LARM_JOINT5"
rleg_rom = 'hrp2_rleg_rom'
lleg_rom = 'hrp2_lleg_rom'
rhand_rom = 'hrp2_rarm_rom'
lhand_rom = 'hrp2_larm_rom'
limbs_names = [rleg_rom,lleg_rom,rhand_rom,lhand_rom]

# FIXME : retrive value from somewhere ? (it's actually in generate_straight_walk_muscod)
MRsole_offset = se3.SE3.Identity()
MRsole_offset.translation = np.matrix([0.0146,  -0.01, -0.105])
MLsole_offset = se3.SE3.Identity()
MLsole_offset.translation = np.matrix([0.0146,  0.01, -0.105])
MRhand_offset = se3.SE3.Identity()
rot = np.matrix([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
MRhand_offset.rotation = rot
MLhand_offset = se3.SE3.Identity()
rot = np.matrix([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]]) # TODO : check this
MRhand_offset.rotation = rot

# display transform :

MRsole_display = se3.SE3.Identity()
MLsole_display = se3.SE3.Identity()
MRhand_display = se3.SE3.Identity()
MRhand_display.translation = np.matrix([0,  0., -0.11])
MLhand_display = se3.SE3.Identity()
MLhand_display.translation = np.matrix([0,  0., -0.11])


def pinnochioQuaternion(q):
    assert len(q)>6, "config vector size must be superior to 7"
    w = q[3]
    q[3:6] = q[4:7]
    q[6] = w
    return q

def addContactLandmark(M,color,viewer):
    global i_sphere
    gui = viewer.client.gui
    name = 's'+str(i_sphere)
    i_sphere += 1    
    gui.addSphere(name,0.02,color)
    gui.setVisibility(name,"ALWAYS_ON_TOP")
    gui.addToGroup(name,viewer.sceneName)
    p = M.translation.transpose().tolist()[0]
    rot = Quaternion(M.rotation)
    p +=  [rot.w]
    p += rot.coeffs().transpose().tolist()[0][0:3]
    gui.applyConfiguration(name,p)
    gui.addLandmark(name,0.1)    
    
    
def displayContactsFromPhase(phase,viewer):
    if phase.LF_patch.active:
        addContactLandmark(phase.LF_patch.placement*MLsole_display,viewer.color.red ,viewer)
    if phase.RF_patch.active:
        addContactLandmark(phase.RF_patch.placement*MRsole_display,viewer.color.green ,viewer)  
    if phase.LH_patch.active:
        addContactLandmark(phase.LH_patch.placement*MLhand_display,viewer.color.yellow ,viewer)
    if phase.RH_patch.active:
        addContactLandmark(phase.RH_patch.placement*MRhand_display,viewer.color.blue ,viewer)                 
    viewer.client.gui.refresh()                    
        

def generateContactSequence(fb,configs,viewer=None):
    print "generate contact sequence from planning : "
    global i_sphere
    i_sphere = 0
    #print "MR offset",MRsole_offset
    #print "ML offset",MLsole_offset  
    n_double_support = len(configs)
    # config only contains the double support stance
    n_steps = n_double_support*2 -1 
    
    cs = ContactSequenceHumanoid(n_steps)
    unusedPatch = cs.contact_phases[0].LF_patch.copy()
    unusedPatch.placement = SE3.Identity()
    unusedPatch.active= False
    
    # for contact state we must create 2 phase (one with all the contact and one with the next replaced contact(s) broken)
    for k in range(0,n_double_support-1):
        # %%%%%%%%%  all the contacts : %%%%%%%%%%%%%
        phase_d = cs.contact_phases[k*2]
        fb.setCurrentConfig(configs[k])
        # compute MRF and MLF : the position of the contacts
        q_rl = fb.getJointPosition(rleg_id)
        q_ll = fb.getJointPosition(lleg_id)
        q_rh = fb.getJointPosition(rhand_id)
        q_lh = fb.getJointPosition(lhand_id)
        
        # feets
        MRF = SE3()
        MLF = SE3()
        MRF.translation = np.matrix(q_rl[0:3])
        MLF.translation = np.matrix(q_ll[0:3])
        rot_rl = Quaternion(q_rl[3],q_rl[4],q_rl[5],q_rl[6])
        rot_ll = Quaternion(q_ll[3],q_ll[4],q_ll[5],q_ll[6])
        MRF.rotation = rot_rl.matrix()
        MLF.rotation = rot_ll.matrix()
        
        # apply the transform ankle -> center of contact
        MRF *= MRsole_offset
        MLF *= MLsole_offset
        
        # hands
        MRH = SE3()
        MLH = SE3()
        MRH.translation = np.matrix(q_rh[0:3])
        MLH.translation = np.matrix(q_lh[0:3])
        rot_rh = Quaternion(q_rh[3],q_rh[4],q_rh[5],q_rh[6])
        rot_lh = Quaternion(q_lh[3],q_lh[4],q_lh[5],q_lh[6])
        MRH.rotation = rot_rh.matrix()
        MLH.rotation = rot_lh.matrix()   
        
        MRH *= MRhand_offset
        MLH *= MLhand_offset      
        
        # initial state : Set all new contacts patch (either with placement computed below or unused)
        if k==0:
            # FIXME : for loop ? how ?
            if fb.isLimbInContact(rleg_rom,k):
                phase_d.RF_patch.placement = MRF
                phase_d.RF_patch.active = True
            else:
                phase_d.RF_patch = unusedPatch.copy()
            if fb.isLimbInContact(lleg_rom,k):
                phase_d.LF_patch.placement = MLF
                phase_d.LF_patch.active = True
            else:
                phase_d.LF_patch = unusedPatch.copy()
            if fb.isLimbInContact(rhand_rom,k):
                phase_d.RH_patch.placement = MRH
                phase_d.RH_patch.active = True
            else:
                phase_d.RH_patch = unusedPatch.copy()
            if fb.isLimbInContact(lhand_rom,k):
                phase_d.LH_patch.placement = MLH
                phase_d.LH_patch.active = True
            else:
                phase_d.LH_patch = unusedPatch.copy()
        else:   
            # we need to copy the unchanged patch from the last simple support phase (and not create a new one with the same placement)
            phase_d.RF_patch = phase_s.RF_patch
            phase_d.RF_patch.active = fb.isLimbInContact(rleg_rom,k)
            phase_d.LF_patch = phase_s.LF_patch
            phase_d.LF_patch.active = fb.isLimbInContact(lleg_rom,k)
            phase_d.RH_patch = phase_s.RH_patch
            phase_d.RH_patch.active = fb.isLimbInContact(rhand_rom,k)
            phase_d.LH_patch = phase_s.LH_patch
            phase_d.LH_patch.active = fb.isLimbInContact(lhand_rom,k)
            
            # now we change the contacts that have moved : 
            variations = fb.getContactsVariations(k-1,k)
            #assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"
            for var in variations:     
                # FIXME : for loop in variation ? how ?
                if var == lleg_rom:
                    phase_d.LF_patch.placement = MLF
                if var == rleg_rom:
                    phase_d.RF_patch.placement = MRF
                if var == lhand_rom:
                    phase_d.LH_patch.placement = MLH
                if var == rhand_rom:
                    phase_d.RH_patch.placement = MRH
                    
        # retrieve the COM position for init and final state (equal for double support phases)
        init_state = phase_d.init_state
        init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
        init_state[3:9] = np.matrix(configs[k][-6:]).transpose()
        phase_d.init_state=init_state
        phase_d.final_state=init_state
        phase_d.reference_configurations.append(np.matrix(pinnochioQuaternion(configs[k][:-6])))
        phase_d.time_trajectory.append((fb.getTimeAtState(k+1) - fb.getTimeAtState(k))*DURATION_n_CONTACTS)
        
        
        if DISPLAY_CONTACTS and viewer:
            displayContactsFromPhase(phase_d,viewer)
        
        
        # %%%%%% simple support : %%%%%%%% 
        phase_s = cs.contact_phases[k*2 + 1]
        # copy previous placement :
        phase_s.RF_patch = phase_d.RF_patch
        phase_s.LF_patch = phase_d.LF_patch
        phase_s.RH_patch = phase_d.RH_patch
        phase_s.LH_patch = phase_d.LH_patch 
        # find the contact to break : 
        variations = fb.getContactsVariations(k,k+1)
        for var in variations:
            if var == lleg_rom:
                phase_s.LF_patch.active = False
            if var == rleg_rom:
                phase_s.RF_patch.active = False
            if var == lhand_rom:
                phase_s.LH_patch.active = False
            if var == rhand_rom:
                phase_s.RH_patch.active = False
        # retrieve the COM position for init and final state 
        phase_s.init_state=init_state.copy()
        final_state = phase_d.final_state.copy()
        fb.setCurrentConfig(configs[k+1])
        final_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
        final_state[3:9] = np.matrix(configs[k+1][-6:]).transpose()        
        phase_s.final_state=final_state
        phase_s.reference_configurations.append(np.matrix(pinnochioQuaternion(configs[k][:-6])))
        phase_s.time_trajectory.append((fb.getTimeAtState(k+1) - fb.getTimeAtState(k))*(1-DURATION_n_CONTACTS))
        
        if DISPLAY_CONTACTS and viewer:
            displayContactsFromPhase(phase_s,viewer)        
        
    # add the final double support stance : 
    phase_d = cs.contact_phases[n_steps-1]
    fb.setCurrentConfig(configs[n_double_support - 1])
    # compute MRF and MLF : the position of the contacts
    q_rl = fb.getJointPosition(rleg_id)
    q_ll = fb.getJointPosition(lleg_id)
    q_rh = fb.getJointPosition(rhand_id)
    q_lh = fb.getJointPosition(lhand_id)

    # feets
    MRF = SE3()
    MLF = SE3()
    MRF.translation = np.matrix(q_rl[0:3])
    MLF.translation = np.matrix(q_ll[0:3])
    rot_rl = Quaternion(q_rl[3],q_rl[4],q_rl[5],q_rl[6])
    rot_ll = Quaternion(q_ll[3],q_ll[4],q_ll[5],q_ll[6])
    MRF.rotation = rot_rl.matrix()
    MLF.rotation = rot_ll.matrix()

    # apply the transform ankle -> center of contact
    MRF *= MRsole_offset
    MLF *= MLsole_offset

    # hands
    MRH = SE3()
    MLH = SE3()
    MRH.translation = np.matrix(q_rh[0:3])
    MLH.translation = np.matrix(q_lh[0:3])
    rot_rh = Quaternion(q_rh[3],q_rh[4],q_rh[5],q_rh[6])
    rot_lh = Quaternion(q_lh[3],q_lh[4],q_lh[5],q_lh[6])
    MRH.rotation = rot_rh.matrix()
    MLH.rotation = rot_lh.matrix()   
    
    MRH *= MRhand_offset
    MLH *= MLhand_offset    
    
    # we need to copy the unchanged patch from the last simple support phase (and not create a new one with the same placement)
    k = n_double_support-1
    phase_d.RF_patch = phase_s.RF_patch
    phase_d.RF_patch.active = fb.isLimbInContact(rleg_rom,k)
    phase_d.LF_patch = phase_s.LF_patch
    phase_d.LF_patch.active = fb.isLimbInContact(lleg_rom,k)
    phase_d.RH_patch = phase_s.RH_patch
    phase_d.RH_patch.active = fb.isLimbInContact(rhand_rom,k)
    phase_d.LH_patch = phase_s.LH_patch
    phase_d.LH_patch.active = fb.isLimbInContact(lhand_rom,k)
    
    # now we change the contacts that have moved : 
    variations = fb.getContactsVariations(k-1,k)
    #assert len(variations)==1, "Several changes of contacts in adjacent states, not implemented yet !"
    for var in variations:     
        # FIXME : for loop in variation ? how ?
        if var == lleg_rom:
            phase_d.LF_patch.placement = MLF
        if var == rleg_rom:
            phase_d.RF_patch.placement = MRF
        if var == lhand_rom:
            phase_d.LH_patch.placement = MLH
        if var == rhand_rom:
            phase_d.RH_patch.placement = MRH
    # retrieve the COM position for init and final state (equal for double support phases)
    init_state = phase_d.init_state
    init_state[0:3] = np.matrix(fb.getCenterOfMass()).transpose()
    init_state[3:9] = np.matrix(configs[-1][-6:]).transpose()        
    phase_d.init_state=init_state
    phase_d.final_state=init_state
    phase_d.reference_configurations.append(np.matrix(pinnochioQuaternion(configs[-1][:-6])))  
    phase_d.time_trajectory.append(0.)
    
    if DISPLAY_CONTACTS and viewer:
        displayContactsFromPhase(phase_d,viewer)
        
        
    return cs


