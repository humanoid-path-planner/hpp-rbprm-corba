from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
from .disp_bezier import * 
import numpy as np
max_acc = 5.
pointsPerPhase=5
ROUND = 1000.

global curves_initGuess
global timings_initGuess 

curves_initGuess = []
timings_initGuess = []

    
def stdVecToMatrix(std_vector):
    vec_l = []
    for vec in std_vector:
        vec_l.append(vec)

    res = np.hstack(tuple(vec_l))
    return res


def compute_time_array(t_qp,t_c):
    t_discretized = []
    t_per_phase=[]
    t = 0
    for phase_id in range (len(t_qp)):
        dt = float(t_qp[phase_id]/pointsPerPhase)
        t_for_phase = []
        for i in range(pointsPerPhase):
            t += dt
            t = round(t*ROUND)/ROUND
            t = min(t,t_c)
            t_discretized.append(t)
            t_for_phase.append(t)
        t_per_phase.append(t_for_phase)
    print("test for timings : "+str(t_discretized))
    return t_discretized,t_per_phase

def check_acceleration_bounds(ddc_qp,t_discretized):
    min = [0,0,0]
    max = [0,0,0]
    for t in t_discretized:
        a = ddc_qp(t).transpose().tolist()[0]
        for i in range(3):
            if abs(a[i]) > max_acc:
                print("Acceleration sup to bound, at time : "+str(t)+"    a = "+str(a))
                return False
            if a[i] < min[i]:
                min[i] = a[i]
            if a[i] > max[i]:
                max[i] = a[i]
    print("acceleration between : "+str(min)+" ; "+str(max))
    return True;

def check_projection(s,c,t):
    return True
    success = s.projectToCOM(c(t).transpose().tolist()[0],500)
    if not success : 
        print("Unable to project to com at time : "+str(t))
        return False  
    else:
        return True

def check_stability(s,c,dc,ddc,t):
    q = s.q()
    q[-6:-3] =  dc(t).transpose().tolist()[0]    
    q[-3:]   = ddc(t).transpose().tolist()[0]
    success = s.fullBody.isConfigBalanced(q,s.getLimbsInContact(),CoM = c(t).transpose().tolist()[0])
    if not success : 
        print("UNSTABLE  at time : "+str(t))
        #print "q = ",q
        return False          
    else : 
        return True

def check_projection_path(s0,s1,c,dc,ddc,t_per_phase):
    kin_valid = True
    stab_valid = True
    moving_limb = s0.contactsVariations(s1)
    s0_orig = State(s0.fullBody,q=s0.q(),limbsIncontact=s0.getLimbsInContact())
    s1_orig = State(s1.fullBody,q=s1.q(),limbsIncontact=s1.getLimbsInContact())
    
    if len(moving_limb)>1:
        print("Too many contact variation between adjacent states")
        return False,False
    if len(moving_limb)==0:
        print("No contact between adjacent states")
        return True,True
    #print "test for phase 0 :"
    for t in t_per_phase[0]:
        if kin_valid and not check_projection(s0,c,t):
            kin_valid = False
        if stab_valid and not check_stability(s0_orig,c,dc,ddc,t):
            stab_valid = False
    smid,success = StateHelper.removeContact(s0,moving_limb[0])
    smid_orig,success_orig = StateHelper.removeContact(s0_orig,moving_limb[0])
    
    if not (success and success_orig):
        print("Error in creation of intermediate state")
        return False,False   
    #print "test for phase 1 :"
    for t in t_per_phase[1]:
        if kin_valid and not check_projection(smid,c,t):
            kin_valid =  False
        if stab_valid and not check_stability(smid_orig,c,dc,ddc,t):
            stab_valid = False            
    #print "test for phase 2"
    # go in reverse order for p2, it's easier for the projection : 
    for i in range(1,len(t_per_phase[2])+1):
        t = t_per_phase[2][-i]
        if kin_valid and not check_projection(s1,c,t):
            kin_valid = False
        if stab_valid and not check_stability(s1_orig,c,dc,ddc,t):
            stab_valid = False             

    return kin_valid,stab_valid



def check_one_transition(ps,fullBody,s0,s1,r=None,pp=None):
    global curves_initGuess
    global timings_initGuess 
    
    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True)
    if len(pid)==0:
        print("unable to compute trajectory")
        return False
    if r!=None and pp != None:
        showPath(r,pp,pid)
    c_qp = fullBody.getPathAsBezier(int(pid[0]))
    dc_qp = c_qp.compute_derivate(1)
    ddc_qp = dc_qp.compute_derivate(1)
    valid = True
    t_qp = [ps.pathLength(int(pid[1])), ps.pathLength(int(pid[2])), ps.pathLength(int(pid[3]))]  
    t_discretized,t_per_phase = compute_time_array(t_qp,c_qp.max())
    print("### test acceleration bounds ###")
    valid = valid and check_acceleration_bounds(ddc_qp,t_discretized)
    print("### test kinematic projection ###")
    valid = valid and check_projection_path(s0,s1,c_qp,dc_qp,ddc_qp,t_per_phase)
    
    curves_initGuess.append(c_qp)
    timings_initGuess.append(t_qp)
    return valid
 

 
def genTimeArray(Ts,dt):
    t = 0.
    res = []
    current_t = 0.
    for phase_id in range(len(Ts)):
        t_phase = []
        while t <= (current_t + Ts[phase_id]):
            t_phase += [t]
            t += dt
        res += [t_phase]
        current_t += Ts[phase_id]
    return res
    
def check_traj_valid(ps,fullBody,s0_,s1_,pIds,dt=0.001):
    # create copy of original states : 
    s0 = State(fullBody,q=s0_.q(),limbsIncontact=s0_.getLimbsInContact())
    s1 = State(fullBody,q=s1_.q(),limbsIncontact=s1_.getLimbsInContact())
    fullBody.setStaticStability(False)
    
    Ts = [ps.pathLength(int(pIds[1])), ps.pathLength(int(pIds[2])), ps.pathLength(int(pIds[3]))]  
    t_array = genTimeArray(Ts,dt)
    #print "Time array : ",t_array
    
    c_qp = fullBody.getPathAsBezier(int(pIds[0]))
    dc_qp = c_qp.compute_derivate(1)
    ddc_qp = dc_qp.compute_derivate(1)
    return check_projection_path(s0,s1,c_qp,dc_qp,ddc_qp,t_array)    
    
    
def check_muscod_traj(fullBody,cs,s0_,s1_):
    if cs.size() != 3 :
        print("Contact sequence must be of size 3 (one step)")
        return False,False
    kin_valid = True
    stab_valid = True
    stab_valid_discretized = True
    # create copy of original states : 
    s0 = State(fullBody,q=s0_.q(),limbsIncontact=s0_.getLimbsInContact())
    s1 = State(fullBody,q=s1_.q(),limbsIncontact=s1_.getLimbsInContact()) 
    moving_limb = s0.contactsVariations(s1)
    smid,success = StateHelper.removeContact(s0,moving_limb[0])
    if not success:
        print("Error in creation of intermediate state")
        return False,False   
    phases = []
    c_array = []
    dc_array = []
    ddc_array = []
    t_array = [] 
    states = [s0,smid,s1]
    states_copy = [State(fullBody,q=s0_.q(),limbsIncontact=s0_.getLimbsInContact()), State(fullBody,q=smid.q(),limbsIncontact=smid.getLimbsInContact()), State(fullBody,q=s1_.q(),limbsIncontact=s1_.getLimbsInContact()) ]
    for k in range(3) : 
        phases += [cs.contact_phases[k]]
        state_traj = stdVecToMatrix(phases[k].state_trajectory).transpose()
        control_traj = stdVecToMatrix(phases[k].control_trajectory).transpose() 
        c_array = state_traj[:,:3]
        dc_array = state_traj[:,3:6]
        ddc_array = control_traj[:,:3]
        t_array = stdVecToMatrix(phases[k].time_trajectory).transpose()    
    
        for i in range(len(c_array)):
            #if kin_valid and not states[k].projectToCOM(c_array[i].tolist()[0],500) :
            #    kin_valid = False
            q = states_copy[k].q()
            q[-6:-3] = dc_array[i].tolist()[0]    
            q[-3:]   = ddc_array[i].tolist()[0]
            if stab_valid_discretized and not states_copy[k].fullBody.isConfigBalanced(q,states_copy[k].getLimbsInContact(),CoM = c_array[i].tolist()[0]) :
                stab_valid_discretized = False  
        for i in range(int(len(c_array)/100)):
            q = states_copy[k].q()
            q[-6:-3] = dc_array[i*100].tolist()[0]    
            q[-3:]   = ddc_array[i*100].tolist()[0]
            if stab_valid and not states_copy[k].fullBody.isConfigBalanced(q,states_copy[k].getLimbsInContact(),CoM = c_array[i*100].tolist()[0]) :
                stab_valid = False              
        
    
    return kin_valid,stab_valid, stab_valid_discretized
   
    
    
    
    
 
def check_contact_plan(ps,r,pp,fullBody,idBegin,idEnd):
    fullBody.client.basic.robot.setExtraConfigSpaceBounds([-0,0,-0,0,-0,0,0,0,0,0,0,0])
    fullBody.setStaticStability(False)
    validPlan = True
    for id_state in range(idBegin,idEnd-1):
        print("#### check for transition between state "+str(id_state) +" and "+str(id_state+1))
        s0 = State(fullBody, sId = id_state)
        s1 = State(fullBody, sId = id_state + 1)
        # make a copy of each state because they are modified by the projection
        s0_ = State(fullBody,q = s0.q(),limbsIncontact=s0.getLimbsInContact())
        s1_ = State(fullBody,q = s1.q(),limbsIncontact=s1.getLimbsInContact())
        
        valid = check_one_transition(ps,fullBody,s0_,s1_,r,pp)        
        validPlan = validPlan and valid 
        
        global curves_initGuess
        global timings_initGuess 
        
    return validPlan, curves_initGuess,timings_initGuess




"""
    
fullBody.client.basic.robot.setExtraConfigSpaceBounds([-0,0,-0,0,-0,0,0,0,0,0,0,0])
fullBody.setStaticStability(False)
s0 = State(fullBody, sId = 0)
s1 = State(fullBody, sId = 1)
check_one_transition(s0,s1,r,pp)

    
    
pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True)
c_qp = fullBody.getPathAsBezier(int(pid[0]))
dc_qp = c_qp.compute_derivate(1)
ddc_qp = dc_qp.compute_derivate(1)
valid = True
t_qp = [ps.pathLength(int(pid[1])), ps.pathLength(int(pid[2])), ps.pathLength(int(pid[3]))]  
t_discretized,t_per_phase = compute_time_array(t_qp)

"""

"""

cs = ContactSequenceHumanoid(0)
cs.loadFromXML('/home/pfernbac/Documents/muscod/uc-dual/PROBLEM/random/res/contact_sequence_trajectory.xml',CONTACT_SEQUENCE_XML_TAG)


"""
