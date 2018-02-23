from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
from disp_bezier import * 
max_acc = 1
pointsPerPhase=4
ROUND = 1000.

global curves_initGuess
global timings_initGuess 

curves_initGuess = []
timings_initGuess = []



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
    print "test for timings : "+str(t_discretized)
    return t_discretized,t_per_phase

def check_acceleration_bounds(ddc_qp,t_discretized):
    min = [0,0,0]
    max = [0,0,0]
    for t in t_discretized:
        a = ddc_qp(t).transpose().tolist()[0]
        for i in range(3):
            if abs(a[i]) > max_acc:
                print "Acceleration sup to bound, at time : "+str(t)+"    a = "+str(a)
                return False
            if a[i] < min[i]:
                min[i] = a[i]
            if a[i] > max[i]:
                max[i] = a[i]
    print "acceleration between : "+str(min)+" ; "+str(max)
    return True;

def check_projection(s,c,dc,ddc,t):
    success = s.projectToCOM(c(t).transpose().tolist()[0],500)
    if not success : 
        print "Unable to project to com at time : "+str(t)
        return False  
    # check dynamic stability
    q = s.q()
    q[-6:-3] =  dc(t).transpose().tolist()[0]    
    q[-3:]   = ddc(t).transpose().tolist()[0]
    success = s.fullBody.isConfigBalanced(q,s.getLimbsInContact())
    if not success : 
        print "UNSTABLE  at time : "+str(t)
        return False      
    return True

def check_projection_path(s0,s1,c,dc,ddc,t_qp,t_per_phase):
    moving_limb = s0.contactsVariations(s1)
    if len(moving_limb)>1:
        print "Too many contact variation between adjacent states"
        return False
    if len(moving_limb)==0:
        print "No contact between adjacent states"
        return True
    print "test for phase 0 :"
    for t in t_per_phase[0]:
        if not check_projection(s0,c,dc,ddc,t):
            return False
    smid,success = StateHelper.removeContact(s0,moving_limb[0])
    if not success:
        print "Error in creation of intermediate state"
        return False    
    print "test for phase 1 :"
    for t in t_per_phase[1]:
        #if t == t_per_phase[0][-1]:
         #   t -= EPS  
        if not check_projection(smid,c,dc,ddc,t):
            return False
    print "test for phase 2"
    # go in reverse order for p2, it's easier for the projection : 
    for i in range(1,len(t_per_phase[2])+1):
        t = t_per_phase[1][-i]
        if not check_projection(s1,c,dc,ddc,t):
            return False
 
    t = t_per_phase[1][-1]
    if not check_projection(s1,c,dc,ddc,t):
        return False
  
    return True
    
        

def check_one_transition(ps,fullBody,s0,s1,r=None,pp=None):
    global curves_initGuess
    global timings_initGuess 
    
    pid = fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,True)
    if len(pid)==0:
        print "unable to compute trajectory"
        return False
    if r!=None and pp != None:
        showPath(r,pp,pid)
    c_qp = fullBody.getPathAsBezier(int(pid[0]))
    dc_qp = c_qp.compute_derivate(1)
    ddc_qp = dc_qp.compute_derivate(1)
    valid = True
    t_qp = [ps.pathLength(int(pid[1])), ps.pathLength(int(pid[2])), ps.pathLength(int(pid[3]))]  
    t_discretized,t_per_phase = compute_time_array(t_qp,c_qp.max())
    print "### test acceleration bounds ###"
    valid = valid and check_acceleration_bounds(ddc_qp,t_discretized)
    print "### test kinematic projection ###"
    valid = valid and check_projection_path(s0,s1,c_qp,dc_qp,ddc_qp,t_qp,t_per_phase)
    
    curves_initGuess.append(c_qp)
    timings_initGuess.append(t_qp)
    return valid
  
        
   
 
def check_contact_plan(ps,r,pp,fullBody,idBegin,idEnd):
    fullBody.client.basic.robot.setExtraConfigSpaceBounds([-0,0,-0,0,-0,0,0,0,0,0,0,0])
    fullBody.setStaticStability(False)
    validPlan = True
    for id_state in range(idBegin,idEnd-1):
        print "#### check for transition between state "+str(id_state) +" and "+str(id_state+1)
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