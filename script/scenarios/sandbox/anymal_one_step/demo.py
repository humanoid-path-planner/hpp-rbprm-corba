import setup_one_step as sos


fsp = sos.fewStepPlanner #planner instance
q_init =  sos.q_init  #initial configuration
initState = sos.initState #initial state. 
viewer = sos.v

### Go somewhere

n_goal = q_init[:7][:]
n_goal[0] += 3
n_goal[1] += 0
#~ n_goal[2] = [0.,0.,0.7071,0.7071]
#~ n_goal[3:7] = [0.,0.,0.7071,0.7071]
#~ sos.fullBody.toggleNonContactingLimb(sos.fullBody.prongFrontId)
states, configs = fsp.goToQuasiStatic(initState,n_goal, displayGuidePath = False)
#~ pId = fsp.guidePath(initState.q()[:7], n_goal, displayPath = True)
#~ fsp.setPlanningContext()
#~ fsp.pathPlayer(pId)
#equivalent to
 # fsp.goToQuasiStatic(self, initState, n_goal, stepsize = 0.002, goalLimbsInContact = None, goalNormals = None, displayGuidePath = False)


#display computed States:

n_goal[3:7] = [0.,0.,0.7071,0.7071]

s = states[-1] #last state computed
states2, configs2 = fsp.goToQuasiStatic(s,n_goal, displayGuidePath = False)

#display configuration
viewer(s.q())

states+=states2
configs+=configs2

configs = [q + [0. for _ in range(6)] for q in configs]

sos.dispContactPlan(states,0.051) #2nd argument is frame rateue

#some helpers: 
s.q() # configuration associated to state
#~ initState. # configuration associated to state

from hpp.corbaserver.rbprm import rbprmstate
target = sos.fullBody.getJointPosition(sos.fullBody.prongFrontId)[:3]
target[2] = 0.


s = rbprmstate.StateHelper.cloneState(states[-1])[0]
fb = sos.fullBody

#~ sos.fullBody.toggleNonContactingLimb(sos.fullBody.prongFrontId)
#~ rbprmstate.StateHelper.addNewContact(s,sos.fullBody.prongFrontId,target,[0.,0.,1.])
