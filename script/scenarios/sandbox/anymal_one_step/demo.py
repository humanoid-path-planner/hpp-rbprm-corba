import setup_one_step as sos


fsp = sos.fewStepPlanner #planner instance
q_init =  sos.q_init  #initial configuration
initState = sos.initState #initial state. 
viewer = sos.v

### Go somewhere

n_goal = q_init[:7][:]
n_goal[0] += 2
n_goal[1] += 1
n_goal[3:7] = [0.,0.,0.7071,0.7071]
states, configs = fsp.goToQuasiStatic(initState,n_goal, displayGuidePath = True)
#equivalent to
 # fsp.goToQuasiStatic(self, initState, n_goal, stepsize = 0.002, goalLimbsInContact = None, goalNormals = None, displayGuidePath = False)


#display computed States:
sos.dispContactPlan(states,0.051) #2nd argument is frame rateue


s = states[-1] #last state computed

#display configuration
viewer(s.q())

#some helpers: 
s.q() # configuration associated to state
#~ initState. # configuration associated to state
