# Importing helper class for setting up a reachability planning problem
from hpp.corbaserver.rbprm.hyq_abstract import Robot

# Importing Gepetto viewer helper class
from hpp.gepetto import Viewer


# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
rbprmBuilder.setJointBounds ("root_joint", [-2,5, -1, 1, 0.3, 4])
# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact ...
rbprmBuilder.setFilter(['hyq_rhleg_rom', 'hyq_lfleg_rom', 'hyq_rfleg_rom','hyq_lhleg_rom'])
rbprmBuilder.setAffordanceFilter('hyq_rhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_rfleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('hyq_lhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_lfleg_rom', ['Support',])
# We also bound the rotations of the torso.
rbprmBuilder.boundSO3([-0.4,0.4,-0.3,0.3,-0.3,0.3])

# Creating an instance of HPP problem solver and the viewer
from hpp.corbaserver.problem_solver import ProblemSolver
ps = ProblemSolver( rbprmBuilder )
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)


from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
#~ afftool.loadObstacleModel (packageName, "darpa", "planning", r, reduceSizes=[0.05,0.,0.])
afftool.loadObstacleModel("hpp_environments", "multicontact/darpa", "planning", vf,reduceSizes=[0.1,0,0])
v = vf.createViewer()
#afftool.visualiseAffordances('Support', v, [0.25, 0.5, 0.5])



# Setting initial and goal configurations
q_init = rbprmBuilder.getCurrentConfig ();
q_init [0:3] = [-2, 0, 0.64]; rbprmBuilder.setCurrentConfig (q_init); v (q_init)
q_goal = q_init [::]
q_goal [0:3] = [3, 0, 0.64]; v (q_goal)
#~ q_goal [0:3] = [-1.5, 0, 0.75]; r (q_goal)

# Choosing a path optimizer
ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# Choosing RBPRM shooter and path validation methods.
# Note that the standard RRT algorithm is used.
ps.client.problem.selectConfigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)

# Solve the problem
t = ps.solve ()
#~ t = 0.
if isinstance(t, list):
	t = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]	

print("computation time for root path ", t)

# Playing the computed path
from hpp.gepetto import PathPlayer
pp = PathPlayer (v)

q_far = q_init [::]
q_far [0:3] = [-2, -3, 0.6]; 
v(q_far)

for i in range(1,10):
	rbprmBuilder.client.problem.optimizePath(i)
        
#~ pp(9)
	

from hpp.corbaserver import Client
 #DEMO code to play root path and final contact plan
cl = Client()
cl.problem.selectProblem("rbprm_path")
rbprmBuilder2 = Robot ("toto")
ps2 = ProblemSolver( rbprmBuilder2 )
cl.problem.selectProblem("default")
cl.problem.movePathToProblem(8,"rbprm_path",rbprmBuilder.getAllJointNames()[1:])
r2 = Viewer (ps2, viewerClient=v.client)
r2(q_far)

