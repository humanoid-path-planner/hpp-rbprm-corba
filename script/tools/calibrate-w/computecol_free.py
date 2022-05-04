from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import sys

# ~ print sys.args

rootJointType = "freeflyer"
packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
urdfNameTested = "hrp2_trunk_flexible"
urdfNameRoms = ["hrp2_larm_rom", "hrp2_rarm_rom", "hrp2_lleg_rom", "hrp2_rleg_rom"]
urdfSuffix = ""
srdfSuffix = ""


scene = sys.argv[len(sys.argv) - 1]

tested = Builder()
tested.loadModel(
    urdfNameTested,
    urdfNameRoms,
    rootJointType,
    meshPackageName,
    packageName,
    urdfSuffix,
    srdfSuffix,
)


# ~ tested.setNormalFilter('hrp2_lleg_rom', [0,0,1], 0.9)
# ~ tested.setFilter(['hrp2_lleg_rom','hrp2_rleg_rom'])

ps = ProblemSolver(tested)
r = Viewer(ps)
r.loadObstacleModel(packageName, scene, "planning")
tested.setJointBounds("base_joint_xyz", [-10.0, 10, -10, 10, 0, 20])
ps.client.problem.selectConFigurationShooter("RbprmShooter")

q_init = tested.getCurrentConfig()
q_init[0:3] = [-10, -0.82, 1.25]
tested.setCurrentConfig(q_init)
r(q_init)
q_goal = q_init[::]
q_goal[0:3] = [-9, -0.65, 1.25]
r(q_goal)

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)
t = ps.solve()

res = {}
x_start = 0
y_start = 0
x_max = 2
y_max = 1.64
iter_step = 0.01

res = {}
x_start = -1.5
y_start = 0
x_max = 2.84
y_max = 2.65
iter_step = 0.01

res = {}

import numpy as np

nbStepsX = int((x_max - x_start) / iter_step)
nbStepsY = int((y_max - y_start) / iter_step)

x_t = []
y_t = []

for x in np.linspace(x_start, x_max, num=nbStepsX):
    ys = {}
    for y in np.linspace(y_start, y_max, num=nbStepsY):
        q = q_init
        q[0] = x
        q[1] = -0.82
        q[1] = 0
        q[2] = y
        if tested.isReachable(q):
            x_t.append(x)
            y_t.append(y)
            ys[y] = True
    res[x] = ys

x_t.append(0)
y_t.append(0)  # for scale

# ~ x_t.append(0.25)
# ~ y_t.append(0)
# ~
# ~ x_t.append(0.3)
# ~ y_t.append(0.15)
# ~ x_t.append(0.6)
# ~ y_t.append(0.15)
# ~
# ~ x_t.append(0.6)
# ~ y_t.append(0.30)
# ~ x_t.append(0.9)
# ~ y_t.append(0.30)
# ~
# ~ x_t.append(0.9)
# ~ y_t.append(0.45)
# ~ x_t.append(1.2)
# ~ y_t.append(0.45)
# ~
# ~ x_t.append(1.2)
# ~ y_t.append(0.6)
# ~ x_t.append(1.80)
# ~ y_t.append(0.6)
# ~
# ~ x_t.append(0.4)
# ~ y_t.append(0.81)
# ~ x_t.append(2)
# ~ y_t.append(1.6)

# ~ x_t.append(2.5)
# ~ y_t.append(2.65) #for scale
# ~
# ~ x_t.append(-1.5)
# ~ y_t.append(0) #for scale

# ~ x_t.append(-0.87)
# ~ y_t.append(0.2) #for scale
# ~ x_t.append(-0.67)
# ~ y_t.append(0.2) #for scale
# ~
# ~ x_t.append(-0.23)
# ~ y_t.append(0.4) #for scale
# ~ x_t.append(-0.03)
# ~ y_t.append(0.4) #for scale
# ~
# ~ x_t.append(0.77)
# ~ y_t.append(0.6) #for scale
# ~ x_t.append(0.97)
# ~ y_t.append(0.6) #for scale
# ~
# ~ x_t.append(2.2)
# ~ y_t.append(1) #for scale
# ~ x_t.append(2.4)
# ~ y_t.append(1) #for scale
# ~
# ~ x_t.append(0.62)
# ~ y_t.append(2.35) #for scale
# ~ x_t.append(1.12)
# ~ y_t.append(2.35) #for scale


import pickle

sFile = "creach_2DGrid_" + scene + ".pkl"
output = open(sFile, "wb")
pickle.dump(res, output)
output.close()

q[2] = y_t[0]
q[0] = x_t[0]
r(q)


import numpy as np
import matplotlib.pyplot as plt

# ~ plt.scatter(x_t, y_t, s=170, marker='s', edgecolors='none')
plt.scatter(x_t, y_t, edgecolors="none")
# ~ plt.scatter(x_t, y_t ,color ='c', edgecolors='none')
plt.show()
