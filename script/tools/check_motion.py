from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = "_reduced"
srdfSuffix = ""

fullBody = FullBody()

fullBody.loadFullBodyModel(
    urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix
)

ps = ProblemSolver(fullBody)
r = Viewer(ps)

justin = "/home/stonneau/dev/justin/hrp2-motions/standup/"


def check(csv):
    qs = []
    file = open(justin + csv, "r+")
    # first retrieve frame range
    for line in file.readlines():
        objs = line.rstrip("\n").split(" ")
        objs.pop(0)
        q = fullBody.getCurrentConfig()
        for i in range(0, len(q) - 1):
            q[i + 7] = float(objs[i])
            qs.append(q)
    return qs


configs = check("standup_2015_11_16_16_23_35.pos")
