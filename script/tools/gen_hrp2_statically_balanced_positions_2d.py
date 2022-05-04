from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import numpy as np

# from hpp.corbaserver.rbprm.tools.cwc_trajectory import *

from hpp import Error as hpperr
from numpy import array, matrix


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
fullBody.setJointBounds("base_joint_xyz", [-1.5, 1.5, -1, 1, 0.45, 0.8])


ps = ProblemSolver(fullBody)
r = Viewer(ps)


rLegId = "hrp2_rleg_rom"
lLegId = "hrp2_lleg_rom"
rarmId = "hrp2_rarm_rom"
larmId = "hrp2_larm_rom"
rLeg = "RLEG_JOINT0"
rLegOffset = [0, 0, -0.105]
rLegLimbOffset = [0, 0, -0.035]  # 0.035
rLegNormal = [0, 0, 1]
rLegx = 0.09
rLegy = 0.05
# fullBody.addLimbDatabase("./db/hrp2_rleg_db.db",rLegId,"forward")
fullBody.addLimb(
    rLegId,
    rLeg,
    "",
    rLegOffset,
    rLegNormal,
    rLegx,
    rLegy,
    100000,
    "manipulability",
    0.01,
    "_6_DOF",
    limbOffset=rLegLimbOffset,
)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
# fullBody.saveLimbDatabase(rLegId, "./db/hrp2_rleg_db.db")

lLeg = "LLEG_JOINT0"
lLegOffset = [0, 0, -0.105]
lLegLimbOffset = [0, 0, 0.035]
lLegNormal = [0, 0, 1]
lLegx = 0.09
lLegy = 0.05
# fullBody.addLimbDatabase("./db/hrp2_lleg_db.db",lLegId,"forward")
fullBody.addLimb(
    lLegId,
    lLeg,
    "",
    lLegOffset,
    rLegNormal,
    lLegx,
    lLegy,
    100000,
    "manipulability",
    0.01,
    "_6_DOF",
    limbOffset=lLegLimbOffset,
)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
# fullBody.saveLimbDatabase(lLegId, "./db/hrp2_lleg_db.db")
q_ref = [
    0.0,
    0.0,
    0.648702,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.261799388,
    0.174532925,
    0.0,
    -0.523598776,
    0.0,
    0.0,
    0.17,
    0.261799388,
    -0.174532925,
    0.0,
    -0.523598776,
    0.0,
    0.0,
    0.17,
    0.0,
    0.0,
    -0.453785606,
    0.872664626,
    -0.41887902,
    0.0,
    0.0,
    0.0,
    -0.453785606,
    0.872664626,
    -0.41887902,
    0.0,
]
fullBody.setReferenceConfig(q_ref)


limbsCOMConstraints = {
    rLegId: {"file": "hrp2/RL_com.ineq", "effector": "RLEG_JOINT5"},
    lLegId: {"file": "hrp2/LL_com.ineq", "effector": "LLEG_JOINT5"},
    larmId: {"file": "hrp2/LA_com.ineq", "effector": "LARM_JOINT5"},
    rarmId: {"file": "hrp2/RA_com.ineq", "effector": "RARM_JOINT5"},
}
# ~ limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},
# ~ lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'},
# ~ rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand} }

# ~

# ~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
# ~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])

from . import quaternion as quat


def _getTransform(qEffector):
    q0 = quat.Quaternion(qEffector[3:7])
    rot = q0.toRotationMatrix()  # compute rotation matrix local -> world
    p = qEffector[0:3]  # (0,0,0) coordinate expressed in effector fram
    rm = np.zeros((4, 4))
    for k in range(0, 3):
        for l in range(0, 3):
            rm[k, l] = rot[k, l]
    for m in range(0, 3):
        rm[m, 3] = qEffector[m]
    rm[3, 3] = 1
    return rm


from numpy.linalg import norm


def __loosely_z_aligned(limb, config):
    fullBody.setCurrentConfig(config)
    effectorName = limbsCOMConstraints[limb]["effector"]
    m = _getTransform(fullBody.getJointPosition(effectorName))
    P, N = fullBody.computeContactForConfig(config, limb)
    # ~ N_world = m.dot(array(N[0]+[1]))[:3]
    N_world = m[:3, :3].dot(array(N[0]))
    N_world = N_world / np.linalg.norm(N_world)
    return N_world.dot(array([0, 0, 1])) > 0.7


def draw_cp(cid, limb, config):
    global r
    # ~ posetc = fullBody.getEffectorPosition(limb, config)
    P, N = fullBody.computeContactForConfig(config, limb)
    fullBody.setCurrentConfig(config)
    effectorName = limbsCOMConstraints[limb]["effector"]
    limbId = limb
    m = _getTransform(fullBody.getJointPosition(effectorName))
    scene = "qds" + limb + str(cid)
    r.client.gui.createScene(scene)
    for i in range(4):
        # ~ pos = posetc[2*i]
        print("P", array(P[i] + [1]))
        print("N", array(N[i] + [1]))
        print(m.dot(array(P[i] + [1])))
        pos = m.dot(array(P[i] + [1]))[:3]
        print("pos", pos)
        r.client.gui.addBox(scene + "/b" + str(i), 0.01, 0.01, 0.01, [1, 0, 0, 1])
        r.client.gui.applyConfiguration(
            scene + "/b" + str(i), pos.tolist() + [1, 0, 0, 0]
        )
        r.client.gui.refresh()
    r.client.gui.addSceneToWindow(scene, 0)


def fill_contact_points(limbs, config, config_pinocchio):
    res = {}
    res["q"] = config_pinocchio[:]
    res["contact_points"] = {}
    res["P"] = []
    res["N"] = []
    for limb in limbs:
        effector = limbsCOMConstraints[limb]["effector"]
        # ~ posetc = fullBody.getEffectorPosition(limb, config)
        P, N = fullBody.computeContactForConfig(config, limb)
        # ~ posetc = fullBody.getEffectorPosition(limb, config)
        res["contact_points"][effector] = {}
        # ~ res["contact_points"][effector]["P"] = [p for i, p in enumerate (posetc) if (i%2 == 0)]
        res["contact_points"][effector]["P"] = P
        # ~ res["P"] += [p for i, p in enumerate (posetc) if (i%2 == 0)]
        res["P"] += P
        # ~ res["contact_points"][effector]["N"] = [n for i, n in enumerate (posetc) if (i%2 == 1)]
        res["contact_points"][effector]["N"] = N
        # ~ res["N"] += [n for i, n in enumerate (posetc) if (i%2 == 1)]
        res["N"] += N
    return res


from numpy import cos, sin, pi


def __eulerToQuat(pitch, roll, yaw):
    t0 = cos(yaw * 0.5)
    t1 = sin(yaw * 0.5)
    t2 = cos(roll * 0.5)
    t3 = sin(roll * 0.5)
    t4 = cos(pitch * 0.5)
    t5 = sin(pitch * 0.5)
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    return [w, x, y, z]
    # ~


# ~ void SampleRotation(model::DevicePtr_t so3, ConfigurationPtr_t config, JointVector_t& jv)
# ~ {
# ~ std::size_t id = 1;
# ~ if(so3->rootJoint())
# ~ {
# ~ Eigen::Matrix <value_type, 3, 1> confso3;
# ~ id+=1;
# ~ model::JointPtr_t joint = so3->rootJoint();
# ~ for(int i =0; i <3; ++i)
# ~ {
# ~ joint->configuration()->uniformlySample (i, confso3);
# ~ }
# ~ Eigen::Quaterniond qt = Eigen::AngleAxisd(confso3(0), Eigen::Vector3d::UnitZ())
# ~ * Eigen::AngleAxisd(confso3(1), Eigen::Vector3d::UnitY())
# ~ * Eigen::AngleAxisd(confso3(2), Eigen::Vector3d::UnitX());
# ~ std::size_t rank = 3;
# ~ (*config)(rank+0) = qt.w();
# ~ (*config)(rank+1) = qt.x();
# ~ (*config)(rank+2) = qt.y();
# ~ (*config)(rank+3) = qt.z();
# ~ }
# ~ if(id < jv.size())
# ~ SampleRotationRec(config,jv,id);
# ~ }

from random import uniform


def _boundSO3(q, num_limbs):
    q[:3] = [0, 0, 0.5]
    limb_weight = float(4 - num_limbs)
    # generate random angle
    rot_y = uniform(-pi / (4 + limb_weight), pi / (4 + limb_weight))
    rot_x = uniform(-pi / 8, pi / (3 + limb_weight))

    rot_z = 0
    q[3:7] = __eulerToQuat(rot_x, rot_y, rot_z)
    return q


def _genbalance(limbs):
    for i in range(10000):
        q = fullBody.client.basic.robot.shootRandomConfig()
        q = _boundSO3(q, len(limbs))
        if (
            fullBody.isConfigValid(q)[0]
            and fullBody.isConfigBalanced(q, limbs, 5)
            and __loosely_z_aligned(limbs[0], q)
            and __loosely_z_aligned(limbs[1], q)
        ):
            # ~ if fullBody.isConfigValid(q)[0] and  __loosely_z_aligned(limbs[0], q) and __loosely_z_aligned(limbs[1], q):
            return q
    print("can't generate equilibrium config")


all_qs = []


def gen(limbs, num_samples=1000, coplanar=True):
    q_0 = fullBody.getCurrentConfig()
    # ~ fullBody.getSampleConfig()
    qs = []
    qs_gepetto = []
    states = []
    for _ in range(num_samples):
        if coplanar:
            q = fullBody.generateGroundContact(limbs)
        else:
            q = _genbalance(limbs)
        q_gep = q[:]
        quat_end = q[4:7]
        q[6] = q[3]
        q[3:6] = quat_end
        qs.append(q)
        qs_gepetto.append(q_gep)
        states.append(fill_contact_points(limbs, q_gep, q))
    global all_qs
    all_qs += [qs_gepetto]
    fname = ""
    for lname in limbs:
        fname += lname + "_"
    fname += "configs"
    if coplanar:
        fname += "_coplanar"
    from pickle import dump

    # ~ f1=open("configs_feet_on_ground_static_eq", 'w+')
    f1 = open(fname, "w+")
    dump(states, f1)
    print("export to file : " + fname)
    f1.close()


j = 0

q_init = [
    0.0,
    0.0,
    0.648702,
    1.0,
    0.0,
    0.0,
    0.0,  # Free flyer 0-6
    0.0,
    0.0,
    0.0,
    0.0,  # CHEST HEAD 7-10
    0.261799388,
    0.174532925,
    0.0,
    -0.523598776,
    0.0,
    0.0,
    0.17,  # LARM       11-17
    0.261799388,
    -0.174532925,
    0.0,
    -0.523598776,
    0.0,
    0.0,
    0.17,  # RARM       18-24
    0.0,
    0.0,
    -0.453785606,
    0.872664626,
    -0.41887902,
    0.0,  # LLEG       25-30
    0.0,
    0.0,
    -0.453785606,
    0.872664626,
    -0.41887902,
    0.0,  # RLEG       31-36
]
r(q_init)

limbs = [[lLegId, rLegId]]
# ~ limbs = [[lLegId,rLegId, rarmId]]
# ~ limbs = [[larmId, rarmId]]

# ~ gen(limbs[0], 10)

"""
for ls in limbs:
	gen(ls, 1000, False)
gen(limbs[0], 1000)
"""

gen(limbs[0], 1000, False)

i = 0
