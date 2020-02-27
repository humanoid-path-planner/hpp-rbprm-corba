from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.tools.display_tools import displayContactSequence
from hpp.corbaserver import ProblemSolver
import time
from abc import abstractmethod

class AbstractContactGenerator:

    fullbody = None
    ps = None
    v = None
    q_ref = None
    weight_postural = None
    q_init = None
    q_goal = None

    def __init__(self, path_planner):
        """
        Constructor, run the guide path and save the results
        :param path_planner: an instance of a child class of AbstractPathPlanner
        """
        path_planner.run()
        self.path_planner = path_planner
        self.pid = self.path_planner.ps.numberPaths() - 1
        self.used_limbs = path_planner.used_limbs
        self.dt = 0.01
        self.robustness = 0
        self.filter_states = True
        self.test_reachability = True
        self.quasi_static = True
        self.erase_previous_states = True
        self.init_contacts = self.used_limbs
        self.end_contacts = self.used_limbs
        self.configs = []

    @abstractmethod
    def load_fullbody(self):
        pass

    def set_joints_bounds(self):
        self.fullbody.setJointBounds ("root_joint", self.path_planner.root_translation_bounds)
        self.fullbody.client.robot.setDimensionExtraConfigSpace(self.path_planner.extra_dof)
        self.fullbody.client.robot.setExtraConfigSpaceBounds(self.path_planner.extra_dof_bounds)

    def set_reference(self, use_postural_task = True):
        self.fullbody.setReferenceConfig(self.q_ref)
        self.fullbody.setCurrentConfig(self.q_ref)
        self.fullbody.setPostureWeights(self.weight_postural)
        self.fullbody.usePosturalTaskContactCreation(use_postural_task)

    def load_limbs(self, heuristic = "fixedStep06", analysis=None, nb_samples=None, octree_size=None):
        self.fullbody.limbs_names = self.used_limbs
        if nb_samples is None:
            nb_samples = self.fullbody.nbSamples
        if octree_size is None:
            octree_size = self.fullbody.octreeSize
        print("Generate limb DB ...")
        t_start = time.time()
        self.fullbody.loadAllLimbs(heuristic, analysis, nb_samples, octree_size)
        t_generate = time.time() - t_start
        print("Databases generated in : " + str(t_generate) + " s")


    def init_problem(self):
        self.ps = ProblemSolver(self.fullbody)
        if self.path_planner.v_max >= 0:
            self.ps.setParameter("Kinodynamic/velocityBound", self.path_planner.v_max)
        if self.path_planner.a_max >= 0:
            self.ps.setParameter("Kinodynamic/accelerationBound", self.path_planner.a_max)

    def init_viewer(self):
        self.v = Viewer(self.ps, viewerClient=self.path_planner.v.client, displayCoM=True)

    def compute_configs_from_guide(self, use_acc_init = True, use_acc_end = False):
        self.q_init = self.q_ref[::]
        self.q_init[0:7] = self.path_planner.ps.configAtParam(self.pid, 0.001)[0:7]
        # do not use 0 but an epsilon in order to avoid the orientation discontinuity that may happen at t=0
        self.q_goal = self.q_init[::]
        self.q_goal[0:7] = self.path_planner.ps.configAtParam(self.pid, self.path_planner.ps.pathLength(self.pid))[0:7]

        # copy extraconfig for start and init configurations
        configSize = self.fullbody.getConfigSize() - self.fullbody.client.robot.getDimensionExtraConfigSpace()
        q_init_guide = self.path_planner.ps.configAtParam(self.pid, 0)
        q_goal_guide = self.path_planner.ps.configAtParam(self.pid, self.path_planner.ps.pathLength(self.pid))
        index_ecs = len(q_init_guide) - self.path_planner.extra_dof
        self.q_init[configSize : configSize + 3] = q_init_guide[index_ecs : index_ecs + 3]
        if use_acc_init:
            self.q_init[configSize + 3 : configSize + 6] = q_init_guide[index_ecs + 3 : index_ecs + 6]
        else:
            self.q_init[configSize + 3 : configSize + 6] = [0, 0, 0]
        self.q_goal[configSize : configSize + 3] = q_goal_guide[index_ecs : index_ecs + 3]
        if use_acc_end:
            self.q_goal[configSize + 3 : configSize + 6] = q_goal_guide[index_ecs + 3 : index_ecs + 6]
        else:
            self.q_goal[configSize + 3 : configSize + 6] = [0, 0, 0]

    def interpolate(self):
        # specify the full body configurations as start and goal state of the problem
        self.fullbody.setStartState(self.q_init, self.init_contacts)
        self.fullbody.setEndState(self.q_goal, self.end_contacts)

        print("Generate contact plan ...")
        t_start = time.time()
        self.configs = self.fullbody.interpolate(0.01,
                                            pathId=self.pid,
                                            robustnessTreshold=self.robustness,
                                            filterStates=self.filter_states,
                                            testReachability=self.test_reachability,
                                            quasiStatic=self.quasi_static,
                                            erasePreviousStates=self.erase_previous_states)
        t_interpolate_configs = time.time() - t_start
        print("Contact plan generated in : " + str(t_interpolate_configs) + " s")
        print("number of configs :", len(self.configs))

    def display_sequence(self):
        displayContactSequence(self.v, self.configs)

    @abstractmethod
    def run(self):
        """
        Must be defined in the child class to run all the methods with the correct arguments.
        """
        # example of definition:
        """
        self.load_fullbody()
        self.set_joints_bounds()
        self.set_reference(True)
        self.load_limbs("fixedStep06")
        self.init_problem()
        self.init_viewer()
        self.compute_configs_from_guide()
        # force root height to be at the reference position:
        self.q_init[2] = self.q_ref[2]
        self.q_goal[2] = self.q_ref[2]
        self.interpolate()
        """
        pass
