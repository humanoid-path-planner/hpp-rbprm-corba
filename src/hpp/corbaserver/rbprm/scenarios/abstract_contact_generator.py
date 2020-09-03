from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.tools.display_tools import displayContactSequence
from hpp.corbaserver import ProblemSolver, Client
import time
from abc import abstractmethod


class AbstractContactGenerator:

    fullbody = None  # rborpm.FullBody instance
    ps = None  # ProblemSolver instance
    v = None  # gepetto.Viewer instance
    q_ref = None  # reference configuration used (depending on the settings)
    weight_postural = None  # weight used for the postural task (depending on the setting)
    q_init = None  # Initial whole body configuration
    q_goal = None  # Desired final whole body configuration
    robot_node_name = None  # name of the robot in the node list of the viewer

    def __init__(self, path_planner):
        """
        Constructor, run the guide path and save the results
        :param path_planner: an instance of a child class of AbstractPathPlanner
        """
        path_planner.run()
        self.path_planner = path_planner
        self.path_planner.hide_rom()
        # ID of the guide path used in problemSolver, default to the last one
        self.pid = self.path_planner.ps.numberPaths() - 1
        # Save the guide planning problem in a specific instance,
        # such that we can use it again even after creating the "fullbody" problem:
        self.cl = Client()
        self.cl.problem.selectProblem("guide_planning")
        path_planner.load_rbprm()
        ProblemSolver(path_planner.rbprmBuilder)
        self.cl.problem.selectProblem("default")
        self.cl.problem.movePathToProblem(self.pid, "guide_planning",
                                          self.path_planner.rbprmBuilder.getAllJointNames()[1:])
        # copy bounds and limbs used from path planning :
        self.used_limbs = path_planner.used_limbs
        self.root_translation_bounds = self.path_planner.root_translation_bounds
        # increase bounds from path planning, to leave room for the root motion during the steps
        for i in range(3):
            self.root_translation_bounds[2 * i] -= 0.1
            self.root_translation_bounds[2 * i + 1] += 0.1
        # settings related to the 'interpolate' method:
        self.dt = 0.01  # discretisation step used
        self.robustness = 0  # robustness treshold
        # (see https://github.com/humanoid-path-planner/hpp-centroidal-dynamics/blob/master/include/hpp/centroidal-dynamics/centroidal_dynamics.hh#L215)
        self.filter_states = True  # if True, after contact generation try to remove all the redundant steps
        self.test_reachability = True  # if True, check feasibility of the contact transition during contact generation
        self.quasi_static = True  # if True, use the 2-PAC method to check feasibility, if False use CROC
        self.erase_previous_states = True  # if False, keep^the previously computed states if 'interpolate' is called a second time
        self.static_stability = True  # if True, the acceleration computed by the guide is ignored during contact planning
        self.init_contacts = self.used_limbs  # limbs in contact in the initial configuration
        # the order of the limbs in the list define the initial order in which the contacts are removed when then cannot be maintained anymore
        self.end_contacts = self.used_limbs  # limbs in contact in the final configuration
        self.configs = []  # will contains the whole body configuration computed after calling 'interpolate'

    @abstractmethod
    def load_fullbody(self):
        """
        Load the robot from urdf and instanciate a fullBody object
        Also initialize the q_ref and weight_postural vectors
        """
        pass

    def set_joints_bounds(self):
        """
        Define the root translation bounds and the extra config bounds
        """
        self.fullbody.setJointBounds("root_joint", self.root_translation_bounds)
        self.fullbody.client.robot.setDimensionExtraConfigSpace(self.path_planner.extra_dof)
        self.fullbody.client.robot.setExtraConfigSpaceBounds(self.path_planner.extra_dof_bounds)

    def set_reference(self, use_postural_task=True):
        """
        Set the reference configuration used and the weight for the postural task
        :param use_postural_task: if True, when projecting configurations to contact a postural task is added to the cost function
        Disabling this setting will greatly reduce the computation time, but may result in weird configurations in contact
        """
        self.fullbody.setReferenceConfig(self.q_ref)
        self.fullbody.setCurrentConfig(self.q_ref)
        self.fullbody.setPostureWeights(self.weight_postural)
        self.fullbody.usePosturalTaskContactCreation(use_postural_task)

    def set_rom_filters(self):
        """
        Define which type of affordance should be considered to create contact with each limb
        By default, use "Support" type for all limbs
        """
        for limb in self.used_limbs:
            self.path_planner.rbprmBuilder.setAffordanceFilter(limb, ['Support'])

    def load_limbs(self, heuristic="fixedStep06", analysis="ReferenceConfiguration", nb_samples=None, octree_size=None,
                   disableEffectorCollision=False):
        """
        Generate the samples used for each limbs in 'used_limbs'
        :param heuristic: the name of the heuristic used,
        see https://github.com/humanoid-path-planner/hpp-rbprm/blob/master/src/sampling/heuristic.cc#L272-L285
        :param analysis: The name of the analysis used,
        see https://github.com/humanoid-path-planner/hpp-rbprm/blob/master/src/sampling/analysis.cc#L318-L335
        :param nb_samples: The number of samples for each limb database. Default is set in the Robot python class
        :param octree_size: The size of each cell of the octree. Default is set in the Robot python class
        """
        self.set_rom_filters()
        self.fullbody.limbs_names = self.used_limbs
        if nb_samples is None:
            nb_samples = self.fullbody.nbSamples
        if octree_size is None:
            octree_size = self.fullbody.octreeSize
        print("Generate limb DB ...")
        t_start = time.time()
        self.fullbody.loadAllLimbs(heuristic, analysis, nb_samples, octree_size,
                                   disableEffectorCollision=disableEffectorCollision)
        t_generate = time.time() - t_start
        print("Databases generated in : " + str(t_generate) + " s")

    def init_problem(self):
        """
        Create a ProblemSolver instance, and set the velocity and acceleration bounds
        """
        self.ps = ProblemSolver(self.fullbody)
        if self.path_planner.v_max >= 0:
            self.ps.setParameter("Kinodynamic/velocityBound", self.path_planner.v_max)
        if self.path_planner.a_max >= 0:
            self.ps.setParameter("Kinodynamic/accelerationBound", self.path_planner.a_max)

    def init_viewer(self):
        """
        Create a Viewer instance
        """
        self.v = Viewer(self.ps, viewerClient=self.path_planner.v.client, displayCoM=True)

    def compute_configs_from_guide(self, use_acc_init=True, use_acc_end=False, set_ref_height=True):
        """
        Compute the wholebody configurations from the reference one and the guide init/goal config
        :param use_acc_init: if True, use the initial acceleration from the guide path
        :param use_acc_end: if True, use the final acceleration from the guide path
        :param set_ref_height: if True, set the root Z position of q_init and q_goal to be equal to q_ref
        """
        self.q_init = self.q_ref[::]
        self.q_init[:7] = self.path_planner.ps.configAtParam(self.pid, 0.001)[0:7]
        # do not use 0 but an epsilon in order to avoid the orientation discontinuity that may happen at t=0
        self.q_goal = self.q_init[::]
        self.q_goal[:7] = self.path_planner.ps.configAtParam(self.pid, self.path_planner.ps.pathLength(self.pid))[0:7]

        # copy extraconfig for start and init configurations
        configSize = self.fullbody.getConfigSize() - self.fullbody.client.robot.getDimensionExtraConfigSpace()
        q_init_guide = self.path_planner.ps.configAtParam(self.pid, 0)
        q_goal_guide = self.path_planner.ps.configAtParam(self.pid, self.path_planner.ps.pathLength(self.pid))
        index_ecs = len(q_init_guide) - self.path_planner.extra_dof
        self.q_init[configSize:configSize + 3] = q_init_guide[index_ecs:index_ecs + 3]
        if use_acc_init:
            self.q_init[configSize + 3:configSize + 6] = q_init_guide[index_ecs + 3:index_ecs + 6]
        else:
            self.q_init[configSize + 3:configSize + 6] = [0, 0, 0]
        self.q_goal[configSize:configSize + 3] = q_goal_guide[index_ecs:index_ecs + 3]
        if use_acc_end:
            self.q_goal[configSize + 3:configSize + 6] = q_goal_guide[index_ecs + 3:index_ecs + 6]
        else:
            self.q_goal[configSize + 3:configSize + 6] = [0, 0, 0]
        if set_ref_height:
            # force root height to be at the reference position:
            self.q_init[2] = self.q_ref[2]
            self.q_goal[2] = self.q_ref[2]
        self.v(self.q_init)

    def set_start_end_states(self):
        """
        Set the current q_init and q_goal as initial/goal state for the contact planning
        """
        self.fullbody.setStartState(self.q_init, self.init_contacts)
        self.fullbody.setEndState(self.q_goal, self.end_contacts)

    def interpolate(self):
        """
        Compute the sequence of configuration in contact between q_init and q_goal
        """
        self.set_start_end_states()
        self.fullbody.setStaticStability(self.static_stability)
        self.v(self.q_init)
        print("Generate contact plan ...")
        t_start = time.time()
        self.configs = self.fullbody.interpolate(self.dt,
                                                 pathId=self.pid,
                                                 robustnessTreshold=self.robustness,
                                                 filterStates=self.filter_states,
                                                 testReachability=self.test_reachability,
                                                 quasiStatic=self.quasi_static,
                                                 erasePreviousStates=self.erase_previous_states)
        t_interpolate_configs = time.time() - t_start
        print("Contact plan generated in : " + str(t_interpolate_configs) + " s")
        print("number of configs :", len(self.configs))

    # Helper methods used to display results

    def display_sequence(self, dt=0.5):
        """
        Display the sequence of configuration in contact,
        requires that self.configs is not empty
        :param dt: the pause (in second) between each configuration
        """
        displayContactSequence(self.v, self.configs, dt)

    def display_init_config(self):
        self.v(self.q_init)

    def display_end_config(self):
        self.v(self.q_goal)

    def play_guide_path(self):
        """
        display the guide path planned
        """
        self.v.client.gui.setVisibility(self.robot_node_name, "OFF")
        self.path_planner.show_rom()
        self.cl.problem.selectProblem("guide_planning")
        # this is not the same problem as in the guide_planner. We only added the final path in this problem,
        # thus there is only 1 path in this problem
        self.path_planner.pp(0)
        self.cl.problem.selectProblem("default")
        self.path_planner.hide_rom()
        self.v.client.gui.setVisibility(self.robot_node_name, "ON")

    def run(self):
        """
        Must be defined in the child class to run all the methods with the correct arguments.
        """
        self.load_fullbody()
        self.set_joints_bounds()
        self.set_reference()
        self.load_limbs()
        self.init_problem()
        self.init_viewer()
        self.compute_configs_from_guide()
        self.interpolate()
