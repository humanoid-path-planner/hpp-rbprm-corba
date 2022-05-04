from .abstract_contact_generator import AbstractContactGenerator


class AnymalContactGenerator(AbstractContactGenerator):
    def __init__(self, path_planner):
        super().__init__(path_planner)
        self.robustness = 5
        self.used_limbs = ["RFleg", "LHleg", "LFleg", "RHleg"]
        self.init_contacts = self.used_limbs[::]
        self.end_contacts = self.used_limbs[::]
        self.robot_node_name = "anymal"

    def set_start_end_states(self):
        # for 3D contacts, we must specify the normals manually
        normals_init = []
        normals_end = []
        for limb_id in self.init_contacts:
            normals_init += [
                self.fullbody.dict_normal[self.fullbody.dict_limb_joint[limb_id]]
            ]
        for limb_id in self.init_contacts:
            normals_end += [
                self.fullbody.dict_normal[self.fullbody.dict_limb_joint[limb_id]]
            ]
        self.fullbody.setStartState(self.q_init, self.init_contacts, normals_init)
        self.fullbody.setEndState(self.q_goal, self.end_contacts, normals_end)

    def load_fullbody(self):
        from anymal_rbprm.anymal import Robot

        self.fullbody = Robot()
        self.q_ref = (
            self.fullbody.referenceConfig[::] + [0] * self.path_planner.extra_dof
        )
        self.weight_postural = (
            self.fullbody.postureWeights[::] + [0] * self.path_planner.extra_dof
        )

    def init_viewer(self):
        super().init_viewer()
        self.v.addLandmark("anymal/base_0", 0.3)

    def load_limbs(
        self, heuristic="fixedStep04", analysis=None, nb_samples=None, octree_size=None
    ):
        super().load_limbs(heuristic, analysis, nb_samples, octree_size)
