from .abstract_contact_generator import AbstractContactGenerator


class SoloContactGenerator(AbstractContactGenerator):
    def __init__(self, path_planner):
        super().__init__(path_planner)
        self.robustness = 5
        self.used_limbs = ['FRleg', 'HLleg', 'FLleg', 'HRleg']
        self.init_contacts = self.used_limbs[::]
        self.end_contacts = self.used_limbs[::]
        self.robot_node_name = "solo"

    def set_start_end_states(self):
        # for 3D contacts, we must specify the normals manually
        normals_init = []
        normals_end = []
        for limb_id in self.init_contacts:
            normals_init += [self.fullbody.dict_normal[self.fullbody.dict_limb_joint[limb_id]]]
        for limb_id in self.init_contacts:
            normals_end += [self.fullbody.dict_normal[self.fullbody.dict_limb_joint[limb_id]]]
        self.fullbody.setStartState(self.q_init, self.init_contacts, normals_init)
        self.fullbody.setEndState(self.q_goal, self.end_contacts, normals_end)

    def load_fullbody(self):
        from solo_rbprm.solo import Robot
        self.fullbody = Robot()
        self.q_ref = self.fullbody.referenceConfig[::] + [0] * self.path_planner.extra_dof
        self.weight_postural = self.fullbody.postureWeights[::] + [0] * self.path_planner.extra_dof
        self.fullbody.setConstrainedJointsBounds()

    def init_viewer(self):
        super().init_viewer()
        self.v.addLandmark('solo/base_link_0', 0.3)

    def load_limbs(self, heuristic="fixedStep04", analysis=None, nb_samples=None, octree_size=None):
        super().load_limbs(heuristic, analysis, nb_samples, octree_size, disableEffectorCollision=True)
