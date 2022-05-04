from hpp.corbaserver.rbprm.scenarios.memmo.talos_randomMove_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.memmo.talos_contact_generator import (
    TalosContactGenerator,
)
import random


class ContactGenerator(TalosContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())
        self.dt = 0.005

    def set_joints_bounds(self):
        super().set_joints_bounds()
        self.fullbody.setConstrainedJointsBounds()
        # constraint z axis and y axis :
        self.fullbody.setJointBounds("leg_left_1_joint", [-0.2, 0.7])
        self.fullbody.setJointBounds("leg_left_3_joint", [-1.3, 0.4])
        self.fullbody.setJointBounds("leg_right_1_joint", [-0.7, 0.2])
        self.fullbody.setJointBounds("leg_right_3_joint", [-1.3, 0.4])

    def load_limbs(self):
        super().load_limbs("fixedStep1", "ReferenceConfiguration", nb_samples=100000)
        # define init gait according to the direction of motion, try to move first the
        # leg on the outside of the turn :
        if self.path_planner.q_goal[1] < 0:
            # goal on the right side of the circle, start motion with right leg first
            gait = [self.fullbody.rLegId, self.fullbody.lLegId]
            print("Right foot first")
        else:
            gait = [self.fullbody.lLegId, self.fullbody.rLegId]
            print("Left foot first")
        self.init_contacts = gait
        self.end_contacts = gait

    def compute_configs_from_guide(self):
        super().compute_configs_from_guide()
        # generate random initial state : root pose at the origin exepct for z
        # translation and both feet in contact with the floor.
        from hpp.corbaserver.rbprm.tools.sample_random_transition import (
            sampleRandomStateFlatFloor,
        )

        limbsInContact = [self.fullbody.rLegId, self.fullbody.lLegId]
        random.seed()
        self.ps.setRandomSeed(
            random.SystemRandom().randint(0, 999999)
        )  # is it really usefull ?

        # floor_Z = -0.00095
        floor_Z = 0.0
        s0 = sampleRandomStateFlatFloor(self.fullbody, limbsInContact, floor_Z)
        self.q_init = s0.q()
        self.v(self.q_init)

    def run(self):
        super().run()
        self.write_status(10)
        self.fullbody.resetJointsBounds()


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
