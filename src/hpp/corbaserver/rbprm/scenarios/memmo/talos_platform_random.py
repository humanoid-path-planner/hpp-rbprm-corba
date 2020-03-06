from hpp.corbaserver.rbprm.scenarios.memmo.talos_platform_random_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.memmo.talos_contact_generator import TalosContactGenerator
import time


class ContactGenerator(TalosContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())
        self.dt = 0.005

    def set_joints_bounds(self):
        super().set_joints_bounds()
        self.fullbody.setConstrainedJointsBounds()

    def set_reference(self):
        self.q_ref = self.fullbody.referenceConfig_legsApart[::] + [0] * 6
        super().set_reference()

    def load_limbs(self):
        super().load_limbs("fixedStep08", nb_samples=100000)
        # define init gait according to the direction of motion, try to move first the leg on the outside of the turn :
        if self.path_planner.q_goal[0] > self.path_planner.q_init[0]:  # go toward x positif
            if self.path_planner.q_goal[1] > self.path_planner.q_init[1]:  # turn left
                gait = [self.fullbody.rLegId, self.fullbody.lLegId]
            else:  # turn right
                gait = [self.fullbody.lLegId, self.fullbody.rLegId]
        else:  # go toward x negatif
            if self.path_planner.q_goal[1] > self.path_planner.q_init[1]:  # turn right
                gait = [self.fullbody.lLegId, self.fullbody.rLegId]
            else:  # turn left
                gait = [self.fullbody.rLegId, self.fullbody.lLegId]
        self.init_contacts = gait
        self.end_contacts = gait

    def run(self):
        super().run()
        self.write_status(50)
        self.fullbody.resetJointsBounds()


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
