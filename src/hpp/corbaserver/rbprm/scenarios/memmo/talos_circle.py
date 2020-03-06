from hpp.corbaserver.rbprm.scenarios.memmo.talos_circle_path import PathPlanner
from hpp.corbaserver.rbprm.scenarios.memmo.talos_contact_generator import TalosContactGenerator
import time


class ContactGenerator(TalosContactGenerator):
    def __init__(self):
        super().__init__(PathPlanner())
        self.pid = 0
        self.dt = 0.005

    def set_joints_bounds(self):
        super().set_joints_bounds()
        self.fullbody.setConstrainedJointsBounds()

    def set_reference(self, use_postural_task=True):
        super().set_reference(use_postural_task)
        # weights used for the postural task depend of the direction of the motion
        if abs(self.path_planner.q_goal[1]) <= abs(self.path_planner.q_goal[0]):
            self.fullbody.setPostureWeights(self.fullbody.postureWeights[::] + [0] * 6)
            print("Use weight for straight walk")
            self.fullbody.usePosturalTaskContactCreation(True)
        else:
            self.fullbody.setPostureWeights(self.fullbody.postureWeights_straff[::] + [0] * 6)
            print("Use weight for straff walk")

    def load_limbs(self, heuristic="fixedStep06", analysis=None, nb_samples=None, octree_size=None):
        # heuristic used depend on the direction of the motion
        if abs(self.path_planner.q_goal[1]) <= abs(self.path_planner.q_goal[0]):
            heuristicR = "fixedStep08"
            heuristicL = "fixedStep08"
        else:
            if self.path_planner.q_goal[1] < 0:
                print("start with right leg")
                heuristicL = "static"
                heuristicR = "fixedStep06"
                self.init_contacts = [self.fullbody.rLegId, self.fullbody.lLegId]
                self.end_contacts = [self.fullbody.rLegId, self.fullbody.lLegId]
            else:
                print("start with left leg")
                heuristicR = "static"
                heuristicL = "fixedStep06"
                self.init_contacts = [self.fullbody.lLegId, self.fullbody.rLegId]
                self.end_contacts = [self.fullbody.lLegId, self.fullbody.rLegId]
        print("Generate limb DB ...")
        t_start = time.time()
        # generate databases :
        nbSamples = 100000
        self.fullbody.addLimb(self.fullbody.rLegId,
                              self.fullbody.rleg,
                              self.fullbody.rfoot,
                              self.fullbody.rLegOffset,
                              self.fullbody.rLegNormal,
                              self.fullbody.rLegx,
                              self.fullbody.rLegy,
                              nbSamples,
                              heuristicR,
                              0.01,
                              kinematicConstraintsPath=self.fullbody.rLegKinematicConstraints,
                              kinematicConstraintsMin=0.85)
        if heuristicR == "static":
            self.fullbody.runLimbSampleAnalysis(self.fullbody.rLegId, "ReferenceConfiguration", True)
        self.fullbody.addLimb(self.fullbody.lLegId,
                              self.fullbody.lleg,
                              self.fullbody.lfoot,
                              self.fullbody.lLegOffset,
                              self.fullbody.rLegNormal,
                              self.fullbody.lLegx,
                              self.fullbody.lLegy,
                              nbSamples,
                              heuristicL,
                              0.01,
                              kinematicConstraintsPath=self.fullbody.lLegKinematicConstraints,
                              kinematicConstraintsMin=0.85)
        if heuristicL == "static":
            self.fullbody.runLimbSampleAnalysis(self.fullbody.lLegId, "ReferenceConfiguration", True)

        t_generate = time.time() - t_start
        print("Databases generated in : " + str(t_generate) + " s")

    def run(self):
        super().run()
        self.fullbody.resetJointsBounds()
        self.write_status(5)


if __name__ == "__main__":
    cg = ContactGenerator()
    cg.run()
