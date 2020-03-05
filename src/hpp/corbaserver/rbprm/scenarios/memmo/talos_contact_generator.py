from hpp.corbaserver.rbprm.scenarios.talos_contact_generator import TalosContactGenerator as Parent

class TalosContactGenerator(Parent):

    """
    Add specific method for benchmark,
    write result of planning / contact generation in file
    """

    def __init__(self,path_planner):
        super().__init__(path_planner)
        self.status_filename = self.path_planner.status_filename
        f = open(self.status_filename, "a")
        if self.path_planner.ps.numberPaths() > 0:
            print("Path planning OK.")
            f.write("Planning_success: True" + "\n")
            f.close()
        else:
            print("Error during path planning")
            f.write("Planning_success: False" + "\n")
            f.close()
            import sys
            sys.exit(1)

    def write_status(self, max_configs):
        if len(self.configs) < 2:
            cg_success = False
            print("Error during contact generation.")
        else:
            cg_success = True
            print("Contact generation Done.")
        if abs(self.configs[-1][0] - self.path_planner.q_goal[0]) < 0.01 and\
                abs(self.configs[-1][1] - self.path_planner.q_goal[1]) < 0.01 and\
                (len(self.fullbody.getContactsVariations(len(self.configs) - 2, len(self.configs) - 1)) == 1):
            print("Contact generation successful.")
            cg_reach_goal = True
        else:
            print("Contact generation failed to reach the goal.")
            cg_reach_goal = False
        if len(self.configs) > max_configs:
            cg_too_many_states = True
            cg_success = False
            print("Discarded contact sequence because it was too long.")
        else:
            cg_too_many_states = False

        f = open(self.status_filename, "a")
        f.write("cg_success: " + str(cg_success) + "\n")
        f.write("cg_reach_goal: " + str(cg_reach_goal) + "\n")
        f.write("cg_too_many_states: " + str(cg_too_many_states) + "\n")
        f.close()
        if (not cg_success) or cg_too_many_states or (not cg_reach_goal):
            import sys
            sys.exit(1)

