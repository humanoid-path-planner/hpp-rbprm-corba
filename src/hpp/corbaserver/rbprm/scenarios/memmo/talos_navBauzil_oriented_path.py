from hpp.corbaserver.rbprm.scenarios.memmo.talos_navBauzil_path import (
    PathPlanner as Parent,
)


class PathPlanner(Parent):
    def set_random_configs(self):
        """
        randomly sample initial and goal configuration :
        """
        from hpp.corbaserver.rbprm.tools.sample_root_config import (
            generate_random_conf_with_orientation,
        )

        self.q_init = generate_random_conf_with_orientation(
            self.rbprmBuilder, self.root_translation_bounds
        )
        self.q_goal = generate_random_conf_with_orientation(
            self.rbprmBuilder, self.root_translation_bounds
        )
        print("q_init= " + str(self.q_init))
        print("q_goal= " + str(self.q_goal))
        # write problem in files :
        with open(self.status_filename, "w") as f:
            f.write("q_init= " + str(self.q_init) + "\n")
            f.write("q_goal= " + str(self.q_goal) + "\n")


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
