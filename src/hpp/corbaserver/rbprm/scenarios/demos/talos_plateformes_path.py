from hpp.corbaserver.rbprm.scenarios.talos_path_planner import TalosPathPlanner


class PathPlanner(TalosPathPlanner):
    def set_rom_filters(self):
        super().set_rom_filters()
        #TEMP fix, because of issue https://github.com/humanoid-path-planner/hpp-fcl/issues/134 in hpp-fcl
        # we need to disable ROM checks in this scenario with really small contact surfaces
        self.rbprmBuilder.setFilter([])

    def run(self):
        self.root_translation_bounds = [-5, 5, -1.5, 1.5, 0.95, 1.3]
        self.init_problem()

        self.q_init[:3] = [0.16, 0.25, 1.14]
        self.q_goal[:3] = [1.09, 0.25, 1.14]

        self.init_viewer("multicontact/plateforme_surfaces",
                         reduce_sizes=[0.18, 0, 0],
                         visualize_affordances=["Support"])
        self.init_planner(kinodynamic=False)
        self.solve()
        self.display_path()
        # self.play_path()
        self.hide_rom()


if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
