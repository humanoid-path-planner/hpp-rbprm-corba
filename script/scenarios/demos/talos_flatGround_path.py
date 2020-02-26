from scenarios.template_talos_path import TalosPathPlanning

path_planning = TalosPathPlanning()
path_planning.init_problem()

path_planning.q_init[0:2] = [0, 0]
path_planning.q_goal[0:2] = [1, 0]

path_planning.init_viewer("multicontact/ground", visualize_affordances=["Support"])
path_planning.init_planner()
path_planning.solve()

path_planning.display_path()

path_planning.play_path()
