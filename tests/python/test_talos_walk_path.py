# Copyright (c) 2020, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import subprocess
from importlib import import_module
import os
import unittest
import time

PATH = "hpp.corbaserver.rbprm.scenarios.demos"


class TestTalosWalkPath(unittest.TestCase):
    def test_talos_walk_path(self):
        subprocess.run(["killall", "hpp-rbprm-server"])
        process = subprocess.Popen("hpp-rbprm-server")
        time.sleep(3)
        module_scenario = import_module(PATH + ".talos_flatGround_path")
        if not hasattr(module_scenario, 'PathPlanner'):
            self.assertTrue(False)
        PathPlanner = getattr(module_scenario, 'PathPlanner')
        planner = PathPlanner()
        planner.run()
        ps = planner.ps
        self.assertEqual(ps.numberPaths(), 2)
        self.assertEqual(ps.pathLength(0), ps.pathLength(1))
        self.assertTrue(ps.pathLength(1) > 6.)
        self.assertTrue(ps.pathLength(1) < 7.)
        self.assertEqual(planner.q_init, ps.configAtParam(1, 0))
        self.assertEqual(planner.q_goal, ps.configAtParam(1, ps.pathLength(1)))
        process.kill()


if __name__ == '__main__':
    unittest.main()
