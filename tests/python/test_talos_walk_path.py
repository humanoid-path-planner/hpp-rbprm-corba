# Copyright (c) 2020, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import os
import unittest
from importlib import import_module

from hpp.corbaserver.rbprm.utils import ServerManager

PATH = "hpp.corbaserver.rbprm.scenarios.demos"


class TestTalosWalkPath(unittest.TestCase):
    def test_talos_walk_path(self):
        with ServerManager('hpp-rbprm-server'):
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


if __name__ == '__main__':
    unittest.main()
