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
            self.assertTrue(hasattr(module_scenario, 'PathPlanner'))
            PathPlanner = getattr(module_scenario, 'PathPlanner')
            planner = PathPlanner()
            planner.run()
            ps = planner.ps
            self.assertEqual(ps.numberPaths(), 1)
            self.assertGreater(ps.pathLength(0), 6.)
            self.assertLess(ps.pathLength(0), 7.)
            self.assertEqual(planner.q_init, ps.configAtParam(0, 0))
            self.assertEqual(planner.q_goal, ps.configAtParam(0, ps.pathLength(0)))


if __name__ == '__main__':
    unittest.main()
