# Copyright (c) 2020, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import unittest
from importlib import import_module

from hpp.corbaserver.rbprm.utils import ServerManager

PATH = "hpp.corbaserver.rbprm.scenarios.demos"


class TestTalosWalkContact(unittest.TestCase):
    def test_talos_walk_contacts(self):
        with ServerManager('hpp-rbprm-server'):
            module_scenario = import_module(PATH + ".talos_flatGround")
            self.assertTrue(hasattr(module_scenario, 'ContactGenerator'))
            ContactGenerator = getattr(module_scenario, 'ContactGenerator')
            cg = ContactGenerator()
            cg.run()
            self.assertGreater(len(cg.configs), 5)
            self.assertLess(len(cg.configs), 10)
            self.assertEqual(cg.q_init, cg.configs[0])
            self.assertEqual(cg.q_goal, cg.configs[-1])


if __name__ == '__main__':
    unittest.main()
