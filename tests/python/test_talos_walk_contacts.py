# Copyright (c) 2020, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import subprocess
from importlib import import_module
import os
import unittest
import time

PATH = "hpp.corbaserver.rbprm.scenarios.demos"


class TestTalosWalkContact(unittest.TestCase):

    def test_talos_walk_contacts(self):
        subprocess.run(["killall", "hpp-rbprm-server"])
        process = subprocess.Popen("hpp-rbprm-server")
        time.sleep(3)
        module_scenario = import_module(PATH+".talos_flatGround")
        if not hasattr(module_scenario, 'ContactGenerator'):
            self.assertTrue(False)
        ContactGenerator = getattr(module_scenario, 'ContactGenerator')
        cg = ContactGenerator()
        cg.run()
        self.assertTrue(len(cg.configs) > 5)
        self.assertTrue(len(cg.configs) < 10)
        self.assertEqual(cg.q_init, cg.configs[0])
        self.assertEqual(cg.q_goal, cg.configs[-1])
        process.kill()


if __name__ == '__main__':
    unittest.main()
