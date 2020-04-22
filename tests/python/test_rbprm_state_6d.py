# Copyright (c) 2020, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import unittest

from hpp.corbaserver.rbprm.rbprmstate import State, StateHelper
from talos_rbprm.talos import Robot

from hpp.corbaserver.rbprm.utils import ServerManager


class TestRBPRMstate6D(unittest.TestCase):
    def test_contacts_6d(self):
        with ServerManager('hpp-rbprm-server'):
            fullbody = Robot()
            fullbody.client.robot.setDimensionExtraConfigSpace(6)
            fullbody.setJointBounds("root_joint", [-10, 10, -10, 10, -10, 10])
            fullbody.client.robot.setExtraConfigSpaceBounds([-10, 10, -10, 10, -10, 10, -10, 10, -10, 10, -10, 10])
            fullbody.loadAllLimbs("static", nbSamples=10000)
            q = fullbody.referenceConfig[::] + [0] * 6
            fullbody.setCurrentConfig(q)
            com = fullbody.getCenterOfMass()
            contacts = [fullbody.rLegId, fullbody.lLegId]
            state = State(fullbody, q=q, limbsIncontact=contacts)
            self.assertTrue(state.isBalanced())
            self.assertTrue(state.isValid()[0])
            self.assertTrue(state.isLimbInContact(fullbody.rLegId))
            self.assertTrue(state.isLimbInContact(fullbody.lLegId))
            self.assertEqual(com, state.getCenterOfMass())

            # remove rf contact :
            state1, success = StateHelper.removeContact(state, fullbody.rLegId)
            self.assertTrue(success)
            self.assertFalse(state1.isLimbInContact(fullbody.rLegId))
            self.assertTrue(state1.isLimbInContact(fullbody.lLegId))
            self.assertFalse(state1.isBalanced())
            self.assertEqual(com, state1.getCenterOfMass())

            # create a new contact :
            n = [0, 0, 1]
            p = [0.15, -0.085, 0.002]
            state2, success = StateHelper.addNewContact(state1, fullbody.rLegId, p, n)
            self.assertTrue(success)
            self.assertTrue(state2.isLimbInContact(fullbody.rLegId))
            self.assertTrue(state2.isLimbInContact(fullbody.lLegId))
            self.assertTrue(state2.isBalanced())
            p2, n2 = state2.getCenterOfContactForLimb(fullbody.rLegId)
            self.assertEqual(n, n2)
            for i in range(3):
                self.assertAlmostEqual(p[i], p2[i], 2)

            # try to replace a contact :
            p = [0.2, 0.085, 0.002]
            state3, success = StateHelper.addNewContact(state2, fullbody.lLegId, p, n)
            self.assertTrue(success)
            self.assertTrue(state3.isLimbInContact(fullbody.rLegId))
            self.assertTrue(state3.isLimbInContact(fullbody.lLegId))
            self.assertTrue(state3.isBalanced())

            # try com projection:
            com_target = com[::]
            com_target[2] -= 0.08
            success = state.projectToCOM(com_target)
            self.assertTrue(success)
            fullbody.setCurrentConfig(state.q())
            com_state = fullbody.getCenterOfMass()
            self.assertEqual(com_state, state.getCenterOfMass())
            for i in range(3):
                self.assertAlmostEqual(com_state[i], com_target[i], 4)

            # try lockOtherJoints:
            p = [0.1, -0.085, 0.002]
            state4, success = StateHelper.addNewContact(state, fullbody.rLegId, p, n, lockOtherJoints=True)
            self.assertTrue(success)
            self.assertTrue(state4.isLimbInContact(fullbody.rLegId))
            self.assertTrue(state4.isLimbInContact(fullbody.lLegId))
            self.assertTrue(state4.isBalanced())
            for i in range(7, 13):
                self.assertAlmostEqual(state.q()[i], state4.q()[i])
            for i in range(19, len(q)):
                self.assertAlmostEqual(state.q()[i], state4.q()[i])

            # try with a rotation
            rot = [0.259, 0, 0, 0.966]
            state5, success = StateHelper.addNewContact(state, fullbody.rLegId, p, n, rotation=rot)
            self.assertTrue(success)
            self.assertTrue(state5.isLimbInContact(fullbody.rLegId))
            self.assertTrue(state5.isLimbInContact(fullbody.lLegId))
            self.assertTrue(state5.isBalanced())
            fullbody.setCurrentConfig(state5.q())
            rf_pose = fullbody.getJointPosition(fullbody.rfoot)
            for i in range(4):
                self.assertAlmostEqual(rf_pose[i + 3], rot[i], 3)


if __name__ == '__main__':
    unittest.main()
