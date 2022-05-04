# Copyright (c) 2020, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import unittest

from hyq_rbprm.hyq import Robot
from hpp.corbaserver.rbprm.rbprmstate import State, StateHelper

from hpp.corbaserver.rbprm.utils import ServerManager


class TestRBPRMstate3D(unittest.TestCase):
    def test_contacts_3d(self):
        with ServerManager("hpp-rbprm-server"):
            fullbody = Robot()
            fullbody.client.robot.setDimensionExtraConfigSpace(6)
            fullbody.setJointBounds("root_joint", [-10, 10, -10, 10, -10, 10])
            fullbody.client.robot.setExtraConfigSpaceBounds(
                [-10, 10, -10, 10, -10, 10, -10, 10, -10, 10, -10, 10]
            )
            fullbody.loadAllLimbs("static", nbSamples=10000)
            q = fullbody.referenceConfig[::] + [0] * 6
            fullbody.setCurrentConfig(q)
            com = fullbody.getCenterOfMass()
            contacts = [
                fullbody.rLegId,
                fullbody.lLegId,
                fullbody.rArmId,
                fullbody.lArmId,
            ]
            state = State(fullbody, q=q, limbsIncontact=contacts)
            self.assertTrue(state.isBalanced())
            self.assertTrue(state.isValid()[0])
            self.assertTrue(state.isLimbInContact(fullbody.rLegId))
            self.assertTrue(state.isLimbInContact(fullbody.lLegId))
            self.assertTrue(state.isLimbInContact(fullbody.rArmId))
            self.assertTrue(state.isLimbInContact(fullbody.lArmId))
            self.assertEqual(com, state.getCenterOfMass())

            # remove rf contact :
            state1, success = StateHelper.removeContact(state, fullbody.rLegId)
            self.assertTrue(success)
            self.assertFalse(state1.isLimbInContact(fullbody.rLegId))
            self.assertTrue(state1.isLimbInContact(fullbody.lLegId))
            self.assertTrue(state.isLimbInContact(fullbody.rArmId))
            self.assertTrue(state.isLimbInContact(fullbody.lArmId))
            self.assertEqual(com, state1.getCenterOfMass())

            # create a new contact :
            n = [0, 0, 1]
            p = [0.45, -0.2, 0.002]
            state2, success = StateHelper.addNewContact(state1, fullbody.rLegId, p, n)
            self.assertTrue(success)
            self.assertTrue(state2.isLimbInContact(fullbody.rLegId))
            self.assertTrue(state2.isLimbInContact(fullbody.lLegId))
            self.assertTrue(state.isLimbInContact(fullbody.rArmId))
            self.assertTrue(state.isLimbInContact(fullbody.lArmId))
            self.assertTrue(state2.isBalanced())
            p_real, n_real = state2.getCenterOfContactForLimb(fullbody.rLegId)
            self.assertEqual(n, n_real)


if __name__ == "__main__":
    unittest.main()
