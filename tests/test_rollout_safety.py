import numpy as np
import unittest

from single_piper_rollout.safety import ActionGate


class ActionGateTest(unittest.TestCase):
    def test_gate_limits_step_and_base_velocity(self):
        state = np.zeros(9)
        state[6] = 35.0
        action = state.copy()
        action[:6] = [0.1, 0.1, -0.1, 0.1, 0.1, 0.1]
        action[6:] = [50.0, 1.0, -1.0]
        safe = ActionGate().apply(action, state)
        self.assertTrue(np.allclose(safe[:6], np.deg2rad([1, 1, -1, 1, 1, 1])))
        self.assertEqual(safe[6], 37.0)
        self.assertEqual(safe[7], 0.15)
        self.assertEqual(safe[8], -0.40)

    def test_gate_rejects_nonfinite_and_hard_joint_violation(self):
        gate = ActionGate()
        state = np.zeros(9)
        bad = state.copy()
        bad[0] = np.nan
        with self.assertRaises(RuntimeError):
            gate.apply(bad, state)
        bad[0] = np.pi
        with self.assertRaises(RuntimeError):
            gate.apply(bad, state)


if __name__ == "__main__":
    unittest.main()
