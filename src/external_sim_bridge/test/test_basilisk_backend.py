import math
import os
import unittest

from external_sim_bridge.backends.basilisk_backend import BasiliskBackend


EXPECT_BASILISK = os.environ.get('ASTRO_EXPECT_BASILISK') == '1'

try:
    import Basilisk  # noqa: F401
except ModuleNotFoundError as error:
    BASILISK_AVAILABLE = False
    BASILISK_IMPORT_ERROR = error
else:
    BASILISK_AVAILABLE = True
    BASILISK_IMPORT_ERROR = None


if EXPECT_BASILISK and not BASILISK_AVAILABLE:
    raise AssertionError(
        'ASTRO_EXPECT_BASILISK=1 but the Basilisk runtime is not importable: '
        f'{BASILISK_IMPORT_ERROR}'
    )


@unittest.skipUnless(BASILISK_AVAILABLE, 'Basilisk runtime is not installed')
class BasiliskBackendSmokeTest(unittest.TestCase):
    def _assert_state_vector(self, state) -> None:
        self.assertEqual(len(state), 6)
        for value in state:
            self.assertTrue(math.isfinite(float(value)))

    def _rollout(self, control, steps: int = 3):
        backend = BasiliskBackend()
        backend.initialize()
        backend.apply_control(control)

        state = None
        for _ in range(steps):
            state = backend.advance()

        self.assertIsNotNone(state)
        self._assert_state_vector(state)
        return state

    def test_zero_control_rollout_returns_finite_state(self):
        state = self._rollout([0.0, 0.0, 0.0])
        self.assertGreater(state[0], 0.0)

    def test_positive_x_control_increases_x_position_and_velocity(self):
        zero_state = self._rollout([0.0, 0.0, 0.0])
        positive_x_state = self._rollout([0.5, 0.0, 0.0])

        self.assertGreater(positive_x_state[0], zero_state[0])
        self.assertGreater(positive_x_state[3], zero_state[3])
