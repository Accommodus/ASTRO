import numpy as np

from external_sim_bridge.backends.base import BackendValidationError
from external_sim_bridge.backends.fake_backend import FakeBackend


def test_fake_backend_zero_thrust_matches_reference_first_step():
    backend = FakeBackend()
    backend.initialize()
    backend.apply_control([0.0, 0.0, 0.0])

    state = backend.advance()

    expected = np.array(
        [
            21.474377061136369,
            2.292875120797660,
            21.081978135884185,
            -0.001232341729441,
            -0.050158638812040,
            -0.002059582195653,
        ],
        dtype=float,
    )
    np.testing.assert_allclose(np.array(state, dtype=float), expected, atol=1e-12)


def test_fake_backend_known_thrust_matches_linear_update():
    backend = FakeBackend()
    backend.initialize()
    control = np.array([1e-7, -2e-7, 5e-8], dtype=float)
    backend.apply_control(control.tolist())

    state = np.array(backend.advance(), dtype=float)

    x0 = np.array(
        [20.0, 20.0, 20.0, 0.00930458, -0.0467472, 0.00798343],
        dtype=float,
    )
    ad = np.array(
        [
            [1.25645279151274, 0.0, 0.0, 349.682205848334, 147.7805051, 0.0],
            [-0.0716204647063059, 1.0, 0.0, -147.7805051, 318.728823393334, 0.0],
            [0.0, 0.0, 0.914515736162421, 0.0, 0.0, 349.682205848334],
            [0.0014040831838806, 0.0, 0.0, 0.914515736162421, 0.809100657054006, 0.0],
            [-0.00059338484671504, 0.0, 0.0, -0.809100657054006, 0.658062944649682, 0.0],
            [0.0, 0.0, -0.000468027727960198, 0.0, 0.0, 0.914515736162421],
        ],
        dtype=float,
    )
    bd = np.array(
        [
            [63868.7072544296, 17836.8364281426, 0.0],
            [-17836.8364281426, 61074.8290177183, 0.0],
            [0.0, 0.0, 63868.7072544296],
            [349.682205848334, 147.7805051, 0.0],
            [-147.7805051, 318.728823393334, 0.0],
            [0.0, 0.0, 349.682205848334],
        ],
        dtype=float,
    )

    expected = ad @ x0 + bd @ control
    np.testing.assert_allclose(state, expected, atol=1e-12)


def test_fake_backend_rejects_wrong_control_dimension():
    backend = FakeBackend()

    try:
        backend.validate_control([1.0, 2.0])
    except BackendValidationError as error:
        assert '3-element control vector' in str(error)
    else:
        raise AssertionError('Expected BackendValidationError for wrong control length')
