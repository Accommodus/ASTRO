from __future__ import annotations

from typing import Sequence

import numpy as np

from external_sim_bridge.backends.base import BackendValidationError


class FakeBackend:
    """Deterministic linear backend that mirrors the current env-node dynamics."""

    def __init__(self) -> None:
        self._state = np.array(
            [20.0, 20.0, 20.0, 0.00930458, -0.0467472, 0.00798343],
            dtype=float,
        )
        self._control = np.zeros(3, dtype=float)
        self._ad = np.array(
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
        self._bd = np.array(
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

    def initialize(self) -> None:
        return None

    def validate_control(self, control: Sequence[float]) -> None:
        if len(control) != 3:
            raise BackendValidationError(
                f'Fake backend expected a 3-element control vector, received {len(control)}'
            )

    def apply_control(self, control: Sequence[float]) -> None:
        self.validate_control(control)
        self._control = np.asarray(control, dtype=float)

    def advance(self) -> list[float]:
        self._state = self._ad @ self._state + self._bd @ self._control
        return self._state.astype(float).tolist()
