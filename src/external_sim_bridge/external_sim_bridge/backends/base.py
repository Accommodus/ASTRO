from __future__ import annotations

from typing import Protocol, Sequence


class BackendValidationError(ValueError):
    """Raised when backend-facing state or control validation fails."""


class ExternalSimBackend(Protocol):
    """Simulator-specific backend contract used by the generic bridge node."""

    def initialize(self) -> None:
        """Initialize any simulator-side state before the bridge starts."""

    def validate_control(self, control: Sequence[float]) -> None:
        """Validate a control vector before the bridge stores it."""

    def apply_control(self, control: Sequence[float]) -> None:
        """Apply the latest validated control command before advancing."""

    def advance(self) -> Sequence[float]:
        """Advance the backend once and return `[x, y, z, vx, vy, vz]`."""
