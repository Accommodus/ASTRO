from external_sim_bridge.backends.base import ExternalSimBackend
from external_sim_bridge.backends.basilisk_backend import BasiliskBackend
from external_sim_bridge.backends.fake_backend import FakeBackend


def create_backend(backend_type: str) -> ExternalSimBackend:
    normalized_backend_type = backend_type.strip().lower()

    if normalized_backend_type == 'fake':
        return FakeBackend()
    if normalized_backend_type == 'basilisk':
        return BasiliskBackend()

    raise ValueError(
        f"Unsupported backend_type '{backend_type}'. Available backends: ['fake', 'basilisk']"
    )
