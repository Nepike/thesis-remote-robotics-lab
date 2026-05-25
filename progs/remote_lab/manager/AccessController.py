from typing import Dict, Optional, Set
from threading import Lock


class AccessController:
    """
    Controls which clients are allowed to send commands to which devices.

    Devices are either *shared* or *exclusive*:
      - Shared:    any client may send commands at any time — no acquire needed.
      - Exclusive: only one client may hold the device at a time.
                   The client must call acquire() before submit_command() will
                   let its commands through.

    Typical two-phase usage for exclusive devices:
        ok = controller.acquire("robot1", client_id)   # lock the device
        if ok:
            manager.submit_command(client_id, ["robot1"], ...)
        ...
        controller.release("robot1", client_id)        # unlock when done

    For shared devices, skip acquire() entirely — check_access() always returns True.

    Thread safety: a plain threading.Lock is used instead of asyncio.Lock because
    AccessController methods are synchronous and may eventually be called from
    network-layer threads (e.g. gRPC handlers) outside the asyncio event loop.
    """

    def __init__(self):
        # Names of devices that require exclusive ownership
        self._exclusive_devices: Set[str] = set()
        # device_name -> client_id of the current owner (exclusive devices only)
        self._device_owner: Dict[str, str] = {}
        self._lock = Lock()

    def register_device(self, device_name: str, shared: bool):
        """Register a device as shared or exclusive. идемпотентна"""
        with self._lock:
            if shared:
                self._exclusive_devices.discard(device_name)
            else:
                self._exclusive_devices.add(device_name)

    def acquire(self, device_name: str, client_id: str) -> bool:
        """
        Attempt to acquire exclusive ownership of a device.

        - Shared devices: always returns True (no ownership is recorded).
        - Exclusive devices: succeeds if the device is free or already owned by this client;
          fails (returns False) if owned by someone else.
        """
        with self._lock:
            if device_name not in self._exclusive_devices:
                return True

            owner = self._device_owner.get(device_name)
            if owner is None or owner == client_id:
                self._device_owner[device_name] = client_id
                return True

            return False

    def release(self, device_name: str, client_id: str):
        """
        Release exclusive ownership of a device.
        No-op if the caller is not the current owner.
        """
        with self._lock:
            if self._device_owner.get(device_name) == client_id:
                del self._device_owner[device_name]

    def release_all(self, client_id: str):
        """
        Release all devices owned by a client.
        Intended to be called on client disconnect.
        """
        with self._lock:
            for device_name in list(self._device_owner):
                if self._device_owner[device_name] == client_id:
                    del self._device_owner[device_name]

    def check_access(self, device_name: str, client_id: str) -> bool:
        """
        Return True if the client is currently allowed to send commands to the device.

        - Shared devices: always True.
        - Exclusive devices: True only if this client holds the ownership lock.

        Note: this does NOT acquire ownership — call acquire() separately first.
        """
        with self._lock:
            if device_name not in self._exclusive_devices:
                return True
            return self._device_owner.get(device_name) == client_id

    def get_owner(self, device_name: str) -> Optional[str]:
        """Return the client_id of the current owner, or None if the device is free."""
        with self._lock:
            return self._device_owner.get(device_name)
