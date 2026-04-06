from typing import Dict, Optional, Set
from threading import Lock
















class AccessController:
    def __init__(self):
        self._exclusive_devices: Set[str] = set()
        self._device_owner: Dict[str, str] = {}
        self._lock = Lock()

    def register_device(self, device_name: str, shared: bool):
        with self._lock:
            if shared:
                self._exclusive_devices.discard(device_name)
            else:
                self._exclusive_devices.add(device_name)

    def acquire(self, device_name: str, client_id: str) -> bool:
        with self._lock:
            if device_name not in self._exclusive_devices:
                return True

            owner = self._device_owner.get(device_name)

            if owner is None or owner == client_id:
                self._device_owner[device_name] = client_id
                return True

            return False

    def release(self, device_name: str, client_id: str):
        with self._lock:
            owner = self._device_owner.get(device_name)

            if owner == client_id:
                del self._device_owner[device_name]

    def release_all(self, client_id: str):
        with self._lock:
            for device in list(self._device_owner):
                if self._device_owner[device] == client_id:
                    del self._device_owner[device]

    def check_access(self, device_name: str, client_id: str) -> bool:
        # проверяет что устройство УЖЕ ЗАКРЕПЛЕНО за пользователем (или shared)
        with self._lock:
            if device_name not in self._exclusive_devices:
                return True

            owner = self._device_owner.get(device_name)

            return owner == client_id


    def get_owner(self, device_name: str) -> Optional[str]:
        with self._lock:
            return self._device_owner.get(device_name)
