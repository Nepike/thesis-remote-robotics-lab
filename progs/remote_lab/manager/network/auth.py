"""
User store and HTTP Basic Auth dependency for FastAPI.

Passwords are hashed with PBKDF2-SHA256 and stored in users.json.

CLI usage (run from the manager/ directory):
    python network/auth.py register <username>   — add a new user (prompts for password)
    python network/auth.py delete  <username>    — remove a user
    python network/auth.py list                  — list all registered users
"""

import argparse
import getpass
import hashlib
import hmac
import json
import os
import sys
from pathlib import Path
from typing import Dict, Optional

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBasic, HTTPBasicCredentials


_ALGORITHM  = "sha256"
_ITERATIONS = 260_000
_security   = HTTPBasic()


def _hash_password(password: str) -> str:
    salt = os.urandom(16)
    dk   = hashlib.pbkdf2_hmac(_ALGORITHM, password.encode(), salt, _ITERATIONS)
    return f"pbkdf2_{_ALGORITHM}:{_ITERATIONS}:{salt.hex()}:{dk.hex()}"


def _verify_password(password: str, stored: str) -> bool:
    try:
        parts = stored.split(":")
        if len(parts) != 4:
            return False
        algo_full, iter_s, salt_hex, hash_hex = parts
        algo     = algo_full.split("_", 1)[1]   # "pbkdf2_sha256" -> "sha256"
        salt     = bytes.fromhex(salt_hex)
        expected = bytes.fromhex(hash_hex)
    except (ValueError, IndexError):
        return False

    dk = hashlib.pbkdf2_hmac(algo, password.encode(), salt, int(iter_s))
    return hmac.compare_digest(dk, expected)


class UserStore:
    """
    Loads users from a JSON file.

    File format:
        {
            "_note": "any key starting with _ is ignored",
            "alice": "<pbkdf2 hash>",
            "bob":   "<pbkdf2 hash>"
        }
    """

    def __init__(self, path: Path):
        with open(path) as f:
            raw = json.load(f)
        # keys starting with _ are treated as comments
        self._users: Dict[str, str] = {k: v for k, v in raw.items() if not k.startswith("_")}

    def verify(self, username: str, password: str) -> bool:
        stored = self._users.get(username)
        if stored is None:
            return False
        return _verify_password(password, stored)


_store: Optional[UserStore] = None


def init_user_store(path: Path):
    """Call once at server startup before accepting connections."""
    global _store
    _store = UserStore(path)


def get_client_id(credentials: HTTPBasicCredentials = Depends(_security)) -> str:
    """
    FastAPI dependency that validates HTTP Basic Auth and returns the username
    as client_id. Use in route handlers: `client_id: str = Depends(get_client_id)`.
    """
    if _store is None:
        raise RuntimeError("UserStore not initialised — call init_user_store() first")

    if not _store.verify(credentials.username, credentials.password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid credentials",
            headers={"WWW-Authenticate": "Basic"},
        )

    return credentials.username


# CLI

_DEFAULT_USERS_PATH = Path(__file__).parent.parent / "users.json"


def _load_raw(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path) as f:
        return json.load(f)


def _save_raw(path: Path, data: dict):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Saved: {path}")


def _cmd_register(args):
    path = Path(args.file)
    data = _load_raw(path)

    username = args.username
    if username.startswith("_"):
        print("Error: usernames starting with _ are reserved.")
        sys.exit(1)
    if username in data:
        print(f"Error: user '{username}' already exists. Delete first if you want to reset.")
        sys.exit(1)

    password = getpass.getpass(f"Password for '{username}': ")
    confirm  = getpass.getpass("Confirm password: ")
    if password != confirm:
        print("Error: passwords do not match.")
        sys.exit(1)
    if not password:
        print("Error: password cannot be empty.")
        sys.exit(1)

    data[username] = _hash_password(password)
    _save_raw(path, data)
    print(f"User '{username}' registered.")


def _cmd_delete(args):
    path = Path(args.file)
    data = _load_raw(path)

    username = args.username
    if username not in data:
        print(f"Error: user '{username}' not found.")
        sys.exit(1)

    del data[username]
    _save_raw(path, data)
    print(f"User '{username}' deleted.")


def _cmd_list(args):
    path = Path(args.file)
    data = _load_raw(path)
    users = [k for k in data if not k.startswith("_")]
    if not users:
        print("No users registered.")
    else:
        print(f"{len(users)} user(s):")
        for u in users:
            print(f"  {u}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="python network/auth.py",
        description="Manage RemoteLab users",
    )
    parser.add_argument(
        "--file", "-f",
        default=str(_DEFAULT_USERS_PATH),
        help="Path to users.json (default: ../users.json)",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_reg = sub.add_parser("register", help="Add a new user")
    p_reg.add_argument("username")
    p_reg.set_defaults(func=_cmd_register)

    p_del = sub.add_parser("delete", help="Remove a user")
    p_del.add_argument("username")
    p_del.set_defaults(func=_cmd_delete)

    p_lst = sub.add_parser("list", help="List all users")
    p_lst.set_defaults(func=_cmd_list)

    parsed = parser.parse_args()
    parsed.func(parsed)
