#!/usr/bin/env python
"""Django's command-line utility for administrative tasks."""
import os
import sys
from remote_web.settings.base import BASE_DIR
import yaml

CONFIG_PATH = BASE_DIR / 'config.yml'
with open(CONFIG_PATH) as f:
    SITE_CONFIG = yaml.safe_load(f.read())


def main():
    """Run administrative tasks."""
    if SITE_CONFIG["env"] == 'dev':
        os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'remote_web.settings.dev')
    else:
        os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'remote_web.settings.prod')

    try:
        from django.core.management import execute_from_command_line
    except ImportError as exc:
        raise ImportError(
            "Couldn't import Django. Are you sure it's installed and "
            "available on your PYTHONPATH environment variable? Did you "
            "forget to activate a virtual environment?"
        ) from exc
    execute_from_command_line(sys.argv)


if __name__ == '__main__':
    main()
