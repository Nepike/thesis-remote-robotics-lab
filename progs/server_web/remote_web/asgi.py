import os
from django.core.asgi import get_asgi_application
from channels.routing import ProtocolTypeRouter, URLRouter
from channels.auth import AuthMiddlewareStack

import core.routing
from remote_web.settings.base import BASE_DIR
import yaml


CONFIG_PATH = BASE_DIR / 'config.yml'
with open(CONFIG_PATH) as f:
    SITE_CONFIG = yaml.safe_load(f.read())


if SITE_CONFIG["env"] == 'dev':
    os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'remote_web.settings.dev')
else:
    os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'remote_web.settings.prod')

application = ProtocolTypeRouter({
    "http": get_asgi_application(),
    "websocket": AuthMiddlewareStack(
        URLRouter(
            core.routing.websocket_urlpatterns
        )
    ),
})
