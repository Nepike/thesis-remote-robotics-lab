from .base import *
import yaml


CONFIG_PATH = BASE_DIR / 'config.yml'
with open(CONFIG_PATH) as f:
    SITE_CONFIG = yaml.safe_load(f.read())


SECRET_KEY = 'django-insecure-l0a7(23+8gt-kc1y7@f7!*yhcf%rti$n$=9by@i-+0@kyq)$w%'
DEBUG = True

ALLOWED_HOSTS = ['*']

STATIC_URL = '/static/'
STATIC_ROOT = BASE_DIR / 'static/'

EMAIL_BACKEND = 'django.core.mail.backends.console.EmailBackend'

SILENCED_SYSTEM_CHECKS = ['django_recaptcha.recaptcha_test_key_error']

# TGBOT_TOKEN = SITE_CONFIG["telegram_bot"].get("token", None)