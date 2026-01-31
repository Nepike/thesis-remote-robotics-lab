from .base import *
import yaml

CONFIG_PATH = BASE_DIR / 'config.yml'
with open(CONFIG_PATH) as f:
    SITE_CONFIG = yaml.safe_load(f.read())


SECRET_KEY = SITE_CONFIG["secret_key"]
DEBUG = False

CSRF_TRUSTED_ORIGINS = [
    'https://remote.nepike.ru',
]

ALLOWED_HOSTS = [
	'remote.nepike.ru',
]

AUTH_PASSWORD_VALIDATORS = [
    {
        'NAME': 'django.contrib.auth.password_validation.UserAttributeSimilarityValidator',
    },
    {
        'NAME': 'django.contrib.auth.password_validation.MinimumLengthValidator',
    },
    {
        'NAME': 'django.contrib.auth.password_validation.CommonPasswordValidator',
    },
    {
        'NAME': 'django.contrib.auth.password_validation.NumericPasswordValidator',
    },
]

STATIC_URL = 'static/'
STATIC_ROOT = BASE_DIR / 'static/'


# EMAIL_BACKEND = 'django.core.mail.backends.smtp.EmailBackend'
# EMAIL_HOST = SITE_CONFIG["email"]["host"]
# EMAIL_PORT = SITE_CONFIG["email"]["port"]
# EMAIL_HOST_USER = SITE_CONFIG["email"]["user"]
# EMAIL_HOST_PASSWORD = SITE_CONFIG["email"]["password"]
# EMAIL_USE_SSL = SITE_CONFIG["email"]["use_ssl"]
# EMAIL_USE_TLS = SITE_CONFIG["email"]["use_tls"]
#
# DEFAULT_FROM_EMAIL = EMAIL_HOST_USER
# SERVER_EMAIL = EMAIL_HOST_USER
# EMAIL_ADMIN = EMAIL_HOST_USER
#
# RECAPTCHA_PUBLIC_KEY = SITE_CONFIG["recaptcha"]["public_key"]
# RECAPTCHA_PRIVATE_KEY = SITE_CONFIG["recaptcha"]["private_key"]
#
#
# TGBOT_TOKEN = SITE_CONFIG["telegram_bot"]["token"]


