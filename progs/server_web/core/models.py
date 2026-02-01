from django.contrib.auth.models import AbstractUser
from django.db import models


class CustomUser(AbstractUser):
    ssh_private_key = models.TextField(
        blank=True,
        null=True,
        help_text="Private SSH key (PEM format)"
    )

    # для избежания конфликтов
    groups = models.ManyToManyField(
        'auth.Group',
        related_name='customuser_set',
        blank=True
    )
    user_permissions = models.ManyToManyField(
        'auth.Permission',
        related_name='customuser_set',
        blank=True
    )