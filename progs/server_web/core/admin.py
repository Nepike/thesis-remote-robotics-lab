# admin.py
from django.contrib import admin
from django.contrib.auth.admin import UserAdmin
from .models import CustomUser

class CustomUserAdmin(UserAdmin):
    model = CustomUser
    list_display = ['email', 'username', 'is_staff']
    fieldsets = UserAdmin.fieldsets + (
        ('Дополнительно', {'fields': ('ssh_private_key',)}),
    )

admin.site.register(CustomUser, CustomUserAdmin)