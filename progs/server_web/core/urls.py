from django.urls import path, include, reverse_lazy, re_path
from . import views

urlpatterns = [
    path('', views.home, name='home'),
]
