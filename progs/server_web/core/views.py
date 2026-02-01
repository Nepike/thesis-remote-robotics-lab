from django.contrib.auth.decorators import login_required
from django.shortcuts import render

@login_required
def home(request):
    return render(request, "core/home.html")


@login_required
def terminal_page(request):
    return render(request, "core/terminal.html")

