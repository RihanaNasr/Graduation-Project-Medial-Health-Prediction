import os
import django

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'cardigo.settings')
django.setup()

from users.models import User
email = "test@vitalcare.com"
password = "Password123!"

if User.objects.filter(email=email).exists():
    user = User.objects.get(email=email)
    user.set_password(password)
    user.save()
    print(f"Updated user: {email}")
else:
    user = User.objects.create_user(
        username="testuser",
        email=email,
        password=password,
        first_name="Test",
        last_name="User"
    )
    print(f"Created user: {email}")
