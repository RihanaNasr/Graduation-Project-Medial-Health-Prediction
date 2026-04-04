import os
import django

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'cardigo.settings')
django.setup()

from users.models import User
users = User.objects.all()
print(f"Total Users: {len(users)}")
for u in users:
    print(f"Username: {u.username}, Email: {u.email}, Staff: {u.is_staff}")
