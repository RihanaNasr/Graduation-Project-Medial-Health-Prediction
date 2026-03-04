# Script to populate initial help contacts for Egypt
import os
import django

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'vitalcare.settings')
django.setup()

from medical.models import HelpContact

# Clear existing contacts to avoid duplicates
HelpContact.objects.all().delete()

# Create Egyptian emergency contacts
help_contacts = [
    {
        'name': 'Ambulance (إسعاف)',
        'phone_number': '123',
        'description': 'Main Egyptian ambulance service for medical emergencies',
        'is_emergency': True
    },
    {
        'name': 'Police (شرطة)',
        'phone_number': '122',
        'description': 'Egyptian police emergency services',
        'is_emergency': True
    },
    {
        'name': 'Fire Department (مطافئ)',
        'phone_number': '180',
        'description': 'Egyptian fire emergency services',
        'is_emergency': True
    },
    {
        'name': 'MOH Egypt (وزارة الصحة)',
        'phone_number': '105',
        'description': 'Ministry of Health general helpline and virus inquiries',
        'is_emergency': False
    },
    {
        'name': 'Poison Control - Cairo Univ.',
        'phone_number': '02-23640300',
        'description': 'Poison Control Center - Cairo University Hospitals (Kasr Al-Ainy)',
        'is_emergency': True
    },
    {
        'name': 'Poison Control - Ain Shams',
        'phone_number': '02-24823314',
        'description': 'Poison Control Center - Ain Shams University Hospitals',
        'is_emergency': True
    },
    {
        'name': 'Mental Health Support',
        'phone_number': '08008880700',
        'description': 'National Mental Health Helpline (Egypt)',
        'is_emergency': False
    },
    {
        'name': 'Electricity Emergency',
        'phone_number': '121',
        'description': 'National electricity emergency service',
        'is_emergency': False
    },
    {
        'name': 'Gas Emergency',
        'phone_number': '129',
        'description': 'National gas emergency service',
        'is_emergency': False
    },
]

for contact_data in help_contacts:
    HelpContact.objects.get_or_create(
        name=contact_data['name'],
        defaults=contact_data
    )

print("✅ Egypt help contacts updated successfully!")
