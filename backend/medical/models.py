from django.db import models
from django.conf import settings


class MedicalRecord(models.Model):
    """Model for storing patient medical records"""
    user = models.ForeignKey(settings.AUTH_USER_MODEL, on_delete=models.CASCADE, related_name='medical_records')
    
    # Basic Information
    blood_type = models.CharField(
        max_length=5,
        choices=[
            ('A+', 'A+'), ('A-', 'A-'),
            ('B+', 'B+'), ('B-', 'B-'),
            ('AB+', 'AB+'), ('AB-', 'AB-'),
            ('O+', 'O+'), ('O-', 'O-'),
        ],
        blank=True,
        null=True
    )
    height = models.FloatField(help_text='Height in cm', blank=True, null=True)
    weight = models.FloatField(help_text='Weight in kg', blank=True, null=True)
    
    # Medical History
    chronic_conditions = models.TextField(blank=True, help_text='List of chronic conditions')
    allergies = models.TextField(blank=True, help_text='Known allergies')
    current_medications = models.TextField(blank=True, help_text='Current medications')
    past_surgeries = models.TextField(blank=True, help_text='Past surgical procedures')
    
    # Dashboard Vitals
    heart_rate = models.IntegerField(default=86, help_text='Heart Rate in bpm')
    blood_pressure = models.CharField(max_length=20, default='120/80', help_text='Blood Pressure')
    spo2 = models.IntegerField(default=98, help_text='SpO2 %')
    temperature = models.FloatField(default=36.6, help_text='Body Temperature in Celsius')
    
    # Daily Activities
    calories = models.IntegerField(default=2100, help_text='Calories burned')
    steps = models.IntegerField(default=8500, help_text='Steps count')
    water = models.FloatField(default=1.8, help_text='Water intake in L')
    
    # Emergency Contact
    emergency_contact_name = models.CharField(max_length=200, blank=True)
    emergency_contact_phone = models.CharField(max_length=15, blank=True)
    emergency_contact_relation = models.CharField(max_length=50, blank=True)
    
    # Timestamps
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    
    def __str__(self):
        return f"Medical Record - {self.user.email}"
    
    class Meta:
        verbose_name = 'Medical Record'
        verbose_name_plural = 'Medical Records'
        ordering = ['-updated_at']


class ChatMessage(models.Model):
    """Model for storing chat messages with AI"""
    user = models.ForeignKey(settings.AUTH_USER_MODEL, on_delete=models.CASCADE, related_name='chat_messages')
    message = models.TextField(help_text='User message')
    response = models.TextField(help_text='AI response')
    timestamp = models.DateTimeField(auto_now_add=True)
    
    def __str__(self):
        return f"Chat - {self.user.email} - {self.timestamp}"
    
    class Meta:
        verbose_name = 'Chat Message'
        verbose_name_plural = 'Chat Messages'
        ordering = ['-timestamp']


class HelpContact(models.Model):
    """Model for emergency/help contacts"""
    name = models.CharField(max_length=200)
    phone_number = models.CharField(max_length=15)
    description = models.TextField(help_text='Description of the service')
    is_emergency = models.BooleanField(default=False)
    created_at = models.DateTimeField(auto_now_add=True)
    
    def __str__(self):
        return self.name
    
    class Meta:
        verbose_name = 'Help Contact'
        verbose_name_plural = 'Help Contacts'
        ordering = ['-is_emergency', 'name']
