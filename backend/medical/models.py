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

    def calculate_severity_rate(self):
        """
        AI Logic: Calculates severity percentage (0-100) based on clinical thresholds.
        80%+ means critical/hospitalization recommended.
        """
        score = 0
        
        # 1. Heart Rate (Dangerous: <50 or >120)
        if self.heart_rate > 140 or self.heart_rate < 40: score += 40
        elif self.heart_rate > 110 or self.heart_rate < 55: score += 20
        
        # 2. SpO2 (Dangerous: <92)
        if self.spo2 < 90: score += 40
        elif self.spo2 < 94: score += 25
        
        # 3. Temperature (Fever: >38.5)
        if self.temperature > 39.5: score += 20
        elif self.temperature > 38.5: score += 10
        
        return min(score, 100)

    def get_hospital_recommendation(self):
        """Logic-based prediction for hospitalization"""
        rate = self.calculate_severity_rate()
        if rate >= 80: return "CRITICAL: Immediate Hospitalization Required"
        if rate >= 50: return "WARNING: Urgent Medical Consultation Recommended"
        return "STABLE: Home Monitoring Sufficient"
    
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


class Alert(models.Model):
    """Model for tracking medical alerts and emergencies"""
    PRIORITY_CHOICES = [
        ('critical', 'Critical'),
        ('warning', 'Warning'),
        ('info', 'Info'),
    ]
    
    user = models.ForeignKey(settings.AUTH_USER_MODEL, on_delete=models.CASCADE, related_name='alerts')
    message = models.TextField()
    priority = models.CharField(max_length=20, choices=PRIORITY_CHOICES, default='info')
    is_resolved = models.BooleanField(default=False)
    timestamp = models.DateTimeField(auto_now_add=True)
    resolved_at = models.DateTimeField(null=True, blank=True)
    
    def __str__(self):
        return f"{self.priority.upper()} - {self.user.email} - {self.timestamp}"
    
    class Meta:
        ordering = ['-timestamp']
