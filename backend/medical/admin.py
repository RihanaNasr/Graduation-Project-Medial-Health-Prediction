from django.contrib import admin
from .models import MedicalRecord, ChatMessage, HelpContact


@admin.register(MedicalRecord)
class MedicalRecordAdmin(admin.ModelAdmin):
    """Admin for medical records"""
    list_display = ['user', 'blood_type', 'height', 'weight', 'updated_at']
    list_filter = ['blood_type', 'created_at']
    search_fields = ['user__email', 'user__username']
    readonly_fields = ['created_at', 'updated_at']


@admin.register(ChatMessage)
class ChatMessageAdmin(admin.ModelAdmin):
    """Admin for chat messages"""
    list_display = ['user', 'timestamp', 'short_message']
    list_filter = ['timestamp']
    search_fields = ['user__email', 'message', 'response']
    readonly_fields = ['timestamp']
    
    def short_message(self, obj):
        return obj.message[:50] + '...' if len(obj.message) > 50 else obj.message
    short_message.short_description = 'Message'


@admin.register(HelpContact)
class HelpContactAdmin(admin.ModelAdmin):
    """Admin for help contacts"""
    list_display = ['name', 'phone_number', 'is_emergency', 'created_at']
    list_filter = ['is_emergency', 'created_at']
    search_fields = ['name', 'phone_number', 'description']
