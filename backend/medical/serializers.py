from rest_framework import serializers
from .models import MedicalRecord, ChatMessage, HelpContact


class MedicalRecordSerializer(serializers.ModelSerializer):
    """Serializer for medical records"""
    
    class Meta:
        model = MedicalRecord
        fields = '__all__'
        read_only_fields = ['user', 'created_at', 'updated_at']


class ChatMessageSerializer(serializers.ModelSerializer):
    """Serializer for chat messages"""
    
    class Meta:
        model = ChatMessage
        fields = ['id', 'message', 'response', 'timestamp']
        read_only_fields = ['id', 'response', 'timestamp']


class ChatRequestSerializer(serializers.Serializer):
    """Serializer for chat requests"""
    message = serializers.CharField(max_length=1000)


class HelpContactSerializer(serializers.ModelSerializer):
    """Serializer for help contacts"""
    
    class Meta:
        model = HelpContact
        fields = '__all__'
