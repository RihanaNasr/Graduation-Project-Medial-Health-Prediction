from rest_framework import serializers
from .models import MedicalRecord, ChatMessage, HelpContact, Alert


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


class AlertSerializer(serializers.ModelSerializer):
    user_email = serializers.EmailField(source='user.email', read_only=True)
    user_name = serializers.SerializerMethodField()

    class Meta:
        model = Alert
        fields = ['id', 'user', 'user_email', 'user_name', 'message', 'priority', 'is_resolved', 'timestamp', 'resolved_at']
        read_only_fields = ['id', 'timestamp', 'resolved_at']

    def get_user_name(self, obj):
        return f"{obj.user.first_name} {obj.user.last_name}"
