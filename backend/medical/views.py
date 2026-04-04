from rest_framework import generics, permissions, status
from rest_framework.response import Response
from rest_framework.views import APIView
from .models import MedicalRecord, ChatMessage, HelpContact
from .serializers import (
    MedicalRecordSerializer,
    ChatMessageSerializer,
    ChatRequestSerializer,
    HelpContactSerializer
)
from .chatbot import MedicalChatbot
import datetime


class HelpContactListView(generics.ListAPIView):
    """List help contacts for the authenticated user"""
    serializer_class = HelpContactSerializer
    permission_classes = [permissions.IsAuthenticated]
    
    def get_queryset(self):
        return HelpContact.objects.filter(user=self.request.user)


class MedicalRecordView(generics.RetrieveUpdateAPIView):
    """Get, create, or update medical record for the authenticated user"""
    serializer_class = MedicalRecordSerializer
    permission_classes = [permissions.IsAuthenticated]
    
    def get_object(self):
        # Get or create medical record for the user
        obj, created = MedicalRecord.objects.get_or_create(user=self.request.user)
        return obj


class MedicalRecordListView(generics.ListAPIView):
    """List all medical records (admin view)"""
    serializer_class = MedicalRecordSerializer
    permission_classes = [permissions.IsAuthenticated]
    
    def get_queryset(self):
        # Users can only see their own records
        return MedicalRecord.objects.filter(user=self.request.user)


class ChatView(APIView):
    """Chat with AI endpoint"""
    permission_classes = [permissions.IsAuthenticated]
    
    def post(self, request):
        serializer = ChatRequestSerializer(data=request.data)
        if serializer.is_valid():
            user_message = serializer.validated_data['message']
            
            # Get AI response
            chatbot = MedicalChatbot()
            ai_response = chatbot.process_message(user_message, user=request.user)
            
            # Save chat message
            chat_message = ChatMessage.objects.create(
                user=request.user,
                message=user_message,
                response=ai_response
            )
            
            return Response({
                'id': chat_message.id,
                'message': user_message,
                'response': ai_response,
                'timestamp': chat_message.timestamp
            }, status=status.HTTP_200_OK)
        
        return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)


class ChatHistoryView(generics.ListAPIView):
    """Get chat history for the authenticated user"""
    serializer_class = ChatMessageSerializer
    permission_classes = [permissions.IsAuthenticated]
    
    def get_queryset(self):
        return ChatMessage.objects.filter(user=self.request.user)


class SOSView(APIView):
    """Simulate an Emergency SOS alert"""
    permission_classes = [permissions.IsAuthenticated]
    
    def post(self, request):
        try:
            record = MedicalRecord.objects.get(user=request.user)
            contact = HelpContact.objects.filter(is_emergency=True).first()
            if not contact:
                contact = HelpContact.objects.first()
            
            contact_info = HelpContactSerializer(contact).data if contact else "No contact found"
            
            sos_payload = {
                "message": "🚨 EMERGENCY SOS: CardiGO Health Alert!",
                "user": request.user.email,
                "vitals": {
                    "heart_rate": record.heart_rate,
                    "blood_pressure": record.blood_pressure,
                    "oxygen": record.spo2
                },
                "contact": contact_info,
                "action": "Sending SMS to emergency contact and calling nearest ambulance..."
            }
            return Response(sos_payload, status=status.HTTP_200_OK)
        except Exception as e:
            return Response({"error": str(e)}, status=status.HTTP_400_BAD_REQUEST)

class ReportExportView(APIView):
    """Generate a digital health report for doctors"""
    permission_classes = [permissions.IsAuthenticated]
    
    def get(self, request):
        try:
            record = MedicalRecord.objects.get(user=request.user)
            history = ChatMessage.objects.filter(user=request.user).order_by('-timestamp')[:5]
            
            report = {
                "title": f"CardiGO Health Summary - {datetime.date.today()}",
                "patient": request.user.email,
                "current_vitals": MedicalRecordSerializer(record).data,
                "risk_history": ChatMessageSerializer(history, many=True).data,
                "doctor_notes": "Patient has been monitoring cardiovascular patterns. Vitals are currently within database-driven safety zones."
            }
            return Response(report, status=status.HTTP_200_OK)
        except Exception as e:
            return Response({"error": str(e)}, status=status.HTTP_400_BAD_REQUEST)
