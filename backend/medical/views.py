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
            ai_response = chatbot.process_message(user_message)
            
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


class HelpContactListView(generics.ListAPIView):
    """List all help/emergency contacts"""
    serializer_class = HelpContactSerializer
    permission_classes = [permissions.IsAuthenticated]
    queryset = HelpContact.objects.all()
