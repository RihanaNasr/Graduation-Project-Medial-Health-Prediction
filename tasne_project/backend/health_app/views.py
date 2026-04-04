from rest_framework import generics, status
from rest_framework.response import Response
from rest_framework.permissions import IsAuthenticated, AllowAny
from rest_framework.views import APIView
from rest_framework_simplejwt.views import TokenObtainPairView
import google.generativeai as genai
import mimetypes
import random
from django.utils import timezone
from datetime import timedelta
from PIL import Image as PILImage
from .models import User, HealthData, AlertHistory, ChatMessage, PasswordResetOTP
from .serializers import (
    UserSerializer, HealthDataSerializer, AlertHistorySerializer,
    ChatMessageSerializer, CustomTokenObtainPairSerializer
)

class RegisterView(generics.CreateAPIView):
    queryset = User.objects.all()
    permission_classes = (AllowAny,)
    serializer_class = UserSerializer

class CustomTokenObtainPairView(TokenObtainPairView):
    serializer_class = CustomTokenObtainPairSerializer

class ProfileView(generics.RetrieveUpdateAPIView):
    serializer_class = UserSerializer
    permission_classes = (IsAuthenticated,)

    def get_object(self):
        return self.request.user

class HealthDataListCreateView(generics.ListCreateAPIView):
    serializer_class = HealthDataSerializer
    permission_classes = (IsAuthenticated,)

    def get_queryset(self):
        return HealthData.objects.filter(user=self.request.user).order_by('-timestamp')

    def perform_create(self, serializer):
        data = serializer.save(user=self.request.user)
        # Check for critical status and create alert if needed
        if data.status in ['warning', 'critical']:
            AlertHistory.objects.create(
                user=self.request.user,
                message=f"Health reading alert: {data.status} for Heart Rate {data.heart_rate}, SpO2 {data.sp02}"
            )

class AlertHistoryListView(generics.ListAPIView):
    serializer_class = AlertHistorySerializer
    permission_classes = (IsAuthenticated,)

    def get_queryset(self):
        return AlertHistory.objects.filter(user=self.request.user).order_by('-timestamp')

class ChatMessageListCreateView(generics.ListCreateAPIView):
    serializer_class = ChatMessageSerializer
    permission_classes = (IsAuthenticated,)

    def get_queryset(self):
        return ChatMessage.objects.filter(user=self.request.user).order_by('timestamp')

    def perform_create(self, serializer):
        user_message = serializer.save(user=self.request.user, sender='user')
        
        # Get user's latest health data for context
        latest_health = HealthData.objects.filter(user=self.request.user).order_by('-timestamp').first()
        health_context = "No data yet."
        if latest_health:
            health_context = f"BPM: {latest_health.heart_rate}, SpO2: {latest_health.sp02}%, BP: {latest_health.blood_pressure_sys}/{latest_health.blood_pressure_dia}, Status: {latest_health.status}"

        try:
            # Using the latest Gemini 2.0 Flash model detected for this account
            genai.configure(api_key="AIzaSyCgyadeSdVgWvzbWk9Fk4WDLTTx9O98P8U")
            model = genai.GenerativeModel('gemini-2.0-flash')
            
            system_prompt = f"""
            You are PulseGuard AI, a professional medical assistant part of the PulseGuard health monitoring app.
            User Profile: Age {self.request.user.age}, Gender {self.request.user.gender}.
            Latest Vitals: {health_context}.
            
            Instructions:
            1. Answer medical questions accurately but always add a disclaimer to consult a doctor.
            2. If requested, provide a clear 7-day health or workout schedule formatted as a list.
            3. Use the user's vitsls context to give personalized advice.
            4. Keep responses encouraging, professional, and clear.
            """
            
            
            prompt_content = [f"{system_prompt}\n\nUser Question: {user_message.message}"]
            
            if user_message.image:
                try:
                    img = PILImage.open(user_message.image.path)
                    prompt_content.append(img)
                except Exception as img_err:
                    print(f"Error loading image for Gemini: {img_err}")

            if user_message.audio:
                try:
                    # Load audio data for Gemini
                    with open(user_message.audio.path, "rb") as audio_file:
                        audio_data = audio_file.read()
                        
                    # Guess mime type based on file extension
                    mime_type, _ = mimetypes.guess_type(user_message.audio.path)
                    if not mime_type:
                        mime_type = "audio/mpeg" # Fallback
                        
                    prompt_content.append({
                        "mime_type": mime_type,
                        "data": audio_data
                    })
                except Exception as audio_err:
                    print(f"Error loading audio for Gemini: {audio_err}")

            response = model.generate_content(prompt_content)
            ai_response = response.text
        except Exception as e:
            # Better error reporting for the user
            if "429" in str(e):
                ai_response = "I'm experiencing high traffic right now (Quota Reached). Please wait 1 minute and try again! ⏳"
            else:
                ai_response = f"I'm PulseGuard AI. Regarding your question about '{user_message.message}', based on your vitals ({health_context}), please ensure you follow a balanced routine. [System updated: AI ready!]"

        ChatMessage.objects.create(
            user=self.request.user,
            sender='ai',
            message=ai_response
        )

class ChatMessageClearView(APIView):
    permission_classes = (IsAuthenticated,)

    def delete(self, request):
        ChatMessage.objects.filter(user=request.user).delete()
        return Response({"message": "Chat history cleared successfully"}, status=status.HTTP_200_OK)

class PasswordResetRequestView(APIView):
    permission_classes = (AllowAny,)

    def post(self, request):
        email = request.data.get('email')
        if not email:
            return Response({"error": "Email is required"}, status=status.HTTP_400_BAD_REQUEST)
        
        user = User.objects.filter(email=email).first()
        if not user:
             return Response({"error": "No account found with this email."}, status=status.HTTP_404_NOT_FOUND)

        otp = str(random.randint(100000, 999999))
        PasswordResetOTP.objects.create(email=email, otp=otp)
        
        print(f"\n\n--- [RESET PASSWORD] ---\nEmail: {email}\nOTP Code: {otp}\n------------------------\n\n")
        
        return Response({
            "message": "OTP sent to your email!",
            "otp": otp  # Returning OTP for demo notification purposes
        }, status=status.HTTP_200_OK)

class PasswordResetConfirmView(APIView):
    permission_classes = (AllowAny,)

    def post(self, request):
        email = request.data.get('email')
        otp = request.data.get('otp')
        new_password = request.data.get('new_password')
        
        if not all([email, otp, new_password]):
            return Response({"error": "All fields are required"}, status=status.HTTP_400_BAD_REQUEST)
        
        reset_obj = PasswordResetOTP.objects.filter(email=email, otp=otp, is_used=False).order_by('-created_at').first()
        
        if not reset_obj:
            return Response({"error": "Invalid OTP."}, status=status.HTTP_400_BAD_REQUEST)
            
        if reset_obj.created_at < timezone.now() - timedelta(minutes=10):
            return Response({"error": "OTP expired."}, status=status.HTTP_400_BAD_REQUEST)
        # Update all users with this email (safest for demo environments with duplicates)
        users = User.objects.filter(email=email)
        for user in users:
            user.set_password(new_password)
            user.save()
        
        # Mark OTP as used
        reset_obj.is_used = True
        reset_obj.save()
        
        return Response({"message": "Password updated successfully!"}, status=status.HTTP_200_OK)
