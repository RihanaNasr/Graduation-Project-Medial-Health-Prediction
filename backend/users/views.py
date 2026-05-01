from rest_framework import status, generics, permissions
from rest_framework.response import Response
from rest_framework.views import APIView
from rest_framework_simplejwt.tokens import RefreshToken
from django.contrib.auth import authenticate, get_user_model

from .serializers import (
    UserRegistrationSerializer,
    UserProfileSerializer,
    UserLoginSerializer,
    ChangePasswordSerializer
)

User = get_user_model()

class RegisterView(APIView):
    """User registration endpoint"""
    permission_classes = [permissions.AllowAny]
    
    def post(self, request):
        serializer = UserRegistrationSerializer(data=request.data)
        if serializer.is_valid():
            user = serializer.save()
            
            # Generate tokens
            refresh = RefreshToken.for_user(user)
            
            return Response({
                'user': UserProfileSerializer(user).data,
                'tokens': {
                    'refresh': str(refresh),
                    'access': str(refresh.access_token),
                }
            }, status=status.HTTP_201_CREATED)
        
        return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)


class LoginView(APIView):
    """User login endpoint"""
    permission_classes = [permissions.AllowAny]
    
    def post(self, request):
        serializer = UserLoginSerializer(data=request.data)
        if serializer.is_valid():
            email = serializer.validated_data['email'].strip()
            password = serializer.validated_data['password']
            
            # Authenticate user (case-insensitive email)
            user = User.objects.filter(email__iexact=email).first()
            if user and not user.check_password(password):
                user = None
            
            if user is not None:
                # Generate tokens
                refresh = RefreshToken.for_user(user)
                
                return Response({
                    'user': UserProfileSerializer(user).data,
                    'tokens': {
                        'refresh': str(refresh),
                        'access': str(refresh.access_token),
                    }
                }, status=status.HTTP_200_OK)
            
            return Response(
                {'detail': 'Invalid credentials'},
                status=status.HTTP_401_UNAUTHORIZED
            )
        
        return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)


class ProfileView(generics.RetrieveUpdateAPIView):
    """Get and update user profile"""
    serializer_class = UserProfileSerializer
    permission_classes = [permissions.IsAuthenticated]
    
    def get_object(self):
        return self.request.user


class ChangePasswordView(APIView):
    """Change password endpoint for authenticated users"""
    permission_classes = [permissions.IsAuthenticated]

    def post(self, request):
        print(f"DEBUG: ChangePasswordView hit by user: {request.user.email}")
        serializer = ChangePasswordSerializer(data=request.data)
        if serializer.is_valid():
            user = request.user
            print(f"DEBUG: Serializer valid. Old password check...")
            if not user.check_password(serializer.data.get("old_password")):
                print(f"DEBUG: Wrong old password for {user.email}")
                return Response({"old_password": ["Wrong password."]}, status=status.HTTP_400_BAD_REQUEST)
            
            print(f"DEBUG: Setting new password for {user.email}")
            user.set_password(serializer.data.get("new_password"))
            user.save()
            print(f"DEBUG: Password saved successfully for {user.email}")
            return Response({"status": "success", "message": "Password updated successfully"}, status=status.HTTP_200_OK)

        print(f"DEBUG: Serializer invalid: {serializer.errors}")
        return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)

