from django.urls import path
from rest_framework_simplejwt.views import TokenRefreshView
from .views import (
    RegisterView, CustomTokenObtainPairView, ProfileView,
    HealthDataListCreateView, AlertHistoryListView, ChatMessageListCreateView,
    ChatMessageClearView, PasswordResetRequestView, PasswordResetConfirmView
)

urlpatterns = [
    path('auth/register/', RegisterView.as_view(), name='register'),
    path('auth/login/', CustomTokenObtainPairView.as_view(), name='login'),
    path('auth/refresh/', TokenRefreshView.as_view(), name='token_refresh'),
    path('user/profile/', ProfileView.as_view(), name='profile'),
    path('health/data/', HealthDataListCreateView.as_view(), name='health_data'),
    path('health/alerts/', AlertHistoryListView.as_view(), name='alerts'),
    path('chat/', ChatMessageListCreateView.as_view(), name='chat'),
    path('chat/clear/', ChatMessageClearView.as_view(), name='chat_clear'),
    path('auth/password-reset-request/', PasswordResetRequestView.as_view(), name='password_reset_request'),
    path('auth/password-reset-confirm/', PasswordResetConfirmView.as_view(), name='password_reset_confirm'),
]
