from django.urls import path
from .views import (
    MedicalRecordView,
    MedicalRecordListView,
    ChatView,
    ChatHistoryView,
    HelpContactListView,
    SOSView,
    ReportExportView
)
from .admin_views import (
    AdminDashboardStatsView,
    AdminUsersListView,
    AdminUserDetailView,
    AdminMedicalRecordsView,
    AdminMedicalRecordDetailView,
    AdminChatMessagesView,
    AdminSystemHealthView,
    AdminHelpContactsView,
    AdminHelpContactDetailView,
    AdminDataExportView,
    AdminAlertsView,
    AdminAlertDetailView,
    AdminOTPsView,
)

urlpatterns = [
    # Existing mobile API routes (untouched)
    path('record/', MedicalRecordView.as_view(), name='medical-record'),
    path('records/', MedicalRecordListView.as_view(), name='medical-records-list'),
    path('chat/', ChatView.as_view(), name='chat'),
    path('chat/history/', ChatHistoryView.as_view(), name='chat-history'),
    path('help/contacts/', HelpContactListView.as_view(), name='help-contacts'),
    path('sos/', SOSView.as_view(), name='sos'),
    path('report/export/', ReportExportView.as_view(), name='report-export'),

    # Admin Dashboard API routes
    path('admin/stats/', AdminDashboardStatsView.as_view(), name='admin-stats'),
    path('admin/users/', AdminUsersListView.as_view(), name='admin-users'),
    path('admin/users/<int:user_id>/', AdminUserDetailView.as_view(), name='admin-user-detail'),
    path('admin/records/', AdminMedicalRecordsView.as_view(), name='admin-records'),
    path('admin/records/<int:record_id>/', AdminMedicalRecordDetailView.as_view(), name='admin-record-detail'),
    path('admin/chats/', AdminChatMessagesView.as_view(), name='admin-chats'),
    path('admin/health/', AdminSystemHealthView.as_view(), name='admin-health'),
    path('admin/help-contacts/', AdminHelpContactsView.as_view(), name='admin-help-contacts'),
    path('admin/help-contacts/<int:contact_id>/', AdminHelpContactDetailView.as_view(), name='admin-help-contact-detail'),
    path('admin/export/', AdminDataExportView.as_view(), name='admin-export'),
    path('admin/alerts/', AdminAlertsView.as_view(), name='admin-alerts'),
    path('admin/alerts/<int:alert_id>/', AdminAlertDetailView.as_view(), name='admin-alert-detail'),
    path('admin/otps/', AdminOTPsView.as_view(), name='admin-otps'),
]
