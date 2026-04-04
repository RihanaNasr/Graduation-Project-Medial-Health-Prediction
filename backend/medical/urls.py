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

urlpatterns = [
    path('record/', MedicalRecordView.as_view(), name='medical-record'),
    path('records/', MedicalRecordListView.as_view(), name='medical-records-list'),
    path('chat/', ChatView.as_view(), name='chat'),
    path('chat/history/', ChatHistoryView.as_view(), name='chat-history'),
    path('help/contacts/', HelpContactListView.as_view(), name='help-contacts'),
    path('sos/', SOSView.as_view(), name='sos'),
    path('report/export/', ReportExportView.as_view(), name='report-export'),
]
