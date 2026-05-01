import { useState } from 'react';
import { Routes, Route, Navigate } from 'react-router-dom';
import Sidebar from './components/Sidebar';
import Header from './components/Header';
import LoginPage from './pages/LoginPage';
import DashboardPage from './pages/DashboardPage';
import UsersPage from './pages/UsersPage';
import MedicalRecordsPage from './pages/MedicalRecordsPage';
import ChatMonitorPage from './pages/ChatMonitorPage';
import SettingsPage from './pages/SettingsPage';
import AlertsPage from './pages/AlertsPage';

function ProtectedRoute({ children }) {
  const token = localStorage.getItem('admin_token');
  if (!token) return <Navigate to="/login" replace />;
  return children;
}

export default function App() {
  const [sidebarOpen, setSidebarOpen] = useState(true);

  const token = localStorage.getItem('admin_token');

  return (
    <Routes>
      <Route path="/login" element={<LoginPage />} />
      <Route
        path="/*"
        element={
          <ProtectedRoute>
            <div className="app-layout">
              <Sidebar isOpen={sidebarOpen} onToggle={() => setSidebarOpen(!sidebarOpen)} />
              <div className="main-content" style={{ marginLeft: sidebarOpen ? 'var(--sidebar-width)' : 'var(--sidebar-collapsed)' }}>
                <Header onMenuToggle={() => setSidebarOpen(!sidebarOpen)} />
                <div className="page-wrapper">
                  <Routes>
                    <Route path="/" element={<DashboardPage />} />
                    <Route path="/users" element={<UsersPage />} />
                    <Route path="/records" element={<MedicalRecordsPage />} />
                    <Route path="/chats" element={<ChatMonitorPage />} />
                    <Route path="/settings" element={<SettingsPage />} />
                    <Route path="/alerts" element={<AlertsPage />} />
                    <Route path="*" element={<Navigate to="/" replace />} />
                  </Routes>
                </div>
              </div>
            </div>
          </ProtectedRoute>
        }
      />
    </Routes>
  );
}
