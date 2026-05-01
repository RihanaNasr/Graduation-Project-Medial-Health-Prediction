import { useState, useEffect, useRef } from 'react';
import { Bell, Search, Menu, LogOut, User as UserIcon, Settings as SettingsIcon, ShieldCheck } from 'lucide-react';
import { useNavigate } from 'react-router-dom';
import api from '../services/api';
import './Header.css';

export default function Header({ onMenuToggle }) {
  const [user, setUser] = useState(null);
  const [searchQuery, setSearchQuery] = useState('');
  const [currentTime, setCurrentTime] = useState(new Date());
  const [showNotifications, setShowNotifications] = useState(false);
  const [showProfile, setShowProfile] = useState(false);
  
  const notificationRef = useRef(null);
  const profileRef = useRef(null);
  const navigate = useNavigate();

  const [realAlerts, setRealAlerts] = useState([]);

  const fetchAlerts = async () => {
    try {
      const data = await api.getAlerts('unresolved');
      if (data) setRealAlerts(data.slice(0, 5)); // Only show top 5 in dropdown
    } catch { /* ignore */ }
  };

  useEffect(() => {
    const saved = localStorage.getItem('admin_user');
    if (saved) setUser(JSON.parse(saved));

    const timer = setInterval(() => setCurrentTime(new Date()), 60000);
    
    // Initial fetch and poll every 30s for notifications
    fetchAlerts();
    const alertTimer = setInterval(fetchAlerts, 30000);
    
    // Close dropdowns when clicking outside
    const handleClickOutside = (event) => {
      if (notificationRef.current && !notificationRef.current.contains(event.target)) {
        setShowNotifications(false);
      }
      if (profileRef.current && !profileRef.current.contains(event.target)) {
        setShowProfile(false);
      }
    };
    
    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      clearInterval(timer);
      clearInterval(alertTimer);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const handleLogout = () => {
    localStorage.clear();
    navigate('/login');
  };

  const getTimeAgo = (timestamp) => {
    const diff = new Date() - new Date(timestamp);
    const mins = Math.floor(diff / 60000);
    if (mins < 1) return 'now';
    if (mins < 60) return `${mins}m ago`;
    const hours = Math.floor(mins / 60);
    if (hours < 24) return `${hours}h ago`;
    return `${Math.floor(hours / 24)}d ago`;
  };

  const greeting = () => {
    const hour = currentTime.getHours();
    if (hour < 12) return 'Good Morning';
    if (hour < 18) return 'Good Afternoon';
    return 'Good Evening';
  };

  return (
    <header className="header">
      <div className="header__left">
        <button className="header__menu-btn" onClick={onMenuToggle} id="menu-toggle">
          <Menu size={20} />
        </button>
        <div className="header__greeting">
          <h2>{greeting()}, <span>{user?.first_name || 'Admin'}</span> 👋</h2>
          <p>{currentTime.toLocaleDateString('en-US', { weekday: 'long', month: 'long', day: 'numeric', year: 'numeric' })}</p>
        </div>
      </div>

      <div className="header__right">
        <div className="header__search">
          <Search size={16} />
          <input
            type="text"
            placeholder="Search anything..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
            className="header__search-input"
            id="header-search"
          />
        </div>

        {/* Notifications Dropdown */}
        <div className="header__dropdown-container" ref={notificationRef}>
          <button 
            className={`header__notification ${showNotifications ? 'active' : ''}`} 
            onClick={() => { setShowNotifications(!showNotifications); setShowProfile(false); }}
            id="btn-notifications"
          >
            <Bell size={20} />
            {realAlerts.length > 0 && (
              <span className="header__notification-badge">{realAlerts.length}</span>
            )}
          </button>
          
          {showNotifications && (
            <div className="dropdown dropdown--notifications glass-card animate-fade-in-up">
              <div className="dropdown__header">
                <h3>Notifications</h3>
                <button className="text-btn">Mark all as read</button>
              </div>
              <div className="dropdown__body">
                {realAlerts.length === 0 ? (
                  <div className="dropdown__empty">
                    <ShieldCheck size={32} color="var(--success-400)" />
                    <p>No new alerts. Your system is safe.</p>
                  </div>
                ) : (
                  realAlerts.map(n => (
                    <div key={n.id} className="dropdown__item notification-item" onClick={() => navigate('/alerts')}>
                      <div className={`notification-dot notification-dot--${n.priority === 'critical' ? 'danger' : n.priority === 'warning' ? 'warning' : 'info'}`} />
                      <div className="notification-content">
                        <p><strong>{n.user_name}:</strong> {n.message}</p>
                        <span>{getTimeAgo(n.timestamp)}</span>
                      </div>
                    </div>
                  ))
                )}
              </div>
              <div className="dropdown__footer">
                <button className="btn btn-ghost btn-sm full-width" onClick={() => navigate('/alerts')}>View All Alerts</button>
              </div>
            </div>
          )}
        </div>

        {/* Profile Dropdown */}
        <div className="header__dropdown-container" ref={profileRef}>
          <div 
            className={`header__avatar ${showProfile ? 'active' : ''}`} 
            onClick={() => { setShowProfile(!showProfile); setShowNotifications(false); }}
            id="admin-avatar"
          >
            <span>{user?.first_name?.charAt(0) || 'A'}{user?.last_name?.charAt(0) || 'D'}</span>
          </div>

          {showProfile && (
            <div className="dropdown dropdown--profile glass-card animate-fade-in-up">
              <div className="dropdown__user-info">
                <div className="dropdown__avatar-large">
                  {user?.first_name?.charAt(0)}{user?.last_name?.charAt(0)}
                </div>
                <div>
                  <h4>{user?.first_name} {user?.last_name}</h4>
                  <p>{user?.email}</p>
                  <span className="badge badge-warning">⭐ Administrator</span>
                </div>
              </div>
              <div className="dropdown__divider" />
              <div className="dropdown__body">
                <button className="dropdown__item" onClick={() => { navigate('/settings'); setShowProfile(false); setTimeout(() => document.getElementById('profile-section')?.scrollIntoView({behavior: 'smooth'}), 100); }}>
                  <UserIcon size={16} /> My Profile
                </button>
                <button className="dropdown__item" onClick={() => { navigate('/settings'); setShowProfile(false); setTimeout(() => document.getElementById('account-section')?.scrollIntoView({behavior: 'smooth'}), 100); }}>
                  <SettingsIcon size={16} /> Account Settings
                </button>
                <button className="dropdown__item" onClick={() => { navigate('/settings'); setShowProfile(false); setTimeout(() => document.getElementById('security-section')?.scrollIntoView({behavior: 'smooth'}), 100); }}>
                  <ShieldCheck size={16} /> Security
                </button>
              </div>
              <div className="dropdown__divider" />
              <button className="dropdown__item dropdown__item--logout" onClick={handleLogout}>
                <LogOut size={16} /> Sign Out
              </button>
            </div>
          )}
        </div>
      </div>
    </header>
  );
}
