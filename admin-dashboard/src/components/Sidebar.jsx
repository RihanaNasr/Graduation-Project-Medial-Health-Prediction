import { NavLink, useNavigate } from 'react-router-dom';
import { Heart, LayoutDashboard, Users, FileHeart, MessageSquare, Settings, LogOut, ChevronLeft, ChevronRight, Activity, Bell } from 'lucide-react';
import api from '../services/api';
import './Sidebar.css';

export default function Sidebar({ isOpen, onToggle }) {
  const navigate = useNavigate();

  const handleLogout = () => {
    api.clearToken();
    navigate('/login');
  };

  const navItems = [
    { path: '/', icon: LayoutDashboard, label: 'Dashboard' },
    { path: '/users', icon: Users, label: 'Users' },
    { path: '/records', icon: FileHeart, label: 'Medical Records' },
    { path: '/chats', icon: MessageSquare, label: 'Chat Monitor' },
    { path: '/alerts', icon: Bell, label: 'Alert Center' },
    { path: '/settings', icon: Settings, label: 'Settings' },
  ];

  return (
    <aside className={`sidebar ${isOpen ? 'sidebar--open' : 'sidebar--collapsed'}`}>
      {/* Logo */}
      <div className="sidebar__logo">
        <div className="sidebar__logo-icon">
          <Heart size={24} color="white" fill="white" />
        </div>
        {isOpen && (
          <div className="sidebar__logo-text">
            <h2>Cardi<span>Go</span></h2>
            <span className="sidebar__logo-sub">Admin Panel</span>
          </div>
        )}
      </div>

      {/* Toggle Button */}
      <button className="sidebar__toggle" onClick={onToggle} id="sidebar-toggle">
        {isOpen ? <ChevronLeft size={18} /> : <ChevronRight size={18} />}
      </button>

      {/* Navigation */}
      <nav className="sidebar__nav">
        <span className="sidebar__section-label">{isOpen ? 'MAIN MENU' : ''}</span>
        {navItems.map((item) => (
          <NavLink
            key={item.path}
            to={item.path}
            end={item.path === '/'}
            className={({ isActive }) =>
              `sidebar__link ${isActive ? 'sidebar__link--active' : ''}`
            }
            id={`nav-${item.label.toLowerCase().replace(/\s/g, '-')}`}
          >
            <item.icon size={20} />
            {isOpen && <span>{item.label}</span>}
          </NavLink>
        ))}
      </nav>

      {/* System Status */}
      {isOpen && (
        <div className="sidebar__status">
          <Activity size={14} />
          <span>System Online</span>
          <div className="sidebar__status-dot" />
        </div>
      )}

      {/* Logout */}
      <button className="sidebar__logout" onClick={handleLogout} id="btn-logout">
        <LogOut size={20} />
        {isOpen && <span>Log Out</span>}
      </button>
    </aside>
  );
}
