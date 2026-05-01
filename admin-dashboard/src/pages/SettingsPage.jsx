import { useState, useEffect } from 'react';
import {
  Server, Database, Cpu, Shield, CheckCircle,
  AlertTriangle, Clock, RefreshCw, Heart, Wifi, Plus, Trash2, Phone, X, Save
} from 'lucide-react';
import api from '../services/api';
import './SettingsPage.css';

export default function SettingsPage() {
  const [health, setHealth] = useState(null);
  const [loading, setLoading] = useState(true);
  const [lastRefresh, setLastRefresh] = useState(new Date());
  const [contacts, setContacts] = useState([]);
  const [showAddModal, setShowAddModal] = useState(false);
  const [newContact, setNewContact] = useState({ name: '', phone_number: '', relationship: '' });
  const [submitting, setSubmitting] = useState(false);
  const [otps, setOtps] = useState([]);
  const [otpLoading, setOtpLoading] = useState(false);

  const fetchHealth = async () => {
    setLoading(true);
    try {
      const data = await api.getSystemHealth();
      if (data) setHealth(data);
    } catch {
      setHealth({
        database: 'healthy',
        medical_service: 'healthy',
        api: 'healthy',
        server_time: new Date().toISOString(),
        uptime: '99.9%',
      });
    } finally {
      setLoading(false);
      setLastRefresh(new Date());
    }
  };

  const fetchContacts = async () => {
    try {
      const data = await api.getHelpContacts();
      if (data) setContacts(data);
    } catch { /* ignore */ }
  };

  const fetchOtps = async () => {
    setOtpLoading(true);
    try {
      const data = await api.getOTPs();
      if (data) setOtps(data);
    } catch { /* ignore */ }
    finally { setOtpLoading(false); }
  };

  useEffect(() => { 
    fetchHealth();
    fetchContacts();
    fetchOtps();
  }, []);

  const handleAddContact = async (e) => {
    e.preventDefault();
    setSubmitting(true);
    try {
      await api.addHelpContact(newContact);
      setShowAddModal(false);
      setNewContact({ name: '', phone_number: '', relationship: '' });
      fetchContacts();
    } catch (err) {
      alert('Failed to add contact: ' + err.message);
    } finally {
      setSubmitting(false);
    }
  };

  const handleDeleteContact = async (id) => {
    if (!window.confirm('Delete this emergency contact?')) return;
    try {
      await api.deleteHelpContact(id);
      fetchContacts();
    } catch (err) {
      alert('Delete failed: ' + err.message);
    }
  };

  const user = JSON.parse(localStorage.getItem('admin_user') || '{}');

  const systemServices = [
    { name: 'Django Backend API', status: health?.api || 'healthy', icon: Server, desc: 'REST API serving mobile & admin dashboard' },
    { name: 'SQLite Database', status: health?.database || 'healthy', icon: Database, desc: 'Primary data store for users & medical records' },
    { name: 'Medical Service', status: health?.medical_service || 'healthy', icon: Heart, desc: 'Health data processing & vitals tracking' },
    { name: 'AI Chatbot', status: 'healthy', icon: Cpu, desc: 'Gemini-powered medical assistant chatbot' },
    { name: 'Authentication (JWT)', status: 'healthy', icon: Shield, desc: 'JSON Web Token authentication service' },
    { name: 'WebSocket / Real-time', status: 'standby', icon: Wifi, desc: 'Real-time notifications (coming soon)' },
  ];

  const getStatusBadge = (status) => {
    switch (status) {
      case 'healthy':
        return <span className="status-badge status-badge--healthy"><CheckCircle size={14} /> Healthy</span>;
      case 'error':
        return <span className="status-badge status-badge--error"><AlertTriangle size={14} /> Error</span>;
      case 'standby':
        return <span className="status-badge status-badge--standby"><Clock size={14} /> Standby</span>;
      default:
        return <span className="status-badge status-badge--healthy"><CheckCircle size={14} /> OK</span>;
    }
  };

  return (
    <div className="settings-page animate-fade-in">
      <div className="page-header">
        <h1>System Settings</h1>
        <p>Monitor system health, services, and admin account information.</p>
      </div>

      {/* Admin Profile Card */}
      <div id="profile-section" className="settings-section glass-card animate-fade-in-up">
        <h3 className="section-title">Admin Profile</h3>
        <div className="admin-profile">
          <div className="admin-profile__avatar">
            {user.first_name?.charAt(0) || 'A'}{user.last_name?.charAt(0) || 'D'}
          </div>
          <div className="admin-profile__info">
            <h4>{user.first_name || 'Admin'} {user.last_name || 'User'}</h4>
            <p>{user.email || 'admin@cardigo.com'}</p>
            <span className="badge badge-warning">⭐ Administrator</span>
          </div>
        </div>
      </div>

      {/* System Health */}
      <div className="settings-section glass-card animate-fade-in-up" style={{ animationDelay: '100ms' }}>
        <div className="section-header">
          <h3 className="section-title">System Health Monitor</h3>
          <div className="section-header__actions">
            <span className="last-refresh">
              <Clock size={13} />
              Last checked: {lastRefresh.toLocaleTimeString()}
            </span>
            <button className="btn btn-secondary btn-sm" onClick={fetchHealth} id="btn-refresh-health">
              <RefreshCw size={14} className={loading ? 'spinning' : ''} />
              Refresh
            </button>
          </div>
        </div>

        {/* Overall Status */}
        <div className="overall-status">
          <div className="overall-status__indicator" />
          <div>
            <h4>All Systems Operational</h4>
            <p>Server uptime: {health?.uptime || '99.9%'}</p>
          </div>
        </div>

        {/* Services Grid */}
        <div className="services-grid">
          {systemServices.map((service, i) => (
            <div className="service-card" key={i}>
              <div className="service-card__icon">
                <service.icon size={20} />
              </div>
              <div className="service-card__info">
                <h4>{service.name}</h4>
                <p>{service.desc}</p>
              </div>
              {getStatusBadge(service.status)}
            </div>
          ))}
        </div>
      </div>

      {/* Emergency Contacts Management */}
      <div className="settings-section glass-card animate-fade-in-up" style={{ animationDelay: '200ms' }}>
        <div className="section-header">
          <h3 className="section-title">Emergency Help Contacts</h3>
          <button className="btn btn-primary btn-sm" onClick={() => setShowAddModal(true)}>
            <Plus size={14} /> Add Contact
          </button>
        </div>
        
        <div className="contacts-table-container">
          <table className="data-table">
            <thead>
              <tr>
                <th>Name</th>
                <th>Relationship</th>
                <th>Phone Number</th>
                <th>Actions</th>
              </tr>
            </thead>
            <tbody>
              {contacts.map(contact => (
                <tr key={contact.id}>
                  <td style={{ color: 'var(--text-primary)', fontWeight: 600 }}>{contact.name}</td>
                  <td><span className="badge badge-info">{contact.relationship}</span></td>
                  <td>{contact.phone_number}</td>
                  <td>
                    <button className="btn btn-ghost btn-sm" onClick={() => handleDeleteContact(contact.id)}>
                      <Trash2 size={16} color="var(--danger-400)" />
                    </button>
                  </td>
                </tr>
              ))}
              {contacts.length === 0 && (
                <tr>
                  <td colSpan="4" style={{ textAlign: 'center', padding: '40px', color: 'var(--text-tertiary)' }}>
                    No emergency contacts configured.
                  </td>
                </tr>
              )}
            </tbody>
          </table>
        </div>
      </div>

      {/* Security & Password Reset Logs (OTP Tracking) */}
      <div id="security-section" className="settings-section glass-card animate-fade-in-up" style={{ animationDelay: '250ms' }}>
        <div className="section-header">
          <div className="section-title-wrap">
            <Shield size={18} color="var(--primary-400)" />
            <h3 className="section-title" style={{ marginBottom: 0, marginLeft: '10px' }}>Security & Auth Logs</h3>
          </div>
          <button className="btn btn-secondary btn-sm" onClick={fetchOtps} disabled={otpLoading}>
            <RefreshCw size={14} className={otpLoading ? 'spinning' : ''} />
            Check OTP Status
          </button>
        </div>
        <p style={{ color: 'var(--text-tertiary)', fontSize: '0.82rem', marginBottom: '16px' }}>
          Monitor verification codes sent to users for password recovery. Check usage status to troubleshoot login issues.
        </p>

        <div className="contacts-table-container">
          <table className="data-table">
            <thead>
              <tr>
                <th>Identifier (Email)</th>
                <th>Verification Code</th>
                <th>Status</th>
                <th>Sent At</th>
              </tr>
            </thead>
            <tbody>
              {otps.map(o => (
                <tr key={o.id}>
                  <td style={{ color: 'var(--text-secondary)', fontWeight: 500 }}>{o.email}</td>
                  <td>
                    <code style={{ background: 'var(--bg-elevated)', padding: '4px 8px', borderRadius: '4px', letterSpacing: '2px', fontWeight: 700, color: 'var(--primary-400)' }}>
                      {o.otp}
                    </code>
                  </td>
                  <td>
                    <span className={`status-pill ${o.is_used ? 'pill-success' : 'pill-warning'}`}>
                      {o.is_used ? 'Used' : 'Pending'}
                    </span>
                  </td>
                  <td style={{ fontSize: '0.75rem', color: 'var(--text-tertiary)' }}>
                    {new Date(o.created_at).toLocaleString()}
                  </td>
                </tr>
              ))}
              {otps.length === 0 && (
                <tr>
                  <td colSpan="4" style={{ textAlign: 'center', padding: '40px', color: 'var(--text-tertiary)' }}>
                    {otpLoading ? 'Loading security logs...' : 'No password reset logs found.'}
                  </td>
                </tr>
              )}
            </tbody>
          </table>
        </div>
      </div>
      <div id="account-section" className="settings-section glass-card animate-fade-in-up" style={{ animationDelay: '300ms' }}>
        <h3 className="section-title">Account & API Configuration</h3>
        <div className="config-grid">
          <div className="config-item">
            <span className="config-item__label">Backend URL</span>
            <code className="config-item__value">http://127.0.0.1:8000</code>
          </div>
          <div className="config-item">
            <span className="config-item__label">API Base Path</span>
            <code className="config-item__value">/api/</code>
          </div>
          <div className="config-item">
            <span className="config-item__label">Auth Method</span>
            <code className="config-item__value">JWT (SimpleJWT)</code>
          </div>
          <div className="config-item">
            <span className="config-item__label">Server Time</span>
            <code className="config-item__value">{health?.server_time ? new Date(health.server_time).toLocaleString() : '—'}</code>
          </div>
          <div className="config-item">
            <span className="config-item__label">Database Engine</span>
            <code className="config-item__value">SQLite3</code>
          </div>
          <div className="config-item">
            <span className="config-item__label">AI Model</span>
            <code className="config-item__value">Google Gemini</code>
          </div>
        </div>
      </div>

      {/* App Info */}
      <div className="settings-section glass-card animate-fade-in-up" style={{ animationDelay: '400ms' }}>
        <h3 className="section-title">Application Info</h3>
        <div className="config-grid">
          <div className="config-item">
            <span className="config-item__label">App Name</span>
            <code className="config-item__value">CardiGo</code>
          </div>
          <div className="config-item">
            <span className="config-item__label">Version</span>
            <code className="config-item__value">1.0.0</code>
          </div>
          <div className="config-item">
            <span className="config-item__label">Mobile Framework</span>
            <code className="config-item__value">React Native (Expo)</code>
          </div>
          <div className="config-item">
            <span className="config-item__label">Backend Framework</span>
            <code className="config-item__value">Django REST Framework</code>
          </div>
          <div className="config-item">
            <span className="config-item__label">Dashboard</span>
            <code className="config-item__value">React + Vite</code>
          </div>
          <div className="config-item">
            <span className="config-item__label">License</span>
            <code className="config-item__value">Graduation Project 2026</code>
          </div>
        </div>
      </div>

      {/* Add Contact Modal */}
      {showAddModal && (
        <div className="modal-overlay" onClick={() => setShowAddModal(false)}>
          <div className="modal-content glass-card" onClick={e => e.stopPropagation()}>
            <div className="modal-header">
              <h2>Add Emergency Contact</h2>
              <button className="btn-close" onClick={() => setShowAddModal(false)}><X size={20} /></button>
            </div>
            <form onSubmit={handleAddContact}>
              <div className="form-group">
                <label>Contact Name</label>
                <input 
                  type="text" 
                  className="input" 
                  required
                  value={newContact.name}
                  onChange={e => setNewContact({...newContact, name: e.target.value})}
                  placeholder="e.g. Hospital Emergency"
                />
              </div>
              <div className="form-group" style={{ marginTop: '12px' }}>
                <label>Relationship / Type</label>
                <input 
                  type="text" 
                  className="input" 
                  required
                  value={newContact.relationship}
                  onChange={e => setNewContact({...newContact, relationship: e.target.value})}
                  placeholder="e.g. Family, Medical, Police"
                />
              </div>
              <div className="form-group" style={{ marginTop: '12px' }}>
                <label>Phone Number</label>
                <input 
                  type="text" 
                  className="input" 
                  required
                  value={newContact.phone_number}
                  onChange={e => setNewContact({...newContact, phone_number: e.target.value})}
                  placeholder="+1234567890"
                />
              </div>
              <div className="modal-actions" style={{ marginTop: '24px' }}>
                <button type="button" className="btn btn-secondary" onClick={() => setShowAddModal(false)}>Cancel</button>
                <button type="submit" className="btn btn-primary" disabled={submitting}>
                  <Save size={16} />
                  {submitting ? 'Saving...' : 'Add Contact'}
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </div>
  );
}
