import { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import { AlertCircle, AlertTriangle, Info, Search, Filter, Clock, ChevronRight, CheckCircle2, User } from 'lucide-react';
import api from '../services/api';
import './AlertsPage.css';

export default function AlertsPage() {
  const navigate = useNavigate();
  const [alerts, setAlerts] = useState([]);
  const [filter, setFilter] = useState('all');
  const [loading, setLoading] = useState(true);
  const [resolvingId, setResolvingId] = useState(null);

  const fetchAlerts = async () => {
    setLoading(true);
    try {
      const data = await api.getAlerts(filter);
      if (data) setAlerts(data);
    } catch (err) {
      console.error("Failed to fetch alerts:", err);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchAlerts();
    const interval = setInterval(fetchAlerts, 5000);
    return () => clearInterval(interval);
  }, [filter]);

  const handleResolve = async (alertId) => {
    setResolvingId(alertId);
    try {
      await api.resolveAlert(alertId);
      // Update local state
      setAlerts(prev => prev.map(a => 
        a.id === alertId ? { ...a, is_resolved: true, resolved_at: new Date().toISOString() } : a
      ));
    } catch (err) {
      console.error("Failed to resolve alert:", err);
    } finally {
      setResolvingId(null);
    }
  };

  const handleViewVitals = (userEmail) => {
    // Navigate to medical records page with search query
    navigate(`/records?search=${userEmail}`);
  };

  const getPriorityIcon = (priority) => {
    switch(priority) {
      case 'critical': return <AlertCircle className="text-danger" size={20} />;
      case 'warning': return <AlertTriangle className="text-warning" size={20} />;
      default: return <Info className="text-info" size={20} />;
    }
  };

  return (
    <div className="alerts-page animate-fade-in">
      <div className="page-header">
        <div className="page-header__left">
          <h1>Health Alert Center</h1>
          <p>Review and resolve critical medical alerts triggered by patient readings.</p>
        </div>
        <div className="page-header__right">
          <button className="btn btn-secondary btn-sm" onClick={fetchAlerts} disabled={loading}>
            Refresh Feed
          </button>
        </div>
      </div>

      <div className="alerts-toolbar glass-card">
        <div className="filter-group">
          <button className={`filter-chip ${filter === 'all' ? 'active' : ''}`} onClick={() => setFilter('all')}>All Alerts</button>
          <button className={`filter-chip ${filter === 'unresolved' ? 'active' : ''}`} onClick={() => setFilter('unresolved')}>Unresolved</button>
          <button className={`filter-chip ${filter === 'critical' ? 'active' : ''}`} onClick={() => setFilter('critical')}>Critical Only</button>
        </div>
        <div className="record-count">{alerts.length} record(s) found</div>
      </div>

      {loading ? (
        <div className="loading-state">
          <div className="loader"></div>
          <p>Analyzing system data...</p>
        </div>
      ) : (
        <div className="alerts-grid stagger">
          {alerts.length === 0 ? (
            <div className="empty-state glass-card">
              <CheckCircle2 size={48} color="var(--success-400)" />
              <h3>All clear!</h3>
              <p>No health alerts currently require your attention.</p>
            </div>
          ) : (
            alerts.map((alert) => (
              <div className={`alert-detail-card glass-card animate-fade-in-up ${alert.is_resolved ? 'resolved' : ''}`} key={alert.id}>
                <div className={`alert-priority-line bg-${alert.priority}`}></div>
                <div className="alert-body">
                  <div className="alert-meta">
                    <div className="alert-icon-box">
                      {getPriorityIcon(alert.priority)}
                    </div>
                    <div className="alert-user-info">
                      <h4>{alert.user_name || 'Anonymous Patient'}</h4>
                      <span>{alert.user_email}</span>
                    </div>
                    <div className="alert-timestamp">
                      <Clock size={12} />
                      {new Date(alert.timestamp).toLocaleString()}
                    </div>
                  </div>
                  
                  <div className="alert-message-box">
                    <p>{alert.message}</p>
                  </div>

                  <div className="alert-footer">
                    <span className={`status-pill ${alert.is_resolved ? 'pill-success' : 'pill-danger'}`}>
                      {alert.is_resolved ? <CheckCircle2 size={12} /> : <AlertCircle size={12} />}
                      {alert.is_resolved ? 'Resolved' : 'Needs Attention'}
                    </span>
                    <div className="alert-actions">
                      {!alert.is_resolved && (
                        <button 
                          className="btn btn-primary btn-sm" 
                          onClick={() => handleResolve(alert.id)}
                          disabled={resolvingId === alert.id}
                        >
                          {resolvingId === alert.id ? 'Resolving...' : 'Resolve Now'}
                        </button>
                      )}
                      <button 
                        className="btn btn-ghost btn-sm"
                        onClick={() => handleViewVitals(alert.user_email)}
                      >
                        View Vitals <ChevronRight size={14} />
                      </button>
                    </div>
                  </div>
                </div>
              </div>
            ))
          )}
        </div>
      )}
    </div>
  );
}
