import { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import {
  Users, FileHeart, MessageSquare, Activity,
  Heart, Footprints, TrendingUp, Thermometer,
  ArrowUpRight, ArrowDownRight, Droplets,
  Zap, Bell, Clock, AlertCircle, AlertTriangle, ShieldCheck
} from 'lucide-react';
import {
  AreaChart, Area, BarChart, Bar, XAxis, YAxis,
  CartesianGrid, Tooltip, ResponsiveContainer, PieChart, Pie, Cell
} from 'recharts';
import api from '../services/api';
import './DashboardPage.css';

/* ---- custom tooltip ---- */
const CustomTooltip = ({ active, payload, label }) => {
  if (!active || !payload?.length) return null;
  return (
    <div className="chart-tooltip">
      <p className="chart-tooltip__label">{label}</p>
      <p className="chart-tooltip__value">{payload[0].value}{payload[0].name === 'rate' ? '%' : ''}</p>
    </div>
  );
};

/* ---- mock fallback data ---- */
const fallbackStats = {
  total_users: 24,
  active_users: 18,
  total_records: 22,
  total_chats: 156,
  total_contacts: 8,
  vitals_avg: { avg_heart_rate: 82, avg_spo2: 97, avg_temperature: 36.7, avg_steps: 7800, avg_calories: 1950 },
  daily_registrations: [
    { date: 'Mon', count: 3 }, { date: 'Tue', count: 5 }, { date: 'Wed', count: 2 },
    { date: 'Thu', count: 7 }, { date: 'Fri', count: 4 }, { date: 'Sat', count: 6 }, { date: 'Sun', count: 3 }
  ],
  daily_chats: [
    { date: 'Mon', count: 12 }, { date: 'Tue', count: 19 }, { date: 'Wed', count: 8 },
    { date: 'Thu', count: 25 }, { date: 'Fri', count: 15 }, { date: 'Sat', count: 22 }, { date: 'Sun', count: 10 }
  ],
  recent_actions: [
    { user: 'System', action: 'Database optimized', time: '2h ago', type: 'success' },
    { user: 'Admin', action: 'Security scan completed', time: '5h ago', type: 'info' },
    { user: 'New User', action: 'Account registered', time: '12m ago', type: 'new' }
  ],
  health_alerts: [
    { id: 1, user: 'Tasneem', message: 'Critical: Heart Rate 40, SpO2 98', time: '14:16', priority: 'critical' },
    { id: 2, user: 'Omar Khaled', message: 'Warning: Heart Rate 120, SpO2 95', time: '01:45', priority: 'warning' },
    { id: 3, user: 'Sara Ahmed', message: 'Fever detected: 39.2°C', time: '10:22', priority: 'warning' },
    { id: 4, user: 'Nour Hassan', message: 'System Sync: Vitals updated', time: '08:12', priority: 'info' },
  ],
  severity_trend: [
    { time: '00:00', rate: 2 }, { time: '04:00', rate: 5 }, { time: '08:00', rate: 3 },
    { time: '12:00', rate: 8 }, { time: '16:00', rate: 4 }, { time: '20:00', rate: 6 }, { time: '23:00', rate: 10 }
  ]
};

const pieColors = ['#ef4444', '#3b82f6'];

export default function DashboardPage() {
  const navigate = useNavigate();
  const [stats, setStats] = useState(fallbackStats);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchStats = async () => {
      try {
        const data = await api.getStats();
        if (data) setStats(data);
      } catch {
        // fallback mock is already set
      } finally {
        setLoading(false);
      }
    };
    fetchStats();
    const interval = setInterval(fetchStats, 5000);
    return () => clearInterval(interval);
  }, []);

  const statCards = [
    {
      label: 'Total Users',
      value: stats.total_users,
      icon: Users,
      change: '+12%',
      up: true,
      color: 'var(--primary-500)',
      bg: 'rgba(239, 68, 68, 0.1)',
    },
    {
      label: 'Active Users',
      value: stats.active_users,
      icon: Activity,
      change: '+5%',
      up: true,
      color: 'var(--success-400)',
      bg: 'rgba(34, 197, 94, 0.1)',
    },
    {
      label: 'Medical Records',
      value: stats.total_records,
      icon: FileHeart,
      change: '+8%',
      up: true,
      color: 'var(--accent-400)',
      bg: 'rgba(59, 130, 246, 0.1)',
    },
    {
      label: 'AI Conversations',
      value: stats.total_chats,
      icon: MessageSquare,
      change: '+23%',
      up: true,
      color: 'var(--warning-500)',
      bg: 'rgba(245, 158, 11, 0.1)',
    },
  ];

  const vitalCards = [
    { label: 'Avg Heart Rate', value: `${Math.round(stats.vitals_avg?.avg_heart_rate || 82)} bpm`, icon: Heart, color: '#ef4444' },
    { label: 'Avg SpO₂', value: `${Math.round(stats.vitals_avg?.avg_spo2 || 97)}%`, icon: Droplets, color: '#3b82f6' },
    { label: 'Avg Temperature', value: `${(stats.vitals_avg?.avg_temperature || 36.7).toFixed(1)}°C`, icon: Thermometer, color: '#22c55e' },
    { label: 'Avg Steps', value: (stats.vitals_avg?.avg_steps || 7800).toLocaleString(), icon: Footprints, color: '#f59e0b' },
  ];

  const userDistribution = [
    { name: 'Active', value: stats.active_users || 18 },
    { name: 'Inactive', value: Math.max(0, (stats.total_users || 0) - (stats.active_users || 0)) },
  ];

  return (
    <div className="dashboard animate-fade-in">
      <div className="page-header">
        <div className="page-header__left">
          <h1>Dashboard Overview</h1>
          <p>Monitor your CardiGo application health and user activity in real-time.</p>
        </div>
        <div className="page-header__right">
          <button className="btn btn-primary btn-sm">
            <Zap size={14} /> System Live
          </button>
        </div>
      </div>

      {/* ---- Stat Cards ---- */}
      <div className="stats-grid stagger">
        {statCards.map((card, i) => (
          <div className="stat-card glass-card animate-fade-in-up" key={i}>
            <div className="stat-card__header">
              <div className="stat-card__icon" style={{ background: card.bg }}>
                <card.icon size={20} color={card.color} />
              </div>
              <span className={`stat-card__change ${card.up ? 'up' : 'down'}`}>
                {card.up ? <ArrowUpRight size={14} /> : <ArrowDownRight size={14} />}
                {card.change}
              </span>
            </div>
            <div className="stat-card__value">{card.value}</div>
            <div className="stat-card__label">{card.label}</div>
          </div>
        ))}
      </div>

      <div className="dashboard-layout">
        <div className="dashboard-main-content">
          <div className="dashboard__charts">
            <div className="chart-card glass-card animate-fade-in-up" style={{ animationDelay: '100ms' }}>
              <div className="chart-card__header">
                <h3>Daily Severity Rate (Avg)</h3>
                <span className="badge badge-error">24H Real-time</span>
              </div>
              <div className="chart-card__body">
                <ResponsiveContainer width="100%" height={240}>
                  <AreaChart data={stats.severity_trend || []}>
                    <defs>
                      <linearGradient id="gradSev" x1="0" y1="0" x2="0" y2="1">
                        <stop offset="5%" stopColor="#ef4444" stopOpacity={0.3} />
                        <stop offset="95%" stopColor="#ef4444" stopOpacity={0} />
                      </linearGradient>
                    </defs>
                    <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" />
                    <XAxis dataKey="time" stroke="var(--text-tertiary)" fontSize={11} tickLine={false} axisLine={false} />
                    <YAxis stroke="var(--text-tertiary)" fontSize={11} tickLine={false} axisLine={false} unit="%" />
                    <Tooltip content={<CustomTooltip />} />
                    <Area type="monotone" dataKey="rate" stroke="#ef4444" strokeWidth={3} fill="url(#gradSev)" />
                  </AreaChart>
                </ResponsiveContainer>
              </div>
            </div>

            <div className="chart-card glass-card animate-fade-in-up" style={{ animationDelay: '200ms' }}>
              <div className="chart-card__header">
                <h3>AI Chat Activity</h3>
                <span className="badge badge-warning">7D View</span>
              </div>
              <div className="chart-card__body">
                <ResponsiveContainer width="100%" height={240}>
                  <BarChart data={stats.daily_chats || []} barSize={26}>
                    <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" />
                    <XAxis dataKey="date" stroke="var(--text-tertiary)" fontSize={11} tickLine={false} axisLine={false} />
                    <YAxis stroke="var(--text-tertiary)" fontSize={11} tickLine={false} axisLine={false} />
                    <Tooltip content={<CustomTooltip />} />
                    <Bar dataKey="count" fill="var(--accent-400)" radius={[4, 4, 0, 0]} />
                  </BarChart>
                </ResponsiveContainer>
              </div>
            </div>
          </div>

          <div className="vitals-section glass-card animate-fade-in-up" style={{ animationDelay: '300ms' }}>
            <div className="vitals-header" style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '20px' }}>
              <h3 style={{ margin: 0 }}>Clinical Vitals & Severity Rate</h3>
              <div className="severity-indicator" style={{ textAlign: 'right' }}>
                <span style={{ fontSize: '0.8rem', color: 'var(--text-tertiary)' }}>Current Severity Rate:</span>
                <div style={{ fontSize: '1.2rem', fontWeight: 'bold', color: 'var(--primary-400)' }}>
                  {Math.round(stats.severity_trend?.[23]?.rate || 0)}%
                </div>
              </div>
            </div>
            <div className="vitals-grid">
              {vitalCards.map((v, i) => (
                <div className="vital-item" key={i}>
                  <div className="vital-item__icon" style={{ background: `${v.color}15` }}>
                    <v.icon size={18} color={v.color} />
                  </div>
                  <div>
                    <div className="vital-item__value" style={{ color: v.color }}>{v.value}</div>
                    <div className="vital-item__label">{v.label}</div>
                  </div>
                </div>
              ))}
            </div>
            
            {(() => {
              const currentSev = Math.round(stats.severity_trend?.[23]?.rate || 0);
              let riskLevel = 'Low Risk';
              let riskColor = '#22c55e'; // success
              let recommendation = 'All patients are currently stable. Standard monitoring applies.';
              let Icon = ShieldCheck;

              if (currentSev >= 80) {
                riskLevel = 'Critical Risk';
                riskColor = '#ef4444'; // danger
                recommendation = 'Immediate hospital transport required for high severity patients.';
                Icon = AlertCircle;
              } else if (currentSev >= 40) {
                riskLevel = 'Moderate Risk';
                riskColor = '#f59e0b'; // warning
                recommendation = 'Urgent medical consultation recommended for at-risk patients.';
                Icon = AlertTriangle;
              }

              return (
                <div className="hospital-report glass-card mt-4" style={{ padding: '16px', background: 'rgba(255,255,255,0.02)', border: `1px solid ${riskColor}33` }}>
                   <div style={{ display: 'flex', alignItems: 'center', gap: '10px', marginBottom: '10px' }}>
                     <Icon size={18} color={riskColor} />
                     <h4 style={{ margin: 0, fontSize: '0.9rem', color: riskColor }}>Automated Hospitalization Prediction</h4>
                   </div>
                   <p style={{ fontSize: '0.85rem', color: 'var(--text-secondary)', lineHeight: 1.5 }}>
                     Based on current thresholds (Heart Rate, SpO₂, Temperature), the AI predicts a <strong style={{color: riskColor}}>{riskLevel}</strong> of emergency across the active patient pool. 
                     <br/><br/>
                     Recommended Action: {recommendation}
                   </p>
                </div>
              );
            })()}
          </div>
        </div>

        <div className="dashboard-sidebar-widgets">
          {/* Health Alert Watch */}
          <div className="actions-card glass-card animate-fade-in-up" style={{ animationDelay: '400ms' }}>
            <div className="actions-card__header">
              <div className="actions-card__title">
                <AlertCircle size={18} color="var(--primary-400)" />
                <h3 style={{ color: 'var(--primary-400)' }}>Health Alert Watch</h3>
              </div>
              <span className="pulse-dot"></span>
            </div>
            <div className="alerts-feed">
              {stats.health_alerts?.map((alert, i) => (
                <div className={`alert-item alert-item--${alert.priority}`} key={alert.id}>
                  <div className="alert-item__header">
                    <span className="alert-item__user">{alert.user}</span>
                    <span className="alert-item__time">{alert.time}</span>
                  </div>
                  <p className="alert-item__msg">{alert.message}</p>
                </div>
              ))}
            </div>
            <button 
              className="btn btn-ghost btn-sm w-full mt-2" 
              style={{ fontSize: '0.75rem' }}
              onClick={() => navigate('/alerts')}
            >
              Open Alert Center
            </button>
          </div>

          {/* System & Security Logs */}
          <div className="actions-card glass-card animate-fade-in-up" style={{ animationDelay: '500ms' }}>
            <div className="actions-card__header">
              <div className="actions-card__title">
                <ShieldCheck size={18} color="var(--success-400)" />
                <h3>Security & System</h3>
              </div>
            </div>
            <div className="actions-list">
              {stats.recent_actions?.map((action, i) => (
                <div className="action-item" key={i}>
                  <div className={`action-indicator action-indicator--${action.type}`} />
                  <div className="action-info">
                    <p className="action-text"><strong>{action.user}</strong> {action.action}</p>
                    <span className="action-time"><Clock size={10} /> {action.time}</span>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
