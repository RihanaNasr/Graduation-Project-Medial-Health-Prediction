import { useState, useEffect } from 'react';
import { Heart, Droplets, Thermometer, Activity, Footprints, Flame, Search, Edit2, X, Save, Download, ChevronDown } from 'lucide-react';
import api from '../services/api';
import './MedicalRecordsPage.css';

const mockRecords = [
  { id: 1, user_email: 'sara.ahmed@gmail.com', user_name: 'Sara Ahmed', blood_type: 'A+', heart_rate: 78, blood_pressure: '118/76', spo2: 98, temperature: 36.5, steps: 9200, calories: 2100, water: 2.1, chronic_conditions: 'None', allergies: 'Penicillin', current_medications: 'None', updated_at: '2026-04-24T10:00:00Z' },
  { id: 2, user_email: 'omar.khaled@gmail.com', user_name: 'Omar Khaled', blood_type: 'B+', heart_rate: 85, blood_pressure: '125/82', spo2: 97, temperature: 36.8, steps: 6500, calories: 1800, water: 1.5, chronic_conditions: 'Hypertension', allergies: 'None', current_medications: 'Amlodipine 5mg', updated_at: '2026-04-23T18:30:00Z' },
  { id: 3, user_email: 'nour.hassan@gmail.com', user_name: 'Nour Hassan', blood_type: 'O+', heart_rate: 72, blood_pressure: '115/72', spo2: 99, temperature: 36.4, steps: 12000, calories: 2400, water: 2.5, chronic_conditions: 'None', allergies: 'Dust', current_medications: 'Vitamin D', updated_at: '2026-04-24T08:45:00Z' },
  { id: 4, user_email: 'reem.admin@cardigo.com', user_name: 'Reem Ehab', blood_type: 'AB+', heart_rate: 76, blood_pressure: '120/80', spo2: 98, temperature: 36.6, steps: 8500, calories: 2100, water: 1.8, chronic_conditions: 'None', allergies: 'None', current_medications: 'None', updated_at: '2026-04-24T23:00:00Z' },
];

export default function MedicalRecordsPage() {
  const [records, setRecords] = useState(mockRecords);
  const [search, setSearch] = useState('');
  const [expandedId, setExpandedId] = useState(null);
  const [editingRecord, setEditingRecord] = useState(null);
  const [loading, setLoading] = useState(false);
  const [exporting, setExporting] = useState(false);
  const [showExportOptions, setShowExportOptions] = useState(false);

  const fetchRecords = async () => {
    setLoading(true);
    try {
      const data = await api.getMedicalRecords(search);
      if (data) setRecords(data);
    } catch { /* keep current */ }
    finally { setLoading(false); }
  };

  useEffect(() => {
    const timeoutId = setTimeout(() => {
      fetchRecords();
    }, 400);
    const interval = setInterval(fetchRecords, 5000);
    return () => {
      clearTimeout(timeoutId);
      clearInterval(interval);
    };
  }, [search]);

  const handleUpdate = async (e) => {
    e.preventDefault();
    setLoading(true);
    try {
      await api.updateMedicalRecord(editingRecord.id, editingRecord);
      setEditingRecord(null);
      fetchRecords();
    } catch (err) {
      alert('Failed to update record: ' + err.message);
    } finally {
      setLoading(false);
    }
  };

  const handleExport = async (format) => {
    setExporting(true);
    setShowExportOptions(false);
    try {
      const blob = await api.exportData('vitals', format);
      api.downloadFile(blob, `cardigo_vitals_export.${format === 'json' ? 'json' : 'csv'}`);
    } catch (err) {
      alert('Export failed: ' + err.message);
    } finally {
      setExporting(false);
    }
  };

  const getHeartRateColor = (hr) => {
    if (hr < 60) return 'var(--accent-400)';
    if (hr > 100) return 'var(--danger-400)';
    return 'var(--success-400)';
  };

  const getSpo2Color = (spo2) => {
    if (spo2 < 95) return 'var(--danger-400)';
    if (spo2 < 97) return 'var(--warning-400)';
    return 'var(--success-400)';
  };

  return (
    <div className="records-page animate-fade-in">
      <div className="page-header">
        <h1>Medical Records</h1>
        <p>View and manage patient medical data, vitals, and history.</p>
      </div>

      {/* Search */}
      <div className="records-toolbar">
        <div className="users-search">
          <Search size={16} />
          <input
            type="text"
            placeholder="Search by patient name or email..."
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            className="users-search__input"
            id="records-search"
          />
        </div>
        <div className="toolbar-right">
          <span className="record-count">{records.length} records found</span>
          <div className="export-dropdown-container">
            <button 
              className="btn btn-secondary btn-sm" 
              onClick={() => setShowExportOptions(!showExportOptions)}
              disabled={exporting}
            >
              <Download size={14} />
              {exporting ? 'Exporting...' : 'Download Report'}
              <ChevronDown size={14} />
            </button>
            {showExportOptions && (
              <div className="export-dropdown glass-card animate-fade-in">
                <button onClick={() => handleExport('csv')}>Export as CSV</button>
                <button onClick={() => handleExport('json')}>Export as JSON</button>
              </div>
            )}
          </div>
        </div>
      </div>

      {/* Record Cards */}
      <div className="records-grid stagger">
        {records.map((record) => (
          <div
            className={`record-card glass-card animate-fade-in-up ${expandedId === record.id ? 'record-card--expanded' : ''}`}
            key={record.id}
            onClick={() => setExpandedId(expandedId === record.id ? null : record.id)}
          >
            {/* Header */}
            <div className="record-card__header">
              <div className="record-card__user">
                <div className="record-card__avatar">
                  {record.user_name.split(' ').map(n => n[0]).join('')}
                </div>
                <div>
                  <h3>{record.user_name}</h3>
                  <p>{record.user_email}</p>
                </div>
              </div>
              <div className="record-card__badges">
                {record.risk_status && record.risk_status !== 'NORMAL' && (
                  <span className={`risk-badge risk-badge--${record.risk_status.toLowerCase()}`}>
                    {record.risk_status}
                  </span>
                )}
              </div>
              <div className="record-card__actions">
                <div className="record-card__blood-type">
                  <Droplets size={14} />
                  <span>{record.blood_type || '—'}</span>
                </div>
                <button
                  className="btn-icon"
                  onClick={(e) => {
                    e.stopPropagation();
                    setEditingRecord({ ...record });
                  }}
                  id={`edit-record-${record.id}`}
                >
                  <Edit2 size={14} />
                </button>
              </div>
            </div>

            {/* Vitals Grid */}
            <div className="record-vitals">
              <div className="record-vital">
                <Heart size={18} color={getHeartRateColor(record.heart_rate)} />
                <div>
                  <span className="record-vital__value" style={{ color: getHeartRateColor(record.heart_rate) }}>
                    {record.heart_rate}
                  </span>
                  <span className="record-vital__unit">bpm</span>
                </div>
              </div>
              <div className="record-vital">
                <Activity size={18} color="var(--accent-400)" />
                <div>
                  <span className="record-vital__value" style={{ color: 'var(--accent-400)' }}>
                    {record.blood_pressure}
                  </span>
                  <span className="record-vital__unit">mmHg</span>
                </div>
              </div>
              <div className="record-vital">
                <Droplets size={18} color={getSpo2Color(record.spo2)} />
                <div>
                  <span className="record-vital__value" style={{ color: getSpo2Color(record.spo2) }}>
                    {record.spo2}%
                  </span>
                  <span className="record-vital__unit">SpO₂</span>
                </div>
              </div>
              <div className="record-vital">
                <Thermometer size={18} color="var(--success-400)" />
                <div>
                  <span className="record-vital__value" style={{ color: 'var(--success-400)' }}>
                    {record.temperature}°
                  </span>
                  <span className="record-vital__unit">Celsius</span>
                </div>
              </div>
            </div>

            {/* Activity Row */}
            <div className="record-activity">
              <div className="record-activity__item">
                <Footprints size={14} /> {record.steps?.toLocaleString() || 0} steps
              </div>
              <div className="record-activity__item">
                <Flame size={14} /> {record.calories || 0} kcal
              </div>
              <div className="record-activity__item">
                <Droplets size={14} /> {record.water || 0}L water
              </div>
            </div>

            {/* Expanded Details */}
            {expandedId === record.id && (
              <div className="record-expanded">
                <div className="record-expanded__section">
                  <h4>Chronic Conditions</h4>
                  <p>{record.chronic_conditions || 'None reported'}</p>
                </div>
                <div className="record-expanded__section">
                  <h4>Allergies</h4>
                  <p>{record.allergies || 'None reported'}</p>
                </div>
                <div className="record-expanded__section">
                  <h4>Current Medications</h4>
                  <p>{record.current_medications || 'None'}</p>
                </div>
                <div className="record-expanded__updated">
                  Last updated: {new Date(record.updated_at).toLocaleString()}
                </div>
              </div>
            )}
          </div>
        ))}

        {records.length === 0 && (
          <div className="empty-state">
            <Search size={48} />
            <p>No records found for your search.</p>
          </div>
        )}
      </div>

      {/* Edit Modal */}
      {editingRecord && (
        <div className="modal-overlay" onClick={() => setEditingRecord(null)}>
          <div className="modal-content glass-card" onClick={e => e.stopPropagation()}>
            <div className="modal-header">
              <h2>Edit Medical Record</h2>
              <button className="btn-close" onClick={() => setEditingRecord(null)}><X size={20} /></button>
            </div>
            <form onSubmit={handleUpdate}>
              <div className="user-detail__info">
                <div className="user-detail__avatar">
                  {editingRecord.user_name.split(' ').map(n => n[0]).join('')}
                </div>
                <div>
                  <h3>{editingRecord.user_name}</h3>
                  <p>{editingRecord.user_email}</p>
                </div>
              </div>

              <div className="modal-grid">
                <div className="input-field">
                  <label>Blood Type</label>
                  <input
                    type="text"
                    value={editingRecord.blood_type || ''}
                    onChange={e => setEditingRecord({...editingRecord, blood_type: e.target.value})}
                  />
                </div>
                <div className="input-field">
                  <label>Heart Rate (bpm)</label>
                  <input
                    type="number"
                    value={editingRecord.heart_rate || ''}
                    onChange={e => setEditingRecord({...editingRecord, heart_rate: parseInt(e.target.value)})}
                  />
                </div>
                <div className="input-field">
                  <label>Blood Pressure (mmHg)</label>
                  <input
                    type="text"
                    value={editingRecord.blood_pressure || ''}
                    onChange={e => setEditingRecord({...editingRecord, blood_pressure: e.target.value})}
                  />
                </div>
                <div className="input-field">
                  <label>SpO₂ (%)</label>
                  <input
                    type="number"
                    value={editingRecord.spo2 || ''}
                    onChange={e => setEditingRecord({...editingRecord, spo2: parseInt(e.target.value)})}
                  />
                </div>
                <div className="input-field">
                  <label>Temperature (°C)</label>
                  <input
                    type="number"
                    step="0.1"
                    value={editingRecord.temperature || ''}
                    onChange={e => setEditingRecord({...editingRecord, temperature: parseFloat(e.target.value)})}
                  />
                </div>
              </div>

              <div className="modal-section">
                <div className="input-field">
                  <label>Chronic Conditions</label>
                  <textarea
                    rows="2"
                    value={editingRecord.chronic_conditions || ''}
                    onChange={e => setEditingRecord({...editingRecord, chronic_conditions: e.target.value})}
                  />
                </div>
                <div className="input-field">
                  <label>Allergies</label>
                  <textarea
                    rows="2"
                    value={editingRecord.allergies || ''}
                    onChange={e => setEditingRecord({...editingRecord, allergies: e.target.value})}
                  />
                </div>
                <div className="input-field">
                  <label>Current Medications</label>
                  <textarea
                    rows="2"
                    value={editingRecord.current_medications || ''}
                    onChange={e => setEditingRecord({...editingRecord, current_medications: e.target.value})}
                  />
                </div>
              </div>

              <div className="modal-actions">
                <button type="button" className="btn btn-secondary" onClick={() => setEditingRecord(null)}>Cancel</button>
                <button type="submit" className="btn btn-primary" disabled={loading}>
                  <Save size={16} />
                  {loading ? 'Saving...' : 'Save Changes'}
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </div>
  );
}
