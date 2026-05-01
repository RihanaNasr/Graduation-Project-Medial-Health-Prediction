import { useState, useEffect } from 'react';
import {
  Search, UserCheck, UserX, Shield, Trash2,
  ChevronDown, Eye, Mail, Phone, Calendar, X, Download, Lock, Check,
} from 'lucide-react';
import api from '../services/api';
import './UsersPage.css';

/* ---- fallback mock ---- */
const mockUsers = [
  { id: 1, email: 'sara.ahmed@gmail.com', username: 'sara_ahmed', first_name: 'Sara', last_name: 'Ahmed', gender: 'Female', phone_number: '+20123456789', is_active: true, is_staff: false, created_at: '2026-04-10T14:30:00Z', last_login: '2026-04-24T10:00:00Z', chat_count: 23, has_medical_record: true, heart_rate: 78, spo2: 98 },
  { id: 2, email: 'omar.khaled@gmail.com', username: 'omar_k', first_name: 'Omar', last_name: 'Khaled', gender: 'Male', phone_number: '+20109876543', is_active: true, is_staff: false, created_at: '2026-04-12T09:15:00Z', last_login: '2026-04-23T18:30:00Z', chat_count: 15, has_medical_record: true, heart_rate: 85, spo2: 97 },
  { id: 3, email: 'nour.hassan@gmail.com', username: 'nour_h', first_name: 'Nour', last_name: 'Hassan', gender: 'Female', phone_number: '+20112233445', is_active: true, is_staff: false, created_at: '2026-04-15T11:00:00Z', last_login: '2026-04-24T08:45:00Z', chat_count: 31, has_medical_record: true, heart_rate: 72, spo2: 99 },
  { id: 4, email: 'ahmed.ali@gmail.com', username: 'ahmed_ali', first_name: 'Ahmed', last_name: 'Ali', gender: 'Male', phone_number: '+20155667788', is_active: false, is_staff: false, created_at: '2026-04-08T16:20:00Z', last_login: null, chat_count: 0, has_medical_record: false, heart_rate: null, spo2: null },
  { id: 5, email: 'reem.admin@cardigo.com', username: 'reem_admin', first_name: 'Reem', last_name: 'Ehab', gender: 'Female', phone_number: '+20199887766', is_active: true, is_staff: true, created_at: '2026-04-01T08:00:00Z', last_login: '2026-04-24T23:00:00Z', chat_count: 45, has_medical_record: true, heart_rate: 76, spo2: 98 },
];

export default function UsersPage() {
  const [users, setUsers] = useState(mockUsers);
  const [search, setSearch] = useState('');
  const [filter, setFilter] = useState('all');
  const [selectedUser, setSelectedUser] = useState(null);
  const [editingUser, setEditingUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [updating, setUpdating] = useState(false);
  const [exporting, setExporting] = useState(false);
  const [showExportOptions, setShowExportOptions] = useState(false);
  const [editTab, setEditTab] = useState('profile'); // 'profile' | 'access'

  const fetchUsers = async () => {
    setLoading(true);
    try {
      const data = await api.getUsers(search, filter);
      if (data) setUsers(data);
    } catch { /* keep current */ }
    finally { setLoading(false); }
  };

  useEffect(() => {
    const timeoutId = setTimeout(() => {
      fetchUsers();
    }, 400); // debounce
    const interval = setInterval(fetchUsers, 5000);
    return () => {
      clearTimeout(timeoutId);
      clearInterval(interval);
    };
  }, [search, filter]);

  const toggleUserActive = async (userId) => {
    const user = users.find(u => u.id === userId);
    if (!user) return;
    try {
      await api.updateUser(userId, { is_active: !user.is_active });
    } catch { /* continue */ }
    setUsers(prev => prev.map(u => u.id === userId ? { ...u, is_active: !u.is_active } : u));
  };

  const handleUpdate = async (e) => {
    e.preventDefault();
    setUpdating(true);
    try {
      await api.updateUser(editingUser.id, editingUser);
      setEditingUser(null);
      setSelectedUser(null);
      fetchUsers();
    } catch (err) {
      alert('Failed to update user: ' + err.message);
    } finally {
      setUpdating(false);
    }
  };

  const deleteUser = async (userId) => {
    if (!window.confirm('Are you sure you want to delete this user?')) return;
    try {
      await api.deleteUser(userId);
    } catch { /* continue */ }
    setUsers(prev => prev.filter(u => u.id !== userId));
    setSelectedUser(null);
  };

  const handleExport = async (format) => {
    setExporting(true);
    setShowExportOptions(false);
    try {
      const blob = await api.exportData('users', format);
      api.downloadFile(blob, `cardigo_users_export.${format === 'json' ? 'json' : 'csv'}`);
    } catch (err) {
      alert('Export failed: ' + err.message);
    } finally {
      setExporting(false);
    }
  };

  const formatDate = (dateStr) => {
    if (!dateStr) return 'Never';
    return new Date(dateStr).toLocaleDateString('en-US', {
      month: 'short', day: 'numeric', year: 'numeric',
    });
  };

  return (
    <div className="users-page animate-fade-in">
      <div className="page-header">
        <h1>User Management</h1>
        <p>View and manage all registered CardiGo users.</p>
      </div>

      {/* Filters */}
      <div className="users-toolbar">
        <div className="users-search">
          <Search size={16} />
          <input
            type="text"
            placeholder="Search users by name or email..."
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            className="users-search__input"
            id="users-search"
          />
        </div>

        <div className="users-filters">
          {['all', 'active', 'inactive', 'admin'].map((f) => (
            <button
              key={f}
              className={`filter-btn ${filter === f ? 'filter-btn--active' : ''}`}
              onClick={() => setFilter(f)}
              id={`filter-${f}`}
            >
              {f.charAt(0).toUpperCase() + f.slice(1)}
              {f === 'all' && <span className="filter-count">{users.length}</span>}
              {f === 'active' && <span className="filter-count">{users.filter(u => u.is_active).length}</span>}
              {f === 'inactive' && <span className="filter-count">{users.filter(u => !u.is_active).length}</span>}
              {f === 'admin' && <span className="filter-count">{users.filter(u => u.is_staff).length}</span>}
            </button>
          ))}
        </div>

        <div className="toolbar-actions">
          <div className="export-dropdown-container">
            <button 
              className="btn btn-secondary btn-sm" 
              onClick={() => setShowExportOptions(!showExportOptions)}
              disabled={exporting}
            >
              <Download size={14} />
              {exporting ? 'Exporting...' : 'Export Data'}
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

      {/* User Table */}
      <div className="table-container glass-card">
        <table className="data-table">
          <thead>
            <tr>
              <th>User</th>
              <th>Email</th>
              <th>Status</th>
              <th>Role</th>
              <th>Chats</th>
              <th>Heart Rate</th>
              <th>Joined</th>
              <th>Actions</th>
            </tr>
          </thead>
          <tbody>
            {users.map((user) => (
              <tr key={user.id}>
                <td>
                  <div className="user-cell">
                    <div className="user-cell__avatar" style={{
                      background: user.is_staff
                        ? 'linear-gradient(135deg, var(--warning-500), var(--primary-500))'
                        : 'linear-gradient(135deg, var(--accent-500), var(--accent-700))'
                    }}>
                      {user.first_name?.charAt(0)}{user.last_name?.charAt(0)}
                    </div>
                    <div>
                      <div className="user-cell__name">{user.first_name} {user.last_name}</div>
                      <div className="user-cell__username">@{user.username}</div>
                    </div>
                  </div>
                </td>
                <td>{user.email}</td>
                <td>
                  <span className={`badge ${user.is_active ? 'badge-success' : 'badge-danger'}`}>
                    {user.is_active ? 'Active' : 'Inactive'}
                  </span>
                </td>
                <td>
                  <span className={`badge ${user.is_staff ? 'badge-warning' : 'badge-info'}`}>
                    {user.is_staff ? '⭐ Admin' : 'User'}
                  </span>
                </td>
                <td style={{ fontWeight: 600 }}>{user.chat_count}</td>
                <td>
                  {user.heart_rate ? (
                    <span style={{ color: user.heart_rate > 100 ? 'var(--danger-400)' : 'var(--success-400)', fontWeight: 600 }}>
                      {user.heart_rate} bpm
                    </span>
                  ) : '—'}
                </td>
                <td style={{ fontSize: '0.82rem' }}>{formatDate(user.created_at)}</td>
                <td>
                  <div className="action-btns">
                    <button className="btn btn-ghost btn-sm" onClick={() => setSelectedUser(user)} title="View Details" id={`view-user-${user.id}`}>
                      <Eye size={16} />
                    </button>
                    <button
                      className="btn btn-ghost btn-sm"
                      onClick={() => setEditingUser({ ...user })}
                      title="Edit User"
                      id={`edit-user-${user.id}`}
                    >
                      <X size={16} style={{ transform: 'rotate(45deg)' }} />
                    </button>
                    <button className="btn btn-ghost btn-sm" onClick={() => toggleUserActive(user.id)} title={user.is_active ? 'Deactivate' : 'Activate'} id={`toggle-user-${user.id}`}>
                      {user.is_active ? <UserX size={16} color="var(--danger-400)" /> : <UserCheck size={16} color="var(--success-400)" />}
                    </button>
                  </div>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
        {users.length === 0 && (
          <div className="empty-state">
            <Search size={40} />
            <p>No users found matching your criteria.</p>
          </div>
        )}
      </div>

      {/* User Detail Modal */}
      {selectedUser && (
        <div className="modal-overlay" onClick={() => setSelectedUser(null)}>
          <div className="modal-content glass-card" onClick={(e) => e.stopPropagation()}>
            <div className="modal-header">
              <h2>User Details</h2>
              <button className="btn-close" onClick={() => setSelectedUser(null)}><X size={20} /></button>
            </div>

            <div className="user-detail">
              <div className="user-detail__top">
                <div className="user-detail__avatar" style={{
                  background: selectedUser.is_staff
                    ? 'linear-gradient(135deg, var(--warning-500), var(--primary-500))'
                    : 'linear-gradient(135deg, var(--accent-500), var(--accent-700))'
                }}>
                  {selectedUser.first_name?.charAt(0)}{selectedUser.last_name?.charAt(0)}
                </div>
                <div>
                  <h3>{selectedUser.first_name} {selectedUser.last_name}</h3>
                  <p style={{ color: 'var(--text-tertiary)' }}>@{selectedUser.username}</p>
                </div>
                <span className={`badge ${selectedUser.is_active ? 'badge-success' : 'badge-danger'}`} style={{ marginLeft: 'auto' }}>
                  {selectedUser.is_active ? 'Active' : 'Inactive'}
                </span>
              </div>

              <div className="user-detail__grid">
                <div className="detail-item">
                  <Mail size={16} />
                  <span>{selectedUser.email}</span>
                </div>
                <div className="detail-item">
                  <Phone size={16} />
                  <span>{selectedUser.phone_number || 'Not provided'}</span>
                </div>
                <div className="detail-item">
                  <Calendar size={16} />
                  <span>Joined {formatDate(selectedUser.created_at)}</span>
                </div>
                <div className="detail-item">
                  <Shield size={16} />
                  <span>{selectedUser.is_staff ? 'Admin' : 'Regular User'}</span>
                </div>
              </div>

              <div className="user-detail__stats" style={{ marginTop: '24px' }}>
                <div className="mini-stat">
                  <span className="mini-stat__value">{selectedUser.chat_count}</span>
                  <span className="mini-stat__label">Chats</span>
                </div>
                <div className="mini-stat">
                  <span className="mini-stat__value">{selectedUser.heart_rate || '—'}</span>
                  <span className="mini-stat__label">Heart Rate</span>
                </div>
                <div className="mini-stat">
                  <span className="mini-stat__value">{selectedUser.spo2 || '—'}</span>
                  <span className="mini-stat__label">SpO₂ %</span>
                </div>
              </div>
            </div>

            <div className="modal-actions">
              <button className="btn btn-secondary" onClick={() => setSelectedUser(null)}>Close</button>
              <button
                className="btn btn-outline-danger"
                onClick={() => {
                  if (window.confirm(`Are you sure you want to delete ${selectedUser.first_name}?`)) deleteUser(selectedUser.id);
                }}
              >
                <Trash2 size={16} /> Delete User
              </button>
            </div>
          </div>
        </div>
      )}

      {/* Edit User Modal */}
      {editingUser && (
        <div className="modal-overlay" onClick={() => setEditingUser(null)}>
          <div className="modal-content glass-card" onClick={e => e.stopPropagation()}>
            <div className="modal-header">
              <h2>Edit User Profile</h2>
              <div className="modal-tabs">
                <button type="button" className={`tab-btn ${editTab === 'profile' ? 'active' : ''}`} onClick={() => setEditTab('profile')}>User Profile</button>
                <button type="button" className={`tab-btn ${editTab === 'access' ? 'active' : ''}`} onClick={() => setEditTab('access')}>Access Controls</button>
              </div>
              <button className="btn-close" onClick={() => setEditingUser(null)}><X size={20} /></button>
            </div>
            <form onSubmit={handleUpdate}>
              {editTab === 'profile' ? (
                <div className="modal-grid" style={{ gridTemplateColumns: '1fr 1fr' }}>
                  <div className="input-field">
                    <label>First Name</label>
                    <input
                      type="text"
                      value={editingUser.first_name || ''}
                      onChange={e => setEditingUser({...editingUser, first_name: e.target.value})}
                    />
                  </div>
                  <div className="input-field">
                    <label>Last Name</label>
                    <input
                      type="text"
                      value={editingUser.last_name || ''}
                      onChange={e => setEditingUser({...editingUser, last_name: e.target.value})}
                    />
                  </div>
                  <div className="input-field">
                    <label>Phone Number</label>
                    <input
                      type="text"
                      value={editingUser.phone_number || ''}
                      onChange={e => setEditingUser({...editingUser, phone_number: e.target.value})}
                    />
                  </div>
                  <div className="input-field">
                    <label>Account Status</label>
                    <select
                      value={editingUser.is_active ? 'active' : 'inactive'}
                      onChange={e => setEditingUser({...editingUser, is_active: e.target.value === 'active'})}
                      className="select-field"
                    >
                      <option value="active">Active (Full Access)</option>
                      <option value="inactive">Inactive (Suspended)</option>
                    </select>
                  </div>
                </div>
              ) : (
                <div className="permissions-section animate-fade-in">
                  <div className="permission-card">
                    <div className="permission-info">
                      <Shield size={20} />
                      <div>
                        <h4>Administrator Privileges</h4>
                        <p>Grants access to the admin dashboard and system settings.</p>
                      </div>
                    </div>
                    <label className="toggle-switch">
                      <input 
                        type="checkbox" 
                        checked={editingUser.is_staff}
                        onChange={e => setEditingUser({...editingUser, is_staff: e.target.checked})}
                      />
                      <span className="toggle-slider"></span>
                    </label>
                  </div>

                  <div className="permission-group-label">Granular Permissions</div>
                  
                  <div className="permission-card">
                    <div className="permission-info">
                      <Lock size={18} />
                      <div>
                        <h4>Manage Users</h4>
                        <p>Allow editing and deleting other user accounts.</p>
                      </div>
                    </div>
                    <label className="toggle-switch">
                      <input type="checkbox" defaultChecked={editingUser.is_staff} />
                      <span className="toggle-slider"></span>
                    </label>
                  </div>

                  <div className="permission-card">
                    <div className="permission-info">
                      <Eye size={18} />
                      <div>
                        <h4>View Medical Records</h4>
                        <p>Access patient vitals, history, and conditions.</p>
                      </div>
                    </div>
                    <label className="toggle-switch">
                      <input type="checkbox" defaultChecked={true} />
                      <span className="toggle-slider"></span>
                    </label>
                  </div>

                  <div className="permission-card">
                    <div className="permission-info">
                      <Download size={18} />
                      <div>
                        <h4>Export Reports</h4>
                        <p>Download platform data in CSV/JSON formats.</p>
                      </div>
                    </div>
                    <label className="toggle-switch">
                      <input type="checkbox" defaultChecked={editingUser.is_staff} />
                      <span className="toggle-slider"></span>
                    </label>
                  </div>
                </div>
              )}
              
              <div className="modal-actions">
                <button type="button" className="btn btn-secondary" onClick={() => setEditingUser(null)}>Cancel</button>
                <button type="submit" className="btn btn-primary" disabled={updating}>
                  {updating ? 'Saving Changes...' : (
                    <>
                      <Check size={16} /> Save Changes
                    </>
                  )}
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </div>
  );
}
