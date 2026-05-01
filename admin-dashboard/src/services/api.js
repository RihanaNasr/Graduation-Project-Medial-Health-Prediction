/**
 * CardiGo Admin Dashboard — API Service
 * Handles all communication with the Django backend.
 */

const API_BASE = 'http://127.0.0.1:8000/api';

class ApiService {
  constructor() {
    this.token = localStorage.getItem('admin_token') || null;
  }

  setToken(token) {
    this.token = token;
    localStorage.setItem('admin_token', token);
  }

  clearToken() {
    this.token = null;
    localStorage.removeItem('admin_token');
    localStorage.removeItem('admin_refresh');
    localStorage.removeItem('admin_user');
  }

  getHeaders() {
    const headers = { 'Content-Type': 'application/json' };
    if (this.token) {
      headers['Authorization'] = `Bearer ${this.token}`;
    }
    return headers;
  }

  async request(method, endpoint, body = null) {
    const config = {
      method,
      headers: this.getHeaders(),
    };
    if (body) config.body = JSON.stringify(body);

    try {
      const response = await fetch(`${API_BASE}${endpoint}`, config);

      if (response.status === 401) {
        // Try to refresh token
        const refreshed = await this.refreshToken();
        if (refreshed) {
          config.headers = this.getHeaders();
          const retryResponse = await fetch(`${API_BASE}${endpoint}`, config);
          if (!retryResponse.ok) throw new Error('Request failed after token refresh');
          return await retryResponse.json();
        }
        this.clearToken();
        window.location.href = '/';
        return null;
      }

      if (!response.ok) {
        const err = await response.json().catch(() => ({}));
        throw new Error(err.detail || err.error || `HTTP ${response.status}`);
      }
      return await response.json();
    } catch (error) {
      console.error(`API Error [${method} ${endpoint}]:`, error);
      throw error;
    }
  }

  async refreshToken() {
    const refresh = localStorage.getItem('admin_refresh');
    if (!refresh) return false;
    try {
      const res = await fetch(`${API_BASE}/auth/token/refresh/`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ refresh }),
      });
      if (!res.ok) return false;
      const data = await res.json();
      this.setToken(data.access);
      return true;
    } catch {
      return false;
    }
  }

  // === Auth ===
  async login(email, password) {
    const data = await this.request('POST', '/auth/login/', { email, password });
    if (data?.tokens) {
      this.setToken(data.tokens.access);
      localStorage.setItem('admin_refresh', data.tokens.refresh);
      localStorage.setItem('admin_user', JSON.stringify(data.user));
    }
    return data;
  }

  // === Dashboard Stats ===
  async getStats() {
    return this.request('GET', '/medical/admin/stats/');
  }

  // === Users ===
  async getUsers(search = '', filter = 'all') {
    return this.request('GET', `/medical/admin/users/?search=${search}&filter=${filter}`);
  }

  async getUserDetail(userId) {
    return this.request('GET', `/medical/admin/users/${userId}/`);
  }

  async updateUser(userId, data) {
    return this.request('PATCH', `/medical/admin/users/${userId}/`, data);
  }

  async deleteUser(userId) {
    return this.request('DELETE', `/medical/admin/users/${userId}/`);
  }

  // === Medical Records ===
  async getMedicalRecords(search = '') {
    return this.request('GET', `/medical/admin/records/?search=${search}`);
  }

  async updateMedicalRecord(recordId, data) {
    return this.request('PATCH', `/medical/admin/records/${recordId}/`, data);
  }

  // === Chat Messages ===
  async getChatMessages() {
    return this.request('GET', '/medical/admin/chats/');
  }

  // === System Health ===
  async getSystemHealth() {
    return this.request('GET', '/medical/admin/health/');
  }

  // === Help Contacts ===
  async getHelpContacts() {
    return this.request('GET', '/medical/admin/help-contacts/');
  }

  async addHelpContact(data) {
    return this.request('POST', '/medical/admin/help-contacts/', data);
  }

  async deleteHelpContact(contactId) {
    return this.request('DELETE', `/medical/admin/help-contacts/${contactId}/`);
  }

  // === Alerts ===
  async getAlerts(filter = 'all') {
    return this.request('GET', `/medical/admin/alerts/?filter=${filter}`);
  }

  async resolveAlert(alertId) {
    return this.request('PATCH', `/medical/admin/alerts/${alertId}/`, { is_resolved: true });
  }

  async getOTPs() {
    return this.request('GET', '/medical/admin/otps/');
  }

  // === Data Export ===
  async exportData(type, format) {
    const config = {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${this.token}`,
      },
    };
    const response = await fetch(`${API_BASE}/medical/admin/export/?type=${type}&format=${format}`, config);
    if (!response.ok) throw new Error('Export failed');
    return await response.blob();
  }

  downloadFile(blob, filename) {
    const url = window.URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.setAttribute('download', filename);
    document.body.appendChild(link);
    link.click();
    link.parentNode.removeChild(link);
  }
}

const api = new ApiService();
export default api;
