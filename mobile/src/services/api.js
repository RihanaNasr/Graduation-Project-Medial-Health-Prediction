import axios from 'axios';
import AsyncStorage from '@react-native-async-storage/async-storage';

// Update this to your backend URL
// For Android emulator: http://10.0.2.2:8000
// For iOS simulator: http://localhost:8000
// For physical device: http://<YOUR_COMPUTER_IP>:8000
// Use a more stable direct connection for the backend
const API_URL = 'http://10.20.228.171:8000/api';
const CHATBOT_URL = 'http://10.20.228.171:5000'; // Our new chatbot server!

const api = axios.create({
    baseURL: API_URL,
    timeout: 10000, // Increase to 10 seconds
    headers: {
        'Content-Type': 'application/json',
        'Bypass-Tunnel-Reminder': 'true', // Required for localtunnel to work with Axios
    },
});

// Add token to requests
api.interceptors.request.use(
    async (config) => {
        const token = await AsyncStorage.getItem('access_token');
        if (token) {
            config.headers.Authorization = `Bearer ${token}`;
        }
        return config;
    },
    (error) => {
        return Promise.reject(error);
    }
);

// Handle token refresh
api.interceptors.response.use(
    (response) => response,
    async (error) => {
        const originalRequest = error.config;

        if (error.response?.status === 401 && !originalRequest._retry) {
            originalRequest._retry = true;

            try {
                const refreshToken = await AsyncStorage.getItem('refresh_token');
                if (!refreshToken) {
                    throw new Error('No refresh token available');
                }
                const response = await axios.post(`${API_URL}/auth/token/refresh/`, {
                    refresh: refreshToken,
                });

                const { access } = response.data;
                await AsyncStorage.setItem('access_token', access);

                originalRequest.headers.Authorization = `Bearer ${access}`;
                return api(originalRequest);
            } catch (refreshError) {
                // Refresh token expired, logout user
                await AsyncStorage.multiRemove(['access_token', 'refresh_token', 'user']);
                return Promise.reject(refreshError);
            }
        }

        return Promise.reject(error);
    }
);

export const authAPI = {
    register: (data) => api.post('/auth/register/', data),
    login: (data) => api.post('/auth/login/', data),
    getProfile: () => api.get('/auth/profile/'),
    updateProfile: (data) => api.put('/auth/profile/', data),
    changePassword: (data) => api.post('/auth/password/change/', data),
};

export const medicalAPI = {
    getRecord: () => api.get('/medical/record/'),
    updateRecord: (data) => api.patch('/medical/record/', data),
    // Pointed to our new assistant
    chat: (message, user_email) => axios.post(`${CHATBOT_URL}/chat`, { message, user_email }),
    getChatHistory: () => api.get('/medical/chat/history/'),
    getHelpContacts: () => api.get('/medical/help/contacts/'),
    triggerSOS: () => api.post('/medical/sos/'),
    exportReport: () => api.get('/medical/report/export/'),
    getRecords: () => api.get('/medical/records/'),
};

export default api;
