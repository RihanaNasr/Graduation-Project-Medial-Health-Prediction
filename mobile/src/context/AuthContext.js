import React, { createContext, useState, useContext, useEffect } from 'react';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { authAPI } from '../services/api';

const AuthContext = createContext({});

export const AuthProvider = ({ children }) => {
    const [user, setUser] = useState(null);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        loadUser();
    }, []);

    const loadUser = async () => {
        try {
            const userData = await AsyncStorage.getItem('user');
            if (userData) {
                setUser(JSON.parse(userData));
            }
        } catch (error) {
            console.error('Error loading user:', error);
        } finally {
            setLoading(false);
        }
    };

    const login = async (email, password) => {
        try {
            const response = await authAPI.login({ email, password });
            const { user, tokens } = response.data;

            await AsyncStorage.multiSet([
                ['user', JSON.stringify(user)],
                ['access_token', tokens.access],
                ['refresh_token', tokens.refresh],
            ]);

            setUser(user);
            return { success: true };
        } catch (error) {
            return {
                success: false,
                error: error.response?.data?.detail || 'Login failed',
            };
        }
    };

    const register = async (userData) => {
        try {
            const response = await authAPI.register(userData);
            const { user, tokens } = response.data;

            await AsyncStorage.multiSet([
                ['user', JSON.stringify(user)],
                ['access_token', tokens.access],
                ['refresh_token', tokens.refresh],
            ]);

            setUser(user);
            return { success: true };
        } catch (error) {
            return {
                success: false,
                error: error.response?.data || 'Registration failed',
            };
        }
    };

    const logout = async () => {
        try {
            await AsyncStorage.multiRemove(['user', 'access_token', 'refresh_token']);
            setUser(null);
        } catch (error) {
            console.error('Error logging out:', error);
        }
    };

    const updateUser = async (userData) => {
        try {
            const response = await authAPI.updateProfile(userData);
            const updatedUser = response.data;
            await AsyncStorage.setItem('user', JSON.stringify(updatedUser));
            setUser(updatedUser);
            return { success: true };
        } catch (error) {
            return {
                success: false,
                error: error.response?.data || 'Update failed',
            };
        }
    };

    return (
        <AuthContext.Provider
            value={{
                user,
                loading,
                login,
                register,
                logout,
                updateUser,
            }}
        >
            {children}
        </AuthContext.Provider>
    );
};

export const useAuth = () => {
    const context = useContext(AuthContext);
    if (!context) {
        throw new Error('useAuth must be used within an AuthProvider');
    }
    return context;
};
