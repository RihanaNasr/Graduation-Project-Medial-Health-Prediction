import React from 'react';
import { StatusBar } from 'expo-status-bar';
import { AuthProvider } from './src/context/AuthContext';
import { ThemeProvider, useTheme } from './src/context/ThemeContext';
import { LanguageProvider } from './src/context/LanguageContext';
import AppNavigator from './src/navigation/AppNavigator';

const AppContent = () => {
    const { isDark } = useTheme();
    return (
        <LanguageProvider>
            <StatusBar style={isDark ? "light" : "dark"} />
            <AppNavigator />
        </LanguageProvider>
    );
};

export default function App() {
    return (
        <AuthProvider>
            <ThemeProvider>
                <AppContent />
            </ThemeProvider>
        </AuthProvider>
    );
}
