import React, { createContext, useContext, useState, useEffect } from 'react';
import AsyncStorage from '@react-native-async-storage/async-storage';

const ThemeContext = createContext();

export const ThemeProvider = ({ children }) => {
    const [isDarkMode, setIsDarkMode] = useState(false);

    useEffect(() => {
        loadTheme();
    }, []);

    const loadTheme = async () => {
        try {
            const savedTheme = await AsyncStorage.getItem('theme');
            if (savedTheme !== null) {
                setIsDarkMode(savedTheme === 'dark');
            }
        } catch (error) {
            console.log('Error loading theme:', error);
        }
    };

    const toggleTheme = async () => {
        try {
            const newTheme = !isDarkMode;
            setIsDarkMode(newTheme);
            await AsyncStorage.setItem('theme', newTheme ? 'dark' : 'light');
        } catch (error) {
            console.log('Error saving theme:', error);
        }
    };

    // Global Theme Colors
    const theme = {
        isDark: isDarkMode,
        colors: {
            background: isDarkMode ? '#0F172A' : '#F4F8FF',
            card: isDarkMode ? '#1E293B' : '#FFFFFF',
            text: isDarkMode ? '#F8FAFC' : '#0F1E3C',
            subtext: isDarkMode ? '#94A3B8' : '#A0AEC0',
            primary: '#3A8EF6',
            secondary: '#5BADFF',
            accent: '#FF4D6D',
            border: isDarkMode ? '#334155' : '#E4ECFD',
        }
    };

    return (
        <ThemeContext.Provider value={{ ...theme, toggleTheme }}>
            {children}
        </ThemeContext.Provider>
    );
};

export const useTheme = () => useContext(ThemeContext);
