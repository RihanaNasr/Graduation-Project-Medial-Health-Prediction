import React from 'react';
import { NavigationContainer } from '@react-navigation/native';
import { createStackNavigator } from '@react-navigation/stack';
import { createBottomTabNavigator } from '@react-navigation/bottom-tabs';
import { useAuth } from '../context/AuthContext';
import { Feather } from '@expo/vector-icons';
import { useLanguage } from '../context/LanguageContext';

// Screens
import LoginScreen from '../screens/LoginScreen';
import RegisterScreen from '../screens/RegisterScreen';
import HomeScreen from '../screens/HomeScreen';
import DashboardScreen from '../screens/DashboardScreen';
import ChatScreen from '../screens/ChatScreen';
import ProfileScreen from '../screens/ProfileScreen';
import MedicalRecordScreen from '../screens/MedicalRecordScreen';
import HelpScreen from '../screens/HelpScreen';
import NotificationsScreen from '../screens/NotificationsScreen';
import LanguageScreen from '../screens/LanguageScreen';
import PrivacyScreen from '../screens/PrivacyScreen';
import HistoryScreen from '../screens/HistoryScreen';
import ChangePasswordScreen from '../screens/ChangePasswordScreen';
import LegalContentScreen from '../screens/LegalContentScreen';

import ForgotPasswordScreen from '../screens/ForgotPasswordScreen';

const Stack = createStackNavigator();
const Tab = createBottomTabNavigator();

const AuthStack = () => (
    <Stack.Navigator
        screenOptions={{
            headerShown: false,
        }}
    >
        <Stack.Screen name="Login" component={LoginScreen} />
        <Stack.Screen name="Register" component={RegisterScreen} />
        <Stack.Screen name="ForgotPassword" component={ForgotPasswordScreen} />
    </Stack.Navigator>
);

const MainTabs = () => {
    const { t } = useLanguage();
    
    return (
        <Tab.Navigator
            screenOptions={({ route }) => ({
                tabBarIcon: ({ focused, color, size }) => {
                    let iconName;
                    if (route.name === 'Home') iconName = 'home';
                    else if (route.name === 'Dashboard') iconName = 'grid';
                    else if (route.name === 'Chat AI') iconName = 'message-circle';
                    else iconName = 'user';

                    return <Feather name={iconName} size={22} color={color} style={{ opacity: focused ? 1 : 0.6 }} />;
                },
                tabBarActiveTintColor: '#3A8EF6',
                tabBarInactiveTintColor: '#A0AEC0',
                headerShown: false,
                tabBarStyle: {
                    backgroundColor: '#ffffff',
                    borderTopLeftRadius: 24,
                    borderTopRightRadius: 24,
                    height: 70,
                    paddingBottom: 16,
                    paddingTop: 12,
                },
                tabBarLabelStyle: {
                    fontSize: 11,
                    fontWeight: '600',
                }
            })}
        >
            <Tab.Screen
                name="Home"
                component={HomeScreen}
                options={{ title: t('home') || 'Home' }}
            />
            <Tab.Screen
                name="Dashboard"
                component={DashboardScreen}
                options={{ title: t('dashboard') || 'Dashboard' }}
            />
            <Tab.Screen
                name="Chat AI"
                component={ChatScreen}
                options={{
                    title: t('chat') || 'Chat AI',
                    tabBarLabel: t('chat') || 'Chat AI',
                        tabBarIcon: ({ color, size }) => (
                            <Feather name="message-circle" size={size} color={color} />
                        ),
                    }} 
                />
                <Tab.Screen 
                    name="HelpTab" 
                    component={HelpScreen} 
                    options={{ 
                        title: t('help'),
                        tabBarLabel: t('help'),
                        tabBarIcon: ({ color, size }) => (
                            <Feather name="help-circle" size={size} color={color} />
                        ),
                    }} 
                />
                <Tab.Screen 
                    name="Profile" 
                    component={ProfileScreen} 
                    options={{ 
                        title: t('profile'),
                        tabBarLabel: t('profile'),
                        tabBarIcon: ({ color, size }) => (
                            <Feather name="user" size={size} color={color} />
                        ),
                    }} 
                />
            </Tab.Navigator>
    );
};

const AppNavigator = () => {
    const { user, loading } = useAuth();

    if (loading) {
        return null; // Or a loading screen
    }

    return (
        <NavigationContainer>
            {user ? (
                <Stack.Navigator screenOptions={{ headerShown: false, headerBackTitle: 'Back', headerTintColor: '#3A8EF6' }}>
                    <Stack.Screen name="Tabs" component={MainTabs} />
                    <Stack.Screen name="Records" component={MedicalRecordScreen} />
                    <Stack.Screen name="Help" component={HelpScreen} />
                    <Stack.Screen name="Notifications" component={NotificationsScreen} options={{ headerShown: true, title: 'Notifications' }} />
                    <Stack.Screen name="Language" component={LanguageScreen} options={{ headerShown: true, title: 'Language' }} />
                    <Stack.Screen name="Privacy" component={PrivacyScreen} options={{ headerShown: true, title: 'Privacy & Security' }} />
                    <Stack.Screen name="History" component={HistoryScreen} options={{ headerShown: true, title: 'History' }} />
                    <Stack.Screen name="ChangePassword" component={ChangePasswordScreen} options={{ headerShown: true, title: 'Change Password' }} />
                    <Stack.Screen name="LegalContent" component={LegalContentScreen} options={{ headerShown: true }} />
                </Stack.Navigator>
            ) : (
                <AuthStack />
            )}
        </NavigationContainer>
    );
};

export default AppNavigator;
