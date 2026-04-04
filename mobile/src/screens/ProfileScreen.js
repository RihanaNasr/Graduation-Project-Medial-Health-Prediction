import React, { useState } from 'react';
import {
    View,
    Text,
    StyleSheet,
    TouchableOpacity,
    ScrollView,
    Platform,
    Switch,
    TextInput,
} from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather, Ionicons } from '@expo/vector-icons';
import { LinearGradient } from 'expo-linear-gradient';
import { useAuth } from '../context/AuthContext';
import { useTheme } from '../context/ThemeContext';

const ProfileScreen = ({ navigation }) => {
    const { user, logout } = useAuth();
    const { isDark, toggleTheme, colors } = useTheme();
    const [alertsEnabled, setAlertsEnabled] = useState(true);
    const [syncEnabled, setSyncEnabled] = useState(true);
    const [emergencyEnabled, setEmergencyEnabled] = useState(false);
    const [isEditingName, setIsEditingName] = useState(false);
    const [tempFirstName, setTempFirstName] = useState(user?.first_name || 'Reem');
    const [tempLastName, setTempLastName] = useState(user?.last_name || 'Ihab');

    const handleNameSave = () => {
        setIsEditingName(false);
        // Normally update via API here
    };

    return (
        <View style={[styles.container, { backgroundColor: colors.background }]}>
            <StatusBar style={isDark ? "light" : "dark"} />

            <ScrollView contentContainerStyle={styles.scrollContent} showsVerticalScrollIndicator={false}>
                {/* Header */}
                <View style={styles.headerRow}>
                    <Text style={[styles.pageTitle, { color: colors.text }]}>Settings</Text>
                    <Text style={[styles.pageSub, { color: colors.subtext }]}>Customize your experience</Text>
                </View>

                {/* Profile Card */}
                <LinearGradient
                    colors={['#3A8EF6', '#5BADFF']}
                    style={styles.profileCard}
                    start={{ x: 0, y: 0 }}
                    end={{ x: 1, y: 1 }}
                >
                    <View style={styles.profileCardInner}>
                        <View style={styles.avatarWrap}>
                            <Text style={styles.avatarEmoji}>👩‍💼</Text>
                        </View>
                        <View style={styles.profileInfo}>
                            {isEditingName ? (
                                <View style={{ flexDirection: 'row' }}>
                                    <TextInput 
                                        style={styles.profileNameInput} 
                                        value={tempFirstName} 
                                        onChangeText={setTempFirstName} 
                                        autoFocus
                                    />
                                    <TextInput 
                                        style={styles.profileNameInput} 
                                        value={tempLastName} 
                                        onChangeText={setTempLastName} 
                                    />
                                </View>
                            ) : (
                                <Text style={styles.profileName}>{tempFirstName} {tempLastName}</Text>
                            )}
                            <Text style={styles.profilePlan}>Premium Plan ✦</Text>
                        </View>
                        <TouchableOpacity style={styles.editBtn} onPress={isEditingName ? handleNameSave : () => setIsEditingName(true)}>
                            <Feather name={isEditingName ? "check" : "edit-2"} size={16} color="white" />
                        </TouchableOpacity>
                    </View>
                </LinearGradient>

                {/* Section: Monitoring */}
                <Text style={[styles.sectionLabel, { color: colors.subtext }]}>MONITORING</Text>
                <View style={[styles.cardBlock, { backgroundColor: colors.card, borderColor: colors.border, borderWidth: isDark ? 1 : 0 }]}>
                    {/* Item */}
                    <View style={styles.listItem}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#FFF0F3' }]}>
                            <Text style={styles.listEmoji}>❤️</Text>
                        </View>
                        <View style={styles.listContent}>
                            <Text style={[styles.listTitle, { color: colors.text }]}>Heart Rate Alerts</Text>
                            <Text style={[styles.listSub, { color: colors.subtext }]}>Notify when abnormal</Text>
                        </View>
                        <Switch
                            trackColor={{ false: '#E4ECFD', true: '#22C55E' }}
                            thumbColor="white"
                            onValueChange={() => setAlertsEnabled(!alertsEnabled)}
                            value={alertsEnabled}
                            style={{ transform: [{ scaleX: 0.9 }, { scaleY: 0.9 }] }}
                        />
                    </View>
                    <View style={styles.divider} />

                    {/* Item */}
                    <View style={styles.listItem}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#F4F8FF' }]}>
                            <Text style={styles.listEmoji}>🔬</Text>
                        </View>
                        <View style={styles.listContent}>
                            <Text style={[styles.listTitle, { color: colors.text }]}>Live Sync</Text>
                            <Text style={[styles.listSub, { color: colors.subtext }]}>Connect hardware device</Text>
                        </View>
                        <Switch
                            trackColor={{ false: '#E4ECFD', true: '#22C55E' }}
                            thumbColor="white"
                            onValueChange={() => setSyncEnabled(!syncEnabled)}
                            value={syncEnabled}
                            style={{ transform: [{ scaleX: 0.9 }, { scaleY: 0.9 }] }}
                        />
                    </View>
                    <View style={styles.divider} />

                    {/* Item */}
                    <View style={styles.listItem}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#FFFBEB' }]}>
                            <Text style={styles.listEmoji}>🚨</Text>
                        </View>
                        <View style={styles.listContent}>
                            <Text style={[styles.listTitle, { color: colors.text }]}>Emergency Alert</Text>
                            <Text style={[styles.listSub, { color: colors.subtext }]}>Auto-call contacts</Text>
                        </View>
                        <Switch
                            trackColor={{ false: '#E4ECFD', true: '#22C55E' }}
                            thumbColor="white"
                            onValueChange={() => setEmergencyEnabled(!emergencyEnabled)}
                            value={emergencyEnabled}
                            style={{ transform: [{ scaleX: 0.9 }, { scaleY: 0.9 }] }}
                        />
                    </View>
                </View>

                {/* Section: App Preferences */}
                <Text style={[styles.sectionLabel, { color: colors.subtext }]}>APP PREFERENCES</Text>
                <View style={[styles.cardBlock, { backgroundColor: colors.card, borderColor: colors.border, borderWidth: isDark ? 1 : 0 }]}>
                    {/* Toggle: Dark Mode */}
                    <View style={styles.listItem}>
                        <View style={[styles.listIconWrap, { backgroundColor: isDark ? '#334155' : '#F4F8FF' }]}>
                            <Text style={styles.listEmoji}>{isDark ? '🌙' : '☀️'}</Text>
                        </View>
                        <View style={styles.listContent}>
                            <Text style={[styles.listTitle, { color: colors.text }]}>Dark Mode</Text>
                            <Text style={[styles.listSub, { color: colors.subtext }]}>Comfortable night viewing</Text>
                        </View>
                        <Switch
                            trackColor={{ false: '#E4ECFD', true: '#3A8EF6' }}
                            thumbColor="white"
                            onValueChange={toggleTheme}
                            value={isDark}
                            style={{ transform: [{ scaleX: 0.9 }, { scaleY: 0.9 }] }}
                        />
                    </View>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={() => navigation.navigate('Notifications')}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#FFFBEB' }]}>
                            <Text style={styles.listEmoji}>🔔</Text>
                        </View>
                        <View style={styles.listContent}>
                            <Text style={[styles.listTitle, { color: colors.text }]}>Notifications</Text>
                            <Text style={[styles.listSub, { color: colors.subtext }]}>Manage reminders</Text>
                        </View>
                        <Feather name="chevron-right" size={18} color={colors.subtext} />
                    </TouchableOpacity>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={() => navigation.navigate('Language')}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#E8F1FE' }]}>
                            <Text style={styles.listEmoji}>🌐</Text>
                        </View>
                        <View style={styles.listContent}>
                            <Text style={[styles.listTitle, { color: colors.text }]}>Language</Text>
                            <Text style={[styles.listSub, { color: colors.subtext }]}>English</Text>
                        </View>
                        <Feather name="chevron-right" size={18} color={colors.subtext} />
                    </TouchableOpacity>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={() => navigation.navigate('Privacy')}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#FFF0F3' }]}>
                            <Text style={styles.listEmoji}>🔒</Text>
                        </View>
                        <View style={styles.listContent}>
                            <Text style={[styles.listTitle, { color: colors.text }]}>Privacy & Security</Text>
                            <Text style={[styles.listSub, { color: colors.subtext }]}>Data & permissions</Text>
                        </View>
                        <Feather name="chevron-right" size={18} color={colors.subtext} />
                    </TouchableOpacity>
                </View>

                {/* Sign Out */}
                <View style={[styles.cardBlock, { marginTop: 24, backgroundColor: colors.card, borderColor: colors.border, borderWidth: isDark ? 1 : 0 }]}>
                    <TouchableOpacity style={styles.listItem} onPress={() => logout()}>
                        <View style={[styles.listIconWrap, { backgroundColor: isDark ? '#334155' : '#FFF0F3' }]}>
                            <Text style={styles.listEmoji}>🚪</Text>
                        </View>
                        <View style={styles.listContent}>
                            <Text style={[styles.listTitle, { color: '#FF4D6D' }]}>Sign Out</Text>
                        </View>
                        <Feather name="chevron-right" size={18} color="#FF4D6D" />
                    </TouchableOpacity>
                </View>

            </ScrollView>
        </View>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#F4F8FF',
    },
    scrollContent: {
        paddingTop: Platform.OS === 'ios' ? 70 : 40,
        paddingHorizontal: 24,
        paddingBottom: 100,
    },
    headerRow: {
        marginBottom: 24,
    },
    pageTitle: {
        fontSize: 28,
        fontWeight: '900',
        color: '#0F1E3C',
        letterSpacing: -0.5,
    },
    pageSub: {
        fontSize: 14,
        color: '#A0AEC0',
        marginTop: 4,
        fontWeight: '500',
    },
    profileCard: {
        borderRadius: 24,
        padding: 24,
        marginBottom: 32,
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 12 },
        shadowOpacity: 0.25,
        shadowRadius: 24,
        elevation: 8,
    },
    profileCardInner: {
        flexDirection: 'row',
        alignItems: 'center',
    },
    avatarWrap: {
        width: 56,
        height: 56,
        backgroundColor: 'rgba(255,255,255,0.25)',
        borderRadius: 20,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 16,
    },
    avatarEmoji: {
        fontSize: 28,
    },
    profileInfo: {
        flex: 1,
    },
    profileName: {
        fontSize: 18,
        fontWeight: '900',
        color: '#FFF',
        marginBottom: 4,
    },
    profileNameInput: {
        fontSize: 18,
        fontWeight: '900',
        color: '#FFF',
        borderBottomWidth: 1,
        borderBottomColor: '#FFF',
        marginRight: 10,
        padding: 0,
    },
    profilePlan: {
        fontSize: 12,
        fontWeight: '700',
        color: 'rgba(255,255,255,0.85)',
    },
    editBtn: {
        width: 40,
        height: 40,
        backgroundColor: 'rgba(255,255,255,0.2)',
        borderRadius: 14,
        alignItems: 'center',
        justifyContent: 'center',
    },
    sectionLabel: {
        fontSize: 11,
        fontWeight: '800',
        color: '#A0AEC0',
        letterSpacing: 1.2,
        marginBottom: 12,
        marginLeft: 8,
    },
    cardBlock: {
        backgroundColor: '#FFF',
        borderRadius: 24,
        paddingHorizontal: 20,
        paddingVertical: 8,
        marginBottom: 24,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 6 },
        shadowOpacity: 0.04,
        shadowRadius: 16,
        elevation: 2,
    },
    listItem: {
        flexDirection: 'row',
        alignItems: 'center',
        paddingVertical: 12,
    },
    listIconWrap: {
        width: 44,
        height: 44,
        borderRadius: 14,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 16,
    },
    listEmoji: {
        fontSize: 20,
    },
    listContent: {
        flex: 1,
    },
    listTitle: {
        fontSize: 15,
        fontWeight: '800',
        color: '#0F1E3C',
        marginBottom: 3,
    },
    listSub: {
        fontSize: 11,
        fontWeight: '600',
        color: '#A0AEC0',
    },
    divider: {
        height: 1,
        backgroundColor: '#F4F8FF',
        marginLeft: 60,
    },
});

export default ProfileScreen;
