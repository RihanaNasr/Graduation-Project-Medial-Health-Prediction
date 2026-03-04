import React from 'react';
import {
    View,
    Text,
    StyleSheet,
    TouchableOpacity,
    ScrollView,
    Platform,
    Image,
    ActivityIndicator,
    TextInput,
} from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather, Ionicons } from '@expo/vector-icons';
import { LinearGradient } from 'expo-linear-gradient';
import { useAuth } from '../context/AuthContext';
import Svg, { Path, Circle } from 'react-native-svg';
import { medicalAPI } from '../services/api';

const HomeScreen = ({ navigation }) => {
    const { user } = useAuth();
    const [record, setRecord] = React.useState(null);
    const [loading, setLoading] = React.useState(true);
    const [isEditing, setIsEditing] = React.useState(false);

    // Form states
    const [form, setForm] = React.useState({
        heart_rate: '86',
        calories: '2100',
        steps: '8500',
        water: '1.8',
    });

    React.useEffect(() => {
        loadRecord();
    }, []);

    const loadRecord = async () => {
        try {
            const response = await medicalAPI.getRecord();
            if (response.data) {
                setRecord(response.data);
                setForm({
                    heart_rate: response.data.heart_rate?.toString() || '86',
                    calories: response.data.calories?.toString() || '2100',
                    steps: response.data.steps?.toString() || '8500',
                    water: response.data.water?.toString() || '1.8',
                });
            }
        } catch (error) {
            console.error('Failed to load records:', error);
        } finally {
            setLoading(false);
        }
    };

    const handleSave = async () => {
        setIsEditing(false);
        try {
            const dataToUpdate = {
                ...record,
                heart_rate: parseInt(form.heart_rate) || 86,
                calories: parseInt(form.calories) || 2100,
                steps: parseInt(form.steps) || 8500,
                water: parseFloat(form.water) || 1.8,
            };
            const response = await medicalAPI.updateRecord(dataToUpdate);
            setRecord(response.data);
        } catch (error) {
            console.error('Failed to update records:', error);
            // Optionally load record again or show error toast
            loadRecord();
        }
    };

    if (loading) {
        return (
            <View style={[styles.container, { justifyContent: 'center', alignItems: 'center' }]}>
                <ActivityIndicator size="large" color="#3A8EF6" />
            </View>
        );
    }

    return (
        <View style={styles.container}>
            <StatusBar style="light" />

            <ScrollView contentContainerStyle={styles.scrollContent} showsVerticalScrollIndicator={false}>
                {/* Header Background */}
                <LinearGradient
                    colors={['#3A8EF6', '#5BADFF']}
                    style={styles.heroHeader}
                    start={{ x: 0, y: 0 }}
                    end={{ x: 1, y: 1 }}
                >
                    <View style={styles.headerTopUser}>
                        <View style={{ flex: 1 }}>
                            <Text style={styles.greetingText}>Good morning,</Text>
                            <Text style={styles.userNameText}>{user?.first_name || 'Ahmed'} 👋</Text>
                        </View>
                        <TouchableOpacity
                            style={styles.editBtnTop}
                            onPress={isEditing ? handleSave : () => setIsEditing(true)}
                        >
                            <Text style={styles.editBtnTopText}>{isEditing ? 'Save' : 'Edit'}</Text>
                        </TouchableOpacity>
                        <TouchableOpacity style={styles.notifBtn} onPress={() => navigation.navigate('Notifications')}>
                            <Feather name="bell" size={20} color="white" />
                            <View style={styles.notifBadge} />
                        </TouchableOpacity>
                    </View>
                </LinearGradient>

                {/* Overlapping Main Card */}
                <View style={styles.mainCardWrap}>
                    <View style={styles.liveCard}>
                        <View style={styles.cardHeader}>
                            <Text style={styles.cardTitle}>LIVE HEART RATE</Text>
                            <View style={styles.statusBadge}>
                                <View style={styles.statusDot} />
                                <Text style={styles.statusText}>Normal</Text>
                            </View>
                        </View>

                        <View style={styles.bpmRow}>
                            {isEditing ? (
                                <TextInput
                                    style={styles.bpmInput}
                                    value={form.heart_rate}
                                    onChangeText={(text) => setForm({ ...form, heart_rate: text })}
                                    keyboardType="numeric"
                                />
                            ) : (
                                <Text style={styles.bpmVal}>{form.heart_rate}</Text>
                            )}
                            <Text style={styles.bpmUnit}>bpm</Text>

                            {/* Graphic */}
                            <View style={styles.graphWrap}>
                                <Svg width="80" height="30" viewBox="0 0 80 30" fill="none">
                                    <Path d="M0 15H15L20 5L30 25L35 15H80" stroke="#3A8EF6" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" />
                                </Svg>
                            </View>
                        </View>

                        <View style={styles.cardDivider} />

                        <View style={styles.statsRow}>
                            <View style={styles.statCol}>
                                <Text style={[styles.statVal, { color: '#3A8EF6' }]}>72</Text>
                                <Text style={styles.statLabel}>Min BPM</Text>
                            </View>
                            <View style={styles.statDivider} />
                            <View style={styles.statCol}>
                                <Text style={[styles.statVal, { color: '#FF4D6D' }]}>112</Text>
                                <Text style={styles.statLabel}>Max BPM</Text>
                            </View>
                            <View style={styles.statDivider} />
                            <View style={styles.statCol}>
                                <Text style={[styles.statVal, { color: '#22C55E' }]}>7h 30m</Text>
                                <Text style={styles.statLabel}>Sleep</Text>
                            </View>
                        </View>
                    </View>
                </View>

                {/* 3 Mini Cards */}
                <View style={styles.miniCardsRow}>
                    <View style={styles.miniCard}>
                        <View style={[styles.iconWrap, { backgroundColor: '#FFF0F3' }]}>
                            <Text style={styles.iconEmoji}>🔥</Text>
                        </View>
                        {isEditing ? (
                            <TextInput
                                style={styles.miniInput}
                                value={form.calories}
                                onChangeText={(text) => setForm({ ...form, calories: text })}
                                keyboardType="numeric"
                            />
                        ) : (
                            <Text style={styles.miniCardVal}>{form.calories}</Text>
                        )}
                        <Text style={styles.miniCardLabel}>Calories</Text>
                    </View>
                    <View style={styles.miniCard}>
                        <View style={[styles.iconWrap, { backgroundColor: '#F4F8FF' }]}>
                            <Text style={styles.iconEmoji}>👣</Text>
                        </View>
                        {isEditing ? (
                            <TextInput
                                style={styles.miniInput}
                                value={form.steps}
                                onChangeText={(text) => setForm({ ...form, steps: text })}
                                keyboardType="numeric"
                            />
                        ) : (
                            <Text style={styles.miniCardVal}>{form.steps}</Text>
                        )}
                        <Text style={styles.miniCardLabel}>Steps</Text>
                    </View>
                    <View style={styles.miniCard}>
                        <View style={[styles.iconWrap, { backgroundColor: '#E8F1FE' }]}>
                            <Text style={styles.iconEmoji}>💧</Text>
                        </View>
                        {isEditing ? (
                            <TextInput
                                style={styles.miniInput}
                                value={form.water}
                                onChangeText={(text) => setForm({ ...form, water: text })}
                                keyboardType="numeric"
                            />
                        ) : (
                            <Text style={styles.miniCardVal}>{form.water}</Text>
                        )}
                        <Text style={styles.miniCardLabel}>Water (L)</Text>
                    </View>
                </View>

                {/* Alert Card */}
                <View style={styles.alertCard}>
                    <View style={styles.alertIconWrap}>
                        <Feather name="alert-triangle" size={18} color="white" />
                    </View>
                    <View style={styles.alertContent}>
                        <Text style={styles.alertTitle}>Elevated Heart Rate Detected</Text>
                        <Text style={styles.alertSub}>Yesterday at 11:45 PM — 128 bpm</Text>
                    </View>
                </View>

                {/* Appointments */}
                <View style={styles.sectionHeader}>
                    <Text style={styles.sectionTitle}>Appointments</Text>
                    <TouchableOpacity>
                        <Text style={styles.sectionLink}>View All</Text>
                    </TouchableOpacity>
                </View>

                <View style={styles.appointmentCard}>
                    <View style={styles.docAvatar}>
                        <Text style={{ fontSize: 24 }}>👩‍⚕️</Text>
                    </View>
                    <View style={styles.docInfo}>
                        <Text style={styles.docName}>Dr. Sarah</Text>
                        <Text style={styles.docSpecialty}>Cardiologist</Text>
                    </View>
                    <View style={styles.aptTimeWrap}>
                        <Text style={styles.aptTime}>10:30 AM</Text>
                        <Text style={styles.aptDate}>Tomorrow</Text>
                    </View>
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
    heroHeader: {
        paddingTop: Platform.OS === 'ios' ? 70 : 50,
        paddingBottom: 110,
        paddingHorizontal: 24,
        borderBottomLeftRadius: 36,
        borderBottomRightRadius: 36,
    },
    headerTopUser: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
    },
    greetingText: {
        fontSize: 14,
        color: 'rgba(255,255,255,0.8)',
        marginBottom: 4,
    },
    userNameText: {
        fontSize: 24,
        fontWeight: '900',
        color: 'white',
        letterSpacing: -0.5,
    },
    notifBtn: {
        width: 44,
        height: 44,
        backgroundColor: 'rgba(255,255,255,0.2)',
        borderRadius: 16,
        alignItems: 'center',
        justifyContent: 'center',
        position: 'relative',
    },
    editBtnTop: {
        backgroundColor: 'rgba(255,255,255,0.25)',
        paddingHorizontal: 14,
        paddingVertical: 8,
        borderRadius: 12,
        marginRight: 10,
    },
    editBtnTopText: {
        color: 'white',
        fontWeight: '700',
        fontSize: 14,
    },
    notifBadge: {
        position: 'absolute',
        top: 10,
        right: 12,
        width: 8,
        height: 8,
        backgroundColor: '#FF4D6D',
        borderRadius: 4,
        borderWidth: 1.5,
        borderColor: '#5BADFF',
    },
    scrollContent: {
        paddingBottom: 100,
    },
    mainCardWrap: {
        paddingHorizontal: 24,
        marginTop: -70,
    },
    liveCard: {
        backgroundColor: 'white',
        borderRadius: 24,
        padding: 24,
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 12 },
        shadowOpacity: 0.12,
        shadowRadius: 30,
        elevation: 10,
    },
    cardHeader: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
        marginBottom: 16,
    },
    cardTitle: {
        fontSize: 11,
        fontWeight: '800',
        color: '#A0AEC0',
        letterSpacing: 1,
    },
    statusBadge: {
        flexDirection: 'row',
        alignItems: 'center',
        backgroundColor: '#EDFBF3',
        paddingHorizontal: 10,
        paddingVertical: 4,
        borderRadius: 12,
    },
    statusDot: {
        width: 6,
        height: 6,
        borderRadius: 3,
        backgroundColor: '#22C55E',
        marginRight: 6,
    },
    statusText: {
        fontSize: 11,
        fontWeight: '700',
        color: '#22C55E',
    },
    bpmRow: {
        flexDirection: 'row',
        alignItems: 'flex-end',
        marginBottom: 24,
    },
    bpmVal: {
        fontSize: 56,
        fontWeight: '900',
        color: '#0F1E3C',
        lineHeight: 60,
    },
    bpmUnit: {
        fontSize: 16,
        fontWeight: '700',
        color: '#5A6A8A',
        marginLeft: 4,
        marginBottom: 8,
    },
    bpmInput: {
        fontSize: 56,
        fontWeight: '900',
        color: '#3A8EF6',
        lineHeight: 60,
        borderBottomWidth: 2,
        borderBottomColor: '#3A8EF6',
        minWidth: 80,
    },
    graphWrap: {
        flex: 1,
        alignItems: 'flex-end',
        paddingBottom: 10,
    },
    cardDivider: {
        height: 1,
        backgroundColor: '#E4ECFD',
        marginVertical: 16,
    },
    statsRow: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
    },
    statCol: {
        flex: 1,
        alignItems: 'center',
    },
    statVal: {
        fontSize: 18,
        fontWeight: '800',
        marginBottom: 2,
    },
    statLabel: {
        fontSize: 11,
        fontWeight: '600',
        color: '#A0AEC0',
    },
    statDivider: {
        width: 1,
        height: 30,
        backgroundColor: '#E4ECFD',
    },
    miniCardsRow: {
        flexDirection: 'row',
        paddingHorizontal: 24,
        marginTop: 16,
        justifyContent: 'space-between',
    },
    miniCard: {
        backgroundColor: 'white',
        borderRadius: 20,
        padding: 16,
        alignItems: 'center',
        width: '31%',
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.04,
        shadowRadius: 10,
        elevation: 2,
    },
    iconWrap: {
        width: 40,
        height: 40,
        borderRadius: 12,
        alignItems: 'center',
        justifyContent: 'center',
        marginBottom: 12,
    },
    iconEmoji: {
        fontSize: 18,
    },
    miniCardVal: {
        fontSize: 16,
        fontWeight: '800',
        color: '#0F1E3C',
        marginBottom: 2,
    },
    miniCardLabel: {
        fontSize: 11,
        fontWeight: '600',
        color: '#A0AEC0',
    },
    miniInput: {
        fontSize: 16,
        fontWeight: '800',
        color: '#3A8EF6',
        marginBottom: 2,
        borderBottomWidth: 1.5,
        borderBottomColor: '#3A8EF6',
        minWidth: 40,
        textAlign: 'center',
    },
    alertCard: {
        marginHorizontal: 24,
        marginTop: 20,
        backgroundColor: '#FFF0F3',
        borderWidth: 1,
        borderColor: '#FFE4E8',
        borderRadius: 20,
        padding: 16,
        flexDirection: 'row',
        alignItems: 'center',
    },
    alertIconWrap: {
        width: 36,
        height: 36,
        backgroundColor: '#FF4D6D',
        borderRadius: 10,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 12,
    },
    alertTitle: {
        fontSize: 13,
        fontWeight: '800',
        color: '#FF4D6D',
        marginBottom: 2,
    },
    alertSub: {
        fontSize: 11,
        fontWeight: '600',
        color: '#FF8FA3',
    },
    sectionHeader: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
        paddingHorizontal: 24,
        marginTop: 24,
        marginBottom: 16,
    },
    sectionTitle: {
        fontSize: 18,
        fontWeight: '800',
        color: '#0F1E3C',
    },
    sectionLink: {
        fontSize: 13,
        fontWeight: '700',
        color: '#3A8EF6',
    },
    appointmentCard: {
        marginHorizontal: 24,
        backgroundColor: 'white',
        borderRadius: 20,
        padding: 16,
        flexDirection: 'row',
        alignItems: 'center',
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.04,
        shadowRadius: 10,
        elevation: 2,
        marginBottom: 20,
    },
    docAvatar: {
        width: 50,
        height: 50,
        backgroundColor: '#E8F1FE',
        borderRadius: 16,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 14,
    },
    docInfo: {
        flex: 1,
    },
    docName: {
        fontSize: 16,
        fontWeight: '800',
        color: '#0F1E3C',
        marginBottom: 2,
    },
    docSpecialty: {
        fontSize: 12,
        fontWeight: '600',
        color: '#5A6A8A',
    },
    aptTimeWrap: {
        alignItems: 'flex-end',
    },
    aptTime: {
        fontSize: 14,
        fontWeight: '800',
        color: '#3A8EF6',
        marginBottom: 2,
    },
    aptDate: {
        fontSize: 11,
        fontWeight: '600',
        color: '#A0AEC0',
    },
});

export default HomeScreen;
