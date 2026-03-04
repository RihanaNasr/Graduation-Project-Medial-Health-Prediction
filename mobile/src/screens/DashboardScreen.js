import React from 'react';
import {
    View,
    Text,
    StyleSheet,
    TouchableOpacity,
    ScrollView,
    Platform,
    ActivityIndicator,
    TextInput,
} from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather } from '@expo/vector-icons';
import { LinearGradient } from 'expo-linear-gradient';
import { medicalAPI } from '../services/api';

const DashboardScreen = ({ navigation }) => {
    const [record, setRecord] = React.useState(null);
    const [loading, setLoading] = React.useState(true);
    const [isEditing, setIsEditing] = React.useState(false);

    // Form states
    const [form, setForm] = React.useState({
        heart_rate: '86',
        blood_pressure: '120/80',
        spo2: '98',
        temperature: '36.6',
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
                    blood_pressure: response.data.blood_pressure || '120/80',
                    spo2: response.data.spo2?.toString() || '98',
                    temperature: response.data.temperature?.toString() || '36.6',
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
                blood_pressure: form.blood_pressure,
                spo2: parseInt(form.spo2) || 98,
                temperature: parseFloat(form.temperature) || 36.6,
            };
            const response = await medicalAPI.updateRecord(dataToUpdate);
            setRecord(response.data);
        } catch (error) {
            console.error('Failed to update records:', error);
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
            <StatusBar style="dark" />

            <ScrollView contentContainerStyle={styles.scrollContent} showsVerticalScrollIndicator={false}>
                {/* Header */}
                <View style={styles.headerRow}>
                    <View style={{ flex: 1 }}>
                        <Text style={styles.pageTitle}>Dashboard</Text>
                        <Text style={styles.pageSub}>Today's health overview</Text>
                    </View>
                    <TouchableOpacity
                        style={[styles.dateBadge, isEditing && { backgroundColor: '#3A8EF6' }]}
                        onPress={isEditing ? handleSave : () => setIsEditing(true)}
                    >
                        <Text style={[styles.dateBadgeText, isEditing && { color: 'white' }]}>
                            {isEditing ? 'Save' : 'Edit'}
                        </Text>
                    </TouchableOpacity>
                </View>

                {/* Risk Card */}
                <LinearGradient
                    colors={['#FF4D6D', '#FF758F']}
                    style={styles.riskCard}
                    start={{ x: 0, y: 0 }}
                    end={{ x: 1, y: 1 }}
                >
                    <View style={styles.riskCardInner}>
                        <View style={styles.riskIconWrap}>
                            <Feather name="activity" size={24} color="#FFF" />
                        </View>
                        <View style={styles.riskContent}>
                            <Text style={styles.riskLabel}>RISK LEVEL</Text>
                            <Text style={styles.riskTitle}>Low Risk ✓</Text>
                            <Text style={styles.riskSub}>All vitals within normal range</Text>
                        </View>
                        <View style={styles.riskValueWrap}>
                            {isEditing ? (
                                <TextInput
                                    style={styles.riskInput}
                                    value={form.heart_rate}
                                    onChangeText={(text) => setForm({ ...form, heart_rate: text })}
                                    keyboardType="numeric"
                                />
                            ) : (
                                <Text style={styles.riskValue}>{form.heart_rate}</Text>
                            )}
                            <Text style={styles.riskUnit}>bpm now</Text>
                        </View>
                    </View>
                </LinearGradient>

                {/* 2x2 Grid */}
                <View style={styles.gridRow}>
                    {/* Item 1 */}
                    <View style={styles.gridCard}>
                        <View style={styles.gridHeader}>
                            <View style={[styles.gridIconWrap, { backgroundColor: '#FFF0F3' }]}>
                                <Text style={styles.gridEmoji}>❤️</Text>
                            </View>
                            <View style={[styles.badge, styles.badgeGreen]}>
                                <Text style={styles.badgeTextGreen}>+2%</Text>
                            </View>
                        </View>
                        <View style={styles.gridData}>
                            {isEditing ? (
                                <TextInput
                                    style={styles.gridInput}
                                    value={form.heart_rate}
                                    onChangeText={(text) => setForm({ ...form, heart_rate: text })}
                                    keyboardType="numeric"
                                />
                            ) : (
                                <Text style={styles.gridVal}>{form.heart_rate}<Text style={styles.gridUnit}> bpm</Text></Text>
                            )}
                            <Text style={styles.gridLabel}>Heart Rate</Text>
                        </View>
                    </View>

                    {/* Item 2 */}
                    <View style={styles.gridCard}>
                        <View style={styles.gridHeader}>
                            <View style={[styles.gridIconWrap, { backgroundColor: '#F4F8FF' }]}>
                                <Text style={styles.gridEmoji}>💉</Text>
                            </View>
                            <View style={[styles.badge, styles.badgeRed]}>
                                <Text style={styles.badgeTextRed}>-1%</Text>
                            </View>
                        </View>
                        <View style={styles.gridData}>
                            {isEditing ? (
                                <TextInput
                                    style={styles.gridInput}
                                    value={form.blood_pressure}
                                    onChangeText={(text) => setForm({ ...form, blood_pressure: text })}
                                />
                            ) : (
                                <Text style={styles.gridVal}>{form.blood_pressure}</Text>
                            )}
                            <Text style={styles.gridLabel}>Blood Pressure</Text>
                        </View>
                    </View>
                </View>

                <View style={styles.gridRow}>
                    {/* Item 3 */}
                    <View style={styles.gridCard}>
                        <View style={styles.gridHeader}>
                            <View style={[styles.gridIconWrap, { backgroundColor: '#EDFBF3' }]}>
                                <Text style={styles.gridEmoji}>🫁</Text>
                            </View>
                            <View style={[styles.badge, styles.badgeGreen]}>
                                <Text style={styles.badgeTextGreen}>+5%</Text>
                            </View>
                        </View>
                        <View style={styles.gridData}>
                            {isEditing ? (
                                <TextInput
                                    style={styles.gridInput}
                                    value={form.spo2}
                                    onChangeText={(text) => setForm({ ...form, spo2: text })}
                                    keyboardType="numeric"
                                />
                            ) : (
                                <Text style={styles.gridVal}>{form.spo2}<Text style={styles.gridUnit}> %</Text></Text>
                            )}
                            <Text style={styles.gridLabel}>SpO2</Text>
                        </View>
                    </View>

                    {/* Item 4 */}
                    <View style={styles.gridCard}>
                        <View style={styles.gridHeader}>
                            <View style={[styles.gridIconWrap, { backgroundColor: '#FFFBEB' }]}>
                                <Text style={styles.gridEmoji}>🌡️</Text>
                            </View>
                            <View style={[styles.badge, styles.badgeGreen]}>
                                <Text style={styles.badgeTextGreen}>Norm</Text>
                            </View>
                        </View>
                        <View style={styles.gridData}>
                            {isEditing ? (
                                <TextInput
                                    style={styles.gridInput}
                                    value={form.temperature}
                                    onChangeText={(text) => setForm({ ...form, temperature: text })}
                                    keyboardType="numeric"
                                />
                            ) : (
                                <Text style={styles.gridVal}>{form.temperature}<Text style={styles.gridUnit}> °C</Text></Text>
                            )}
                            <Text style={styles.gridLabel}>Temp</Text>
                        </View>
                    </View>
                </View>

                {/* Chart Section */}
                <View style={styles.chartSection}>
                    <Text style={styles.chartTitle}>Weekly Heart Rate Trend</Text>
                    <View style={styles.chartContainer}>
                        <View style={styles.barWrap}><View style={[styles.bar, { height: `${Math.min(100, Math.max(20, (parseInt(form.heart_rate) || 86) * 0.4))}%`, backgroundColor: '#E8F1FE' }]} /><Text style={styles.barLabel}>Mon</Text></View>
                        <View style={styles.barWrap}><View style={[styles.bar, { height: `${Math.min(100, Math.max(20, (parseInt(form.heart_rate) || 86) * 0.6))}%`, backgroundColor: '#E8F1FE' }]} /><Text style={styles.barLabel}>Tue</Text></View>
                        <View style={styles.barWrap}><View style={[styles.bar, { height: `${Math.min(100, Math.max(20, (parseInt(form.heart_rate) || 86) * 0.5))}%`, backgroundColor: '#E8F1FE' }]} /><Text style={styles.barLabel}>Wed</Text></View>
                        <View style={styles.barWrap}><View style={[styles.bar, { height: `${Math.min(100, Math.max(20, (parseInt(form.heart_rate) || 86) * 0.9))}%`, backgroundColor: '#FF8FA3' }]} /><Text style={styles.barLabelRed}>Thu</Text></View>
                        <View style={styles.barWrap}><View style={[styles.bar, { height: `${Math.min(100, Math.max(20, (parseInt(form.heart_rate) || 86) * 0.55))}%`, backgroundColor: '#E8F1FE' }]} /><Text style={styles.barLabel}>Fri</Text></View>
                        <View style={styles.barWrap}><View style={[styles.bar, { height: `${Math.min(100, Math.max(20, (parseInt(form.heart_rate) || 86) * 0.8))}%`, backgroundColor: '#3A8EF6' }]} /><Text style={styles.barLabelBlue}>Sat</Text></View>
                        <View style={styles.barWrap}><View style={[styles.bar, { height: `${Math.min(100, Math.max(20, (parseInt(form.heart_rate) || 86) * 0.45))}%`, backgroundColor: '#E8F1FE' }]} /><Text style={styles.barLabel}>Sun</Text></View>
                    </View>
                </View>

                {/* Bottom Actions */}
                <View style={styles.actionsRow}>
                    <TouchableOpacity style={styles.actionCard} onPress={() => navigation.navigate('Records')}>
                        <View style={[styles.actionIconWrap, { backgroundColor: '#FFF0F3' }]}>
                            <Text style={styles.actionEmoji}>📋</Text>
                        </View>
                        <View>
                            <Text style={styles.actionTitle}>Fill Records</Text>
                            <Text style={styles.actionSub}>Update info</Text>
                        </View>
                    </TouchableOpacity>

                    <TouchableOpacity style={styles.actionCard} onPress={() => navigation.navigate('History')}>
                        <View style={[styles.actionIconWrap, { backgroundColor: '#EDFBF3' }]}>
                            <Text style={styles.actionEmoji}>📊</Text>
                        </View>
                        <View>
                            <Text style={styles.actionTitle}>History</Text>
                            <Text style={styles.actionSub}>All data</Text>
                        </View>
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
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
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
    dateBadge: {
        backgroundColor: '#E8F1FE',
        paddingHorizontal: 16,
        paddingVertical: 8,
        borderRadius: 20,
    },
    dateBadgeText: {
        fontSize: 14,
        fontWeight: '700',
        color: '#3A8EF6',
    },
    riskCard: {
        borderRadius: 24,
        padding: 24,
        marginBottom: 16,
        shadowColor: '#FF4D6D',
        shadowOffset: { width: 0, height: 12 },
        shadowOpacity: 0.3,
        shadowRadius: 24,
        elevation: 8,
    },
    riskCardInner: {
        flexDirection: 'row',
        alignItems: 'center',
    },
    riskIconWrap: {
        width: 50,
        height: 50,
        backgroundColor: 'rgba(255,255,255,0.25)',
        borderRadius: 16,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 16,
    },
    riskContent: {
        flex: 1,
    },
    riskLabel: {
        fontSize: 10,
        fontWeight: '800',
        color: 'rgba(255,255,255,0.8)',
        letterSpacing: 1,
        marginBottom: 2,
    },
    riskTitle: {
        fontSize: 18,
        fontWeight: '900',
        color: '#FFF',
        marginBottom: 2,
    },
    riskSub: {
        fontSize: 11,
        fontWeight: '500',
        color: 'rgba(255,255,255,0.9)',
    },
    riskValueWrap: {
        alignItems: 'center',
    },
    riskValue: {
        fontSize: 36,
        fontWeight: '900',
        color: '#FFF',
        lineHeight: 40,
    },
    riskInput: {
        fontSize: 36,
        fontWeight: '900',
        color: '#FFF',
        lineHeight: 40,
        borderBottomWidth: 2,
        borderBottomColor: '#FFF',
        minWidth: 60,
        textAlign: 'center',
    },
    riskUnit: {
        fontSize: 11,
        fontWeight: '600',
        color: 'rgba(255,255,255,0.8)',
    },
    gridRow: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        marginBottom: 16,
    },
    gridCard: {
        backgroundColor: '#FFF',
        borderRadius: 24,
        width: '48%',
        padding: 16,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.04,
        shadowRadius: 10,
        elevation: 2,
    },
    gridHeader: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
        marginBottom: 20,
    },
    gridIconWrap: {
        width: 44,
        height: 44,
        borderRadius: 14,
        alignItems: 'center',
        justifyContent: 'center',
    },
    gridEmoji: {
        fontSize: 20,
    },
    badge: {
        paddingHorizontal: 8,
        paddingVertical: 4,
        borderRadius: 10,
    },
    badgeGreen: {
        backgroundColor: '#EDFBF3',
    },
    badgeRed: {
        backgroundColor: '#FFF0F3',
    },
    badgeTextGreen: {
        fontSize: 12,
        fontWeight: '800',
        color: '#22C55E',
    },
    badgeTextRed: {
        fontSize: 12,
        fontWeight: '800',
        color: '#FF4D6D',
    },
    gridData: {},
    gridVal: {
        fontSize: 22,
        fontWeight: '900',
        color: '#0F1E3C',
    },
    gridUnit: {
        fontSize: 14,
        color: '#5A6A8A',
        fontWeight: '600',
    },
    gridInput: {
        fontSize: 18,
        fontWeight: '900',
        color: '#3A8EF6',
        borderBottomWidth: 1.5,
        borderBottomColor: '#3A8EF6',
        minWidth: 50,
        paddingVertical: 0,
        marginVertical: 2,
    },
    gridLabel: {
        fontSize: 12,
        fontWeight: '600',
        color: '#A0AEC0',
        marginTop: 4,
    },
    chartSection: {
        backgroundColor: '#FFF',
        borderRadius: 24,
        padding: 24,
        marginTop: 8,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.04,
        shadowRadius: 10,
        elevation: 2,
    },
    chartTitle: {
        fontSize: 16,
        fontWeight: '800',
        color: '#0F1E3C',
        marginBottom: 20,
    },
    chartContainer: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'flex-end',
        height: 120,
    },
    barWrap: {
        alignItems: 'center',
        width: '12%',
        height: '100%',
        justifyContent: 'flex-end',
    },
    bar: {
        width: '100%',
        borderRadius: 6,
        marginBottom: 8,
    },
    barLabel: {
        fontSize: 10,
        fontWeight: '600',
        color: '#A0AEC0',
    },
    barLabelRed: {
        fontSize: 10,
        fontWeight: '700',
        color: '#FF4D6D',
    },
    barLabelBlue: {
        fontSize: 10,
        fontWeight: '800',
        color: '#3A8EF6',
    },
    actionsRow: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        marginTop: 20,
    },
    actionCard: {
        backgroundColor: '#FFF',
        borderRadius: 20,
        width: '48%',
        padding: 16,
        flexDirection: 'row',
        alignItems: 'center',
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.04,
        shadowRadius: 10,
        elevation: 2,
    },
    actionIconWrap: {
        width: 40,
        height: 40,
        borderRadius: 12,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 10,
    },
    actionEmoji: {
        fontSize: 18,
    },
    actionTitle: {
        fontSize: 13,
        fontWeight: '800',
        color: '#0F1E3C',
        marginBottom: 2,
    },
    actionSub: {
        fontSize: 10,
        fontWeight: '600',
        color: '#A0AEC0',
    },
});

export default DashboardScreen;
