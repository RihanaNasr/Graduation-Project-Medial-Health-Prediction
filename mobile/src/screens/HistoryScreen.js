import React, { useState, useEffect } from 'react';
import { View, Text, StyleSheet, FlatList, ActivityIndicator, Image } from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather, Ionicons } from '@expo/vector-icons';
import { LinearGradient } from 'expo-linear-gradient';
import { medicalAPI } from '../services/api';
import { useTheme } from '../context/ThemeContext';

const HistoryScreen = () => {
    const [records, setRecords] = useState([]);
    const [loading, setLoading] = useState(true);
    const { isDark, colors } = useTheme();

    useEffect(() => {
        loadHistory();
    }, []);

    const loadHistory = async () => {
        try {
            const response = await medicalAPI.getRecords();
            setRecords(response.data);
        } catch (error) {
            console.error('Failed to load history:', error);
        } finally {
            setLoading(false);
        }
    };

    const renderItem = ({ item }) => (
        <View style={[styles.recordCard, { backgroundColor: colors.card, borderColor: colors.border, borderWidth: isDark ? 1 : 0 }]}>
            <View style={styles.cardHeader}>
                <View style={styles.dateLabelWrap}>
                    <Feather name="calendar" size={14} color={colors.subtext} />
                    <Text style={[styles.dateText, { color: colors.subtext }]}>{item.updated_at ? new Date(item.updated_at).toLocaleDateString() : 'N/A'}</Text>
                </View>
                <View style={[styles.riskBadge, { backgroundColor: isDark ? '#1E293B' : '#EDFBF3' }]}>
                    <Text style={[styles.riskBadgeText, { color: '#22C55E' }]}>Normal</Text>
                </View>
            </View>

            <View style={styles.statsGrid}>
                <View style={styles.statCol}>
                    <Text style={[styles.statVal, { color: colors.text }]}>{item.heart_rate} <Text style={[styles.statUnit, { color: colors.subtext }]}>bpm</Text></Text>
                    <Text style={[styles.statLabel, { color: colors.subtext }]}>Heart Rate</Text>
                </View>
                <View style={[styles.statDivider, { backgroundColor: colors.border }]} />
                <View style={styles.statCol}>
                    <Text style={[styles.statVal, { color: colors.text }]}>{item.blood_pressure}</Text>
                    <Text style={[styles.statLabel, { color: colors.subtext }]}>Pressure</Text>
                </View>
                <View style={[styles.statDivider, { backgroundColor: colors.border }]} />
                <View style={styles.statCol}>
                    <Text style={[styles.statVal, { color: colors.text }]}>{item.spo2}<Text style={[styles.statUnit, { color: colors.subtext }]}>%</Text></Text>
                    <Text style={[styles.statLabel, { color: colors.subtext }]}>SpO2</Text>
                </View>
            </View>
        </View>
    );

    if (loading) {
        return (
            <View style={[styles.container, { justifyContent: 'center', backgroundColor: colors.background }]}>
                <ActivityIndicator size="large" color="#3A8EF6" />
            </View>
        );
    }

    return (
        <View style={[styles.container, { backgroundColor: colors.background }]}>
            <StatusBar style={isDark ? "light" : "dark"} />
            
            <View style={styles.header}>
                <Text style={[styles.headerTitle, { color: colors.text }]}>Medical History</Text>
                <Text style={[styles.headerSub, { color: colors.subtext }]}>Track your cardiovascular trends</Text>
            </View>

            <FlatList
                data={records}
                renderItem={renderItem}
                keyExtractor={(item) => item.id.toString()}
                contentContainerStyle={styles.listContent}
                ListEmptyComponent={() => (
                    <View style={styles.emptyState}>
                        <Text style={styles.emptyEmoji}>📊</Text>
                        <Text style={[styles.emptyTitle, { color: colors.text }]}>No Records Yet</Text>
                        <Text style={[styles.emptySub, { color: colors.subtext }]}>Start updating your vitals on the Dashboard to see your history here.</Text>
                    </View>
                )}
            />
        </View>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#F4F8FF',
    },
    header: {
        paddingHorizontal: 24,
        paddingTop: 60,
        paddingBottom: 20,
    },
    headerTitle: {
        fontSize: 28,
        fontWeight: '900',
        color: '#0F1E3C',
        letterSpacing: -0.5,
    },
    headerSub: {
        fontSize: 14,
        color: '#A0AEC0',
        marginTop: 4,
        fontWeight: '600',
    },
    listContent: {
        paddingHorizontal: 24,
        paddingBottom: 40,
    },
    recordCard: {
        backgroundColor: '#FFF',
        borderRadius: 24,
        padding: 20,
        marginBottom: 16,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.04,
        shadowRadius: 12,
        elevation: 2,
    },
    cardHeader: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
        marginBottom: 16,
    },
    dateLabelWrap: {
        flexDirection: 'row',
        alignItems: 'center',
    },
    dateText: {
        fontSize: 13,
        fontWeight: '700',
        color: '#A0AEC0',
        marginLeft: 6,
    },
    riskBadge: {
        paddingHorizontal: 12,
        paddingVertical: 5,
        borderRadius: 12,
    },
    riskBadgeText: {
        fontSize: 11,
        fontWeight: '800',
    },
    statsGrid: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
    },
    statCol: {
        alignItems: 'center',
        flex: 1,
    },
    statVal: {
        fontSize: 18,
        fontWeight: '800',
        color: '#0F1E3C',
        marginBottom: 2,
    },
    statUnit: {
        fontSize: 11,
        color: '#5A6A8A',
        fontWeight: '600',
    },
    statLabel: {
        fontSize: 11,
        fontWeight: '600',
        color: '#A0AEC0',
    },
    statDivider: {
        width: 1,
        height: 24,
        backgroundColor: '#E4ECFD',
    },
    emptyState: {
        marginTop: 100,
        alignItems: 'center',
        paddingHorizontal: 40,
    },
    emptyEmoji: {
        fontSize: 60,
        marginBottom: 16,
    },
    emptyTitle: {
        fontSize: 20,
        fontWeight: '800',
        color: '#0F1E3C',
        marginBottom: 8,
    },
    emptySub: {
        fontSize: 14,
        color: '#5A6A8A',
        textAlign: 'center',
        lineHeight: 20,
    },
});

export default HistoryScreen;
