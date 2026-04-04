import React from 'react';
import { View, Text, StyleSheet, ScrollView, TouchableOpacity, Linking, Image, TextInput } from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather, MaterialCommunityIcons, FontAwesome5 } from '@expo/vector-icons';
import { LinearGradient } from 'expo-linear-gradient';
import { useTheme } from '../context/ThemeContext';

const HelpScreen = () => {
    const { isDark, colors } = useTheme();

    const makeCall = (number) => {
        Linking.openURL(`tel:${number}`);
    };

    return (
        <View style={[styles.container, { backgroundColor: colors.background }]}>
            <StatusBar style="light" />
            
            <LinearGradient colors={['#22C55E', '#10B981']} style={styles.header}>
                <View style={styles.headerContent}>
                    <View style={styles.iconCircle}>
                        <Text style={{ fontSize: 32 }}>🚨</Text>
                    </View>
                    <Text style={styles.headerTitle}>Emergency Numbers</Text>
                    <Text style={styles.headerSub}>Egypt • Available 24/7 • All free</Text>
                </View>
            </LinearGradient>

            <ScrollView contentContainerStyle={styles.scrollArea} showsVerticalScrollIndicator={false}>
                <View style={[styles.searchBar, { backgroundColor: colors.card, borderColor: colors.border }]}>
                    <Feather name="search" size={20} color="#A0AEC0" />
                    <TextInput 
                        placeholder="Search emergency or help..." 
                        placeholderTextColor="#A0AEC0"
                        style={[styles.searchInput, { color: colors.text }]}
                    />
                </View>

                <Text style={styles.sectionLabel}>UNIFIED EMERGENCY</Text>
                <TouchableOpacity 
                    style={styles.unifiedCard} 
                    onPress={() => makeCall('112')}
                >
                    <LinearGradient colors={['#FF4D6D', '#FF758F']} style={styles.unifiedGradient}>
                        <View style={styles.unifiedIconWrap}>
                            <Feather name="phone" size={32} color="#FF4D6D" />
                        </View>
                        <View style={styles.unifiedTextWrap}>
                            <Text style={styles.unifiedNumber}>112</Text>
                            <Text style={styles.unifiedTarget}>Unified Emergency — Egypt</Text>
                            <Text style={styles.unifiedSub}>All emergencies • Any phone • Free ✓</Text>
                        </View>
                    </LinearGradient>
                </TouchableOpacity>

                <View style={styles.sectionRow}>
                    <Text style={styles.sectionLabel}>SPECIALIZED LINES</Text>
                    <Text style={styles.badgeText}>6 NUMBERS</Text>
                </View>

                <View style={styles.grid}>
                    <EmergencyBox title="Ambulance" icon="ambulance" color="#3A8EF6" number="123" onPress={() => makeCall('123')} />
                    <EmergencyBox title="Police" icon="shield-alt" color="#1E293B" number="122" onPress={() => makeCall('122')} />
                    <EmergencyBox title="Fire Brigade" icon="fire" color="#FF9F1C" number="180" onPress={() => makeCall('180')} />
                    <EmergencyBox title="Health Ministry" icon="hospital-user" color="#9333EA" number="137" onPress={() => makeCall('137')} />
                </View>

                <Text style={styles.sectionLabel}>UTILITIES & ROADS</Text>
                <View style={[styles.listBlock, { backgroundColor: colors.card, borderColor: colors.border }]}>
                    <UtilityItem title="Highway Rescue" icon="car" number="136" onPress={() => makeCall('136')} />
                    <View style={styles.divider} />
                    <UtilityItem title="Electricity Emergency" icon="bolt" number="121" onPress={() => makeCall('121')} />
                    <View style={styles.divider} />
                    <UtilityItem title="Water Emergency" icon="tint" number="125" onPress={() => makeCall('125')} />
                    <View style={styles.divider} />
                    <UtilityItem title="Gas Emergency" icon="gas-pump" number="129" onPress={() => makeCall('129')} />
                </View>

                <Text style={styles.sectionLabel}>HEART EMERGENCY TIPS</Text>
                <View style={[styles.tipsCard, { backgroundColor: colors.card, borderColor: colors.border }]}>
                    <View style={styles.tipsHeader}>
                        <Text style={{ fontSize: 24, marginRight: 12 }}>❤️</Text>
                        <Text style={[styles.tipsTitle, { color: colors.text }]}>If your heart rate spikes suddenly</Text>
                    </View>
                    <TipStep num="1" text="Stay calm — sit or lie down in a safe place immediately" />
                    <TipStep num="2" text="Call 123 if BPM exceeds 150 or you feel chest pain" />
                    <TipStep num="3" text="Alert someone nearby — do not drive alone" />
                    <TipStep num="4" text="If heart stops — call 123 and begin CPR immediately" />
                </View>
            </ScrollView>
        </View>
    );
};

const EmergencyBox = ({ title, icon, color, number, onPress }) => {
    const { colors } = useTheme();
    return (
        <TouchableOpacity style={[styles.gridBox, { backgroundColor: colors.card, borderColor: colors.border }]} onPress={onPress}>
            <View style={[styles.boxIconWrap, { backgroundColor: `${color}15` }]}>
                <FontAwesome5 name={icon} size={20} color={color} />
            </View>
            <Text style={[styles.boxNumber, { color: color }]}>{number}</Text>
            <Text style={[styles.boxTitle, { color: colors.text }]}>{title}</Text>
            <Text style={styles.activityLabel}>• 24/7 Active</Text>
            <TouchableOpacity style={styles.callNowBtn} onPress={onPress}>
                <Feather name="phone-call" size={12} color={color} />
                <Text style={[styles.callNowText, { color: color }]}>Call Now</Text>
            </TouchableOpacity>
        </TouchableOpacity>
    );
};

const UtilityItem = ({ title, icon, number, onPress }) => {
    const { colors } = useTheme();
    return (
        <TouchableOpacity style={styles.listItem} onPress={onPress}>
            <View style={styles.listIconWrap}>
                 <FontAwesome5 name={icon} size={18} color="#3A8EF6" />
            </View>
            <View style={styles.listTextWrap}>
                <Text style={[styles.listTitleText, { color: colors.text }]}>{title}</Text>
                <Text style={styles.listSubText}>Road accidents • Highways</Text>
            </View>
            <View style={styles.listRight}>
                <Text style={styles.utilityNumber}>{number}</Text>
                <TouchableOpacity style={styles.miniCall} onPress={onPress}>
                    <Feather name="phone" size={14} color="#3A8EF6" />
                    <Text style={styles.miniCallText}>Call</Text>
                </TouchableOpacity>
            </View>
        </TouchableOpacity>
    );
};

const TipStep = ({ num, text }) => {
    const { colors } = useTheme();
    return (
        <View style={styles.tipRow}>
            <View style={styles.tipNum}>
                <Text style={styles.tipNumText}>{num}</Text>
            </View>
            <Text style={[styles.tipText, { color: colors.subtext }]}>{text}</Text>
        </View>
    );
};

const styles = StyleSheet.create({
    container: { flex: 1 },
    header: { paddingTop: 60, paddingBottom: 40, paddingHorizontal: 24, borderBottomLeftRadius: 40, borderBottomRightRadius: 40 },
    headerContent: { alignItems: 'center' },
    iconCircle: { width: 80, height: 80, backgroundColor: 'rgba(255,255,255,0.2)', borderRadius: 40, alignItems: 'center', justifyContent: 'center', marginBottom: 16 },
    headerTitle: { fontSize: 28, color: 'white', fontWeight: '900' },
    headerSub: { fontSize: 14, color: 'rgba(255,255,255,0.85)', marginTop: 4, fontWeight: '700' },
    scrollArea: { padding: 20 },
    searchBar: { flexDirection: 'row', alignItems: 'center', borderRadius: 20, paddingHorizontal: 20, height: 56, marginBottom: 24, borderWidth: 1 },
    searchInput: { flex: 1, marginLeft: 12, fontSize: 16, fontWeight: '600' },
    sectionLabel: { fontSize: 12, fontWeight: '900', color: '#A0AEC0', letterSpacing: 1, marginBottom: 16 },
    unifiedCard: { borderRadius: 24, overflow: 'hidden', marginBottom: 24 },
    unifiedGradient: { padding: 24, flexDirection: 'row', alignItems: 'center' },
    unifiedIconWrap: { width: 70, height: 70, backgroundColor: 'rgba(255,255,255,0.4)', borderRadius: 24, alignItems: 'center', justifyContent: 'center' },
    unifiedTextWrap: { marginLeft: 20, flex: 1 },
    unifiedNumber: { fontSize: 42, fontWeight: '900', color: 'white', lineHeight: 48 },
    unifiedTarget: { fontSize: 16, fontWeight: '800', color: 'white' },
    unifiedSub: { fontSize: 12, color: 'rgba(255,255,255,0.8)', marginTop: 4 },
    sectionRow: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', marginBottom: 16 },
    badgeText: { fontSize: 11, fontWeight: '900', color: '#3A8EF6', backgroundColor: '#E8F1FE', paddingHorizontal: 8, paddingVertical: 4, borderRadius: 8 },
    grid: { flexDirection: 'row', flexWrap: 'wrap', justifyContent: 'space-between' },
    gridBox: { width: '48%', padding: 16, borderRadius: 20, borderWidth: 1, marginBottom: 16 },
    boxIconWrap: { width: 44, height: 44, borderRadius: 14, alignItems: 'center', justifyContent: 'center', marginBottom: 12 },
    boxNumber: { fontSize: 28, fontWeight: '900', marginBottom: 4 },
    boxTitle: { fontSize: 15, fontWeight: '800' },
    activityLabel: { fontSize: 10, color: '#22C55E', fontWeight: '800', marginVertical: 8 },
    callNowBtn: { flexDirection: 'row', alignItems: 'center', backgroundColor: '#F4F8FF', paddingVertical: 8, paddingHorizontal: 12, borderRadius: 12, justifyContent: 'center' },
    callNowText: { fontSize: 12, fontWeight: '800', marginLeft: 6 },
    listBlock: { borderRadius: 24, borderWidth: 1, overflow: 'hidden', marginBottom: 24 },
    listItem: { padding: 16, flexDirection: 'row', alignItems: 'center' },
    listIconWrap: { width: 44, height: 44, backgroundColor: '#F4F8FF', borderRadius: 14, alignItems: 'center', justifyContent: 'center', marginRight: 12 },
    listTextWrap: { flex: 1 },
    listTitleText: { fontSize: 15, fontWeight: '800' },
    listSubText: { fontSize: 11, color: '#A0AEC0', fontWeight: '600' },
    listRight: { alignItems: 'flex-end' },
    utilityNumber: { fontSize: 20, fontWeight: '900', color: '#0F1E3C', marginBottom: 4 },
    miniCall: { flexDirection: 'row', alignItems: 'center', backgroundColor: '#E8F1FE', paddingHorizontal: 10, paddingVertical: 5, borderRadius: 10 },
    miniCallText: { fontSize: 11, color: '#3A8EF6', fontWeight: '800', marginLeft: 4 },
    divider: { height: 1, backgroundColor: '#F4F8FF', marginHorizontal: 20 },
    tipsCard: { borderRadius: 24, padding: 24, borderWidth: 1, marginBottom: 40 },
    tipsHeader: { flexDirection: 'row', alignItems: 'center', marginBottom: 20 },
    tipsTitle: { fontSize: 18, fontWeight: '900', flex: 1 },
    tipRow: { flexDirection: 'row', marginBottom: 15, alignItems: 'flex-start' },
    tipNum: { width: 24, height: 24, borderRadius: 12, backgroundColor: '#E8F1FE', alignItems: 'center', justifyContent: 'center', marginRight: 12, marginTop: 2 },
    tipNumText: { color: '#3A8EF6', fontSize: 13, fontWeight: '900' },
    tipText: { fontSize: 14, lineHeight: 22, flex: 1, fontWeight: '600' }
});

export default HelpScreen;
