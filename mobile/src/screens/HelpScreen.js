import React, { useState, useEffect } from 'react';
import {
    View,
    Text,
    TextInput,
    StyleSheet,
    TouchableOpacity,
    ScrollView,
    Linking,
    ActivityIndicator,
    Platform,
} from 'react-native';
import { Ionicons, Feather } from '@expo/vector-icons';
import { LinearGradient } from 'expo-linear-gradient';
import { medicalAPI } from '../services/api';
import { StatusBar } from 'expo-status-bar';

const HelpScreen = () => {
    const [contacts, setContacts] = useState([]);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        loadHelpContacts();
    }, []);

    const loadHelpContacts = async () => {
        try {
            const response = await medicalAPI.getHelpContacts();
            setContacts(response.data);
        } catch (error) {
            console.error('Error loading help contacts:', error);
        } finally {
            setLoading(false);
        }
    };

    const handleCall = (phoneNumber) => {
        Linking.openURL(`tel:${phoneNumber}`);
    };

    return (
        <View style={styles.container}>
            <StatusBar style="light" />
            <LinearGradient
                colors={['#22C55E', '#4ADE80']}
                style={styles.heroHeader}
                start={{ x: 0, y: 0 }}
                end={{ x: 1, y: 1 }}
            >
                <View style={styles.heroBgCircle} />
                <View style={styles.headerTop}>
                    <TouchableOpacity style={styles.backBtn}>
                        <Feather name="chevron-left" size={24} color="white" />
                    </TouchableOpacity>
                    <Text style={styles.headerTitle}>Help Center</Text>
                    <TouchableOpacity style={styles.helpBtn}>
                        <Feather name="info" size={18} color="white" />
                    </TouchableOpacity>
                </View>

                <View style={styles.heroContent}>
                    <View style={styles.heroIconWrap}>
                        <Text style={styles.heroIcon}>🚨</Text>
                    </View>
                    <View>
                        <Text style={styles.heroBigTitle}>Emergency{'\n'}Numbers</Text>
                        <Text style={styles.heroSubTitle}>Available 24/7 · All free</Text>
                    </View>
                </View>
            </LinearGradient>

            <View style={styles.searchWrap}>
                <Feather name="search" size={18} color="#A0AEC0" />
                <TextInput
                    style={styles.searchInput}
                    placeholder="Search emergency or help..."
                    placeholderTextColor="#A0AEC0"
                />
            </View>

            <ScrollView contentContainerStyle={styles.scrollContent} showsVerticalScrollIndicator={false}>

                <Text style={styles.sectionLabelLabel}>Unified Emergency</Text>
                <TouchableOpacity style={styles.sosCard} onPress={() => handleCall('112')}>
                    <LinearGradient
                        colors={['#FF4D6D', '#FF7A90']}
                        style={styles.sosGradient}
                        start={{ x: 0, y: 0 }}
                        end={{ x: 1, y: 1 }}
                    >
                        <View style={styles.sosIconWrap}>
                            <Text style={styles.sosIcon}>📞</Text>
                        </View>
                        <View>
                            <Text style={styles.sosNum}>112</Text>
                            <Text style={styles.sosTitle}>Unified Emergency</Text>
                            <Text style={styles.sosDesc}>All emergencies · Any phone · Free ✓</Text>
                        </View>
                    </LinearGradient>
                </TouchableOpacity>

                <View style={styles.labelRow}>
                    <Text style={styles.sectionLabelLabel}>Specialized Lines</Text>
                    <View style={styles.badge}><Text style={styles.badgeText}>6 numbers</Text></View>
                </View>

                <View style={styles.grid}>
                    <TouchableOpacity style={styles.gridCard} onPress={() => handleCall('123')}>
                        <View style={[styles.cardTopBar, { backgroundColor: '#FF4D6D' }]} />
                        <View style={[styles.gridIconWrap, { backgroundColor: '#FFF0F3' }]}><Text>🚑</Text></View>
                        <Text style={styles.gridNum}>123</Text>
                        <Text style={styles.gridName}>Ambulance</Text>
                        <TouchableOpacity style={[styles.gridBtn, { backgroundColor: '#FFF0F3' }]} onPress={() => handleCall('123')}>
                            <Feather name="phone-call" size={10} color="#FF4D6D" />
                            <Text style={[styles.gridBtnText, { color: '#FF4D6D' }]}>Call Now</Text>
                        </TouchableOpacity>
                    </TouchableOpacity>

                    <TouchableOpacity style={styles.gridCard} onPress={() => handleCall('122')}>
                        <View style={[styles.cardTopBar, { backgroundColor: '#3A8EF6' }]} />
                        <View style={[styles.gridIconWrap, { backgroundColor: '#E8F1FE' }]}><Text>👮</Text></View>
                        <Text style={styles.gridNum}>122</Text>
                        <Text style={styles.gridName}>Police</Text>
                        <TouchableOpacity style={[styles.gridBtn, { backgroundColor: '#E8F1FE' }]} onPress={() => handleCall('122')}>
                            <Feather name="phone-call" size={10} color="#3A8EF6" />
                            <Text style={[styles.gridBtnText, { color: '#3A8EF6' }]}>Call Now</Text>
                        </TouchableOpacity>
                    </TouchableOpacity>

                    <TouchableOpacity style={styles.gridCard} onPress={() => handleCall('180')}>
                        <View style={[styles.cardTopBar, { backgroundColor: '#F59E0B' }]} />
                        <View style={[styles.gridIconWrap, { backgroundColor: '#FFFBEB' }]}><Text>🔥</Text></View>
                        <Text style={styles.gridNum}>180</Text>
                        <Text style={styles.gridName}>Fire Brigade</Text>
                        <TouchableOpacity style={[styles.gridBtn, { backgroundColor: '#FFFBEB' }]} onPress={() => handleCall('180')}>
                            <Feather name="phone-call" size={10} color="#F59E0B" />
                            <Text style={[styles.gridBtnText, { color: '#F59E0B' }]}>Call Now</Text>
                        </TouchableOpacity>
                    </TouchableOpacity>

                    <TouchableOpacity style={styles.gridCard} onPress={() => handleCall('137')}>
                        <View style={[styles.cardTopBar, { backgroundColor: '#22C55E' }]} />
                        <View style={[styles.gridIconWrap, { backgroundColor: '#EDFBF3' }]}><Text>🏥</Text></View>
                        <Text style={styles.gridNum}>137</Text>
                        <Text style={styles.gridName}>Health Ministry</Text>
                        <TouchableOpacity style={[styles.gridBtn, { backgroundColor: '#EDFBF3' }]} onPress={() => handleCall('137')}>
                            <Feather name="phone-call" size={10} color="#22C55E" />
                            <Text style={[styles.gridBtnText, { color: '#22C55E' }]}>Call Now</Text>
                        </TouchableOpacity>
                    </TouchableOpacity>
                </View>

                {contacts && contacts.length > 0 && (
                    <>
                        <Text style={styles.sectionLabelLabel}>Other API Contacts</Text>
                        {loading ? (
                            <ActivityIndicator size="small" color="#22C55E" />
                        ) : (
                            contacts.map(item => (
                                <TouchableOpacity
                                    key={item.id}
                                    style={[styles.ewCard, item.is_emergency && { borderLeftColor: '#FF4D6D' }]}
                                    onPress={() => handleCall(item.phone_number)}
                                >
                                    <View style={[styles.ewIconWrap, { backgroundColor: item.is_emergency ? '#FFF0F3' : '#E8F1FE' }]}>
                                        <Ionicons name={item.is_emergency ? "alert-circle" : "call"} size={18} color={item.is_emergency ? "#FF4D6D" : "#3A8EF6"} />
                                    </View>
                                    <View>
                                        <Text style={styles.ewName}>{item.name}</Text>
                                        <Text style={styles.ewSub}>{item.description}</Text>
                                    </View>
                                    <View style={styles.ewRight}>
                                        <Text style={[styles.ewNum, item.is_emergency && { color: '#FF4D6D' }]}>{item.phone_number}</Text>
                                        <View style={[styles.ewBtn, { backgroundColor: item.is_emergency ? '#FFF0F3' : '#E8F1FE' }]}>
                                            <Text style={[styles.ewBtnText, { color: item.is_emergency ? '#FF4D6D' : '#3A8EF6' }]}>Call</Text>
                                        </View>
                                    </View>
                                </TouchableOpacity>
                            ))
                        )}
                    </>
                )}

                <Text style={styles.sectionLabelLabel}>Heart Emergency Tips</Text>
                <View style={styles.tipsCard}>
                    <View style={styles.tipsHead}>
                        <View style={styles.tipsIcon}><Text style={{ fontSize: 12 }}>❤️</Text></View>
                        <Text style={styles.tipsTitle}>If your heart rate spikes suddenly</Text>
                    </View>
                    <View style={styles.tipRow}>
                        <View style={styles.tipNum}><Text style={styles.tipNumTxt}>1</Text></View>
                        <Text style={styles.tipTxt}>Stay calm — sit or lie down immediately</Text>
                    </View>
                    <View style={styles.tipRow}>
                        <View style={styles.tipNum}><Text style={styles.tipNumTxt}>2</Text></View>
                        <Text style={styles.tipTxt}>Call 123 if BPM {'>'} 150 or chest pain occurs</Text>
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
        paddingTop: Platform.OS === 'ios' ? 60 : 30,
        paddingBottom: 50,
        borderBottomLeftRadius: 36,
        borderBottomRightRadius: 36,
        overflow: 'hidden',
        position: 'relative',
    },
    heroBgCircle: {
        position: 'absolute',
        top: -30,
        right: -30,
        width: 140,
        height: 140,
        borderRadius: 70,
        backgroundColor: 'rgba(255,255,255,0.1)',
    },
    headerTop: {
        flexDirection: 'row',
        alignItems: 'center',
        justifyContent: 'space-between',
        paddingHorizontal: 24,
        marginBottom: 20,
    },
    backBtn: {
        width: 36,
        height: 36,
        backgroundColor: 'rgba(255,255,255,0.25)',
        borderRadius: 12,
        alignItems: 'center',
        justifyContent: 'center',
    },
    headerTitle: {
        fontSize: 16,
        fontWeight: '800',
        color: 'white',
    },
    helpBtn: {
        width: 36,
        height: 36,
        backgroundColor: 'rgba(255,255,255,0.25)',
        borderRadius: 12,
        alignItems: 'center',
        justifyContent: 'center',
    },
    heroContent: {
        flexDirection: 'row',
        alignItems: 'center',
        paddingHorizontal: 24,
    },
    heroIconWrap: {
        width: 52,
        height: 52,
        backgroundColor: 'rgba(255,255,255,0.25)',
        borderRadius: 16,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 14,
    },
    heroIcon: {
        fontSize: 26,
    },
    heroBigTitle: {
        fontSize: 24,
        fontWeight: '900',
        color: 'white',
        lineHeight: 28,
    },
    heroSubTitle: {
        fontSize: 12,
        color: 'rgba(255,255,255,0.85)',
        marginTop: 4,
    },
    searchWrap: {
        marginHorizontal: 24,
        marginTop: -24,
        backgroundColor: '#fff',
        borderRadius: 16,
        paddingHorizontal: 16,
        paddingVertical: 14,
        flexDirection: 'row',
        alignItems: 'center',
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 8 },
        shadowOpacity: 0.1,
        shadowRadius: 24,
        elevation: 6,
        zIndex: 10,
    },
    searchInput: {
        flex: 1,
        marginLeft: 10,
        fontSize: 14,
        color: '#0F1E3C',
    },
    scrollContent: {
        paddingTop: 24,
        paddingBottom: 40,
    },
    sectionLabelLabel: {
        fontSize: 11,
        fontWeight: '800',
        color: '#A0AEC0',
        textTransform: 'uppercase',
        letterSpacing: 1,
        paddingHorizontal: 24,
        marginBottom: 10,
        marginTop: 10,
    },
    sosCard: {
        marginHorizontal: 24,
        marginBottom: 10,
        borderRadius: 20,
        overflow: 'hidden',
        shadowColor: '#FF4D6D',
        shadowOffset: { width: 0, height: 8 },
        shadowOpacity: 0.28,
        shadowRadius: 22,
        elevation: 8,
    },
    sosGradient: {
        flexDirection: 'row',
        alignItems: 'center',
        padding: 18,
    },
    sosIconWrap: {
        width: 50,
        height: 50,
        backgroundColor: 'rgba(255,255,255,0.22)',
        borderRadius: 16,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 14,
    },
    sosIcon: {
        fontSize: 24,
    },
    sosNum: {
        fontSize: 28,
        fontWeight: '900',
        color: 'white',
    },
    sosTitle: {
        fontSize: 12,
        color: 'rgba(255,255,255,0.9)',
        fontWeight: '700',
    },
    sosDesc: {
        fontSize: 10,
        color: 'rgba(255,255,255,0.7)',
        marginTop: 2,
    },
    labelRow: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
        paddingRight: 24,
    },
    badge: {
        backgroundColor: '#E8F1FE',
        paddingVertical: 4,
        paddingHorizontal: 10,
        borderRadius: 10,
    },
    badgeText: {
        fontSize: 10,
        fontWeight: '700',
        color: '#3A8EF6',
    },
    grid: {
        flexDirection: 'row',
        flexWrap: 'wrap',
        paddingHorizontal: 20,
        justifyContent: 'space-between',
    },
    gridCard: {
        width: '48%',
        backgroundColor: '#fff',
        borderRadius: 16,
        padding: 14,
        marginBottom: 12,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.05,
        shadowRadius: 10,
        elevation: 2,
        position: 'relative',
        overflow: 'hidden',
    },
    cardTopBar: {
        position: 'absolute',
        top: 0,
        left: 0,
        right: 0,
        height: 4,
    },
    gridIconWrap: {
        width: 36,
        height: 36,
        borderRadius: 12,
        alignItems: 'center',
        justifyContent: 'center',
        marginBottom: 8,
        marginTop: 4,
    },
    gridNum: {
        fontSize: 22,
        fontWeight: '900',
        color: '#0F1E3C',
    },
    gridName: {
        fontSize: 11,
        fontWeight: '700',
        color: '#5A6A8A',
        marginBottom: 8,
    },
    gridBtn: {
        flexDirection: 'row',
        alignItems: 'center',
        justifyContent: 'center',
        paddingVertical: 6,
        borderRadius: 10,
    },
    gridBtnText: {
        fontSize: 10,
        fontWeight: '700',
        marginLeft: 4,
    },
    ewCard: {
        backgroundColor: '#fff',
        marginHorizontal: 24,
        marginBottom: 8,
        padding: 14,
        borderRadius: 16,
        flexDirection: 'row',
        alignItems: 'center',
        borderLeftWidth: 4,
        borderLeftColor: '#3A8EF6',
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.05,
        shadowRadius: 10,
        elevation: 2,
    },
    ewIconWrap: {
        width: 40,
        height: 40,
        borderRadius: 12,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 12,
    },
    ewName: {
        fontSize: 11,
        fontWeight: '700',
        color: '#5A6A8A',
    },
    ewSub: {
        fontSize: 10,
        color: '#A0AEC0',
        marginTop: 2,
    },
    ewRight: {
        marginLeft: 'auto',
        alignItems: 'flex-end',
    },
    ewNum: {
        fontSize: 18,
        fontWeight: '900',
        color: '#0F1E3C',
        marginBottom: 4,
    },
    ewBtn: {
        paddingVertical: 4,
        paddingHorizontal: 10,
        borderRadius: 8,
    },
    ewBtnText: {
        fontSize: 10,
        fontWeight: '700',
    },
    tipsCard: {
        marginHorizontal: 24,
        backgroundColor: '#fff',
        borderRadius: 16,
        padding: 16,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.06,
        shadowRadius: 12,
        elevation: 2,
    },
    tipsHead: {
        flexDirection: 'row',
        alignItems: 'center',
        marginBottom: 12,
    },
    tipsIcon: {
        width: 28,
        height: 28,
        backgroundColor: '#FFF0F3',
        borderRadius: 10,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 10,
    },
    tipsTitle: {
        fontSize: 13,
        fontWeight: '800',
        color: '#0F1E3C',
    },
    tipRow: {
        flexDirection: 'row',
        alignItems: 'flex-start',
        marginBottom: 10,
    },
    tipNum: {
        width: 20,
        height: 20,
        backgroundColor: '#E8F1FE',
        borderRadius: 8,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 10,
        marginTop: 2,
    },
    tipNumTxt: {
        fontSize: 11,
        fontWeight: '800',
        color: '#3A8EF6',
    },
    tipTxt: {
        flex: 1,
        fontSize: 12,
        color: '#5A6A8A',
        lineHeight: 18,
    },
});

export default HelpScreen;
