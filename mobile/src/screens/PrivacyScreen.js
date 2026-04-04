import React, { useState } from 'react';
import { View, Text, StyleSheet, ScrollView, Switch, TouchableOpacity } from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather } from '@expo/vector-icons';

const PrivacyScreen = ({ navigation }) => {
    const [location, setLocation] = useState(false);
    const [healthSync, setHealthSync] = useState(true);
    const [doctorShare, setDoctorShare] = useState(true);
    const [biometrics, setBiometrics] = useState(true);

    return (
        <View style={styles.container}>
            <StatusBar style="dark" />
            <ScrollView contentContainerStyle={styles.content} showsVerticalScrollIndicator={false}>

                <Text style={styles.sectionLabel}>DATA PERMISSIONS</Text>
                <View style={styles.cardBlock}>
                    <View style={styles.listItem}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#E8F1FE' }]}>
                            <Feather name="map-pin" size={18} color="#3A8EF6" />
                        </View>
                        <View style={styles.listContent}>
                            <Text style={styles.listTitle}>Location Services</Text>
                            <Text style={styles.listSub}>Track fitness routes</Text>
                        </View>
                        <Switch
                            trackColor={{ false: '#E4ECFD', true: '#22C55E' }}
                            thumbColor="white"
                            onValueChange={setLocation}
                            value={location}
                            style={{ transform: [{ scaleX: 0.9 }, { scaleY: 0.9 }] }}
                        />
                    </View>
                    <View style={styles.divider} />

                    <View style={styles.listItem}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#FFF0F3' }]}>
                            <Feather name="heart" size={18} color="#FF4D6D" />
                        </View>
                        <View style={styles.listContent}>
                            <Text style={styles.listTitle}>Health Data Sync</Text>
                            <Text style={styles.listSub}>Sync with Apple/Google Health</Text>
                        </View>
                        <Switch
                            trackColor={{ false: '#E4ECFD', true: '#22C55E' }}
                            thumbColor="white"
                            onValueChange={setHealthSync}
                            value={healthSync}
                            style={{ transform: [{ scaleX: 0.9 }, { scaleY: 0.9 }] }}
                        />
                    </View>
                    <View style={styles.divider} />

                    <View style={styles.listItem}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#EDFBF3' }]}>
                            <Feather name="users" size={18} color="#22C55E" />
                        </View>
                        <View style={styles.listContent}>
                            <Text style={styles.listTitle}>Doctor Data Sharing</Text>
                            <Text style={styles.listSub}>Allow physicians to view history</Text>
                        </View>
                        <Switch
                            trackColor={{ false: '#E4ECFD', true: '#22C55E' }}
                            thumbColor="white"
                            onValueChange={setDoctorShare}
                            value={doctorShare}
                            style={{ transform: [{ scaleX: 0.9 }, { scaleY: 0.9 }] }}
                        />
                    </View>
                </View>

                <Text style={styles.sectionLabel}>SECURITY</Text>
                <View style={styles.cardBlock}>
                    <View style={styles.listItem}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#FFFBEB' }]}>
                            <Feather name="lock" size={18} color="#F59E0B" />
                        </View>
                        <View style={styles.listContent}>
                            <Text style={styles.listTitle}>Biometric Lock</Text>
                            <Text style={styles.listSub}>Require Face/Touch ID to open</Text>
                        </View>
                        <Switch
                            trackColor={{ false: '#E4ECFD', true: '#22C55E' }}
                            thumbColor="white"
                            onValueChange={setBiometrics}
                            value={biometrics}
                            style={{ transform: [{ scaleX: 0.9 }, { scaleY: 0.9 }] }}
                        />
                    </View>
                    <View style={styles.divider} />
                    <TouchableOpacity style={styles.listItem} onPress={() => navigation.navigate('ChangePassword')}>
                        <View style={[styles.listIconWrap, { backgroundColor: '#F4F8FF' }]}>
                            <Feather name="key" size={18} color="#0F1E3C" />
                        </View>
                        <View style={styles.listContent}>
                            <Text style={styles.listTitle}>Change Password</Text>
                            <Text style={styles.listSub}>Update your account login</Text>
                        </View>
                        <Feather name="chevron-right" size={18} color="#A0AEC0" />
                    </TouchableOpacity>
                </View>

                <Text style={styles.sectionLabel}>LEGAL</Text>
                <View style={styles.cardBlock}>
                    <TouchableOpacity 
                        style={styles.listItem} 
                        onPress={() => navigation.navigate('LegalContent', { 
                            title: 'Privacy Policy', 
                            content: 'Your privacy is critical to us. CardiGO uses 256-bit AES encryption to protect your medical data both at rest and in transit. \n\n1. Data Collection: We collect vitals such as Heart Rate, SpO2, and Blood Pressure solely for health monitoring.\n\n2. AI Processing: Your data is processed by local and secure cloud AI models to provide health insights. \n\n3. Doctor Sharing: We only share your data with healthcare providers you explicitly authorize.\n\n4. Your Rights: You have the right to export or delete your health records at any time.' 
                        })}
                    >
                        <View style={styles.listContent}>
                            <Text style={styles.listTitle}>Privacy Policy</Text>
                        </View>
                        <Feather name="external-link" size={18} color="#A0AEC0" />
                    </TouchableOpacity>
                    <View style={styles.divider} />
                    <TouchableOpacity 
                        style={styles.listItem}
                        onPress={() => navigation.navigate('LegalContent', { 
                            title: 'Terms of Service', 
                            content: 'By using CardiGO, you agree to the following terms:\n\n1. Scope of Service: CardiGO is an assistant for monitoring health metrics and is NOT a replacement for professional medical advice, diagnosis, or treatment.\n\n2. Emergency Use: In a life-threatening emergency, always contact local emergency services immediately.\n\n3. Data Accuracy: You are responsible for ensuring the accuracy of the health records you input.\n\n4. Account Security: You must keep your credentials secure.' 
                        })}
                    >
                        <View style={styles.listContent}>
                            <Text style={styles.listTitle}>Terms of Service</Text>
                        </View>
                        <Feather name="external-link" size={18} color="#A0AEC0" />
                    </TouchableOpacity>
                </View>

            </ScrollView>
        </View>
    );
};

const styles = StyleSheet.create({
    container: { flex: 1, backgroundColor: '#F4F8FF' },
    content: { padding: 24, paddingTop: 20, paddingBottom: 60 },
    sectionLabel: {
        fontSize: 11,
        fontWeight: '800',
        color: '#A0AEC0',
        letterSpacing: 1.2,
        marginBottom: 12,
        marginLeft: 8,
        marginTop: 10,
    },
    cardBlock: {
        backgroundColor: '#FFF',
        borderRadius: 24,
        paddingHorizontal: 20,
        paddingVertical: 8,
        marginBottom: 16,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 6 },
        shadowOpacity: 0.04,
        shadowRadius: 16,
        elevation: 2,
    },
    listItem: {
        flexDirection: 'row',
        alignItems: 'center',
        paddingVertical: 14,
    },
    listIconWrap: {
        width: 40,
        height: 40,
        borderRadius: 12,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 14,
    },
    listContent: {
        flex: 1,
    },
    listTitle: {
        fontSize: 15,
        fontWeight: '700',
        color: '#0F1E3C',
        marginBottom: 2,
    },
    listSub: {
        fontSize: 11,
        fontWeight: '600',
        color: '#A0AEC0',
    },
    divider: {
        height: 1,
        backgroundColor: '#F4F8FF',
        marginLeft: 54,
    },
});

export default PrivacyScreen;
