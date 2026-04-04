import React, { useState } from 'react';
import {
    View,
    Text,
    StyleSheet,
    TouchableOpacity,
    TextInput,
    Alert,
    Platform,
    KeyboardAvoidingView,
    ScrollView
} from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather, Ionicons } from '@expo/vector-icons';
import { LinearGradient } from 'expo-linear-gradient';

const ForgotPasswordScreen = ({ navigation }) => {
    const [step, setStep] = useState(1); // 1: Email, 2: Code & New Password
    const [email, setEmail] = useState('');
    const [code, setCode] = useState('');
    const [newPassword, setNewPassword] = useState('');
    const [showNotif, setShowNotif] = useState(false);

    const handleSendCode = () => {
        if (!email.includes('@')) {
            Alert.alert("Error", "Please enter a valid email address.");
            return;
        }

        // Success Alert as per screenshot
        Alert.alert(
            "Success",
            "OTP code sent! Check your notifications (or server console).",
            [{ 
                text: "OK", 
                onPress: () => {
                    setStep(2);
                    // Show custom notification after a short delay
                    setTimeout(() => setShowNotif(true), 1500);
                } 
            }]
        );
    };

    const handleChangePassword = () => {
        if (code.length < 6) {
            Alert.alert("Error", "Please enter the 6-digit verification code.");
            return;
        }
        if (newPassword.length < 6) {
            Alert.alert("Error", "Password must be at least 6 characters.");
            return;
        }

        // 1. Initial Success Alert
        Alert.alert(
            "Success",
            "Your password has been changed successfully!",
            [{ 
                text: "OK", 
                onPress: () => {
                    // 2. Simulated iOS "Save Password" Prompt
                    setTimeout(() => {
                        Alert.alert(
                            "Save Password",
                            `Would you like to save this password in your iCloud Keychain for "CardiGo"?`,
                            [
                                { text: "Not Now", style: "cancel", onPress: () => navigation.navigate('Login') },
                                { 
                                    text: "Save Password", 
                                    onPress: () => {
                                        Alert.alert("Password Saved", "Your new password is now securely stored in your keychain.");
                                        navigation.navigate('Login');
                                    } 
                                }
                            ]
                        );
                    }, 800);
                } 
            }]
        );
    };

    return (
        <KeyboardAvoidingView 
            behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
            style={styles.container}
        >
            <StatusBar style="light" />

            {/* HIGH-FIDELITY SIMULATED NOTIFICATION */}
            {showNotif && (
                <TouchableOpacity 
                    style={styles.notifCard} 
                    onPress={() => {
                        setCode('929939');
                        setShowNotif(false);
                    }}
                >
                    <View style={styles.notifHeader}>
                        <View style={styles.notifLead}>
                            <View style={styles.notifIconWrap}>
                                <Text style={styles.notifIconEmoji}>🔐</Text>
                            </View>
                            <Text style={styles.notifAppName}>CardiGo</Text>
                        </View>
                        <Text style={styles.notifTime}>now</Text>
                    </View>
                    <Text style={styles.notifTitle}>CardiGo Reset Code</Text>
                    <Text style={styles.notifBody}>
                        Your verification code is: 929939. Tap to auto-fill.
                    </Text>
                    <View style={styles.notifHandle} />
                </TouchableOpacity>
            )}
            
            {/* Header Section */}
            <LinearGradient
                colors={['#3A8EF6', '#5BADFF']}
                style={styles.header}
                start={{ x: 0, y: 0 }}
                end={{ x: 1, y: 1 }}
            >
                <TouchableOpacity 
                    style={styles.backBtn}
                    onPress={() => step === 1 ? navigation.goBack() : setStep(1)}
                >
                    <Feather name="arrow-left" size={24} color="white" />
                </TouchableOpacity>
                <Text style={styles.headerTitle}>
                    {step === 1 ? "Forgot\nPassword?" : "Reset\nPassword"}
                </Text>
                <Text style={styles.headerSub}>
                    {step === 1 
                        ? "Don't worry! Enter your registered email to receive a reset code."
                        : "Verify your identity using the 6-digit code sent to your email."
                    }
                </Text>
            </LinearGradient>

            <ScrollView contentContainerStyle={styles.formContent}>
                {step === 1 ? (
                    <View style={styles.stepContainer}>
                        <Text style={styles.fieldLabel}>EMAIL ADDRESS</Text>
                        <View style={styles.inputWrap}>
                            <Feather name="mail" size={18} color="#A0AEC0" />
                            <TextInput
                                style={styles.input}
                                placeholder="example@gmail.com"
                                placeholderTextColor="#A0AEC0"
                                value={email}
                                onChangeText={setEmail}
                                keyboardType="email-address"
                                autoCapitalize="none"
                            />
                        </View>

                        <TouchableOpacity 
                            style={styles.btnPrimary}
                            onPress={handleSendCode}
                        >
                            <Text style={styles.btnText}>Send Reset Code</Text>
                            <Feather name="send" size={18} color="white" style={styles.btnIcon} />
                        </TouchableOpacity>
                    </View>
                ) : (
                    <View style={styles.stepContainer}>
                        <Text style={styles.fieldLabel}>VERIFICATION CODE</Text>
                        <View style={styles.inputWrap}>
                            <Feather name="shield" size={18} color="#A0AEC0" />
                            <TextInput
                                style={styles.input}
                                placeholder="123456"
                                placeholderTextColor="#A0AEC0"
                                value={code}
                                onChangeText={(val) => {
                                    setCode(val);
                                    if (val.length === 6) setShowNotif(false);
                                }}
                                keyboardType="number-pad"
                                maxLength={6}
                            />
                        </View>

                        <Text style={[styles.fieldLabel, { marginTop: 20 }]}>NEW PASSWORD</Text>
                        <View style={styles.inputWrap}>
                            <Feather name="lock" size={18} color="#A0AEC0" />
                            <TextInput
                                style={styles.input}
                                placeholder="••••••••"
                                placeholderTextColor="#A0AEC0"
                                value={newPassword}
                                onChangeText={setNewPassword}
                                secureTextEntry
                            />
                        </View>

                        <TouchableOpacity 
                            style={styles.btnPrimary}
                            onPress={handleChangePassword}
                        >
                            <Text style={styles.btnText}>Change Password</Text>
                            <Ionicons name="checkmark-circle-outline" size={20} color="white" style={styles.btnIcon} />
                        </TouchableOpacity>
                    </View>
                )}
            </ScrollView>
        </KeyboardAvoidingView>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#F4F8FF',
    },
    header: {
        paddingTop: 60,
        paddingHorizontal: 24,
        paddingBottom: 40,
        borderBottomLeftRadius: 32,
        borderBottomRightRadius: 32,
    },
    backBtn: {
        width: 40,
        height: 40,
        alignItems: 'flex-start',
        justifyContent: 'center',
        marginBottom: 20,
    },
    headerTitle: {
        fontSize: 32,
        fontWeight: '900',
        color: 'white',
        lineHeight: 38,
        marginBottom: 12,
    },
    headerSub: {
        fontSize: 14,
        color: 'rgba(255,255,255,0.8)',
        lineHeight: 20,
        fontWeight: '600',
    },
    formContent: {
        padding: 24,
        paddingTop: 32,
    },
    stepContainer: {
        width: '100%',
    },
    fieldLabel: {
        fontSize: 10,
        fontWeight: '800',
        color: '#A0AEC0',
        letterSpacing: 1,
        marginBottom: 8,
    },
    inputWrap: {
        flexDirection: 'row',
        alignItems: 'center',
        backgroundColor: 'rgba(160, 174, 192, 0.1)',
        borderRadius: 16,
        paddingHorizontal: 16,
        height: 56,
        borderWidth: 1.5,
        borderColor: 'rgba(160, 174, 192, 0.05)',
    },
    input: {
        flex: 1,
        marginLeft: 12,
        fontSize: 15,
        color: '#0F1E3C',
        fontWeight: '600',
    },
    btnPrimary: {
        backgroundColor: '#3A8EF6',
        borderRadius: 16,
        height: 56,
        flexDirection: 'row',
        alignItems: 'center',
        justifyContent: 'center',
        marginTop: 32,
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 8 },
        shadowOpacity: 0.2,
        shadowRadius: 16,
        elevation: 6,
    },
    btnText: {
        color: 'white',
        fontSize: 16,
        fontWeight: '900',
    },
    btnIcon: {
        marginLeft: 8,
    },
    // HIGH-FIDELITY NOTIFICATION STYLES
    notifCard: {
        position: 'absolute',
        top: 50,
        left: 20,
        right: 20,
        zIndex: 9999,
        backgroundColor: '#FCFDFF',
        borderRadius: 20,
        padding: 16,
        paddingBottom: 10,
        borderWidth: 1,
        borderColor: '#E8F1FE',
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 10 },
        shadowOpacity: 0.15,
        shadowRadius: 20,
        elevation: 10,
    },
    notifHeader: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
        marginBottom: 8,
    },
    notifLead: {
        flexDirection: 'row',
        alignItems: 'center',
    },
    notifIconWrap: {
        width: 24,
        height: 24,
        borderRadius: 6,
        backgroundColor: '#E8F1FE',
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 8,
    },
    notifIconEmoji: {
        fontSize: 12,
    },
    notifAppName: {
        fontSize: 12,
        fontWeight: '800',
        color: '#A0AEC0',
        letterSpacing: 0.5,
    },
    notifTime: {
        fontSize: 11,
        color: '#A0AEC0',
        fontWeight: '600',
    },
    notifTitle: {
        fontSize: 14,
        fontWeight: '900',
        color: '#0F1E3C',
        marginBottom: 2,
    },
    notifBody: {
        fontSize: 13,
        color: '#5A6A8A',
        lineHeight: 18,
        fontWeight: '500',
    },
    notifHandle: {
        width: 36,
        height: 4,
        backgroundColor: '#E8F1FE',
        borderRadius: 2,
        alignSelf: 'center',
        marginTop: 12,
    }
});

export default ForgotPasswordScreen;
