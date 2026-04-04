import React, { useState } from 'react';
import {
    View,
    Text,
    TextInput,
    StyleSheet,
    TouchableOpacity,
    KeyboardAvoidingView,
    Platform,
    Alert,
    ActivityIndicator,
    ScrollView,
    Vibration,
    Pressable,
    Modal,
    Linking,
} from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { LinearGradient } from 'expo-linear-gradient';
import { Feather, Ionicons } from '@expo/vector-icons';
import * as WebBrowser from 'expo-web-browser';
import Svg, { Path } from 'react-native-svg';
import { useAuth } from '../context/AuthContext';

const LoginScreen = ({ navigation }) => {
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [loading, setLoading] = useState(false);
    const [socialLoading, setSocialLoading] = useState(false);
    const [showGoogleSim, setShowGoogleSim] = useState(false);
    const [googleStep, setGoogleStep] = useState(1); // 1: list, 2: input
    const [customAccount, setCustomAccount] = useState(null);
    const [tempEmail, setTempEmail] = useState('');
    const { login } = useAuth();

    const handleLogin = async () => {
        if (!email || !password) {
            Alert.alert('Error', 'Please fill in all fields');
            return;
        }

        setLoading(true);
        const result = await login(email, password);
        setLoading(false);

        if (!result.success) {
            Alert.alert('Login Failed', result.error);
        }
    };

    const handleSocialSelect = async (provider, chosenEmail = null) => {
        setSocialLoading(true);
        // Simulator delay
        setTimeout(async () => {
            const loginEmail = chosenEmail || tempEmail || 'reem.ihab@gmail.com';
            const firstName = loginEmail.split('@')[0];
            
            const result = await login(null, null, true, {
                email: loginEmail,
                first_name: firstName.charAt(0).toUpperCase() + firstName.slice(1),
                last_name: 'User'
            });
            
            setSocialLoading(false);
            if (result.success) {
                setShowGoogleSim(false);
                setGoogleStep(1);
            }
        }, 1500);
    };

    const handleSocialLogin = async (provider) => {
        try {
            setSocialLoading(true);
            const authUrl = provider === 'Google' 
                ? 'https://accounts.google.com' 
                : 'https://appleid.apple.com';
            
            await WebBrowser.openBrowserAsync(authUrl);
            
            // SECURITY CHECK: Ensure only REEM can unlock this profile
            Alert.prompt(
                "🔐 Secure Medical Unlock",
                `Identity verification required for: ${provider === 'Google' ? 'reem.ehab@gmail.com' : 'CardiGo User'}.\n\nPlease enter your 4-digit Security PIN to continue.`,
                [
                    { text: "Cancel", style: "cancel", onPress: () => setSocialLoading(false) },
                    { 
                        text: "Verify", 
                        onPress: async (pin) => {
                            if (pin === "2024") { // Your demo PIN
                                await login(null, null, true, {
                                    email: provider === 'Google' ? 'reem.ehab@gmail.com' : 'reem_ihab@icloud.com',
                                    first_name: provider === 'Google' ? 'Reem' : 'Apple',
                                    last_name: 'Ehab'
                                });
                                setShowGoogleSim(false);
                            } else {
                                Alert.alert("Access Denied", "Incorrect Security PIN. Unauthorized access to medical records is prohibited.");
                                setSocialLoading(false);
                            }
                        }
                    }
                ],
                'secure-text'
            );
        } catch (error) {
            Alert.alert("Error", "Could not connect to secure authentication portal.");
            setSocialLoading(false);
        }
    };

    return (
        <KeyboardAvoidingView
            behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
            style={styles.container}
        >
            <StatusBar style="light" />
            <ScrollView contentContainerStyle={styles.scrollContent} bounces={false}>

                <LinearGradient
                    colors={['#3A8EF6', '#5BADFF']}
                    style={styles.hero}
                    start={{ x: 0, y: 0 }}
                    end={{ x: 1, y: 1 }}
                >
                    <View style={styles.heroBgCircleTop} />
                    <View style={styles.heroBgCircleBottom} />

                    <View style={styles.logoRow}>
                        <View style={styles.pulseIconWrap}>
                            <Svg width="28" height="28" viewBox="0 0 28 28" fill="none">
                                <Path d="M4 14H8L10 8L13 20L16 10L18 16L20 14H24" stroke="white" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" />
                            </Svg>
                        </View>
                        <Text style={styles.logoText}>
                            Cardi<Text style={{ opacity: 0.7 }}>Go</Text>
                        </Text>
                    </View>

                    <Text style={styles.heroTitle}>Your Heart,{'\n'}Our Priority.</Text>
                    <Text style={styles.heroSub}>Smart monitoring that keeps you safe, 24/7</Text>
                </LinearGradient>

                <View style={styles.formContainer}>
                    <View style={styles.fieldGroup}>
                        <Text style={styles.fieldLabel}>EMAIL ADDRESS</Text>
                        <View style={styles.inputWrap}>
                            <Feather name="mail" size={18} color="#A0AEC0" />
                            <TextInput
                                style={styles.input}
                                placeholder="name@example.com"
                                placeholderTextColor="#A0AEC0"
                                value={email}
                                onChangeText={setEmail}
                                keyboardType="email-address"
                                autoCapitalize="none"
                                autoComplete="email"
                            />
                        </View>
                    </View>

                    <View style={styles.fieldGroup}>
                        <Text style={styles.fieldLabel}>PASSWORD</Text>
                        <View style={styles.inputWrap}>
                            <Feather name="lock" size={18} color="#A0AEC0" />
                            <TextInput
                                style={styles.input}
                                placeholder="••••••••"
                                placeholderTextColor="#A0AEC0"
                                value={password}
                                onChangeText={setPassword}
                                secureTextEntry
                                autoComplete="password"
                            />
                        </View>
                    </View>

                    <View style={styles.forgotPassRow}>
                        <TouchableOpacity onPress={() => navigation.navigate('ForgotPassword')}>
                            <Text style={styles.forgotPassText}>Forgot Password?</Text>
                        </TouchableOpacity>
                    </View>

                    <TouchableOpacity
                        style={styles.btnShadowWrap}
                        onPress={handleLogin}
                        disabled={loading}
                    >
                        <LinearGradient
                            colors={['#3A8EF6', '#5BADFF']}
                            style={styles.btnPrimary}
                            start={{ x: 0, y: 0 }}
                            end={{ x: 1, y: 1 }}
                        >
                            {loading ? (
                                <ActivityIndicator color="#fff" />
                            ) : (
                                <Text style={styles.btnPrimaryText}>Sign In →</Text>
                            )}
                        </LinearGradient>
                    </TouchableOpacity>

                    <View style={styles.dividerRow}>
                        <View style={styles.dividerLine} />
                        <Text style={styles.dividerText}>or continue with</Text>
                        <View style={styles.dividerLine} />
                    </View>

                    <View style={styles.socialRow}>
                        <Pressable 
                            style={({ pressed }) => [styles.socialBtn, { opacity: pressed ? 0.6 : 1 }]} 
                            onPress={() => handleSocialLogin('Google')}
                        >
                            <Ionicons name="logo-google" size={18} color="#0F1E3C" />
                            <Text style={styles.socialBtnText}>Google</Text>
                        </Pressable>
                        <Pressable 
                            style={({ pressed }) => [styles.socialBtn, { opacity: pressed ? 0.6 : 1 }]} 
                            onPress={() => handleSocialLogin('Apple')}
                        >
                            <Ionicons name="logo-apple" size={18} color="#0F1E3C" />
                            <Text style={styles.socialBtnText}>Apple</Text>
                        </Pressable>
                    </View>

                    <TouchableOpacity
                        style={styles.signupWrap}
                        onPress={() => navigation.navigate('Register')}
                    >
                        <Text style={styles.signupText}>
                            Don't have an account? <Text style={styles.signupBold}>Sign Up</Text>
                        </Text>
                    </TouchableOpacity>
                </View>

            </ScrollView>

            <Modal visible={showGoogleSim} animationType="slide" transparent={true}>
                <KeyboardAvoidingView 
                    behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
                    style={styles.modalOverlay}
                >
                    <View style={styles.googleModal}>
                        <View style={styles.googleHeader}>
                            <Ionicons name="logo-google" size={24} color="#4285F4" />
                            <Text style={styles.googleTitle}>
                                {googleStep === 1 ? 'Choose an account' : 'Sign in'}
                            </Text>
                            <Text style={styles.googleSubtitle}>to continue to CardiGo</Text>
                        </View>
                        
                        {socialLoading ? (
                            <View style={styles.googleSigningIn}>
                                <ActivityIndicator size="large" color="#4285F4" />
                                <Text style={styles.googleSigningText}>Signing in...</Text>
                                <Text style={styles.googleSigningSub}>Finalizing secure connection</Text>
                            </View>
                        ) : googleStep === 1 ? (
                            <>
                                <TouchableOpacity style={styles.googleAccount} onPress={() => handleSocialSelect('Google')}>
                                    <View style={[styles.googleAvatar, { backgroundColor: '#3A8EF6' }]}>
                                        <Text style={styles.googleAvatarText}>R</Text>
                                    </View>
                                    <View style={{ flex: 1 }}>
                                        <Text style={styles.googleAccountName}>Reem Ehab</Text>
                                        <Text style={styles.googleAccountEmail}>reem.ihab@gmail.com</Text>
                                    </View>
                                </TouchableOpacity>

                                {customAccount && (
                                    <TouchableOpacity style={styles.googleAccount} onPress={() => handleSocialSelect('Google')}>
                                        <View style={[styles.googleAvatar, { backgroundColor: '#FF4D6D' }]}>
                                            <Text style={styles.googleAvatarText}>{customAccount.charAt(0).toUpperCase()}</Text>
                                        </View>
                                        <View style={{ flex: 1 }}>
                                            <Text style={styles.googleAccountName}>New Account</Text>
                                            <Text style={styles.googleAccountEmail}>{customAccount}</Text>
                                        </View>
                                    </TouchableOpacity>
                                )}

                                <TouchableOpacity 
                                    style={styles.googleAccount} 
                                    onPress={() => setGoogleStep(2)}
                                >
                                    <View style={[styles.googleAvatar, { backgroundColor: '#E2E8F0' }]}>
                                        <Feather name="user-plus" size={16} color="#4A5568" />
                                    </View>
                                    <Text style={styles.googleUseAnother}>Use another account</Text>
                                </TouchableOpacity>

                                <View style={styles.googleFooter}>
                                    <Text style={styles.googleFooterText}>
                                        To continue, Google will share your name, email address, and profile picture with CardiGo. 
                                        <Text style={{ color: '#4285F4' }}> Privacy Policy</Text>
                                    </Text>
                                </View>
                                
                                <TouchableOpacity style={styles.googleClose} onPress={() => { setShowGoogleSim(false); setGoogleStep(1); }}>
                                    <Text style={styles.googleCloseText}>Cancel</Text>
                                </TouchableOpacity>
                            </>
                        ) : (
                            <View style={styles.googleInputView}>
                                <TextInput
                                    style={styles.googleInput}
                                    placeholder="Email or phone"
                                    placeholderTextColor="#5F6368"
                                    value={tempEmail}
                                    onChangeText={setTempEmail}
                                    autoFocus
                                />
                                <TouchableOpacity style={styles.googleForgotText}>
                                    <Text style={{ color: '#4285F4', fontWeight: 'bold' }}>Forgot email?</Text>
                                </TouchableOpacity>
                                
                                <View style={styles.googleActionRow}>
                                    <TouchableOpacity onPress={() => setGoogleStep(1)}>
                                        <Text style={{ color: '#4285F4', fontWeight: 'bold' }}>Create account</Text>
                                    </TouchableOpacity>
                                    <TouchableOpacity 
                                        style={[styles.googleNextBtn, !tempEmail.trim() && { opacity: 0.5 }]} 
                                        disabled={!tempEmail.trim()}
                                        onPress={() => {
                                            setCustomAccount(tempEmail);
                                            setGoogleStep(1);
                                        }}
                                    >
                                        <Text style={styles.googleNextText}>Next</Text>
                                    </TouchableOpacity>
                                </View>
                            </View>
                        )}
                    </View>
                </KeyboardAvoidingView>
            </Modal>
        </KeyboardAvoidingView>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#F4F8FF',
    },
    scrollContent: {
        flexGrow: 1,
    },
    hero: {
        paddingTop: 80,
        paddingBottom: 40,
        paddingHorizontal: 32,
        borderBottomLeftRadius: 40,
        borderBottomRightRadius: 40,
        overflow: 'hidden',
        position: 'relative',
    },
    heroBgCircleTop: {
        position: 'absolute',
        top: -40,
        right: -40,
        width: 180,
        height: 180,
        borderRadius: 90,
        backgroundColor: 'rgba(255,255,255,0.1)',
    },
    heroBgCircleBottom: {
        position: 'absolute',
        bottom: -60,
        left: -30,
        width: 140,
        height: 140,
        borderRadius: 70,
        backgroundColor: 'rgba(255,255,255,0.07)',
    },
    logoRow: {
        flexDirection: 'row',
        alignItems: 'center',
        marginBottom: 28,
    },
    pulseIconWrap: {
        width: 48,
        height: 48,
        backgroundColor: 'rgba(255,255,255,0.2)',
        borderRadius: 16,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 10,
    },
    logoText: {
        fontSize: 24,
        fontWeight: 'bold',
        color: '#fff',
    },
    heroTitle: {
        fontSize: 28,
        fontWeight: '900',
        color: '#fff',
        lineHeight: 34,
        marginBottom: 8,
    },
    heroSub: {
        fontSize: 14,
        color: 'rgba(255,255,255,0.75)',
    },
    formContainer: {
        paddingHorizontal: 24,
        paddingTop: 28,
        paddingBottom: 40,
        flex: 1,
        backgroundColor: '#F4F8FF',
    },
    fieldGroup: {
        marginBottom: 16,
    },
    fieldLabel: {
        fontSize: 12,
        fontWeight: '700',
        color: '#5A6A8A',
        marginBottom: 6,
        letterSpacing: 0.5,
    },
    inputWrap: {
        backgroundColor: '#fff',
        borderWidth: 1.5,
        borderColor: '#E4ECFD',
        borderRadius: 12,
        paddingHorizontal: 16,
        paddingVertical: 14,
        flexDirection: 'row',
        alignItems: 'center',
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 2 },
        shadowOpacity: 0.08,
        shadowRadius: 12,
        elevation: 2,
    },
    input: {
        flex: 1,
        marginLeft: 10,
        fontSize: 14,
        color: '#A0AEC0',
        padding: 0,
    },
    forgotPassRow: {
        alignItems: 'flex-end',
        marginTop: -6,
        marginBottom: 20,
    },
    forgotPassText: {
        fontSize: 12,
        fontWeight: '700',
        color: '#3A8EF6',
    },
    btnShadowWrap: {
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 8 },
        shadowOpacity: 0.35,
        shadowRadius: 24,
        elevation: 8,
        marginBottom: 24,
    },
    btnPrimary: {
        borderRadius: 12,
        paddingVertical: 16,
        alignItems: 'center',
    },
    btnPrimaryText: {
        color: '#fff',
        fontSize: 15,
        fontWeight: '700',
        letterSpacing: 0.3,
    },
    dividerRow: {
        flexDirection: 'row',
        alignItems: 'center',
        marginBottom: 24,
    },
    dividerLine: {
        flex: 1,
        height: 1,
        backgroundColor: '#E4ECFD',
    },
    dividerText: {
        marginHorizontal: 12,
        fontSize: 13,
        color: '#A0AEC0',
    },
    socialRow: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        marginBottom: 30,
    },
    socialBtn: {
        flex: 1,
        backgroundColor: '#fff',
        borderWidth: 1.5,
        borderColor: '#E4ECFD',
        borderRadius: 12,
        paddingVertical: 13,
        flexDirection: 'row',
        alignItems: 'center',
        justifyContent: 'center',
        marginHorizontal: 6,
    },
    socialBtnText: {
        marginLeft: 8,
        fontSize: 13,
        fontWeight: '600',
        color: '#0F1E3C',
    },
    signupWrap: {
        alignItems: 'center',
    },
    signupText: {
        fontSize: 13,
        color: '#5A6A8A',
    },
    signupBold: {
        fontWeight: '700',
        color: '#3A8EF6',
    },
    // Google Simulator Styles
    modalOverlay: {
        flex: 1,
        backgroundColor: 'rgba(0,0,0,0.5)',
        justifyContent: 'flex-end',
    },
    googleModal: {
        backgroundColor: 'white',
        borderTopLeftRadius: 30,
        borderTopRightRadius: 30,
        paddingTop: 32,
        paddingHorizontal: 24,
        paddingBottom: 40,
    },
    googleHeader: {
        alignItems: 'center',
        marginBottom: 30,
    },
    googleTitle: {
        fontSize: 22,
        fontWeight: '700',
        color: '#1A73E8',
        marginTop: 18,
    },
    googleSubtitle: {
        fontSize: 15,
        color: '#5F6368',
        marginTop: 6,
    },
    googleAccount: {
        flexDirection: 'row',
        alignItems: 'center',
        paddingVertical: 18,
        borderBottomWidth: 1,
        borderBottomColor: '#F1F3F4',
    },
    googleAvatar: {
        width: 44,
        height: 44,
        borderRadius: 22,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 16,
    },
    googleAvatarText: {
        color: 'white',
        fontSize: 18,
        fontWeight: 'bold',
    },
    googleAccountName: {
        fontSize: 16,
        fontWeight: '600',
        color: '#202124',
    },
    googleAccountEmail: {
        fontSize: 13,
        color: '#5F6368',
    },
    googleUseAnother: {
        fontSize: 15,
        color: '#5F6368',
        fontWeight: '600',
    },
    googleFooter: {
        marginTop: 24,
        paddingHorizontal: 8,
    },
    googleFooterText: {
        fontSize: 12,
        color: '#5F6368',
        lineHeight: 18,
        textAlign: 'center',
    },
    googleClose: {
        marginTop: 32,
        alignItems: 'center',
    },
    googleCloseText: {
        fontSize: 16,
        fontWeight: '700',
        color: '#4285F4',
    },
    googleSigningIn: {
        alignItems: 'center',
        paddingVertical: 60,
    },
    googleSigningText: {
        fontSize: 18,
        fontWeight: '700',
        color: '#202124',
        marginTop: 20,
    },
    googleSigningSub: {
        fontSize: 14,
        color: '#5F6368',
        marginTop: 8,
    },
    googleInputView: {
        paddingVertical: 20,
    },
    googleInput: {
        borderWidth: 1,
        borderColor: '#DADCE0',
        borderRadius: 4,
        padding: 16,
        fontSize: 16,
        color: '#202124',
    },
    googleForgotText: {
        marginTop: 8,
        marginBottom: 40,
    },
    googleActionRow: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        alignItems: 'center',
    },
    googleNextBtn: {
        backgroundColor: '#1A73E8',
        paddingHorizontal: 24,
        paddingVertical: 10,
        borderRadius: 4,
    },
    googleNextText: {
        color: 'white',
        fontWeight: '700',
        fontSize: 14,
    },
});

export default LoginScreen;
