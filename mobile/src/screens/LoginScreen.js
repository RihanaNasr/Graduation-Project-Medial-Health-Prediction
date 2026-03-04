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
    ScrollView
} from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { LinearGradient } from 'expo-linear-gradient';
import { Feather, Ionicons } from '@expo/vector-icons';
import Svg, { Path } from 'react-native-svg';
import { useAuth } from '../context/AuthContext';

const LoginScreen = ({ navigation }) => {
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [loading, setLoading] = useState(false);
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
                        <TouchableOpacity>
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
                        <TouchableOpacity style={styles.socialBtn}>
                            <Ionicons name="logo-google" size={18} color="#0F1E3C" />
                            <Text style={styles.socialBtnText}>Google</Text>
                        </TouchableOpacity>
                        <TouchableOpacity style={styles.socialBtn}>
                            <Ionicons name="logo-apple" size={18} color="#0F1E3C" />
                            <Text style={styles.socialBtnText}>Apple</Text>
                        </TouchableOpacity>
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
});

export default LoginScreen;
