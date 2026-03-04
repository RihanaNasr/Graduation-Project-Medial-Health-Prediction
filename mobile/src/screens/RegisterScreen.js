import React, { useState } from 'react';
import {
    View,
    Text,
    TextInput,
    StyleSheet,
    TouchableOpacity,
    ScrollView,
    KeyboardAvoidingView,
    Platform,
    Alert,
    ActivityIndicator,
} from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Picker } from '@react-native-picker/picker';
import { Feather } from '@expo/vector-icons';
import { LinearGradient } from 'expo-linear-gradient';
import Svg, { Path } from 'react-native-svg';
import { useAuth } from '../context/AuthContext';

const RegisterScreen = ({ navigation }) => {
    const [formData, setFormData] = useState({
        email: '',
        username: '',
        first_name: '',
        last_name: '',
        password: '',
        password_confirm: '',
        phone_number: '',
        gender: '',
    });
    const [loading, setLoading] = useState(false);
    const { register } = useAuth();

    const updateField = (field, value) => {
        setFormData({ ...formData, [field]: value });
    };

    const handleRegister = async () => {
        if (!formData.email || !formData.password || !formData.first_name) {
            Alert.alert('Error', 'Please fill in all required fields');
            return;
        }

        if (formData.password !== formData.password_confirm) {
            Alert.alert('Error', 'Passwords do not match');
            return;
        }

        setLoading(true);
        const result = await register(formData);
        setLoading(false);

        if (!result.success) {
            console.log('Registration error details:', result.error);
            const errorMsg = typeof result.error === 'object'
                ? JSON.stringify(result.error, null, 2)
                : result.error;
            Alert.alert('Registration Failed', errorMsg);
        }
    };

    return (
        <KeyboardAvoidingView
            behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
            style={styles.container}
        >
            <StatusBar style="light" />

            {/* Top Gradient Hero */}
            <LinearGradient
                colors={['#3A8EF6', '#5BADFF']}
                style={styles.authHero}
                start={{ x: 0, y: 0 }}
                end={{ x: 1, y: 1 }}
            >
                <View style={styles.heroBgCircle} />

                <View style={styles.logoPulse}>
                    <Svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="white" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
                        <Path d="M22 12h-4l-3 9L9 3l-3 9H2" />
                    </Svg>
                </View>

                <Text style={styles.authTitle}>Create Account</Text>
                <Text style={styles.authSub}>Join CardiGo to start monitoring</Text>
            </LinearGradient>

            <ScrollView contentContainerStyle={styles.scrollContent} bounces={false}>
                <View style={styles.authCard}>

                    <View style={styles.row}>
                        <View style={styles.halfWidth}>
                            <Text style={styles.label}>First Name *</Text>
                            <View style={styles.inputWrap}>
                                <Feather name="user" size={18} color="#A0AEC0" style={styles.inputIcon} />
                                <TextInput
                                    style={styles.input}
                                    placeholder="Jane"
                                    placeholderTextColor="#A0AEC0"
                                    value={formData.first_name}
                                    onChangeText={(value) => updateField('first_name', value)}
                                />
                            </View>
                        </View>

                        <View style={styles.halfWidth}>
                            <Text style={styles.label}>Last Name *</Text>
                            <View style={styles.inputWrap}>
                                <TextInput
                                    style={[styles.input, { paddingLeft: 14 }]}
                                    placeholder="Doe"
                                    placeholderTextColor="#A0AEC0"
                                    value={formData.last_name}
                                    onChangeText={(value) => updateField('last_name', value)}
                                />
                            </View>
                        </View>
                    </View>

                    <Text style={styles.label}>Email Address *</Text>
                    <View style={styles.inputWrap}>
                        <Feather name="mail" size={18} color="#A0AEC0" style={styles.inputIcon} />
                        <TextInput
                            style={styles.input}
                            placeholder="your.email@example.com"
                            placeholderTextColor="#A0AEC0"
                            value={formData.email}
                            onChangeText={(value) => updateField('email', value)}
                            keyboardType="email-address"
                            autoCapitalize="none"
                        />
                    </View>

                    <Text style={styles.label}>Username *</Text>
                    <View style={styles.inputWrap}>
                        <Feather name="at-sign" size={18} color="#A0AEC0" style={styles.inputIcon} />
                        <TextInput
                            style={styles.input}
                            placeholder="Choose a username"
                            placeholderTextColor="#A0AEC0"
                            value={formData.username}
                            onChangeText={(value) => updateField('username', value)}
                            autoCapitalize="none"
                        />
                    </View>

                    <Text style={styles.label}>Phone Number</Text>
                    <View style={styles.inputWrap}>
                        <Feather name="phone" size={18} color="#A0AEC0" style={styles.inputIcon} />
                        <TextInput
                            style={styles.input}
                            placeholder="+1 234 567 8900"
                            placeholderTextColor="#A0AEC0"
                            value={formData.phone_number}
                            onChangeText={(value) => updateField('phone_number', value)}
                            keyboardType="phone-pad"
                        />
                    </View>

                    <Text style={styles.label}>Gender</Text>
                    <View style={[styles.inputWrap, styles.pickerWrap]}>
                        <Picker
                            selectedValue={formData.gender}
                            onValueChange={(value) => updateField('gender', value)}
                            style={{ flex: 1, color: '#0F1E3C', marginLeft: -8 }}
                        >
                            <Picker.Item label="Select Gender" value="" color="#A0AEC0" />
                            <Picker.Item label="Male" value="Male" />
                            <Picker.Item label="Female" value="Female" />
                            <Picker.Item label="Other" value="Other" />
                        </Picker>
                    </View>

                    <Text style={styles.label}>Password *</Text>
                    <View style={styles.inputWrap}>
                        <Feather name="lock" size={18} color="#A0AEC0" style={styles.inputIcon} />
                        <TextInput
                            style={styles.input}
                            placeholder="At least 8 characters"
                            placeholderTextColor="#A0AEC0"
                            value={formData.password}
                            onChangeText={(value) => updateField('password', value)}
                            secureTextEntry
                        />
                    </View>

                    <Text style={styles.label}>Confirm Password *</Text>
                    <View style={styles.inputWrap}>
                        <Feather name="check-circle" size={18} color="#A0AEC0" style={styles.inputIcon} />
                        <TextInput
                            style={styles.input}
                            placeholder="Re-enter password"
                            placeholderTextColor="#A0AEC0"
                            value={formData.password_confirm}
                            onChangeText={(value) => updateField('password_confirm', value)}
                            secureTextEntry
                        />
                    </View>

                    <TouchableOpacity style={styles.btnWrap} onPress={handleRegister} disabled={loading}>
                        <LinearGradient
                            colors={['#3A8EF6', '#5BADFF']}
                            style={styles.btnPrimary}
                            start={{ x: 0, y: 0 }}
                            end={{ x: 1, y: 0 }}
                        >
                            {loading ? (
                                <ActivityIndicator color="#fff" />
                            ) : (
                                <Text style={styles.btnPrimaryText}>Create Account</Text>
                            )}
                        </LinearGradient>
                    </TouchableOpacity>

                    <View style={styles.footerWrap}>
                        <Text style={styles.footerText}>Already have an account? </Text>
                        <TouchableOpacity onPress={() => navigation.navigate('Login')}>
                            <Text style={styles.footerLink}>Sign In</Text>
                        </TouchableOpacity>
                    </View>

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
    authHero: {
        paddingTop: Platform.OS === 'ios' ? 70 : 40,
        paddingBottom: 80,
        paddingHorizontal: 24,
        alignItems: 'center',
        borderBottomLeftRadius: 36,
        borderBottomRightRadius: 36,
        position: 'relative',
        overflow: 'hidden',
    },
    heroBgCircle: {
        position: 'absolute',
        top: -40,
        left: -40,
        width: 140,
        height: 140,
        borderRadius: 70,
        backgroundColor: 'rgba(255,255,255,0.08)',
    },
    logoPulse: {
        width: 64,
        height: 64,
        backgroundColor: 'rgba(255,255,255,0.2)',
        borderRadius: 20,
        alignItems: 'center',
        justifyContent: 'center',
        marginBottom: 20,
        borderWidth: 2,
        borderColor: 'rgba(255,255,255,0.3)',
    },
    authTitle: {
        fontSize: 28,
        fontWeight: '900',
        color: 'white',
        letterSpacing: -0.5,
    },
    authSub: {
        fontSize: 14,
        color: 'rgba(255,255,255,0.85)',
        marginTop: 6,
    },
    scrollContent: {
        flexGrow: 1,
        paddingHorizontal: 20,
        paddingBottom: 40,
    },
    authCard: {
        backgroundColor: '#fff',
        borderRadius: 24,
        padding: 24,
        marginTop: -40,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 10 },
        shadowOpacity: 0.08,
        shadowRadius: 30,
        elevation: 10,
    },
    label: {
        fontSize: 12,
        fontWeight: '700',
        color: '#5A6A8A',
        marginBottom: 8,
        marginTop: 16,
    },
    inputWrap: {
        flexDirection: 'row',
        alignItems: 'center',
        backgroundColor: '#F4F8FF',
        borderWidth: 1,
        borderColor: '#E4ECFD',
        borderRadius: 14,
        height: 52,
        overflow: 'hidden',
    },
    pickerWrap: {
        paddingRight: 10,
    },
    inputIcon: {
        paddingLeft: 16,
        paddingRight: 10,
    },
    input: {
        flex: 1,
        height: '100%',
        fontSize: 15,
        color: '#0F1E3C',
    },
    row: {
        flexDirection: 'row',
        justifyContent: 'space-between',
        marginTop: 16,
    },
    halfWidth: {
        width: '48%',
    },
    btnWrap: {
        marginTop: 32,
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 8 },
        shadowOpacity: 0.3,
        shadowRadius: 20,
        elevation: 8,
    },
    btnPrimary: {
        height: 56,
        borderRadius: 16,
        alignItems: 'center',
        justifyContent: 'center',
    },
    btnPrimaryText: {
        color: 'white',
        fontSize: 16,
        fontWeight: '800',
    },
    footerWrap: {
        flexDirection: 'row',
        justifyContent: 'center',
        alignItems: 'center',
        marginTop: 24,
    },
    footerText: {
        fontSize: 14,
        color: '#A0AEC0',
    },
    footerLink: {
        fontSize: 14,
        color: '#3A8EF6',
        fontWeight: '700',
    },
});

export default RegisterScreen;
