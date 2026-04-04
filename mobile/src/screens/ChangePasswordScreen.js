import React, { useState } from 'react';
import { View, Text, StyleSheet, TextInput, TouchableOpacity, Alert, KeyboardAvoidingView, Platform, ScrollView } from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather } from '@expo/vector-icons';
import { useTheme } from '../context/ThemeContext';

const ChangePasswordScreen = ({ navigation }) => {
    const { isDark, colors } = useTheme();
    const [oldPassword, setOldPassword] = useState('');
    const [newPassword, setNewPassword] = useState('');
    const [confirmPassword, setConfirmPassword] = useState('');
    const [showPasswords, setShowPasswords] = useState(false);

    const handleUpdate = () => {
        if (!oldPassword || !newPassword || !confirmPassword) {
            Alert.alert("Error", "Please fill in all fields.");
            return;
        }
        if (newPassword !== confirmPassword) {
            Alert.alert("Error", "New passwords do not match.");
            return;
        }
        if (newPassword.length < 6) {
            Alert.alert("Error", "Password must be at least 6 characters.");
            return;
        }

        // Simulate API call
        Alert.alert(
            "Success", 
            "Your password has been updated successfully.",
            [{ text: "OK", onPress: () => navigation.goBack() }]
        );
    };

    return (
        <KeyboardAvoidingView 
            behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
            style={[styles.container, { backgroundColor: colors.background }]}
        >
            <StatusBar style={isDark ? "light" : "dark"} />
            <ScrollView contentContainerStyle={styles.scrollContent}>
                <View style={styles.header}>
                    <Text style={[styles.title, { color: colors.text }]}>Security Update</Text>
                    <Text style={[styles.subtitle, { color: colors.subtext }]}>Ensure your account remains safe with a strong, updated password.</Text>
                </View>

                <View style={styles.form}>
                    <View style={styles.inputGroup}>
                        <Text style={[styles.label, { color: colors.subtext }]}>CURRENT PASSWORD</Text>
                        <View style={[styles.inputWrapper, { backgroundColor: colors.card, borderColor: colors.border }]}>
                            <Feather name="lock" size={20} color={colors.subtext} style={styles.inputIcon} />
                            <TextInput
                                style={[styles.input, { color: colors.text }]}
                                placeholder="Enter current password"
                                placeholderTextColor="#A0AEC0"
                                secureTextEntry={!showPasswords}
                                value={oldPassword}
                                onChangeText={setOldPassword}
                            />
                        </View>
                    </View>

                    <View style={styles.inputGroup}>
                        <Text style={[styles.label, { color: colors.subtext }]}>NEW PASSWORD</Text>
                        <View style={[styles.inputWrapper, { backgroundColor: colors.card, borderColor: colors.border }]}>
                            <Feather name="shield" size={20} color={colors.subtext} style={styles.inputIcon} />
                            <TextInput
                                style={[styles.input, { color: colors.text }]}
                                placeholder="Min. 6 characters"
                                placeholderTextColor="#A0AEC0"
                                secureTextEntry={!showPasswords}
                                value={newPassword}
                                onChangeText={setNewPassword}
                            />
                        </View>
                    </View>

                    <View style={styles.inputGroup}>
                        <Text style={[styles.label, { color: colors.subtext }]}>CONFIRM NEW PASSWORD</Text>
                        <View style={[styles.inputWrapper, { backgroundColor: colors.card, borderColor: colors.border }]}>
                            <Feather name="shield" size={20} color={colors.subtext} style={styles.inputIcon} />
                            <TextInput
                                style={[styles.input, { color: colors.text }]}
                                placeholder="Re-type new password"
                                placeholderTextColor="#A0AEC0"
                                secureTextEntry={!showPasswords}
                                value={confirmPassword}
                                onChangeText={setConfirmPassword}
                            />
                        </View>
                    </View>

                    <TouchableOpacity 
                        style={styles.toggleShow} 
                        onPress={() => setShowPasswords(!showPasswords)}
                    >
                        <Feather name={showPasswords ? "eye-off" : "eye"} size={16} color="#3A8EF6" />
                        <Text style={styles.toggleShowText}>{showPasswords ? "Hide Passwords" : "Show Passwords"}</Text>
                    </TouchableOpacity>

                    <TouchableOpacity style={styles.submitBtn} onPress={handleUpdate}>
                        <Text style={styles.submitBtnText}>Update Password</Text>
                    </TouchableOpacity>
                </View>

                <View style={styles.tips}>
                    <Text style={[styles.tipTitle, { color: colors.text }]}>Password Tips:</Text>
                    <Text style={[styles.tipText, { color: colors.subtext }]}>• Use at least 8 characters</Text>
                    <Text style={[styles.tipText, { color: colors.subtext }]}>• Include symbols and numbers</Text>
                    <Text style={[styles.tipText, { color: colors.subtext }]}>• Avoid using your birth date</Text>
                </View>
            </ScrollView>
        </KeyboardAvoidingView>
    );
};

const styles = StyleSheet.create({
    container: { flex: 1 },
    scrollContent: { padding: 24, paddingTop: 40 },
    header: { marginBottom: 32 },
    title: { fontSize: 28, fontWeight: '900', marginBottom: 8 },
    subtitle: { fontSize: 14, lineHeight: 20 },
    form: { marginBottom: 30 },
    inputGroup: { marginBottom: 20 },
    label: { fontSize: 11, fontWeight: '800', letterSpacing: 1.2, marginBottom: 10, marginLeft: 4 },
    inputWrapper: { flexDirection: 'row', alignItems: 'center', borderRadius: 16, borderWidth: 1, paddingHorizontal: 16, height: 56 },
    inputIcon: { marginRight: 12 },
    input: { flex: 1, fontSize: 16, fontWeight: '600' },
    toggleShow: { flexDirection: 'row', alignItems: 'center', marginBottom: 30, marginLeft: 4 },
    toggleShowText: { fontSize: 14, fontWeight: '700', color: '#3A8EF6', marginLeft: 8 },
    submitBtn: { backgroundColor: '#3A8EF6', height: 56, borderRadius: 16, alignItems: 'center', justifyContent: 'center', shadowColor: '#3A8EF6', shadowOffset: { width: 0, height: 8 }, shadowOpacity: 0.2, shadowRadius: 15, elevation: 5 },
    submitBtnText: { color: 'white', fontSize: 16, fontWeight: '800' },
    tips: { backgroundColor: 'rgba(58, 142, 246, 0.05)', padding: 20, borderRadius: 20 },
    tipTitle: { fontSize: 15, fontWeight: '800', marginBottom: 10 },
    tipText: { fontSize: 14, marginBottom: 5, fontWeight: '500' }
});

export default ChangePasswordScreen;
