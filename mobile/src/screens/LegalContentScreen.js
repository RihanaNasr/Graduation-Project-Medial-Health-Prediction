import React from 'react';
import { View, Text, StyleSheet, ScrollView, TouchableOpacity } from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather } from '@expo/vector-icons';
import { useTheme } from '../context/ThemeContext';

const LegalContentScreen = ({ route, navigation }) => {
    const { isDark, colors } = useTheme();
    const { title, content } = route.params;

    return (
        <View style={[styles.container, { backgroundColor: colors.background }]}>
            <StatusBar style={isDark ? "light" : "dark"} />
            <ScrollView contentContainerStyle={styles.content} showsVerticalScrollIndicator={false}>
                <View style={styles.header}>
                    <Text style={[styles.title, { color: colors.text }]}>{title}</Text>
                    <Text style={[styles.date, { color: colors.subtext }]}>Effective: August 20, 2026</Text>
                </View>

                <View style={[styles.card, { backgroundColor: colors.card, borderColor: colors.border }]}>
                    <Text style={[styles.legalText, { color: colors.text }]}>{content}</Text>
                </View>
                
                <TouchableOpacity style={styles.backBtn} onPress={() => navigation.goBack()}>
                    <Text style={styles.backBtnText}>I Understand</Text>
                </TouchableOpacity>
            </ScrollView>
        </View>
    );
};

const styles = StyleSheet.create({
    container: { flex: 1 },
    content: { padding: 24, paddingTop: 40, paddingBottom: 60 },
    header: { marginBottom: 24 },
    title: { fontSize: 28, fontWeight: '900', marginBottom: 8 },
    date: { fontSize: 13, fontWeight: '700', letterSpacing: 0.5 },
    card: { padding: 24, borderRadius: 24, borderWidth: 1, shadowColor: '#0F1E3C', shadowOffset: { width: 0, height: 6 }, shadowOpacity: 0.04, shadowRadius: 15, elevation: 2 },
    legalText: { fontSize: 16, lineHeight: 26, fontWeight: '500', textAlign: 'justify' },
    backBtn: { backgroundColor: '#3A8EF6', height: 56, borderRadius: 16, alignItems: 'center', justifyContent: 'center', marginTop: 30 },
    backBtnText: { color: 'white', fontSize: 16, fontWeight: '800' }
});

export default LegalContentScreen;
