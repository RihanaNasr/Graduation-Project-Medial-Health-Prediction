import React from 'react';
import { View, Text, StyleSheet, ScrollView } from 'react-native';
import { StatusBar } from 'expo-status-bar';

const HistoryScreen = () => {
    return (
        <View style={styles.container}>
            <StatusBar style="dark" />
            <ScrollView contentContainerStyle={styles.content}>
                <View style={styles.emptyState}>
                    <Text style={styles.emoji}>📊</Text>
                    <Text style={styles.title}>All Data History</Text>
                    <Text style={styles.subtext}>Your comprehensive past health records and charts will appear here in future updates.</Text>
                </View>
            </ScrollView>
        </View>
    );
};

const styles = StyleSheet.create({
    container: { flex: 1, backgroundColor: '#F4F8FF' },
    content: { flexGrow: 1, justifyContent: 'center', alignItems: 'center', padding: 24 },
    emptyState: { alignItems: 'center' },
    emoji: { fontSize: 60, marginBottom: 16 },
    title: { fontSize: 22, fontWeight: '800', color: '#0F1E3C', marginBottom: 8 },
    subtext: { fontSize: 14, color: '#5A6A8A', textAlign: 'center' },
});

export default HistoryScreen;
