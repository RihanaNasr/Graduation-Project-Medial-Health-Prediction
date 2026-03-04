import React from 'react';
import { View, Text, StyleSheet, ScrollView, TouchableOpacity } from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather } from '@expo/vector-icons';

const NotificationsScreen = () => {
    const notifications = [
        {
            id: '1',
            type: 'alert',
            title: 'Elevated Heart Rate Detected',
            message: 'Your heart rate reached 120 bpm, which is above your normal resting rate.',
            time: '2 hours ago',
            read: false,
            color: '#FF4D6D',
            bg: '#FFF0F3',
            icon: 'activity',
        },
        {
            id: '2',
            type: 'reminder',
            title: 'Hydration Reminder',
            message: 'It is time to drink a glass of water to reach your daily goal.',
            time: '4 hours ago',
            read: true,
            color: '#3A8EF6',
            bg: '#E8F1FE',
            icon: 'droplet',
        },
        {
            id: '3',
            type: 'appointment',
            title: 'Upcoming Appointment',
            message: 'Reminder: You have an appointment with Dr. Sarah tomorrow at 10:30 AM.',
            time: 'Yesterday',
            read: true,
            color: '#F59E0B',
            bg: '#FFFBEB',
            icon: 'calendar',
        },
        {
            id: '4',
            type: 'report',
            title: 'Weekly Report Ready',
            message: 'Your health trends for the past week show great improvement. Keep it up!',
            time: '2 days ago',
            read: true,
            color: '#22C55E',
            bg: '#EDFBF3',
            icon: 'trending-up',
        },
    ];

    return (
        <View style={styles.container}>
            <StatusBar style="dark" />
            <ScrollView contentContainerStyle={styles.content} showsVerticalScrollIndicator={false}>
                {notifications.map((notif, index) => (
                    <TouchableOpacity
                        key={notif.id}
                        style={[styles.notifCard, !notif.read && styles.unreadCard]}
                    >
                        <View style={[styles.iconWrap, { backgroundColor: notif.bg }]}>
                            <Feather name={notif.icon} size={20} color={notif.color} />
                        </View>
                        <View style={styles.textWrap}>
                            <Text style={[styles.title, !notif.read && styles.unreadText]}>
                                {notif.title}
                            </Text>
                            <Text style={styles.message} numberOfLines={2}>
                                {notif.message}
                            </Text>
                            <Text style={styles.time}>{notif.time}</Text>
                        </View>
                        {!notif.read && <View style={styles.unreadDot} />}
                    </TouchableOpacity>
                ))}
            </ScrollView>
        </View>
    );
};

const styles = StyleSheet.create({
    container: { flex: 1, backgroundColor: '#F4F8FF' },
    content: { padding: 24, paddingTop: 20 },
    notifCard: {
        flexDirection: 'row',
        backgroundColor: '#FFF',
        borderRadius: 20,
        padding: 16,
        marginBottom: 16,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.03,
        shadowRadius: 12,
        elevation: 2,
    },
    unreadCard: {
        backgroundColor: '#F8FBFF',
        borderWidth: 1,
        borderColor: '#E8F1FE',
    },
    iconWrap: {
        width: 48,
        height: 48,
        borderRadius: 14,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 16,
    },
    textWrap: {
        flex: 1,
    },
    title: {
        fontSize: 15,
        fontWeight: '700',
        color: '#0F1E3C',
        marginBottom: 4,
    },
    unreadText: {
        fontWeight: '900',
        color: '#3A8EF6',
    },
    message: {
        fontSize: 13,
        color: '#5A6A8A',
        lineHeight: 18,
        marginBottom: 8,
    },
    time: {
        fontSize: 11,
        fontWeight: '600',
        color: '#A0AEC0',
    },
    unreadDot: {
        width: 10,
        height: 10,
        borderRadius: 5,
        backgroundColor: '#3A8EF6',
        marginTop: 6,
        marginLeft: 8,
    }
});

export default NotificationsScreen;
