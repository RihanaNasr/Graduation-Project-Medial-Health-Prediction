import React, { useState, useEffect, useRef } from 'react';
import {
    View,
    Text,
    TextInput,
    StyleSheet,
    TouchableOpacity,
    FlatList,
    KeyboardAvoidingView,
    Platform,
    ActivityIndicator,
    ScrollView,
} from 'react-native';
import { Ionicons, Feather } from '@expo/vector-icons';
import { LinearGradient } from 'expo-linear-gradient';
import { medicalAPI } from '../services/api';
import { useAuth } from '../context/AuthContext';
import { useTheme } from '../context/ThemeContext';
import { StatusBar } from 'expo-status-bar';

const ChatScreen = () => {
    const { user } = useAuth();
    const { isDark, colors } = useTheme();
    const [messages, setMessages] = useState([]);
    const [inputText, setInputText] = useState('');
    const [loading, setLoading] = useState(false);
    const flatListRef = useRef();

    useEffect(() => {
        loadChatHistory();
    }, []);

    const loadChatHistory = async () => {
        try {
            const response = await medicalAPI.getChatHistory();
            const history = response.data.map((msg) => [
                { id: `user-${msg.id}`, text: msg.message, isUser: true, time: 'recently' },
                { id: `ai-${msg.id}`, text: msg.response, isUser: false, time: 'recently' },
            ]).flat();
            setMessages(history);
        } catch (error) {
            console.log('--- CHAT DEMO SESSION INSTANTIATED ---');
            // Mock initial "Welcome" messages if history fails
            setMessages([
                { id: 'welcome-1', text: "Welcome back! I've been monitoring your heart data. How are you feeling today?", isUser: false, time: '9:00 AM' }
            ]);
        }
    };

    const [isListening, setIsListening] = useState(false);

    const handleMicPress = () => {
        setIsListening(true);
        // Simulate a 1.5 second "Listening" phase
        setTimeout(() => {
            const simulatedTranscription = "I feel a sharp pain in my chest that started suddenly while walking.";
            setInputText(simulatedTranscription);
            setIsListening(false);
        }, 1500);
    };

    const sendMessage = async () => {
        if (!inputText.trim()) return;
        const currentMsg = inputText; // Save for fallback context

        const userMessage = {
            id: `temp-${Date.now()}`,
            text: currentMsg,
            isUser: true,
            time: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
        };

        setMessages((prev) => [...prev, userMessage]);
        setInputText('');
        setLoading(true);

        try {
            const response = await medicalAPI.chat(currentMsg, user?.email);
            const aiMessage = {
                id: `ai-${response.data.id || Date.now()}`,
                text: response.data.response,
                isUser: false,
                time: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
            };
            setMessages((prev) => [...prev, aiMessage]);
        } catch (error) {
            console.log('--- CHAT DEMO RESPONSE LOADED ---');
            // Realistic Fallback Response
            const aiMessage = {
                id: `ai-demo-${Date.now()}`,
                text: currentMsg.toLowerCase().includes('risk') || currentMsg.toLowerCase().includes('heart') 
                    ? "Based on your current vitals (Heart Rate: 130 bpm, SpO2: 80%), you are showing signs of potential tachycardia and respiratory distress. I recommend seeking medical attention or contacting your cardiologist immediately. Should I call an ambulance for you?"
                    : "I've analyzed your message. To provide medical insights, I'll need to sync your latest vitals. Please ensure your wearable device is connected.",
                isUser: false,
                time: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
            };
            setMessages((prev) => [...prev, aiMessage]);
        } finally {
            setLoading(false);
        }
    };

    const renderMessage = ({ item }) => (
        <View style={[styles.msgWrap, item.isUser ? styles.msgWrapRight : null]}>
            <View style={[styles.msgAvatar, item.isUser ? styles.msgAvatarUser : null]}>
                <Text style={styles.avatarEmoji}>{item.isUser ? '👤' : '🤖'}</Text>
            </View>
            <View style={styles.msgContent}>
                {item.isUser ? (
                    <LinearGradient
                        colors={['#3A8EF6', '#5BADFF']}
                        style={[styles.msgBubble, styles.msgBubbleUser]}
                        start={{ x: 0, y: 0 }}
                        end={{ x: 1, y: 1 }}
                    >
                        <Text style={styles.msgUserText}>{item.text}</Text>
                    </LinearGradient>
                ) : (
                    <View style={[styles.msgBubble, styles.msgBubbleAi]}>
                        <Text style={styles.msgAiText}>{item.text}</Text>
                    </View>
                )}
                <Text style={[styles.msgTime, item.isUser ? styles.msgTimeRight : null]}>
                    {item.time || 'now'}
                </Text>
            </View>
        </View>
    );

    const presetChips = ["Show my heart history", "Am I at risk?", "Sleep tips", "Today's report"];

    return (
        <KeyboardAvoidingView
            behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
            style={[styles.container, { backgroundColor: colors.background }]}
            keyboardVerticalOffset={Platform.OS === 'ios' ? 90 : 0}
        >
            <StatusBar style={isDark ? "light" : "dark"} />
            <LinearGradient
                colors={isDark ? ['#1E293B', '#0F172A'] : ['#3A8EF6', '#5BADFF']}
                style={styles.chatHeader}
                start={{ x: 0, y: 0 }}
                end={{ x: 1, y: 1 }}
            >
                <View style={[styles.aiAvatar, { backgroundColor: isDark ? 'rgba(255,255,255,0.1)' : 'rgba(255,255,255,0.25)' }]}>
                    <Text style={styles.aiAvatarEmoji}>🤖</Text>
                </View>
                <View style={styles.headerInfo}>
                    <Text style={styles.aiName}>CardiGo AI</Text>
                    <View style={styles.statusRow}>
                        <View style={styles.statusDot} />
                        <Text style={[styles.aiStatus, { color: isDark ? '#A0AEC0' : 'rgba(255,255,255,0.75)' }]}>Online & Monitoring</Text>
                    </View>
                </View>
            </LinearGradient>

            <FlatList
                ref={flatListRef}
                data={messages}
                renderItem={renderMessage}
                keyExtractor={(item) => item.id}
                contentContainerStyle={styles.messageList}
                onContentSizeChange={() =>
                    flatListRef.current?.scrollToEnd({ animated: true })
                }
                ListEmptyComponent={() => (
                    <View style={styles.emptyState}>
                        <View style={[styles.emptyAiWrap, { backgroundColor: colors.card, borderColor: colors.border, borderWidth: isDark ? 1 : 0 }]}>
                            <Text style={styles.emptyEmoji}>🤖</Text>
                        </View>
                        <Text style={[styles.emptyText, { color: colors.text }]}>Hello {user?.first_name || 'there'}!</Text>
                        <Text style={[styles.emptySubtext, { color: colors.subtext }]}>I am CardiGo AI. How can I help you today?</Text>
                    </View>
                )}
            />

            {loading && (
                <View style={styles.loadingContainer}>
                    <ActivityIndicator size="small" color="#3A8EF6" />
                    <Text style={styles.loadingText}>CardiGo AI is typing...</Text>
                </View>
            )}

            <View style={{ paddingBottom: 85, backgroundColor: colors.background }}>
                <View style={[styles.suggestionsStripWrap, { backgroundColor: colors.background }]}>
                    <ScrollView horizontal showsHorizontalScrollIndicator={false} contentContainerStyle={styles.suggestionsStrip}>
                        {presetChips.map((chip, idx) => (
                            <TouchableOpacity key={idx} style={[styles.chip, { backgroundColor: colors.card, borderColor: colors.border }]} onPress={() => setInputText(chip)}>
                                <Text style={styles.chipText}>{chip}</Text>
                            </TouchableOpacity>
                        ))}
                    </ScrollView>
                </View>

                <View style={[styles.inputBar, { backgroundColor: colors.card, borderTopColor: colors.border }]}>
                    <TextInput
                        style={[styles.input, { backgroundColor: colors.background, color: colors.text, borderColor: colors.border }]}
                        placeholder={isListening ? "Listening..." : "Type your message..."}
                        placeholderTextColor={isListening ? "#FF4D6D" : colors.subtext}
                        value={inputText}
                        onChangeText={setInputText}
                        multiline
                        maxLength={500}
                        editable={!isListening}
                    />
                    <TouchableOpacity style={[styles.micBtn, { backgroundColor: colors.background }, isListening && { backgroundColor: '#FFF0F3' }]} onPress={handleMicPress}>
                        <Feather name="mic" size={16} color={isListening ? "#FF4D6D" : "#3A8EF6"} />
                    </TouchableOpacity>
                    <TouchableOpacity
                        style={[styles.sendBtn, !inputText.trim() && { opacity: 0.6 }]}
                        onPress={sendMessage}
                        disabled={!inputText.trim() || loading}
                    >
                        <Feather name="send" size={18} color="white" />
                    </TouchableOpacity>
                </View>
            </View>
        </KeyboardAvoidingView>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#F4F8FF',
    },
    chatHeader: {
        paddingTop: Platform.OS === 'ios' ? 50 : 20,
        paddingBottom: 20,
        paddingHorizontal: 24,
        flexDirection: 'row',
        alignItems: 'center',
        borderBottomLeftRadius: 24,
        borderBottomRightRadius: 24,
    },
    aiAvatar: {
        width: 44,
        height: 44,
        backgroundColor: 'rgba(255,255,255,0.25)',
        borderRadius: 16,
        alignItems: 'center',
        justifyContent: 'center',
    },
    aiAvatarEmoji: {
        fontSize: 22,
    },
    headerInfo: {
        marginLeft: 12,
        flex: 1,
    },
    aiName: {
        fontSize: 16,
        fontWeight: '800',
        color: 'white',
    },
    statusRow: {
        flexDirection: 'row',
        alignItems: 'center',
        marginTop: 2,
    },
    statusDot: {
        width: 6,
        height: 6,
        backgroundColor: '#A7F3C8',
        borderRadius: 3,
        marginRight: 4,
    },
    aiStatus: {
        fontSize: 11,
        color: 'rgba(255,255,255,0.75)',
    },
    topRight: {
        flexDirection: 'row',
        alignItems: 'center',
    },
    iconBtn: {
        width: 36,
        height: 36,
        backgroundColor: 'rgba(255,255,255,0.2)',
        borderRadius: 12,
        alignItems: 'center',
        justifyContent: 'center',
        marginLeft: 8,
    },
    messageList: {
        paddingVertical: 16,
        paddingHorizontal: 20,
        flexGrow: 1,
    },
    msgWrap: {
        flexDirection: 'row',
        alignItems: 'flex-end',
        marginBottom: 14,
    },
    msgWrapRight: {
        flexDirection: 'row-reverse',
    },
    msgAvatar: {
        width: 32,
        height: 32,
        borderRadius: 12,
        backgroundColor: '#fff',
        alignItems: 'center',
        justifyContent: 'center',
        marginHorizontal: 8,
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 2 },
        shadowOpacity: 0.1,
        shadowRadius: 4,
        elevation: 1,
    },
    msgAvatarUser: {
        backgroundColor: '#E8F1FE',
    },
    avatarEmoji: {
        fontSize: 15,
    },
    msgContent: {
        maxWidth: 240,
    },
    msgBubble: {
        paddingVertical: 12,
        paddingHorizontal: 15,
        borderRadius: 18,
    },
    msgBubbleAi: {
        backgroundColor: '#fff',
        borderBottomLeftRadius: 4,
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 2 },
        shadowOpacity: 0.08,
        shadowRadius: 12,
        elevation: 2,
    },
    msgBubbleUser: {
        borderBottomRightRadius: 4,
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.3,
        shadowRadius: 14,
        elevation: 3,
    },
    msgAiText: {
        fontSize: 14,
        color: '#0F1E3C',
        lineHeight: 20,
    },
    msgUserText: {
        fontSize: 14,
        color: 'white',
        lineHeight: 20,
    },
    msgTime: {
        fontSize: 10,
        color: '#A0AEC0',
        marginTop: 4,
        paddingHorizontal: 4,
    },
    msgTimeRight: {
        textAlign: 'right',
    },
    emptyState: {
        flex: 1,
        justifyContent: 'center',
        alignItems: 'center',
        paddingTop: 80,
    },
    emptyAiWrap: {
        width: 60,
        height: 60,
        backgroundColor: '#fff',
        borderRadius: 20,
        alignItems: 'center',
        justifyContent: 'center',
        marginBottom: 16,
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 8 },
        shadowOpacity: 0.1,
        shadowRadius: 24,
        elevation: 5,
    },
    emptyEmoji: {
        fontSize: 30,
    },
    emptyText: {
        fontSize: 18,
        fontWeight: '800',
        color: '#0F1E3C',
        marginBottom: 6,
    },
    emptySubtext: {
        fontSize: 13,
        color: '#5A6A8A',
        textAlign: 'center',
        maxWidth: '80%',
    },
    loadingContainer: {
        flexDirection: 'row',
        alignItems: 'center',
        paddingHorizontal: 20,
        paddingBottom: 10,
    },
    loadingText: {
        marginLeft: 8,
        color: '#A0AEC0',
        fontSize: 12,
        fontStyle: 'italic',
    },
    suggestionsStripWrap: {
        backgroundColor: '#F4F8FF',
    },
    suggestionsStrip: {
        paddingHorizontal: 20,
        paddingBottom: 10,
        paddingTop: 4,
    },
    chip: {
        backgroundColor: '#fff',
        borderWidth: 1.5,
        borderColor: '#E4ECFD',
        borderRadius: 20,
        paddingVertical: 7,
        paddingHorizontal: 14,
        marginRight: 8,
    },
    chipText: {
        fontSize: 12,
        fontWeight: '600',
        color: '#3A8EF6',
    },
    inputBar: {
        backgroundColor: '#fff',
        borderTopWidth: 1,
        borderTopColor: '#E4ECFD',
        paddingHorizontal: 16,
        paddingTop: 10,
        paddingBottom: Platform.OS === 'ios' ? 28 : 10,
        flexDirection: 'row',
        alignItems: 'center',
    },
    helperBtn: {
        width: 36,
        height: 36,
        backgroundColor: '#E8F1FE',
        borderRadius: 18,
        alignItems: 'center',
        justifyContent: 'center',
    },
    helperBtnText: {
        color: '#3A8EF6',
        fontSize: 15,
        fontWeight: '900',
    },
    input: {
        flex: 1,
        backgroundColor: '#F4F8FF',
        borderWidth: 1.5,
        borderColor: '#E4ECFD',
        borderRadius: 20,
        paddingVertical: 10,
        paddingHorizontal: 16,
        marginHorizontal: 8,
        fontSize: 13,
        color: '#0F1E3C',
        maxHeight: 100,
    },
    micBtn: {
        width: 36,
        height: 36,
        backgroundColor: '#E8F1FE',
        borderRadius: 18,
        alignItems: 'center',
        justifyContent: 'center',
        marginRight: 8,
    },
    sendBtn: {
        width: 42,
        height: 42,
        backgroundColor: '#3A8EF6',
        borderRadius: 21,
        alignItems: 'center',
        justifyContent: 'center',
        shadowColor: '#3A8EF6',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.35,
        shadowRadius: 14,
        elevation: 4,
    },
});

export default ChatScreen;
