import React, { useState } from 'react';
import { View, Text, StyleSheet, ScrollView, TouchableOpacity, Switch, Alert } from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather } from '@expo/vector-icons';
import { useTheme } from '../context/ThemeContext';
import { useLanguage } from '../context/LanguageContext';

const LanguageScreen = () => {
    const { isDark, colors } = useTheme();
    const { language, toggleLanguage, t } = useLanguage();
    const [liveText, setLiveText] = useState(true);

    const handleSelectLanguage = (langCode, langName) => {
        toggleLanguage(langCode);
        // Simulation for graduation demo
        Alert.alert(
            t('language_updated'), 
            `${t('lang_set_to')}${langName}.`,
            [{ text: "OK" }]
        );
    };

    const handlePress = (item) => {
        Alert.alert(`${item} Settings`, `Options for ${item} will be available in the next app update.`);
    };

    const handleAddLanguage = () => {
        Alert.alert("Add Language", "Select a new language to add to your preferred list:\n\n- French\n- Spanish\n- German", [
            { text: "Cancel", style: "cancel" },
            { text: "OK" }
        ]);
    };

    return (
        <View style={[styles.container, { backgroundColor: colors.background }]}>
            <StatusBar style={isDark ? "light" : "dark"} />
            <ScrollView contentContainerStyle={styles.content} showsVerticalScrollIndicator={false}>

                <Text style={[styles.sectionLabel, { color: colors.subtext }]}>PREFERRED LANGUAGES</Text>
                <View style={[styles.cardBlock, { backgroundColor: colors.card, borderColor: colors.border, borderWidth: isDark ? 1 : 0 }]}>
                    <TouchableOpacity style={styles.listItem} onPress={() => handleSelectLanguage("en", "English")}>
                        <View style={styles.listContent}>
                            <Text style={[styles.listTitle, { color: colors.text }]}>English</Text>
                            <Text style={styles.listSub}>App Language</Text>
                        </View>
                        {language === 'en' ? (
                            <Feather name="check" size={20} color="#3A8EF6" />
                        ) : (
                            <Feather name="menu" size={20} color="#64748B" />
                        )}
                    </TouchableOpacity>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handleSelectLanguage("ar", "العربية")}>
                        <View style={styles.listContent}>
                            <Text style={[styles.listTitle, { color: colors.text }]}>العربية</Text>
                            <Text style={styles.listSub}>Arabic</Text>
                        </View>
                        {language === 'ar' ? (
                            <Feather name="check" size={20} color="#3A8EF6" />
                        ) : (
                            <Feather name="menu" size={20} color="#64748B" />
                        )}
                    </TouchableOpacity>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={handleAddLanguage}>
                        <Text style={styles.addLanguageText}>Add Language...</Text>
                    </TouchableOpacity>
                </View>
                <Text style={styles.footerText}>Apps and websites will use the first language in this list that they support.</Text>

                <View style={[styles.cardBlock, { backgroundColor: colors.card, borderColor: colors.border, borderWidth: isDark ? 1 : 0 }]}>
                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Region")}>
                        <Text style={[styles.listTitle, { color: colors.text }]}>Region</Text>
                        <View style={styles.rightContent}>
                            <Text style={[styles.valueText, { color: colors.subtext }]}>Egypt</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Calendar")}>
                        <Text style={[styles.listTitle, { color: colors.text }]}>Calendar</Text>
                        <View style={styles.rightContent}>
                            <Text style={[styles.valueText, { color: colors.subtext }]}>Gregorian</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Temperature")}>
                        <Text style={[styles.listTitle, { color: colors.text }]}>Temperature</Text>
                        <View style={styles.rightContent}>
                            <Text style={[styles.valueText, { color: colors.subtext }]}>°C</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Measurement System")}>
                        <Text style={[styles.listTitle, { color: colors.text }]}>Measurement System</Text>
                        <View style={styles.rightContent}>
                            <Text style={[styles.valueText, { color: colors.subtext }]}>Metric</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("First Day of Week")}>
                        <Text style={[styles.listTitle, { color: colors.text }]}>First Day of Week</Text>
                        <View style={styles.rightContent}>
                            <Text style={[styles.valueText, { color: colors.subtext }]}>Saturday</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Date Format")}>
                        <Text style={[styles.listTitle, { color: colors.text }]}>Date Format</Text>
                        <View style={styles.rightContent}>
                            <Text style={[styles.valueText, { color: colors.subtext }]}>19/08/2026</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={[styles.divider, { backgroundColor: colors.border }]} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Number Format")}>
                        <Text style={[styles.listTitle, { color: colors.text }]}>Number Format</Text>
                        <View style={styles.rightContent}>
                            <Text style={[styles.valueText, { color: colors.subtext }]}>1,234,567.89</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                </View>

                <View style={[styles.cardBlock, { marginTop: 16, backgroundColor: colors.card, borderColor: colors.border, borderWidth: isDark ? 1 : 0 }]}>
                    <View style={styles.listItem}>
                        <Text style={[styles.listTitle, { flex: 1, color: colors.text }]}>Live Text</Text>
                        <Switch
                            trackColor={{ false: '#E4ECFD', true: '#22C55E' }}
                            thumbColor="white"
                            onValueChange={setLiveText}
                            value={liveText}
                            style={{ transform: [{ scaleX: 0.9 }, { scaleY: 0.9 }], marginRight: -6 }}
                        />
                    </View>
                </View>
                <Text style={styles.footerText}>Select text in images to copy or take action.</Text>

                <View style={styles.exampleBlock}>
                    <Text style={[styles.exampleTitle, { color: colors.text }]}>Region Format Example</Text>
                    <Text style={[styles.exampleText, { color: colors.subtext }]}>12:34 AM</Text>
                    <Text style={[styles.exampleText, { color: colors.subtext }]}>Wednesday, 19 August 2026</Text>
                </View>

            </ScrollView>
        </View>
    );
};

const styles = StyleSheet.create({
    container: { flex: 1, backgroundColor: '#F4F8FF' },
    content: { padding: 20, paddingBottom: 60 },
    sectionLabel: {
        fontSize: 11,
        fontWeight: '800',
        color: '#A0AEC0',
        letterSpacing: 1.2,
        marginBottom: 8,
        marginLeft: 8,
        marginTop: 10,
    },
    cardBlock: {
        backgroundColor: '#FFF',
        borderRadius: 20,
        paddingHorizontal: 16,
        marginBottom: 8,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.03,
        shadowRadius: 12,
        elevation: 2,
    },
    listItem: {
        flexDirection: 'row',
        alignItems: 'center',
        paddingVertical: 14,
        minHeight: 48,
    },
    listContent: {
        flex: 1,
    },
    listTitle: {
        fontSize: 16,
        fontWeight: '600',
        color: '#0F1E3C',
    },
    listSub: {
        fontSize: 12,
        color: '#A0AEC0',
        marginTop: 2,
    },
    addLanguageText: {
        fontSize: 16,
        fontWeight: '600',
        color: '#3A8EF6',
    },
    rightContent: {
        flexDirection: 'row',
        alignItems: 'center',
    },
    valueText: {
        fontSize: 16,
        color: '#5A6A8A',
        marginRight: 4,
    },
    divider: {
        height: 1,
        backgroundColor: '#F4F8FF',
    },
    footerText: {
        fontSize: 12,
        color: '#A0AEC0',
        marginLeft: 12,
        marginRight: 12,
        marginBottom: 24,
        lineHeight: 16,
    },
    exampleBlock: {
        alignItems: 'center',
        marginTop: 20,
        marginBottom: 40,
    },
    exampleTitle: {
        fontSize: 14,
        fontWeight: '600',
        color: '#0F1E3C',
        marginBottom: 12,
    },
    exampleText: {
        fontSize: 14,
        color: '#5A6A8A',
        marginBottom: 4,
    }
});

export default LanguageScreen;
