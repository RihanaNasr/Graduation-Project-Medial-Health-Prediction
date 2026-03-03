import React, { useState } from 'react';
import { View, Text, StyleSheet, ScrollView, TouchableOpacity, Switch, Alert } from 'react-native';
import { StatusBar } from 'expo-status-bar';
import { Feather } from '@expo/vector-icons';

const LanguageScreen = () => {
    const [liveText, setLiveText] = useState(true);

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
        <View style={styles.container}>
            <StatusBar style="dark" />
            <ScrollView contentContainerStyle={styles.content} showsVerticalScrollIndicator={false}>

                <Text style={styles.sectionLabel}>PREFERRED LANGUAGES</Text>
                <View style={styles.cardBlock}>
                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("English")}>
                        <View style={styles.listContent}>
                            <Text style={styles.listTitle}>English</Text>
                            <Text style={styles.listSub}>App Language</Text>
                        </View>
                        <Feather name="menu" size={20} color="#A0AEC0" />
                    </TouchableOpacity>
                    <View style={styles.divider} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Arabic")}>
                        <View style={styles.listContent}>
                            <Text style={styles.listTitle}>العربية</Text>
                            <Text style={styles.listSub}>Arabic</Text>
                        </View>
                        <Feather name="menu" size={20} color="#A0AEC0" />
                    </TouchableOpacity>
                    <View style={styles.divider} />

                    <TouchableOpacity style={styles.listItem} onPress={handleAddLanguage}>
                        <Text style={styles.addLanguageText}>Add Language...</Text>
                    </TouchableOpacity>
                </View>
                <Text style={styles.footerText}>Apps and websites will use the first language in this list that they support.</Text>

                <View style={styles.cardBlock}>
                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Region")}>
                        <Text style={styles.listTitle}>Region</Text>
                        <View style={styles.rightContent}>
                            <Text style={styles.valueText}>Egypt</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={styles.divider} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Calendar")}>
                        <Text style={styles.listTitle}>Calendar</Text>
                        <View style={styles.rightContent}>
                            <Text style={styles.valueText}>Gregorian</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={styles.divider} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Temperature")}>
                        <Text style={styles.listTitle}>Temperature</Text>
                        <View style={styles.rightContent}>
                            <Text style={styles.valueText}>°C</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={styles.divider} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Measurement System")}>
                        <Text style={styles.listTitle}>Measurement System</Text>
                        <View style={styles.rightContent}>
                            <Text style={styles.valueText}>Metric</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={styles.divider} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("First Day of Week")}>
                        <Text style={styles.listTitle}>First Day of Week</Text>
                        <View style={styles.rightContent}>
                            <Text style={styles.valueText}>Saturday</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={styles.divider} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Date Format")}>
                        <Text style={styles.listTitle}>Date Format</Text>
                        <View style={styles.rightContent}>
                            <Text style={styles.valueText}>19/08/2026</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                    <View style={styles.divider} />

                    <TouchableOpacity style={styles.listItem} onPress={() => handlePress("Number Format")}>
                        <Text style={styles.listTitle}>Number Format</Text>
                        <View style={styles.rightContent}>
                            <Text style={styles.valueText}>1,234,567.89</Text>
                            <Feather name="chevron-right" size={18} color="#A0AEC0" />
                        </View>
                    </TouchableOpacity>
                </View>

                <View style={[styles.cardBlock, { marginTop: 16 }]}>
                    <View style={styles.listItem}>
                        <Text style={[styles.listTitle, { flex: 1 }]}>Live Text</Text>
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
                    <Text style={styles.exampleTitle}>Region Format Example</Text>
                    <Text style={styles.exampleText}>12:34 AM</Text>
                    <Text style={styles.exampleText}>Wednesday, 19 August 2026</Text>
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
