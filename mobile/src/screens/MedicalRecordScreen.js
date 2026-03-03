import React, { useState, useEffect } from 'react';
import {
    View,
    Text,
    TextInput,
    StyleSheet,
    TouchableOpacity,
    ScrollView,
    Alert,
    ActivityIndicator,
    Platform,
} from 'react-native';
import { Picker } from '@react-native-picker/picker';
import { Ionicons, Feather } from '@expo/vector-icons';
import { LinearGradient } from 'expo-linear-gradient';
import { medicalAPI } from '../services/api';
import { StatusBar } from 'expo-status-bar';

const MedicalRecordScreen = () => {
    const [record, setRecord] = useState(null);
    const [editing, setEditing] = useState(false);
    const [loading, setLoading] = useState(true);
    const [saving, setSaving] = useState(false);

    useEffect(() => {
        loadMedicalRecord();
    }, []);

    const loadMedicalRecord = async () => {
        try {
            const response = await medicalAPI.getRecord();
            setRecord(response.data);
        } catch (error) {
            console.error('Error loading medical record:', error);
            // Initialize empty record if not exists
            setRecord({
                blood_type: '',
                height: '',
                weight: '',
                chronic_conditions: '',
                allergies: '',
                current_medications: '',
                past_surgeries: '',
                emergency_contact_name: '',
                emergency_contact_phone: '',
                emergency_contact_relation: '',
            });
        } finally {
            setLoading(false);
        }
    };

    const handleSave = async () => {
        setSaving(true);
        try {
            await medicalAPI.updateRecord(record);
            setEditing(false);
            Alert.alert('Success', 'Medical record updated successfully');
        } catch (error) {
            Alert.alert('Error', 'Failed to update medical record');
        } finally {
            setSaving(false);
        }
    };

    const updateField = (field, value) => {
        setRecord({ ...record, [field]: value });
    };

    if (loading) {
        return (
            <View style={styles.loadingContainer}>
                <ActivityIndicator size="large" color="#3A8EF6" />
            </View>
        );
    }

    return (
        <View style={styles.container}>
            <StatusBar style="dark" />

            <View style={styles.header}>
                <Text style={styles.title}>Medical Profile</Text>
                {!editing ? (
                    <TouchableOpacity style={styles.headerBtn} onPress={() => setEditing(true)}>
                        <Feather name="edit-2" size={18} color="#3A8EF6" />
                    </TouchableOpacity>
                ) : null}
            </View>

            <ScrollView contentContainerStyle={styles.scrollContent} showsVerticalScrollIndicator={false}>

                {editing && (
                    <View style={styles.editBanner}>
                        <Feather name="info" size={16} color="#F59E0B" />
                        <Text style={styles.editBannerTxt}>You are currently editing your medical records.</Text>
                    </View>
                )}

                <Text style={styles.sectionLabelLabel}>Basic Information</Text>
                <View style={styles.card}>
                    <View style={styles.field}>
                        <Text style={styles.label}>Blood Type</Text>
                        {editing ? (
                            <View style={styles.pickerContainer}>
                                <Picker
                                    selectedValue={record.blood_type}
                                    onValueChange={(value) => updateField('blood_type', value)}
                                    style={{ height: 50 }}
                                >
                                    <Picker.Item label="Select Blood Type" value="" />
                                    <Picker.Item label="A+" value="A+" />
                                    <Picker.Item label="A-" value="A-" />
                                    <Picker.Item label="B+" value="B+" />
                                    <Picker.Item label="B-" value="B-" />
                                    <Picker.Item label="AB+" value="AB+" />
                                    <Picker.Item label="AB-" value="AB-" />
                                    <Picker.Item label="O+" value="O+" />
                                    <Picker.Item label="O-" value="O-" />
                                </Picker>
                            </View>
                        ) : (
                            <Text style={styles.value}>{record.blood_type || 'Not set'}</Text>
                        )}
                    </View>

                    <View style={styles.divider} />

                    <View style={styles.row}>
                        <View style={styles.halfField}>
                            <Text style={styles.label}>Height (cm)</Text>
                            {editing ? (
                                <TextInput
                                    style={styles.input}
                                    value={String(record.height || '')}
                                    onChangeText={(value) => updateField('height', value)}
                                    keyboardType="numeric"
                                    placeholder="170"
                                    placeholderTextColor="#A0AEC0"
                                />
                            ) : (
                                <Text style={styles.value}>{record.height ? `${record.height} cm` : 'Not set'}</Text>
                            )}
                        </View>
                        <View style={styles.dividerVertical} />
                        <View style={styles.halfField}>
                            <Text style={styles.label}>Weight (kg)</Text>
                            {editing ? (
                                <TextInput
                                    style={styles.input}
                                    value={String(record.weight || '')}
                                    onChangeText={(value) => updateField('weight', value)}
                                    keyboardType="numeric"
                                    placeholder="70"
                                    placeholderTextColor="#A0AEC0"
                                />
                            ) : (
                                <Text style={styles.value}>{record.weight ? `${record.weight} kg` : 'Not set'}</Text>
                            )}
                        </View>
                    </View>
                </View>

                <Text style={styles.sectionLabelLabel}>Medical History</Text>
                <View style={styles.card}>
                    <View style={styles.field}>
                        <Text style={styles.label}>Chronic Conditions</Text>
                        {editing ? (
                            <TextInput
                                style={[styles.input, styles.textArea]}
                                value={record.chronic_conditions}
                                onChangeText={(value) => updateField('chronic_conditions', value)}
                                multiline
                                numberOfLines={3}
                                placeholder="List any chronic conditions..."
                                placeholderTextColor="#A0AEC0"
                            />
                        ) : (
                            <Text style={styles.value}>{record.chronic_conditions || 'None reported'}</Text>
                        )}
                    </View>

                    <View style={styles.divider} />

                    <View style={styles.field}>
                        <Text style={styles.label}>Allergies</Text>
                        {editing ? (
                            <TextInput
                                style={[styles.input, styles.textArea]}
                                value={record.allergies}
                                onChangeText={(value) => updateField('allergies', value)}
                                multiline
                                numberOfLines={3}
                                placeholder="List any known allergies..."
                                placeholderTextColor="#A0AEC0"
                            />
                        ) : (
                            <Text style={styles.value}>{record.allergies || 'None reported'}</Text>
                        )}
                    </View>

                    <View style={styles.divider} />

                    <View style={styles.field}>
                        <Text style={styles.label}>Current Medications</Text>
                        {editing ? (
                            <TextInput
                                style={[styles.input, styles.textArea]}
                                value={record.current_medications}
                                onChangeText={(value) => updateField('current_medications', value)}
                                multiline
                                numberOfLines={3}
                                placeholder="List current medications..."
                                placeholderTextColor="#A0AEC0"
                            />
                        ) : (
                            <Text style={styles.value}>{record.current_medications || 'None reported'}</Text>
                        )}
                    </View>

                    <View style={styles.divider} />

                    <View style={[styles.field, { marginBottom: 0 }]}>
                        <Text style={styles.label}>Past Surgeries</Text>
                        {editing ? (
                            <TextInput
                                style={[styles.input, styles.textArea]}
                                value={record.past_surgeries}
                                onChangeText={(value) => updateField('past_surgeries', value)}
                                multiline
                                numberOfLines={3}
                                placeholder="List past surgical procedures..."
                                placeholderTextColor="#A0AEC0"
                            />
                        ) : (
                            <Text style={styles.value}>{record.past_surgeries || 'None reported'}</Text>
                        )}
                    </View>
                </View>

                <Text style={styles.sectionLabelLabel}>Emergency Contact</Text>
                <View style={styles.card}>
                    <View style={styles.field}>
                        <Text style={styles.label}>Contact Name</Text>
                        {editing ? (
                            <TextInput
                                style={styles.input}
                                value={record.emergency_contact_name}
                                onChangeText={(value) => updateField('emergency_contact_name', value)}
                                placeholder="Full name"
                                placeholderTextColor="#A0AEC0"
                            />
                        ) : (
                            <Text style={styles.value}>{record.emergency_contact_name || 'Not set'}</Text>
                        )}
                    </View>

                    <View style={styles.divider} />

                    <View style={styles.field}>
                        <Text style={styles.label}>Contact Phone</Text>
                        {editing ? (
                            <TextInput
                                style={styles.input}
                                value={record.emergency_contact_phone}
                                onChangeText={(value) => updateField('emergency_contact_phone', value)}
                                keyboardType="phone-pad"
                                placeholder="+1 234 567 8900"
                                placeholderTextColor="#A0AEC0"
                            />
                        ) : (
                            <Text style={styles.value}>{record.emergency_contact_phone || 'Not set'}</Text>
                        )}
                    </View>

                    <View style={styles.divider} />

                    <View style={[styles.field, { marginBottom: 0 }]}>
                        <Text style={styles.label}>Relationship</Text>
                        {editing ? (
                            <TextInput
                                style={styles.input}
                                value={record.emergency_contact_relation}
                                onChangeText={(value) => updateField('emergency_contact_relation', value)}
                                placeholder="e.g., Spouse, Parent, Sibling"
                                placeholderTextColor="#A0AEC0"
                            />
                        ) : (
                            <Text style={styles.value}>{record.emergency_contact_relation || 'Not set'}</Text>
                        )}
                    </View>
                </View>

                {editing && (
                    <View style={styles.buttonContainer}>
                        <TouchableOpacity
                            style={[styles.button, styles.cancelButton]}
                            onPress={() => {
                                setEditing(false);
                                loadMedicalRecord();
                            }}
                        >
                            <Text style={styles.cancelButtonText}>Cancel</Text>
                        </TouchableOpacity>

                        <TouchableOpacity style={{ flex: 1, marginLeft: 6 }} onPress={handleSave} disabled={saving}>
                            <LinearGradient
                                colors={['#3A8EF6', '#5BADFF']}
                                style={[styles.button, styles.saveButton]}
                                start={{ x: 0, y: 0 }}
                                end={{ x: 1, y: 0 }}
                            >
                                {saving ? (
                                    <ActivityIndicator color="#fff" />
                                ) : (
                                    <Text style={styles.saveButtonText}>Save</Text>
                                )}
                            </LinearGradient>
                        </TouchableOpacity>
                    </View>
                )}

            </ScrollView>
        </View>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#F4F8FF',
    },
    loadingContainer: {
        flex: 1,
        justifyContent: 'center',
        alignItems: 'center',
        backgroundColor: '#F4F8FF',
    },
    header: {
        flexDirection: 'row',
        alignItems: 'center',
        justifyContent: 'space-between',
        paddingTop: Platform.OS === 'ios' ? 60 : 40,
        paddingBottom: 20,
        paddingHorizontal: 24,
        backgroundColor: '#fff',
        borderBottomWidth: 1,
        borderBottomColor: '#E4ECFD',
    },
    title: {
        fontSize: 24,
        fontWeight: '900',
        color: '#0F1E3C',
    },
    headerBtn: {
        width: 40,
        height: 40,
        backgroundColor: '#E8F1FE',
        borderRadius: 12,
        alignItems: 'center',
        justifyContent: 'center',
    },
    scrollContent: {
        paddingHorizontal: 24,
        paddingBottom: 80,
    },
    editBanner: {
        flexDirection: 'row',
        alignItems: 'center',
        backgroundColor: '#FFFBEB',
        padding: 12,
        borderRadius: 12,
        marginTop: 20,
        borderWidth: 1,
        borderColor: '#FEF3C7',
    },
    editBannerTxt: {
        fontSize: 12,
        color: '#B45309',
        marginLeft: 8,
        fontWeight: '600',
    },
    sectionLabelLabel: {
        fontSize: 11,
        fontWeight: '800',
        color: '#A0AEC0',
        textTransform: 'uppercase',
        letterSpacing: 1,
        marginBottom: 10,
        marginTop: 20,
    },
    card: {
        backgroundColor: '#fff',
        borderRadius: 20,
        padding: 20,
        shadowColor: '#0F1E3C',
        shadowOffset: { width: 0, height: 4 },
        shadowOpacity: 0.05,
        shadowRadius: 10,
        elevation: 2,
    },
    field: {
        marginBottom: 16,
    },
    row: {
        flexDirection: 'row',
        alignItems: 'center',
    },
    halfField: {
        flex: 1,
    },
    label: {
        fontSize: 11,
        fontWeight: '700',
        color: '#A0AEC0',
        marginBottom: 6,
    },
    value: {
        fontSize: 15,
        fontWeight: '600',
        color: '#0F1E3C',
    },
    input: {
        backgroundColor: '#F4F8FF',
        borderRadius: 10,
        padding: 14,
        fontSize: 14,
        fontWeight: '500',
        color: '#0F1E3C',
        borderWidth: 1,
        borderColor: '#E4ECFD',
    },
    textArea: {
        minHeight: 100,
        textAlignVertical: 'top',
    },
    pickerContainer: {
        backgroundColor: '#F4F8FF',
        borderRadius: 10,
        borderWidth: 1,
        borderColor: '#E4ECFD',
        overflow: 'hidden',
    },
    divider: {
        height: 1,
        backgroundColor: '#F4F8FF',
        marginVertical: 16,
    },
    dividerVertical: {
        width: 1,
        height: '100%',
        backgroundColor: '#F4F8FF',
        marginHorizontal: 16,
    },
    buttonContainer: {
        flexDirection: 'row',
        marginTop: 24,
    },
    button: {
        paddingVertical: 16,
        borderRadius: 14,
        alignItems: 'center',
        justifyContent: 'center',
    },
    cancelButton: {
        flex: 1,
        marginRight: 6,
        backgroundColor: '#E4ECFD',
    },
    cancelButtonText: {
        color: '#5A6A8A',
        fontWeight: '700',
        fontSize: 14,
    },
    saveButtonText: {
        color: '#fff',
        fontWeight: '700',
        fontSize: 14,
    },
});

export default MedicalRecordScreen;
