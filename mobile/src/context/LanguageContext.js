import React, { createContext, useContext, useState, useEffect } from 'react';
import AsyncStorage from '@react-native-async-storage/async-storage';

const LanguageContext = createContext();

const translations = {
    en: {
        greeting: "Good morning,",
        edit: "Edit",
        save: "Save",
        live_hr: "LIVE HEART RATE",
        normal: "Normal",
        min_bpm: "Min BPM",
        max_bpm: "Max BPM",
        sleep: "Sleep",
        calories: "Calories",
        steps: "Steps",
        water: "Water (L)",
        sos: "SOS",
        elevated_hr: "Elevated Heart Rate Detected",
        appointments: "Appointments",
        view_all: "View All",
        dashboard: "Dashboard",
        chat: "Chat AI",
        profile: "Profile",
        history: "History",
        risk: "Risk Assessment",
        low_risk: "Low Risk",
        generate_report: "Generate Health Report",
        weekly_trend: "Weekly Heart Rate Trend",
        language_updated: "Language Updated",
        lang_set_to: "App language has been set to ",
        home: "Home",
        emergency_sos: "Emergency SOS",
        tap_for_help: "Tap for help",
        tomorrow: "Tomorrow",
        heart_rate: "Heart Rate",
        blood_pressure: "Blood Pressure",
        temperature: "Temperature",
        help: "Help",
    },
    ar: {
        greeting: "صباح الخير،",
        edit: "تعديل",
        save: "حفظ",
        live_hr: "نبض القلب المباشر",
        normal: "طبيعي",
        min_bpm: "أقل نبض",
        max_bpm: "أعلى نبض",
        sleep: "النوم",
        calories: "السعرات",
        steps: "الخطوات",
        water: "الماء (لتر)",
        sos: "طوارئ",
        elevated_hr: "تم اكتشاف ارتفاع في نبض القلب",
        appointments: "المواعيد",
        view_all: "عرض الكل",
        dashboard: "لوحة التحكم",
        chat: "دردشة الذكاء الاصطناعي",
        profile: "الملف الشخصي",
        history: "السجل",
        risk: "تقييم المخاطر",
        low_risk: "مخاطر منخفضة",
        generate_report: "إنشاء تقرير صحي",
        weekly_trend: "اتجاه نبض القلب الأسبوعي",
        language_updated: "تم تحديث اللغة",
        lang_set_to: "تم ضبط لغة التطبيق إلى ",
        home: "الرئيسية",
        emergency_sos: "نداء استغاثة",
        tap_for_help: "اضغط للمساعدة",
        tomorrow: "غداً",
        heart_rate: "نبض القلب",
        blood_pressure: "ضغط الدم",
        temperature: "درجة الحرارة",
        help: "المساعدة",
    }
};

export const LanguageProvider = ({ children }) => {
    const [language, setLanguage] = useState('en');

    useEffect(() => {
        loadLanguage();
    }, []);

    const loadLanguage = async () => {
        const stored = await AsyncStorage.getItem('appLanguage');
        if (stored) setLanguage(stored);
    };

    const toggleLanguage = async (lang) => {
        setLanguage(lang);
        await AsyncStorage.setItem('appLanguage', lang);
    };

    const t = (key) => {
        return translations[language][key] || key;
    };

    return (
        <LanguageContext.Provider value={{ language, toggleLanguage, t }}>
            {children}
        </LanguageContext.Provider>
    );
};

export const useLanguage = () => useContext(LanguageContext);
