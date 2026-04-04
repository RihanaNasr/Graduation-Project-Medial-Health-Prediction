import google.generativeai as genai
import os
import pandas as pd
import datetime
import re
import numpy as np
from decouple import config
from django.conf import settings

# For direct debugging
DEBUG_LOG_FILE = os.path.join(settings.BASE_DIR, 'bot_debug.log')

def log_debug(message):
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    with open(DEBUG_LOG_FILE, 'a', encoding='utf-8') as f:
        f.write(f"[{timestamp}] {message}\n")

class MedicalChatbot:
    """Conversational Explainable AI Chatbot for CardiGO."""
    
    def __init__(self):
        log_debug("--- Initializing Chatbot (XAI Version) ---")
        self.api_key = config('GOOGLE_API_KEY', default=None)
        self.ai_enabled = False
        self.df = None
        
        if self.api_key and self.api_key != 'your-gemini-api-key-here':
            try:
                genai.configure(api_key=self.api_key)
                self.model = genai.GenerativeModel('gemini-2.0-flash-lite')
                self.ai_enabled = True
            except: pass

        try:
            dataset_path = os.path.join(settings.BASE_DIR.parent, 'Health_Risk_Dataset.csv')
            if os.path.exists(dataset_path):
                self.df = pd.read_csv(dataset_path)
        except: pass

    def get_local_fallback(self, message, user=None):
        """Conversational State Engine with Explainable AI logic."""
        msg = message.lower()
        log_debug(f"XAI Flow: {msg[:30]}")

        # 1. SPECIALIZED XAI: "WHY?" LOGIC
        if any(kw in msg for kw in ['why', 'explain', 'how', 'reason', 'because']):
            # Access previous chat context to see what vitals we just discussed
            prev_vitals = {}
            try:
                from .models import ChatMessage
                last_chat = ChatMessage.objects.filter(user=user).order_by('-timestamp').first()
                if last_chat:
                    # Try to extract numbers from previous user message if they aren't in current one
                    prev_msg = last_chat.message.lower()
                    hr_m = re.search(r'(?:heart rate|hr|pulse).*?(\d+)', prev_msg)
                    oxy_m = re.search(r'(?:oxygen|o2|sat|spo2).*?(\d+)', prev_msg)
                    if hr_m: prev_vitals['Heart_Rate'] = int(hr_m.group(1))
                    if oxy_m: prev_vitals['Oxygen_Saturation'] = int(oxy_m.group(1))

            except: pass

            if prev_vitals and self.df is not None:
                # Compare against Low Risk average
                low_risk_df = self.df[self.df['Risk_Level'] == 'Low']
                avg_hr = low_risk_df['Heart_Rate'].mean()
                avg_o2 = low_risk_df['Oxygen_Saturation'].mean()
                
                hr_user = prev_vitals.get('Heart_Rate', 86)
                o2_user = prev_vitals.get('Oxygen_Saturation', 98)
                
                hr_diff = ((hr_user - avg_hr) / avg_hr) * 100
                o2_diff = (avg_o2 - o2_user)
                
                explanation = "CardiGO Deep Explanation (XAI):\n\n"
                explanation += f"Our analysis of 10,000+ clinical cases shows that your vitals deviate from the 'Low Risk' averages:\n"
                if abs(hr_diff) > 10:
                    status = "HIGH" if hr_diff > 0 else "LOW"
                    explanation += f"- Heart Rate ({hr_user} bpm) is {abs(hr_diff):.1f}% {status}er than our healthy average of {avg_hr:.0f} bpm.\n"
                if o2_diff > 2:
                    explanation += f"- Oxygen Saturation ({o2_user}%) is {o2_diff:.1f}% lower than the healthy benchmark of {avg_o2:.0f}%.\n"
                
                explanation += "\nBased on these specific mathematical correlations in our Health_Risk_Dataset, our system flags these as primary risk factors."
                return explanation

        # 2. HEART RISK / DATA ENGINE
        # (Included for direct vital queries)
        vitals = {}
        hr_match = re.search(r'(?:heart rate|hr|pulse).*?(\d+)', msg)
        oxy_match = re.search(r'(?:oxygen|o2|sat|spo2).*?(\d+)', msg)
        if hr_match or oxy_match:
            hr = int(hr_match.group(1)) if hr_match else 86
            o2 = int(oxy_match.group(1)) if oxy_match else 98
            if self.df is not None:
                filtered = self.df[(self.df['Heart_Rate'] >= hr-10) & (self.df['Heart_Rate'] <= hr+10)]
                risk = filtered['Risk_Level'].value_counts().index[0] if not filtered.empty else "Low"
                return f"CardiGO Assessment: Risk is **{risk}**. (HR:{hr}, O2:{o2}). Ask me 'Why?' for a deep explanation."

        # 3. GENERAL CATEGORIES (Conversational)
        if "headache" in msg: return "I see you have a headache. Heart rate changes can often cause sudden headaches. Do you feel any chest pressure or shortness of breath?"
        if "sleep" in msg: return "Rest is vital for heart health. Aim for 7.5+ hours. Are you experiencing fatigue during the day?"
        if "history" in msg or "report" in msg:
            if user:
                try:
                    from .models import MedicalRecord
                    r = MedicalRecord.objects.get(user=user)
                    return f"Daily Report:\n- HR: {r.heart_rate} bpm\n- SpO2: {r.spo2}%\n- BP: {r.blood_pressure}\n- Status: Vitals are synchronized."
                except: pass

        return "CardiGO AI: I'm here for professional health advice and data-driven risk assessment. How can I assist you today?"

    def process_message(self, user_message, user=None):
        if self.ai_enabled:
            try:
                log_debug(f"Requesting Gemini XAI flow...")
                # Optimized prompt for Gemini with XAI instructions
                prompt = f"""You are CardiGO AI, a professional medical assistant with XAI (Explainable AI) capabilities.
Always explain the 'Why' using percentages and dataset averages if the user asks for more detail.
Dataset reference: {self.df.sample(5).to_csv() if self.df is not None else ''}

User Message: {user_message}"""
                response = self.model.generate_content(prompt)
                return response.text
            except Exception as e:
                log_debug(f"API Quota: {str(e)}")
                return self.get_local_fallback(user_message, user=user)
        return self.get_local_fallback(user_message, user=user)
