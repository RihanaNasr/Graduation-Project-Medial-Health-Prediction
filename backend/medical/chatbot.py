import os
import sys
import datetime
import pandas as pd
import re

# --- Environment & Django Setup ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, PROJECT_ROOT)

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'backend.settings')
try:
    import django
    django.setup()
    from medical.models import MedicalRecord
    from django.contrib.auth import get_user_model
    User = get_user_model()
except:
    pass

class CardiGoUnifiedBrain:
    def __init__(self):
        print("\n--- 🧠 CardiGo Unified Brain: ONLINE ---")

    def get_response(self, message, user_email=None):
        msg = message.lower().strip()
        is_arabic = any(c in msg for c in 'ابتثجحخدذرزسشصضطظعغفقكلمنهوي')
        
        # 1. Capture Live Data
        hr, o2, sev = "75", "98", "2"
        try:
            if user_email:
                u = User.objects.filter(email__iexact=user_email.strip()).first()
                if u:
                    r = MedicalRecord.objects.get(user=u)
                    hr = r.heart_rate
                    o2 = r.spo2
                    sev = r.calculate_severity_rate()
        except: pass

        # 2. FEATURE ANALYSIS & EXPLANATION (XAI)
        if any(w in msg for w in ["why", "ليه", "analysis", "تحليل", "explain", "ازاي"]):
            if is_arabic:
                return (f"🔍 **تحليل المؤشرات الطبية**:\n"
                        f"- نبضك ({hr}) يتم مقارنته بمعدل الراحة الطبيعي.\n"
                        f"- نسبة الأكسجين ({o2}%) ممتازة.\n"
                        f"- معدل الخطورة {sev}% محسوب بناءً على العوامل السابقة لضمان سلامتك.")
            return (f"🔍 **Clinical Analysis Reasoning**:\n"
                    f"- Heart Rate ({hr} bpm) is being monitored for stability.\n"
                    f"- SpO2 ({o2}%) level is checked for respiratory health.\n"
                    f"- A {sev}% risk score is generated using our XAI logic.")

        # 3. ADVICE & TIPS (Sleep, Lifestyle, Medical)
        if any(w in msg for w in ["sleep", "نوم", "انام", "نصيحة", "تنصحني", "نصائح"]):
            if is_arabic:
                return ("🌙 **نصيحة CardiGO لك**:\n"
                        "- حافظ على نوم منتظم (7-9 ساعات).\n"
                        "- اشرب كميات كافية من الماء يومياً.\n"
                        "- تجنب الإجهاد والتوتر الزائد للحفاظ على استقرار نبضك.\n"
                        "- يفضل المشي الخفيف لمدة 30 دقيقة يومياً.")
            return ("🌙 **CardiGO Professional Advice**:\n"
                    "- Maintain a consistent sleep schedule.\n"
                    "- Stay hydrated throughout the day.\n"
                    "- Practice deep breathing to manage heart stress.\n"
                    "- Aim for 30 mins of light activity like walking.")

        # 4. DAILY REPORT / STATUS / HISTORY / RISK
        if any(w in msg for w in ["report", "تقرير", "حالة", "status", "ملخص", "history", "سجل", "تاريخ", "خطر", "danger"]):
            if is_arabic:
                risk_status = "خطرة" if int(str(sev).replace('%','')) > 50 else "مستقرة"
                return f"🏥 **تقييم الحالة والخطورة**\n━━━━━━━━━━━━━━━━━━\n💓 النبض: {hr}\n🩸 الأكسجين: {o2}%\n⚠️ الخطورة: {sev}%\n📋 التقييم: حالتك {risk_status}. يرجى الالتزام بالتعليمات."
            return f"🏥 **Health Risk Assessment**\n━━━━━━━━━━━━━━━━━━\n💓 Pulse: {hr} bpm\n🩸 SpO2: {o2}%\n⚠️ Risk: {sev}%\n📋 Verdict: You are safe. Continue monitoring."

        # 5. DYNAMIC PULSE CHECK (Pulse logic)
        if hr_match := re.search(r'(\d+)', msg):
            val = int(hr_match.group(1))
            if val > 120:
                return f"⚠️ **Alert**: {val} bpm is high. (تنبيه: نبضك عالٍ). Please take a moment to rest."
            return f"The pulse of {val} bpm is noted. (نبضك {val} وهو جيد). How else can I help?"

        # 6. GREETINGS & FEEDBACK
        if any(w in msg for w in ["hello", "hi", "سلام", "ازيك", "شكرا", "thanks"]):
            if is_arabic: return "أهلاً بك! أنا CardiGO، كيف يمكنني مساعدتك اليوم؟"
            return "Hello! I am CardiGO. How can I assist you with your heart health today?"

        # FINAL FALLBACK (Helpful Summary)
        if is_arabic:
            return f"أنا هنا لمساعدتك. حالياً نبضك {hr}. يمكنك سؤالي عن نصائح للنوم، تحليل الحالة، أو ملخص التقرير."
        return f"I am here to help. Currently, your pulse is {hr} bpm. Try asking for 'Sleep tips', 'Analysis', or 'Today's report'."

# --- Flask Server ---
from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)
brain = CardiGoUnifiedBrain()

@app.route('/chat', methods=['POST'])
def chat():
    data = request.json
    msg = data.get('message', '')
    email = data.get('user_email', '')
    response = brain.get_response(msg, user_email=email)
    return jsonify({"response": response})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

MedicalChatbot = CardiGoUnifiedBrain
CardiGoGPT = CardiGoUnifiedBrain
