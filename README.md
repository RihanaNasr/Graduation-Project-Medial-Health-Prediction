# 🩺 CardiGO - AI Cardiac Health Companion

**CardiGO** is a high-impact, feature-rich medical application designed to monitor cardiovascular health through AI-driven analytics, real-time risk assessment, and safety-critical features. This project serves as a comprehensive bridge between home monitoring and professional care.

---

## 🚀 Key Elite Features

### 1. Hybrid AI Chatbot (Explainable AI - XAI)
- **Intelligent Diagnosis:** Advanced symptom analysis using Google Gemini Pro.
- **Explainable Mode:** Users can ask "Why?" to understand the medical reasoning behind AI risk assessments.
- **Voice-to-Symptom:** Accessibility-first microphone integration for hands-free symptom descriptions.
- **Local Fallback Engine:** A robust statistical fallback ensures the chatbot remains functional even during API quota exhaustion.

### 2. Safety & Emergency (SOS)
- **One-Tap Emergency SOS:** A safety-critical feature that instantly triggers a backend alert.
- **Automated Vitals Dispatch:** Sends the user's latest heart rate, SpO2, and BP directly to their emergency contacts and simulated emergency services.

### 3. Smart Analytics Dashboard
- **Weekly Trend Monitoring:** Fully dynamic chart mapping cardiovascular data over the last 7 days.
- **Anomaly Detection:** Automatically highlights high heart rate readings in **RED** to warn users of potential risks.
- **Tappable Details:** Detailed analytics available for every day of history.

### 4. Professional Health Export
- **PDF-Ready Report:** Generates a professional summary for doctors, including vital trends, risk history, and physician-compliant notes.
- **Tele-Health Bridge:** Direct sharing from the appointment screen to medical specialists like "Dr. Sarah."

### 5. Premium Mobile Experience
- **Dark Mode Persistence:** Global theme switching with persistent state across app sessions.
- **Premium Aesthetics:** Slate-dark palette for low-light legibility and vivid gradient UIs.
- **State Management:** Seamless synchronization between the Django REST Backend and React Native front-end.

---

## 🛠️ Tech Stack

### Backend (Django)
- **Framework:** Django 4.2.9 + REST Framework
- **Auth:** JWT (JSON Web Tokens) with rotation and blacklist support.
- **Database:** SQLite (Development) / PostgreSQL (Production ready)
- **AI:** Google Generative AI (Gemini Pro) + Local Statistical Inference.

### Frontend (React Native)
- **App Platform:** Expo / React Native
- **Styling:** LinearGradients, Svg Vector Icons, and Vanilla CSS.
- **State:** React Context API + AsyncStorage.
- **Network:** Axios with configurable timeout buffers.

---

## 📁 Project Structure

```
GRAD_PROJECT/
├── backend/              # Django REST API Hub
│   ├── cardigo/          # Core Settings & URLs
│   ├── medical/          # AI Engine, Models & Views
│   ├── users/            # Token Authentication
│   └── .env              # API Environment Keys
├── mobile/               # Mobile Frontend Application
│   ├── src/
│   │   ├── screens/      # Dashboard, Chat, SOS, History
│   │   ├── context/      # Theme & Auth Context
│   │   └── services/     # API Axios Services
│   └── App.js            # Main Theme Provider Entry
└── README.md
```

## 🎯 Quick Start Guide

### 1. Backend Setup
```bash
cd backend
python -m venv venv
.\venv\Scripts\activate
pip install -r requirements.txt
python manage.py migrate
python manage.py runserver 0.0.0.0:8000
```

### 2. Mobile Setup
```bash
cd mobile
npm install
npx expo start --port 8082
```
*Note: Ensure your phone is on the same Wi-Fi as your laptop to reach the backend IP.*

---

## 🛡️ License
CardiGO - Graduation Project 2026.
Created by Ahmed & Team.
