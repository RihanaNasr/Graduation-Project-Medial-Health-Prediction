# CardiGO MVP Deliverables

This document contains all the necessary materials and guides for your MVP submission based on your project's current state.

## 1. MVP Features Checklist (1 page)

| Feature | Component | Status | Evidence |
| :--- | :--- | :--- | :--- |
| **User Registration & Login** | Backend API / Mobile App | Implemented | Demo video / Live Demo |
| **Hardware Data Capture** | ESP32 Sensor System | Implemented | Hardware shown in Demo |
| **Cardiovascular Risk Prediction** | ML Pipeline / Backend API | Implemented | Screenshot / Demo video |
| **Real-time Dashboard** | Mobile UI | Implemented | Demo video |
| **Medical Chatbot** | AI Logic / Mobile UI | Partial | Screen recording |
| **Medical Records Mgmt** | Backend API / Mobile App | Implemented | Demo video |

---

## 2. Demo Video Script & Flow (2–3 minutes)

*Format: MP4 | Max Size: 100 MB | Requirement: All 4 members must speak.*

**Part 1: Problem Context (15-20 seconds)**
*   **Speaker:** Tasneem
*   **Action:** Presenter speaks to the camera or over an introductory slide.
*   **Script Idea:** "Cardiovascular diseases are a leading cause of mortality globally. CardiGO aims to provide continuous health monitoring and early risk prediction using wearable sensors and machine learning to save lives."

**Part 2: Hardware & Data Capture (30-40 seconds)**
*   **Speaker:** Reem
*   **Action:** Show the ESP32 hardware prototype with sensors capturing data from a user.
*   **Script Idea:** "Our hardware prototype captures physiological data like heart rate and oxygen saturation in real-time using an ESP32 microcontroller, which securely transmits this data to our backend."

**Part 3: Model Prediction & Backend (40 seconds)**
*   **Speaker:** Abdelrahman
*   **Action:** Show a split-screen or quick transition of the backend receiving the payload, running the ML model, and returning a result.
*   **Script Idea:** "Once the data is received, our data preprocessing pipeline cleans it and feeds it into our trained machine learning model. The model then instantly outputs a cardiovascular risk assessment."

**Part 4: Mobile App Workflow (45 seconds)**
*   **Speaker:** Rihana
*   **Action:** Screen recording of the mobile app. Show logging in, viewing the dashboard, and interacting with the chatbot.
*   **Script Idea:** "On the patient app, users can log in to view their risk assessments on a dynamic dashboard. They can also manage their medical records or use our AI chatbot for immediate medical guidance."

---

## 3. Two-Slide MVP Summary

### Slide 1: MVP Overview
*   **What the MVP does:** CardiGO provides continuous physiological monitoring using hardware sensors and applies machine learning to predict early cardiovascular risks, offering patients a centralized health dashboard and an AI chatbot assistant.
*   **Key Implemented Features:**
    *   Hardware data capture via ESP32.
    *   Machine learning predictive model.
    *   Mobile app dashboard for real-time risk assessment.
    *   Secure user authentication & records management.
    *   Medical chatbot (Rule-based).
*   **System Architecture Snapshot:**
    *   *Hardware (ESP32/Sensors)* ➔ *Backend API & ML Engine* ➔ *Mobile Client App*

### Slide 2: Learning & Contributions
*   **Key Technical Challenges:** 
    *   Integrating real-time hardware data transmission with the backend API.
    *   Optimizing machine learning model inference latency.
    *   Testing cross-platform mobile UI responsiveness.
*   **What the Team Learned:** Processing raw sensor data, applying theoretical models (Scikit-learn) into deployable REST APIs, and end-to-end system integration.
*   **Responsibilities:**
    *   **Reem Ehab:** Hardware assembly setup, Backend API, Model deployment.
    *   **Tasneem Ashraf:** ML model design/training, hardware testing.
    *   **Rihana Nasr:** Mobile UI/UX development, performance metric benchmarking.
    *   **Abdelrahman Khaled:** Dataset preparation, preprocessing pipelines, App-backend integration.

---

## 4. Live Demo Preparation Guide

*You must be ready to demonstrate the system running and answer individual questions about your specific components.*

**Testing Checklist (Before Demo):**
- [ ] Ensure local servers (Backend API & DB) are running smoothly.
- [ ] Connect mobile device/emulator to the correct local API IP address.
- [ ] Power on ESP32 and establish connection.
- [ ] Perform a full dry-run of the end-to-end flow.

**Supervisor Q&A Cheat Sheet:**
*   **Reem:** Be ready to explain API endpoints, JWT authentication security, and ESP32 wiring/connection logic.
*   **Tasneem:** Be prepared to justify the choice of ML algorithms (Random Forest, SVM), explain evaluation metrics (Accuracy, F1-score), and discuss baseline metrics.
*   **Rihana:** Be ready to discuss the mobile frontend state management, API request handling in the mobile app, and how UI charts update in real-time.
*   **Abdelrahman:** Be prepared to explain dataset cleaning techniques, how missing values were handled, and the specifics of the data pipeline integration.

**Acknowledging Limitations (If Asked):**
*   *Current Bottleneck:* The chatbot is rule-based and NLP fine-tuning is partially implemented.
*   *Future Integration:* Explainable AI (XAI) visualizations (like SHAP) inside the mobile UI are planned for the final submission.
