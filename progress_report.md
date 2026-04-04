# Implementation & Testing Progress Report 1

## Cover Page
- **Project Title:** CardiGO - Cardiovascular Health Prediction System
- **Project Number:** [Insert Project Number]
- **Team Members + IDs + Programs:**
    - Reem Ehab (ID: 202201373) - DSAI
    - Rihana Nasr (ID: [Insert ID]) - DSAI
    - Tasneem Ashraf (ID: [Insert ID]) - DSAI
    - Abdelrahman Khaled (ID: [Insert ID]) - DSAI
- **Supervisor:** [Insert Supervisor Name]
- **Submission Date:** March 5, 2026

---

## 1. Project Status Overview

### 1.1 Overall Completion Status
- **% completion toward MVP:** Approximately 65% completed.
- **Major milestones achieved:**
    - Completion of dataset collection and preprocessing pipelines.
    - Setup of hardware prototype (ESP32, MAX30102, MAX30205 sensors).
    - Initial design and training of machine learning models (Random Forest, SVM, CatBoost) achieving baseline metrics.
    - Development and deployment of backend APIs mapping to ML model inferences.
    - Implementation of foundational mobile application UI (Flutter) and connection to APIs.
- **Current blockers:** 
    - Full system integration between the real-time continuous hardware data transmission, the backend API, and the mobile application dashboard.
    - Integration of explainable AI (XAI) frameworks natively into the mobile interfaces.

### 1.2 Alignment with Final Design
- **Is the architecture unchanged?** Yes, the core architecture remains unchanged.
- **If changed, explain why:** N/A. The flow from Wearable Sensor -> Backend/ML Engine -> Mobile UI is proceeding as originally proposed.

---

## 2. Implementation Progress

### 2.1 Component A: Hardware & Sensor System
- **Purpose:** To collect real-time physiological data including heart rate, blood oxygen saturation, and body temperature.
- **Implemented features:** ESP32 hardware setup and sensor wiring for continuous signal capture.
- **Technology stack:** C/C++, ESP32, MAX30102, MAX30205.
- **Evidence:** Hardware assembled and tested; basic real-time data transmission logs established over serial/Bluetooth.

### 2.2 Component B: Machine Learning & Backend Logic
- **Purpose:** To ingest physiological data and output early cardiovascular risk predictions using robust machine learning models.
- **Implemented features:** Data cleaning, normalization, feature engineering, model training/serialization, and Flask API endpoint creation for model inference.
- **Technology stack:** Python, Scikit-learn, TensorFlow/Keras, Flask/Django.
- **Evidence:** Trained models exported successfully. Flask API running and returning prediction responses dynamically when fed JSON payloads.

### 2.3 Component C: Mobile Application UI
- **Purpose:** To provide patients and practitioners with an accessible interface to view real-time risk assessments, health patterns, and receive medical chatbot assistance.
- **Implemented features:** Cross-platform Flutter application beta, user registration/login screens, and real-time sensor monitoring views.
- **Technology stack:** Flutter, Dart, REST API Integration.
- **Evidence:** Screens developed, API requests logging successfully on backend, user session handling functional.

---

## 3. Program-Specific Technical Evidence

### DSAI (Data Science & Artificial Intelligence)
- **Data collection or preprocessing started?** Yes, both open-source tabular datasets and dynamic database integration have commenced.
- **Model baseline implemented?** Yes, baselines using Random Forest, Logistic Regression, XGBoost/CatBoost, and SVM have been set.
- **Dataset cleaning completed?** Yes, missing values handled and datasets split and normalized.
- **Pipeline partially automated?** Yes, utilizing custom Python data processing scripts for scaling and feature engineering.

### SWD (Software Development)
- **Which architectural layers are implemented?** The hardware data capture layer, backend API (Flask/Django), and frontend client (Flutter).
- **Which algorithms or core logic are completed?** ML Inference logic, JWT-based authentication logic, API routing.
- **Which design patterns are already used?** MVC (Model-View-Controller) on the backend, RESTful API patterns.
- **Framework integration status?** Flask API successfully mapped to Scikit-learn models; Flutter partially connected to endpoints.

### IT (Information Technology)
- **Which systems are integrated?** The Database is integrated with the backend API, and preliminary connections between the mobile app and backend are active.
- **Deployment environment prepared?** Local server environment setup complete; cloud deployment environments currently being configured.
- **Security configuration started?** Yes, implemented JWT token authentication and encrypted password hashing.
- **Infrastructure setup status?** Backend API endpoints are functional over local network networks, pending final public deployment.

---

## 4. Testing Summary

### 4.1 Testing Methods Conducted
- **Unit tests:** Evaluating isolated backend API endpoints.
- **Manual tests:** End-to-end user interface testing, validating screen navigation and component rendering.
- **Integration tests:** Testing data flow between the Flutter app and the Python Backend.
- **Model validation:** Testing accuracy, precision, recall, F-1 score, and ROC-AUC scores against validation datasets.
- **Security tests:** Validating token expiration and route protection.

### 4.2 Test Evidence
- **Test examples:** Validating that `POST /api/medical/records/` correctly rejects malformed data.
- **Coverage:** High coverage on ML validation datasets (testing sets).
- **Sample test results:** The ML model achieved strong baseline accuracy during offline validation testing. Hardware tests confirm accurate readings.

### 4.3 Issues and Bugs Identified
| Issue ID | Description | Severity | Status | Fix Plan |
|---|---|---|---|---|
| BUG-001 | Latency in ML prediction response when batching data | Medium | Open | Optimize backend inference load time and scale database queries. |
| BUG-002 | Bluetooth connection dropping sporadically | High | Open | Adjust connection timeout limits in ESP32 logic. |
| BUG-003 | UI overflow issues on small screen devices | Low | Fixed | Wrap columns in `SingleChildScrollView`. |

---

## 5. Work Plan Toward MVP (Next 2 Weeks)

### Structured Plan:
| Task | Owner | Deadline | Risk | Notes |
|---|---|---|---|---|
| Complete Chatbot Integration | Rihana / Abdelrahman | Week 10 | High | NLP tuning & integration might take longer. |
| DB Validation & App Testing | Tasneem | Week 11 | Medium | Ensuring schema matches ML expected input exactly. |
| ML UI/UX Finalization | Rihana | Week 11 | Low | Refining dashboard visualization components. |
| Full App-Backend Integration | Abdelrahman | Week 12 | Medium | API endpoint data structuring adjustments. |

**Summary for MVP:**
- **What will be finished before MVP:** The hardware-software pipeline, core ML risk predictions, the mobile dashboard for displaying results, and basic chatbot functionally.
- **What remains partially implemented:** Real-time explainable AI (SHAP/LIME) dynamic graphs rendering inside the app.
- **What risks could delay MVP:** Delays in integrating the medical chatbot NLP logic smoothly over the API.

---

## 6. Individual Contribution Summary

| Member | Implemented Tasks | % Contribution |
|---|---|---|
| **Reem Ehab** | Hardware assembly support, ESP32 setup, Backend API development, Model deployment and UI testing. | 25% |
| **Tasneem Ashraf** | Machine learning model design/training (Scikit-learn), hardware testing, comparative model evaluation. | 25% |
| **Rihana Nasr** | Model validation, Flutter UI/UX development, performance metric benchmarking, testing protocols. | 25% |
| **Abdelrahman Khaled** | Complete dataset collection, implementation of data preprocessing pipelines, App-backend integration. | 25% |
