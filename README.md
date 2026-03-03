# VitalCare - Medical Mobile Application

A comprehensive medical mobile application built with Django REST Framework and React Native.

## Features

### Authentication
- User Registration (Sign Up)
- User Login
- JWT Token-based authentication
- Profile Management

### Core Functionality
- **Dashboard**: Interactive AI chatbot for medical assistance
- **Medical Records**: Store and manage patient health information
- **Profile**: View and update user profile
- **Help & Support**: Access to emergency contacts and information

### AI Integration
- Medical chatbot for health queries
- Rule-based medical advice system

## Tech Stack

### Backend
- Django 4.x
- Django REST Framework
- SQLite/PostgreSQL
- JWT Authentication
- CORS enabled

### Frontend
- React Native
- React Navigation
- Axios for API calls
- AsyncStorage for local data
- Modern UI components

## Project Structure

```
VitalCare/
├── backend/              # Django backend
│   ├── vitalcare/       # Main Django project
│   ├── users/           # User authentication & profiles
│   ├── medical/         # Medical records & chatbot
│   └── requirements.txt
├── mobile/              # React Native frontend
│   ├── src/
│   │   ├── screens/    # App screens
│   │   ├── components/ # Reusable components
│   │   ├── navigation/ # Navigation setup
│   │   └── services/   # API services
│   └── package.json
└── README.md
```

## Getting Started

### Backend Setup
```bash
cd backend
python -m venv venv
venv\Scripts\activate  # Windows
pip install -r requirements.txt
python manage.py migrate
python manage.py createsuperuser
python manage.py runserver
```

### Frontend Setup
```bash
cd mobile
npm install
npx react-native run-android  # For Android
npx react-native run-ios       # For iOS
```

## API Endpoints

- `POST /api/auth/register/` - User registration
- `POST /api/auth/login/` - User login
- `GET /api/auth/profile/` - Get user profile
- `PUT /api/auth/profile/` - Update profile
- `POST /api/medical/records/` - Create medical record
- `GET /api/medical/records/` - List medical records
- `POST /api/medical/chat/` - AI chatbot interaction
- `GET /api/help/contacts/` - Get emergency contacts

## License

MIT License
