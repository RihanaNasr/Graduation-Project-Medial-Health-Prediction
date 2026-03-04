# VitalCare - Project Structure & Documentation

## 📱 Project Overview

VitalCare is a comprehensive medical mobile application built with Django REST Framework (backend) and React Native (frontend). The app provides AI-powered medical assistance, health record management, and emergency support contacts.

## 🎯 Features

### ✅ Implemented Features

1. **User Authentication**
   - Email-based registration
   - Secure JWT token authentication
   - Profile management
   - Persistent login sessions

2. **AI Medical Chatbot**
   - Rule-based medical assistance
   - Pattern matching for health topics
   - Emergency detection
   - Chat history tracking

3. **Medical Records Management**
   - Personal health information storage
   - Blood type, height, weight tracking
   - Chronic conditions and allergies
   - Current medications and past surgeries
   - Emergency contact information

4. **Profile Management**
   - View and edit personal information
   - Update contact details
   - Secure logout functionality

5. **Help & Support**
   - Emergency contact numbers (911, Poison Control, etc.)
   - Medical helplines
   - Direct call functionality
   - VitalCare support information

## 📂 Project Structure

```
GRAD_PROJECT/
├── backend/                      # Django REST API
│   ├── vitalcare/               # Main Django project
│   │   ├── settings.py         # Django configuration
│   │   ├── urls.py             # Main URL routing
│   │   └── ...
│   ├── users/                   # User authentication app
│   │   ├── models.py           # Custom User model
│   │   ├── serializers.py      # User serializers
│   │   ├── views.py            # Auth endpoints
│   │   ├── urls.py             # User URLs
│   │   └── admin.py            # Admin configuration
│   ├── medical/                 # Medical features app
│   │   ├── models.py           # Medical records, chat, help contacts
│   │   ├── serializers.py      # Medical serializers
│   │   ├── views.py            # Medical endpoints
│   │   ├── urls.py             # Medical URLs
│   │   ├── chatbot.py          # AI chatbot logic
│   │   └── admin.py            # Admin configuration
│   ├── manage.py                # Django management script
│   ├── requirements.txt         # Python dependencies
│   ├── populate_contacts.py    # Script to add help contacts
│   └── db.sqlite3              # SQLite database
│
├── mobile/                      # React Native app
│   ├── src/
│   │   ├── screens/            # App screens
│   │   │   ├── LoginScreen.js          # Login page
│   │   │   ├── RegisterScreen.js       # Registration page
│   │   │   ├── DashboardScreen.js      # AI Chat interface
│   │   │   ├── ProfileScreen.js        # User profile
│   │   │   ├── MedicalRecordScreen.js  # Health records
│   │   │   └── HelpScreen.js           # Emergency contacts
│   │   ├── navigation/         # Navigation setup
│   │   │   └── AppNavigator.js         # App routing
│   │   ├── context/            # React context
│   │   │   └── AuthContext.js          # Authentication state
│   │   └── services/           # API services
│   │       └── api.js                   # Axios API client
│   ├── assets/                 # Images and icons
│   ├── App.js                  # Main app entry point
│   ├── package.json            # npm dependencies
│   ├── app.json                # Expo configuration
│   └── babel.config.js         # Babel configuration
│
├── README.md                    # Project overview
└── SETUP_GUIDE.md              # Detailed setup instructions
```

## 🔌 API Endpoints

### Authentication Endpoints
| Method | Endpoint | Description | Auth Required |
|--------|----------|-------------|---------------|
| POST | `/api/auth/register/` | User registration | No |
| POST | `/api/auth/login/` | User login | No |
| GET | `/api/auth/profile/` | Get user profile | Yes |
| PUT | `/api/auth/profile/` | Update user profile | Yes |

### Medical Endpoints
| Method | Endpoint | Description | Auth Required |
|--------|----------|-------------|---------------|
| GET | `/api/medical/record/` | Get medical record | Yes |
| PUT | `/api/medical/record/` | Update medical record | Yes |
| POST | `/api/medical/chat/` | Send chat message | Yes |
| GET | `/api/medical/chat/history/` | Get chat history | Yes |
| GET | `/api/medical/help/contacts/` | Get help contacts | Yes |

## 💾 Database Models

### User Model
- email (unique)
- username
- first_name, last_name
- phone_number
- date_of_birth
- gender
- profile_picture
- created_at, updated_at

### Medical Record Model
- user (ForeignKey)
- blood_type
- height, weight
- chronic_conditions
- allergies
- current_medications
- past_surgeries
- emergency_contact_name
- emergency_contact_phone
- emergency_contact_relation

### Chat Message Model
- user (ForeignKey)
- message (user input)
- response (AI response)
- timestamp

### Help Contact Model
- name
- phone_number
- description
- is_emergency (boolean)

## 🎨 Design & UI

### Color Scheme
- Primary: `#6366F1` (Indigo)
- Success: `#10B981` (Green)
- Error: `#EF4444` (Red)
- Background: `#F9FAFB` (Light Gray)
- Text: `#1F2937` (Dark Gray)

### Key UI Components
- Modern card-based layouts
- Gradient headers
- Smooth animations
- Icon-based navigation
- Responsive forms
- Material Design inspired

## 🔐 Security Features

1. **JWT Authentication**
   - Secure token-based authentication
   - Automatic token refresh
   - Token expiration handling

2. **Password Security**
   - Minimum 8 characters required
   - Django's built-in password hashing
   - Password confirmation validation

3. **Data Privacy**
   - Users can only access their own data
   - API authentication required for all endpoints
   - CORS configured for mobile app access

## 🚀 Getting Started

### Quick Start
1. **Backend:**
   ```bash
   cd backend
   python -m venv venv
   venv\Scripts\activate
   pip install -r requirements.txt
   python manage.py migrate
   python populate_contacts.py
   python manage.py runserver
   ```

2. **Mobile:**
   ```bash
   cd mobile
   npm install
   npm start
   ```

See `SETUP_GUIDE.md` for detailed instructions.

## 📱 App Screens

### 1. Login Screen
- Email and password input
- Form validation
- "Sign Up" navigation link
- Beautiful gradient design

### 2. Registration Screen
- Comprehensive user information form
- Password confirmation
- Scrollable layout
- Field validation

### 3. Dashboard (AI Chat)
- Chat interface with AI assistant
- Message history
- Real-time messaging
- Loading indicators
- Empty state for new users

### 4. Medical Records
- View/Edit toggle
- Comprehensive health information
- Emergency contact details
- Organized sections
- Data persistence

### 5. Profile
- User information display
- Edit functionality
- Logout option
- Join date display

### 6. Help & Support
- Emergency contact list
- Direct call functionality
- Warning banner for emergencies
- App information
- Contact descriptions

## 🤖 AI Chatbot Features

The medical chatbot uses pattern matching to detect:
- Greetings
- Symptom descriptions
- Medication questions
- Emergency situations
- General health queries

All responses include a medical disclaimer.

## 🔧 Technologies Used

### Backend
- Python 3.8+
- Django 4.2.9
- Django REST Framework 3.14.0
- djangorestframework-simplejwt 5.3.1
- django-cors-headers 4.3.1
- SQLite (development)

### Frontend
- React Native 0.73
- Expo 50.0
- React Navigation 6.x
- Axios 1.6.5
- AsyncStorage 1.21.0
- Expo Vector Icons

## 📝 Future Enhancements

Potential features to add:
1. **Hardware Integration**
   - ESP32 vital signs monitoring
   - Real-time health data tracking
   - Bluetooth connectivity

2. **Advanced AI**
   - Machine learning models
   - Symptom checker
   - Health predictions

3. **Additional Features**
   - Medication reminders
   - Appointment scheduling
   - Health data visualization
   - Doctor connections
   - Prescription management

4. **Social Features**
   - Family health sharing
   - Care team collaboration
   - Health communities

5. **Deployment**
   - Production server setup
   - App Store/Play Store publishing
   - Cloud database migration
   - Professional hosting

## 🐛 Known Issues

- Asset images need to be added to `mobile/assets/`
- Mobile app requires network connection to backend
- Limited chatbot intelligence (rule-based only)

## 📄 License

MIT License

## 👨‍💻 Development

### Backend Development
- Admin panel: `http://localhost:8000/admin`
- API root: `http://localhost:8000/api/`
- Database: SQLite (`db.sqlite3`)

### Mobile Development
- Expo dev server: `http://localhost:19000`
- Hot reload enabled
- Console logs in Expo dev tools

## 🎓 Learning Resources

- [Django Documentation](https://docs.djangoproject.com/)
- [Django REST Framework](https://www.django-rest-framework.org/)
- [React Native Documentation](https://reactnative.dev/)
- [Expo Documentation](https://docs.expo.dev/)
- [React Navigation](https://reactnavigation.org/)

---

**VitalCare** - Your Personal Health Companion 🏥💙
