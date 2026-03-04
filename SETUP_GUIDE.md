# VitalCare Setup Guide

This guide will help you get the VitalCare mobile app up and running.

## Prerequisites

Before you begin, make sure you have the following installed:

### For Backend (Django)
- Python 3.8 or higher
- pip (Python package manager)

### For Mobile (React Native)
- Node.js 16 or higher
- npm or yarn
- Expo CLI: `npm install -g expo-cli`
- Android Studio (for Android) or Xcode (for iOS on Mac)
- Expo Go app on your physical device (optional)

## Backend Setup

### 1. Navigate to the backend directory
```bash
cd backend
```

### 2. Create a virtual environment (if not already done)
```bash
python -m venv venv
```

### 3. Activate the virtual environment
**Windows:**
```powershell
venv\Scripts\activate
```

**Mac/Linux:**
```bash
source venv/bin/activate
```

### 4. Install dependencies
```bash
pip install -r requirements.txt
```

### 5. Run migrations
```bash
python manage.py makemigrations
python manage.py migrate
```

### 6. Create a superuser (admin account)
```bash
python manage.py createsuperuser
```
Follow the prompts to create your admin account.

### 7. Populate help contacts (optional but recommended)
```bash
python populate_contacts.py
```

### 8. Run the development server
```bash
python manage.py runserver
```

The backend API will now be available at `http://localhost:8000`

### 9. Access the admin panel
Open your browser and go to `http://localhost:8000/admin`
Log in with the superuser credentials you created.

## Mobile App Setup

### 1. Navigate to the mobile directory
Open a new terminal window:
```bash
cd mobile
```

### 2. Install dependencies
```bash
npm install
```

### 3. Configure the API URL

Edit `mobile/src/services/api.js` and update the `API_URL` based on your setup:

- **For Android Emulator:** `http://10.0.2.2:8000/api`
- **For iOS Simulator:** `http://localhost:8000/api`
- **For Physical Device:** `http://<YOUR_COMPUTER_IP>:8000/api`

To find your computer's IP address:
- **Windows:** Run `ipconfig` in Command Prompt and look for IPv4 Address
- **Mac/Linux:** Run `ifconfig` or `ip addr` in Terminal

### 4. Start the Expo development server
```bash
npm start
```

### 5. Run the app

You have three options:

**Option A: Using an Android Emulator**
1. Start Android Studio and launch an emulator
2. Press `a` in the Expo terminal

**Option B: Using Expo Go on your phone**
1. Install Expo Go from the App Store (iOS) or Play Store (Android)
2. Scan the QR code shown in the Expo terminal
3. Make sure your phone and computer are on the same WiFi network

**Option C: Using iOS Simulator (Mac only)**
1. Press `i` in the Expo terminal

## Testing the Application

### 1. Start the Django backend
Make sure the backend server is running on `http://localhost:8000`

### 2. Start the mobile app
Make sure the Expo development server is running

### 3. Create an account
- Open the app
- Click "Sign Up"
- Fill in your details and create an account

### 4. Test all features
- **Login:** Test logging in with your credentials
- **Dashboard:** Try chatting with the AI assistant
- **Medical Records:** Fill in your medical information
- **Profile:** View and edit your profile
- **Help:** View emergency contacts

## API Endpoints

The backend provides the following API endpoints:

### Authentication
- `POST /api/auth/register/` - User registration
- `POST /api/auth/login/` - User login
- `GET /api/auth/profile/` - Get user profile
- `PUT /api/auth/profile/` - Update user profile

### Medical
- `GET /api/medical/record/` - Get medical record
- `PUT /api/medical/record/` - Update medical record
- `POST /api/medical/chat/` - Send message to AI chatbot
- `GET /api/medical/chat/history/` - Get chat history
- `GET /api/medical/help/contacts/` - Get help contacts

## Troubleshooting

### Backend Issues

**Problem:** `ModuleNotFoundError: No module named 'rest_framework'`
**Solution:** Make sure you activated the virtual environment and ran `pip install -r requirements.txt`

**Problem:** Database errors
**Solution:** Delete `db.sqlite3` and run migrations again:
```bash
rm db.sqlite3
python manage.py migrate
```

### Mobile App Issues

**Problem:** "Unable to connect to backend"
**Solution:** 
1. Make sure the Django server is running
2. Check the API_URL in `src/services/api.js`
3. For physical devices, use your computer's IP address
4. Disable any firewalls that might block the connection

**Problem:** "Expo Go failed to connect"
**Solution:**
1. Make sure your phone and computer are on the same WiFi network
2. Try restarting the Expo development server
3. Try clearing the Expo cache: `expo start -c`

**Problem:** Dependencies installation errors
**Solution:**
```bash
rm -rf node_modules
npm cache clean --force
npm install
```

## Development Tips

### Backend Development
- Use Django admin panel to manage data: `http://localhost:8000/admin`
- View API documentation by accessing endpoints directly
- Check logs in the terminal where Django is running

### Mobile Development
- Enable Hot Reload in Expo for faster development
- Use React Native Debugger for debugging
- Check console logs in the Expo dev tools

## Next Steps

Now that you have VitalCare running, you can:

1. **Customize the chatbot:** Modify `backend/medical/chatbot.py` to improve AI responses
2. **Add more features:** Extend the medical records with more fields
3. **Improve UI:** Customize the mobile app design in the screen files
4. **Deploy:** Prepare for production deployment

## Need Help?

If you encounter any issues:
1. Check the error messages carefully
2. Review this setup guide again
3. Check the Django logs and Expo logs
4. Ensure all dependencies are installed correctly

Enjoy using VitalCare! 🏥💙
