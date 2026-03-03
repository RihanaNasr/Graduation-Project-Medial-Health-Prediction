# VitalCare API Testing Guide

Quick reference for testing the VitalCare API endpoints.

## Base URL
```
http://localhost:8000/api
```

## Testing with curl or Postman

### 1. Register a New User

**Endpoint:** `POST /auth/register/`

**Request Body:**
```json
{
  "email": "test@example.com",
  "username": "testuser",
  "first_name": "John",
  "last_name": "Doe",
  "password": "Test12345",
  "password_confirm": "Test12345",
  "phone_number": "+1234567890",
  "gender": "Male"
}
```

**Example curl:**
```bash
curl -X POST http://localhost:8000/api/auth/register/ \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "username": "testuser",
    "first_name": "John",
    "last_name": "Doe",
    "password": "Test12345",
    "password_confirm": "Test12345"
  }'
```

**Response:**
```json
{
  "user": {
    "id": 1,
    "email": "test@example.com",
    "username": "testuser",
    "first_name": "John",
    "last_name": "Doe",
    ...
  },
  "tokens": {
    "refresh": "eyJ0eXAiOiJKV1QiLCJhbGc...",
    "access": "eyJ0eXAiOiJKV1QiLCJhbGc..."
  }
}
```

### 2. Login

**Endpoint:** `POST /auth/login/`

**Request Body:**
```json
{
  "email": "test@example.com",
  "password": "Test12345"
}
```

**Example curl:**
```bash
curl -X POST http://localhost:8000/api/auth/login/ \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Test12345"
  }'
```

**Save the access token from the response!**

### 3. Get User Profile

**Endpoint:** `GET /auth/profile/`

**Headers:** `Authorization: Bearer <access_token>`

**Example curl:**
```bash
curl -X GET http://localhost:8000/api/auth/profile/ \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN"
```

### 4. Update Profile

**Endpoint:** `PUT /auth/profile/`

**Headers:** `Authorization: Bearer <access_token>`

**Request Body:**
```json
{
  "first_name": "Jane",
  "last_name": "Smith",
  "phone_number": "+1987654321"
}
```

### 5. Get Medical Record

**Endpoint:** `GET /medical/record/`

**Headers:** `Authorization: Bearer <access_token>`

**Example curl:**
```bash
curl -X GET http://localhost:8000/api/medical/record/ \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN"
```

### 6. Update Medical Record

**Endpoint:** `PUT /medical/record/`

**Headers:** `Authorization: Bearer <access_token>`

**Request Body:**
```json
{
  "blood_type": "A+",
  "height": 175,
  "weight": 70,
  "allergies": "Peanuts, Dust",
  "chronic_conditions": "None",
  "current_medications": "Vitamin D",
  "emergency_contact_name": "Jane Doe",
  "emergency_contact_phone": "+1234567890",
  "emergency_contact_relation": "Spouse"
}
```

**Example curl:**
```bash
curl -X PUT http://localhost:8000/api/medical/record/ \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "blood_type": "A+",
    "height": 175,
    "weight": 70
  }'
```

### 7. Chat with AI

**Endpoint:** `POST /medical/chat/`

**Headers:** `Authorization: Bearer <access_token>`

**Request Body:**
```json
{
  "message": "I have a headache. What should I do?"
}
```

**Example curl:**
```bash
curl -X POST http://localhost:8000/api/medical/chat/ \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "I have a headache. What should I do?"
  }'
```

**Response:**
```json
{
  "id": 1,
  "message": "I have a headache. What should I do?",
  "response": "I understand you're experiencing symptoms. Please remember this is general advice...",
  "timestamp": "2026-02-14T20:00:00Z"
}
```

### 8. Get Chat History

**Endpoint:** `GET /medical/chat/history/`

**Headers:** `Authorization: Bearer <access_token>`

**Example curl:**
```bash
curl -X GET http://localhost:8000/api/medical/chat/history/ \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN"
```

### 9. Get Help Contacts

**Endpoint:** `GET /medical/help/contacts/`

**Headers:** `Authorization: Bearer <access_token>`

**Example curl:**
```bash
curl -X GET http://localhost:8000/api/medical/help/contacts/ \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN"
```

**Response:**
```json
[
  {
    "id": 1,
    "name": "Emergency Services",
    "phone_number": "911",
    "description": "For life-threatening emergencies...",
    "is_emergency": true,
    "created_at": "2026-02-14T18:00:00Z"
  },
  ...
]
```

## Using Postman

1. **Create a new request**
2. **Set the method** (GET, POST, PUT)
3. **Enter the URL** (e.g., `http://localhost:8000/api/auth/login/`)
4. **Add headers:**
   - `Content-Type: application/json`
   - `Authorization: Bearer <your_token>` (for protected endpoints)
5. **Add request body** (for POST/PUT requests)
6. **Send the request**

## Testing Workflow

### Complete Test Sequence

1. **Register a new user** → Save tokens
2. **Login** → Confirm tokens work
3. **Get profile** → Verify user data
4. **Update profile** → Test data modification
5. **Get medical record** → Should return empty or default record
6. **Update medical record** → Add health information
7. **Send chat message** → Test AI chatbot
8. **Get chat history** → Verify message was saved
9. **Get help contacts** → View emergency contacts

## Common Response Codes

- `200 OK` - Successful GET/PUT
- `201 Created` - Successful POST (creation)
- `400 Bad Request` - Invalid data
- `401 Unauthorized` - Invalid/missing token
- `404 Not Found` - Resource doesn't exist
- `500 Internal Server Error` - Server error

## Tips

1. **Always save your access token** after login/register
2. **Include the Authorization header** for protected endpoints
3. **Check Content-Type header** for JSON requests
4. **Tokens expire after 1 day** - login again if you get 401 errors
5. **Use the Django admin panel** to view data: `http://localhost:8000/admin`

## Django Admin Panel

Access: `http://localhost:8000/admin`

**Create superuser:**
```bash
python manage.py createsuperuser
```

In the admin panel you can:
- View all users
- Manage medical records
- See chat messages
- Edit help contacts
- Monitor application data

---

Happy testing! 🚀
