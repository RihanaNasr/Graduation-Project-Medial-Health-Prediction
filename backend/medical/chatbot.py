import re
import random
import google.generativeai as genai
import os
from decouple import config


class MedicalChatbot:
    """Hybrid medical chatbot: Rules-based for specific symptoms, AI (Gemini) for everything else"""
    
    def __init__(self):
        # Configure Gemini
        self.api_key = config('GOOGLE_API_KEY', default=None)
        self.ai_enabled = False
        
        if self.api_key and self.api_key != 'your-gemini-api-key-here':
            try:
                genai.configure(api_key=self.api_key)
                self.model = genai.GenerativeModel('gemini-2.0-flash-lite-preview-12-2025')
                self.ai_enabled = True
            except Exception as e:
                print(f"Error configuring Gemini: {e}")

        self.responses = {
            'greeting': [
                "Hello! I'm VitalCare AI. How can I assist you with your health concerns today?",
                "Hi there! I'm here to help you with medical information. What's on your mind?",
                "Welcome to VitalCare! How may I help you today?"
            ],
            'headache': [
                "For a headache, try resting in a dark, quiet room. Stay hydrated, and you might consider over-the-counter pain relief like Ibuprofen or Paracetamol if suitable for you. If it's severe or accompanied by a stiff neck, seek medical help.",
                "Headaches can often be relieved by staying hydrated, reducing screen time, or using a warm/cold compress on your forehead. Ensure you're getting enough sleep."
            ],
            'fever': [
                "To manage a fever, stay hydrated with plenty of water or electrolyte drinks. Get plenty of rest and keep the room cool. You can use over-the-counter medications like acetaminophen to reduce the temperature.",
                "If you have a fever, rest and fluids are key. Monitor your temperature and seek medical attention if it goes above 39.4°C (103°F) or lasts more than three days."
            ],
            'cough_cold': [
                "For a cough or cold, try warm salt water gargles, honey with warm water, and staying hydrated. Humidifiers can also help soothe your throat and nasal passages.",
                "Make sure to get plenty of rest and drink lots of fluids. Over-the-counter cough suppressants or decongestants may help manage your symptoms."
            ],
            'stomach_ache': [
                "For a stomach ache, try drinking ginger tea or peppermint tea. Avoid heavy or spicy foods until you feel better. If the pain is sharp and localized, please consult a doctor.",
                "Resting with a heating pad on your abdomen can help. Stick to the BRAT diet (Bananas, Rice, Applesauce, Toast) if you're also feeling nauseous."
            ],
            'symptoms': [
                "I understand you're experiencing symptoms. Please describe them in more detail so I can offer better suggestions. Remember to consult a professional for a proper diagnosis.",
                "Thank you for sharing. General advice for most symptoms includes rest, hydration, and monitoring. What other symptoms are you feeling?"
            ],
            'medication': [
                "For medication-related questions, I recommend consulting with your doctor or pharmacist for personalized advice.",
                "Medication information varies by individual. Please consult your healthcare provider for specific recommendations."
            ],
            'emergency': [
                "⚠️ This sounds like an emergency! Please call emergency services (911) or visit the nearest emergency room immediately!",
                "⚠️ For urgent medical situations, please seek immediate medical attention or call emergency services!"
            ],
            'general_health': [
                "Maintaining a healthy lifestyle includes regular exercise, balanced diet, adequate sleep, and stress management.",
                "General health tips: Stay hydrated, eat nutritious foods, exercise regularly, get enough sleep, and schedule regular check-ups."
            ],
            'heart_rate': [
                "If your heart rate is high, try to sit or lie down and focus on deep, slow breaths. Avoid caffeine or nicotine. If it's accompanied by chest pain, dizziness, or shortness of breath, please seek medical attention immediately.",
                "A high heart rate can be caused by stress, exercise, or dehydration. Try relaxation techniques and drink some water. If it persists while resting, consult a doctor."
            ],
            'chest_pain': [
                "⚠️ Chest pain can be a sign of a serious condition. Please rest immediately. If the pain is sharp, crushing, or spreads to your arm/jaw, call emergency services (911) right away!",
                "⚠️ For any chest discomfort, it is safest to seek immediate medical evaluation. Do not ignore persistent chest pain."
            ],
            'help_request': [
                "I'm here to help you find solutions! Could you tell me exactly what you're feeling? For example, are you having a headache, high heart rate, or fever?",
                "I can give you home care advice if you tell me your symptoms (like 'I have a cough' or 'my heart is racing'). What's going on?"
            ],
            'default': [
                "I want to provide a specific solution for you! Could you describe your symptoms? (e.g., 'I have a headache' or 'I feel dizzy')",
                "I'm VitalCare AI. If you tell me what you're feeling (e.g., pain, fever, heart rate), I can give you some helpful advice!",
                "I'm not quite sure about that one. Could you try rephrasing or telling me about a specific symptom?"
            ]
        }
        
        self.patterns = {
            'greeting': r'\b(hi|hello|hey|good morning|good afternoon|good evening)\b',
            'chest_pain': r'\b(chest pain|heart pain|tight chest|chest pressure)\b',
            'emergency': r'\b(emergency|can\'t breathe|severe bleeding|unconscious|stroke|seizure)\b',
            'heart_rate': r'\b(heart rate|pulse|heart racing|palpitations|tachycardia|heart beating fast)\b',
            'headache': r'\b(headache|head ache|migraine|throbbing head|head hurts)\b',
            'fever': r'\b(fever|high temp|chills|shivering|burning up)\b',
            'cough_cold': r'\b(cough|cold|sore throat|flu|congestion|runny nose|sneez)\b',
            'stomach_ache': r'\b(stomach|nausea|vomit|diarrhea|cramp|belly)\b',
            'symptoms': r'\b(symptom|illness|unwell|sick|ailment)\b',
            'medication': r'\b(medicine|medication|drug|pill|prescription|dose)\b',
            'help_request': r'\b(what should i do|help me|need advice|how to treat|how to fix|medical help)\b',
        }
    
    def get_ai_response(self, message):
        """Get response from Gemini AI as a GPT-like fallback"""
        if not self.ai_enabled:
            return "I'd love to give you a detailed AI answer, but my AI core (Gemini API) isn't configured yet. Please add a valid GOOGLE_API_KEY to the backend .env file to enable GPT-like responses!"
        
        try:
            # System instructions for the AI
            prompt = f"You are VitalCare AI, a professional medical assistant chatbot. Provide helpful, detailed, and empathetic medical advice, plans, and solutions. If the user asks for a schedule or diet plan, provide a complete one. Always include a reminder to consult a doctor. User message: {message}"
            response = self.model.generate_content(prompt)
            return response.text
        except Exception as e:
            print(f"Gemini API Error: {str(e)}")
            return random.choice(self.responses['default'])

    def get_response(self, message):
        """Generate response based on user message"""
        message_lower = message.lower()
        
        # 1. ONLY check for CRITICAL or SIMPLE patterns first
        # We removed 'general_health' (diet/exercise) so AI handles those complex requests
        priority_order = ['chest_pain', 'emergency', 'heart_rate', 'headache', 'fever', 'cough_cold', 'stomach_ache', 'medication', 'greeting']
        
        for category in priority_order:
            if category in self.patterns and re.search(self.patterns[category], message_lower):
                return random.choice(self.responses[category])
        
        # 2. For EVERYTHING else (diet plans, schedules, complex questions), use the AI
        return self.get_ai_response(message)
    
    def process_message(self, user_message):
        """Process user message and return AI response"""
        response = self.get_response(user_message)
        
        # Add disclaimer (only if it's not already a very long AI response which likely has its own)
        if len(response) < 500:
            disclaimer = "\n\n💡 Note: This is general health information. Always consult with qualified healthcare professionals for medical advice."
            return response + disclaimer
        
        return response
