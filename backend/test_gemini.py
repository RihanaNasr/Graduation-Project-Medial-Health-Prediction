import google.generativeai as genai
import os
from decouple import config

# Manually load the key from .env
api_key = "AIzaSyBNdVKPJG0ODhg3kOgYX-vDT_4WUuN-KXU"
genai.configure(api_key=api_key)

try:
    model = genai.GenerativeModel('gemini-1.5-flash')
    response = model.generate_content("Say hello")
    print(f"Success: {response.text}")
except Exception as e:
    print(f"Error: {e}")
