import google.generativeai as genai
api_key = "AIzaSyBNdVKPJG0ODhg3kOgYX-vDT_4WUuN-KXU"
genai.configure(api_key=api_key)
try:
    models = [m.name for m in genai.list_models() if 'generateContent' in m.supported_generation_methods]
    print("AVAILABLE_MODELS:" + ",".join(models))
except Exception as e:
    print(f"Error: {e}")
