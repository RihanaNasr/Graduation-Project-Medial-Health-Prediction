import express from "express";
import cors from "cors";
import dotenv from "dotenv";
import OpenAI from "openai";
import path from "path";
import { fileURLToPath } from "url";

dotenv.config();

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const app = express();
app.use(cors());
app.use(express.json());

// Serve static files (index.html)
app.use(express.static(__dirname));

const client = new OpenAI({
  apiKey: process.env.GROQ_API_KEY || process.env.OPENAI_API_KEY,
  baseURL: "https://api.groq.com/openai/v1"
});

// Initialize chat memory with a system prompt
const SYSTEM_PROMPT = {
  role: "system",
  content: `You are CardiGo, a specialized cardiovascular assistant. Your mission is to help patients understand heart health, explain medical terms related to cardiology, and provide guidance on heart-healthy lifestyles. Use a professional yet empathetic tone. 

DISCLAIMER: Always emphasize that you are an AI assistant and not a medical doctor. Users should always consult with a healthcare professional for diagnosis or treatment.`
};

let messages = [SYSTEM_PROMPT];

app.post("/chat", async (req, res) => {
  try {
    const userMessage = req.body.message;
    if (!userMessage) {
      return res.status(400).json({ error: "Message is required" });
    }

    // Keep history lean (Last 10 messages + System Prompt)
    if (messages.length > 11) {
      messages = [SYSTEM_PROMPT, ...messages.slice(-10)];
    }

    messages.push({ role: "user", content: userMessage });

    const response = await client.chat.completions.create({
      model: "llama-3.1-8b-instant",
      messages: messages,
      max_tokens: 1000,
      temperature: 0.7,
    });

    const botReply = response.choices[0].message.content;
    messages.push({ role: "assistant", content: botReply });

    res.json({ response: botReply });
  } catch (error) {
    console.error("OpenAI API Error:", error);
    
    // Fallback if rate limit or context length exceeded
    if (error.status === 413 || error.status === 429) {
      messages = [SYSTEM_PROMPT]; // Reset history on error
      return res.status(500).json({ 
        error: "System busy. Conversation history cleared to restore service. Please try again." 
      });
    }

    res.status(500).json({ error: "Failed to fetch response from AI Assistant." });
  }
});

const PORT = process.env.PORT || 5000;
app.listen(PORT, () => console.log(`🚀 CardiGo Assistant Server running on http://localhost:${PORT}`));
