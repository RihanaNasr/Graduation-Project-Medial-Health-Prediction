import OpenAI from "openai";
import dotenv from "dotenv";
dotenv.config();

const client = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });

async function check() {
  try {
    console.log("Testing OpenAI connection with model gpt-4o-mini...");
    const response = await client.chat.completions.create({
      model: "gpt-4o-mini",
      messages: [{ role: "user", content: "Hi" }],
    });
    console.log("✅ Success! Bot says:", response.choices[0].message.content);
  } catch (error) {
    console.error("❌ OpenAI Connection Failed!");
    console.error("Error Code:", error.code);
    console.error("Error Message:", error.message);
  }
}

check();
