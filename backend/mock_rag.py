from typing import Optional, List, Dict

class MockRAGChatbot:
    """Mock chatbot for testing without OpenAI API"""
    
    def __init__(self):
        print("✓ Mock RAG Chatbot initialized (no API calls)")
        self.responses = {
            "hi": "Hello! I'm your AI assistant for Physical AI & Humanoid Robotics. How can I help you today?",
            "hello": "Hi there! Welcome to the Physical AI course. What would you like to learn about?",
            "ros": "ROS 2 (Robot Operating System 2) is a middleware framework for robotics. It provides tools and libraries for building robot applications.",
            "gazebo": "Gazebo is a 3D robot simulator that allows you to test robots in realistic environments before deploying to real hardware.",
            "unity": "Unity is a game engine used for high-fidelity rendering and visualization of robotic simulations.",
            "isaac": "NVIDIA Isaac is a platform for AI-powered robotics, including simulation, perception, and navigation tools.",
            "urdf": "URDF (Unified Robot Description Format) is an XML format for representing robot models in ROS.",
        }
    
    async def get_response(
        self,
        message: str,
        selected_text: Optional[str] = None,
        chapter: Optional[str] = None,
        user_profile: Optional[dict] = None
    ) -> dict:
        """Generate mock response"""
        
        message_lower = message.lower()
        
        # Check for keywords
        response = None
        for keyword, answer in self.responses.items():
            if keyword in message_lower:
                response = answer
                break
        
        if not response:
            response = f"""I received your question: "{message}"

I'm a demo chatbot running without OpenAI API. Here's what I can help you with:

📚 **Course Topics:**
- ROS 2 fundamentals and architecture
- Gazebo simulation and physics
- Unity rendering and visualization  
- NVIDIA Isaac platform
- URDF robot modeling
- Vision-Language-Action systems

💡 **Try asking about:**
- "What is ROS 2?"
- "Tell me about Gazebo"
- "Explain URDF"
- "What is NVIDIA Isaac?"

Note: To get full AI-powered responses, please add OpenAI API credits to your account."""
        
        return {
            "response": response,
            "sources": [{"type": "mock", "note": "Demo response (OpenAI API not available)"}]
        }
    
    async def personalize_content(self, content: str, user_background: dict) -> str:
        """Mock personalization"""
        return f"""[Personalized Content - Demo Mode]

{content}

---
Note: Full personalization requires OpenAI API credits. This is a demo response."""
    
    async def translate_content(self, content: str, target_language: str = "ur") -> str:
        """Mock translation"""
        if target_language == "ur":
            return f"""[اردو ترجمہ - ڈیمو موڈ]

{content}

---
نوٹ: مکمل ترجمہ کے لیے OpenAI API کریڈٹس درکار ہیں۔"""
        return content
