from typing import Optional, List, Dict
import os
from openai import AsyncOpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import hashlib

class RAGChatbot:
    def __init__(self):
        print("Initializing RAG Chatbot...")
        
        # Check environment variables
        openai_key = os.getenv("OPENAI_API_KEY")
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_key = os.getenv("QDRANT_API_KEY")
        
        if not openai_key:
            raise ValueError("OPENAI_API_KEY not found in environment")
        if not qdrant_url:
            raise ValueError("QDRANT_URL not found in environment")
        if not qdrant_key:
            raise ValueError("QDRANT_API_KEY not found in environment")
            
        print(f"OpenAI API Key: {openai_key[:20]}...")
        print(f"Qdrant URL: {qdrant_url}")
        print(f"Qdrant API Key: {qdrant_key[:20]}...")
        
        self.openai_client = AsyncOpenAI(api_key=openai_key)
        
        try:
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_key,
                timeout=10
            )
            print("Qdrant client created successfully")
        except Exception as e:
            print(f"Error creating Qdrant client: {e}")
            raise
            
        self.collection_name = "physical_ai_book"
        self._ensure_collection()
        print("RAG Chatbot initialized successfully!")
    
    def _ensure_collection(self):
        """Ensure Qdrant collection exists"""
        try:
            self.qdrant_client.get_collection(self.collection_name)
        except Exception as e:
            print(f"Warning: Could not connect to Qdrant: {e}")
            try:
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
                )
            except Exception as create_error:
                print(f"Warning: Could not create collection: {create_error}")
    
    async def embed_text(self, text: str) -> List[float]:
        """Generate embeddings using OpenAI"""
        response = await self.openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=text
        )
        return response.data[0].embedding
    
    async def index_content(self, content: str, metadata: dict):
        """Index book content into Qdrant"""
        embedding = await self.embed_text(content)
        
        point_id = hashlib.md5(content.encode()).hexdigest()
        
        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=[
                PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "content": content,
                        **metadata
                    }
                )
            ]
        )
    
    async def search_similar(self, query: str, limit: int = 5) -> List[dict]:
        """Search for similar content in Qdrant"""
        query_embedding = await self.embed_text(query)
        
        results = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )
        
        return [
            {
                "content": hit.payload["content"],
                "score": hit.score,
                "metadata": {k: v for k, v in hit.payload.items() if k != "content"}
            }
            for hit in results
        ]
    
    async def get_response(
        self,
        message: str,
        selected_text: Optional[str] = None,
        chapter: Optional[str] = None,
        user_profile: Optional[dict] = None
    ) -> dict:
        """Generate RAG response"""
        
        # If user selected text, prioritize that
        if selected_text:
            context = selected_text
            sources = [{"type": "selected_text", "content": selected_text[:200]}]
        else:
            # Search for relevant content
            search_results = await self.search_similar(message)
            context = "\n\n".join([r["content"] for r in search_results[:3]])
            sources = [
                {
                    "type": "book_content",
                    "chapter": r["metadata"].get("chapter", "Unknown"),
                    "score": r["score"]
                }
                for r in search_results[:3]
            ]
        
        # Build system prompt with user context
        system_prompt = """You are an expert AI assistant for the Physical AI & Humanoid Robotics course.
        Answer questions based on the provided context from the book. Be helpful, accurate, and concise."""
        
        if user_profile:
            system_prompt += f"""
            
            User Background:
            - Software: {user_profile.get('software_background', 'Not specified')}
            - Hardware: {user_profile.get('hardware_background', 'Not specified')}
            - Programming: {user_profile.get('programming_experience', 'Not specified')}
            - Robotics: {user_profile.get('robotics_experience', 'Not specified')}
            
            Tailor your response to their experience level.
            """
        
        # Generate response
        response = await self.openai_client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {message}"}
            ],
            temperature=0.7,
            max_tokens=1000
        )
        
        return {
            "response": response.choices[0].message.content,
            "sources": sources
        }
    
    async def personalize_content(self, content: str, user_background: dict) -> str:
        """Personalize content based on user background"""
        
        prompt = f"""Adapt the following technical content for a learner with this background:
        
        Software Background: {user_background.get('software_background', 'Not specified')}
        Hardware Background: {user_background.get('hardware_background', 'Not specified')}
        Programming Experience: {user_background.get('programming_experience', 'Not specified')}
        Robotics Experience: {user_background.get('robotics_experience', 'Not specified')}
        
        Original Content:
        {content}
        
        Provide a personalized version that:
        1. Adjusts technical depth to their level
        2. Adds relevant examples from their background
        3. Highlights connections to their experience
        4. Suggests prerequisites if needed
        
        Keep the same structure but make it more relevant to them.
        """
        
        response = await self.openai_client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.7,
            max_tokens=2000
        )
        
        return response.choices[0].message.content
    
    async def translate_content(self, content: str, target_language: str = "ur") -> str:
        """Translate content to target language (Urdu)"""
        
        language_names = {
            "ur": "Urdu (اردو)",
            "en": "English"
        }
        
        prompt = f"""Translate the following technical content to {language_names.get(target_language, target_language)}.
        
        Maintain:
        - Technical terms (keep English terms in parentheses)
        - Code snippets (don't translate)
        - Formatting and structure
        - Accuracy of technical concepts
        
        Content:
        {content}
        """
        
        response = await self.openai_client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.3,
            max_tokens=2000
        )
        
        return response.choices[0].message.content
