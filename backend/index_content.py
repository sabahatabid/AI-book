"""
Script to index book content into Qdrant vector database
Run this after deployment to populate the RAG system
"""

import asyncio
import os
from pathlib import Path
from rag import RAGChatbot
from dotenv import load_dotenv

load_dotenv()

async def index_markdown_files():
    """Index all markdown files from docs directory"""
    
    chatbot = RAGChatbot()
    docs_dir = Path("../docs")
    
    if not docs_dir.exists():
        print("Error: docs directory not found")
        return
    
    # Find all markdown files
    md_files = list(docs_dir.rglob("*.md"))
    print(f"Found {len(md_files)} markdown files")
    
    for md_file in md_files:
        print(f"\nProcessing: {md_file}")
        
        # Read content
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract metadata from path
        relative_path = md_file.relative_to(docs_dir)
        parts = relative_path.parts
        
        if len(parts) > 1:
            module = parts[0]
            chapter = parts[-1].replace('.md', '')
        else:
            module = "intro"
            chapter = parts[0].replace('.md', '')
        
        # Split content into chunks (by sections)
        sections = content.split('\n## ')
        
        for i, section in enumerate(sections):
            if not section.strip():
                continue
            
            # Add heading back
            if i > 0:
                section = '## ' + section
            
            # Skip very short sections
            if len(section) < 100:
                continue
            
            # Index the section
            metadata = {
                "module": module,
                "chapter": chapter,
                "section": i,
                "file": str(relative_path)
            }
            
            try:
                await chatbot.index_content(section, metadata)
                print(f"  ✓ Indexed section {i} ({len(section)} chars)")
            except Exception as e:
                print(f"  ✗ Error indexing section {i}: {e}")
    
    print("\n✅ Indexing complete!")
    print(f"Total files processed: {len(md_files)}")

async def test_search():
    """Test the search functionality"""
    chatbot = RAGChatbot()
    
    test_queries = [
        "What is ROS 2?",
        "How do I create a URDF file?",
        "Explain Gazebo simulation",
        "What is NVIDIA Isaac?"
    ]
    
    print("\n🔍 Testing search functionality...\n")
    
    for query in test_queries:
        print(f"Query: {query}")
        results = await chatbot.search_similar(query, limit=3)
        
        if results:
            print(f"  Found {len(results)} results:")
            for r in results:
                print(f"    - {r['metadata']['chapter']} (score: {r['score']:.3f})")
        else:
            print("  No results found")
        print()

async def main():
    print("=" * 60)
    print("Physical AI Book - Content Indexing")
    print("=" * 60)
    
    # Check environment variables
    required_vars = ["OPENAI_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    missing = [var for var in required_vars if not os.getenv(var)]
    
    if missing:
        print(f"\n❌ Error: Missing environment variables: {', '.join(missing)}")
        print("Please set them in backend/.env")
        return
    
    print("\n✓ Environment variables configured")
    
    # Index content
    await index_markdown_files()
    
    # Test search
    await test_search()
    
    print("\n" + "=" * 60)
    print("✅ All done! Your RAG system is ready.")
    print("=" * 60)

if __name__ == "__main__":
    asyncio.run(main())
