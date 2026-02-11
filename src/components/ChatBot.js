import React, { useState, useEffect, useRef } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const API_URL = 'http://localhost:8000';

export default function ChatBot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const messagesEndRef = useRef(null);

  // Only run on client side
  if (!ExecutionEnvironment.canUseDOM) {
    return null;
  }

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await fetch(`${API_URL}/api/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ message: input }),
      });

      if (!response.ok) throw new Error('API Error');

      const data = await response.json();
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: data.response || 'I received your message!',
        sources: data.sources || [],
      }]);
    } catch (error) {
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: '⚠️ Backend not available. Please start the backend server.',
      }]);
    } finally {
      setLoading(false);
    }
  };

  if (!isOpen) {
    return (
      <button
        onClick={() => setIsOpen(true)}
        style={{
          position: 'fixed',
          bottom: '30px',
          right: '30px',
          width: '65px',
          height: '65px',
          borderRadius: '50%',
          background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
          border: 'none',
          cursor: 'pointer',
          boxShadow: '0 8px 24px rgba(102, 126, 234, 0.4)',
          fontSize: '28px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          zIndex: 9999,
          transition: 'all 0.3s ease',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.transform = 'scale(1.1)';
          e.currentTarget.style.boxShadow = '0 12px 32px rgba(102, 126, 234, 0.5)';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'scale(1)';
          e.currentTarget.style.boxShadow = '0 8px 24px rgba(102, 126, 234, 0.4)';
        }}
      >
        💬
      </button>
    );
  }

  return (
    <div style={{
      position: 'fixed',
      bottom: '30px',
      right: '30px',
      width: '420px',
      height: '650px',
      background: 'white',
      borderRadius: '16px',
      boxShadow: '0 12px 48px rgba(0, 0, 0, 0.2)',
      display: 'flex',
      flexDirection: 'column',
      zIndex: 9999,
      overflow: 'hidden',
    }}>
      {/* Header */}
      <div style={{
        padding: '20px',
        background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
        color: 'white',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
      }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
          <div style={{
            width: '40px',
            height: '40px',
            borderRadius: '50%',
            background: 'white',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            fontSize: '20px',
          }}>
            🤖
          </div>
          <div>
            <h3 style={{ margin: 0, fontSize: '18px', fontWeight: '700' }}>AI Assistant</h3>
            <p style={{ margin: 0, fontSize: '12px', opacity: 0.9 }}>Physical AI Expert</p>
          </div>
        </div>
        <button
          onClick={() => setIsOpen(false)}
          style={{
            background: 'rgba(255,255,255,0.2)',
            border: 'none',
            color: 'white',
            width: '32px',
            height: '32px',
            borderRadius: '50%',
            cursor: 'pointer',
            fontSize: '20px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            transition: 'background 0.2s',
          }}
          onMouseEnter={(e) => e.currentTarget.style.background = 'rgba(255,255,255,0.3)'}
          onMouseLeave={(e) => e.currentTarget.style.background = 'rgba(255,255,255,0.2)'}
        >
          ×
        </button>
      </div>

      {/* Messages */}
      <div style={{
        flex: 1,
        overflowY: 'auto',
        padding: '20px',
        background: '#f8f9fa',
        display: 'flex',
        flexDirection: 'column',
        gap: '12px',
      }}>
        {messages.length === 0 && (
          <div style={{ textAlign: 'center', padding: '40px 20px', color: '#666' }}>
            <div style={{ fontSize: '48px', marginBottom: '16px' }}>👋</div>
            <h4 style={{ margin: '0 0 8px 0', fontSize: '18px' }}>Hi! I'm your AI assistant</h4>
            <p style={{ margin: '0 0 20px 0', fontSize: '14px' }}>
              Ask me anything about Physical AI & Humanoid Robotics!
            </p>
            <div style={{ fontSize: '12px', opacity: 0.7 }}>
              💡 Try: "What is ROS 2?" or "Explain URDF"
            </div>
          </div>
        )}

        {messages.map((msg, idx) => (
          <div
            key={idx}
            style={{
              alignSelf: msg.role === 'user' ? 'flex-end' : 'flex-start',
              maxWidth: '75%',
              padding: '12px 16px',
              borderRadius: '12px',
              background: msg.role === 'user'
                ? 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)'
                : 'white',
              color: msg.role === 'user' ? 'white' : '#333',
              boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
              fontSize: '14px',
              lineHeight: '1.5',
            }}
          >
            {msg.content}
            {msg.sources?.length > 0 && (
              <div style={{ fontSize: '11px', marginTop: '8px', opacity: 0.8 }}>
                📚 {msg.sources.map(s => s.chapter).join(', ')}
              </div>
            )}
          </div>
        ))}

        {loading && (
          <div style={{ alignSelf: 'flex-start', padding: '12px 16px', background: 'white', borderRadius: '12px', boxShadow: '0 2px 8px rgba(0,0,0,0.1)' }}>
            <div style={{ display: 'flex', gap: '6px' }}>
              {[0, 1, 2].map(i => (
                <div
                  key={i}
                  style={{
                    width: '8px',
                    height: '8px',
                    borderRadius: '50%',
                    background: '#667eea',
                    animation: `pulse 1.4s infinite ease-in-out ${i * 0.2}s`,
                  }}
                />
              ))}
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div style={{
        padding: '16px',
        background: 'white',
        borderTop: '1px solid #e0e0e0',
        display: 'flex',
        gap: '12px',
      }}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={(e) => e.key === 'Enter' && !e.shiftKey && sendMessage()}
          placeholder="Type your question..."
          style={{
            flex: 1,
            padding: '12px 16px',
            borderRadius: '24px',
            border: '2px solid #e0e0e0',
            fontSize: '14px',
            outline: 'none',
            transition: 'border-color 0.2s',
          }}
          onFocus={(e) => e.currentTarget.style.borderColor = '#667eea'}
          onBlur={(e) => e.currentTarget.style.borderColor = '#e0e0e0'}
        />
        <button
          onClick={sendMessage}
          disabled={!input.trim() || loading}
          style={{
            padding: '12px 24px',
            borderRadius: '24px',
            border: 'none',
            background: input.trim() && !loading
              ? 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)'
              : '#ccc',
            color: 'white',
            cursor: input.trim() && !loading ? 'pointer' : 'not-allowed',
            fontWeight: '600',
            fontSize: '14px',
            transition: 'all 0.2s',
          }}
        >
          Send
        </button>
      </div>

      <style>{`
        @keyframes pulse {
          0%, 100% { opacity: 0.4; transform: scale(0.8); }
          50% { opacity: 1; transform: scale(1); }
        }
      `}</style>
    </div>
  );
}
