import React, { useState } from 'react';
import axios from 'axios';

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export default function TranslateButton({ content, onTranslated }) {
  const [loading, setLoading] = useState(false);
  const [isUrdu, setIsUrdu] = useState(false);

  const handleTranslate = async () => {
    setLoading(true);
    try {
      const targetLang = isUrdu ? 'en' : 'ur';
      
      const response = await axios.post(
        `${API_URL}/api/translate`,
        {
          content,
          target_language: targetLang,
        }
      );

      onTranslated(response.data.translated_content);
      setIsUrdu(!isUrdu);
    } catch (error) {
      console.error('Translation error:', error);
      alert('Failed to translate content. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <button
      className="translate-button"
      onClick={handleTranslate}
      disabled={loading}
    >
      {loading ? '⏳ Translating...' : isUrdu ? '🇬🇧 English' : '🇵🇰 اردو Urdu'}
    </button>
  );
}
