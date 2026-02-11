import React, { useState } from 'react';
import axios from 'axios';

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export default function PersonalizeButton({ content, onPersonalized }) {
  const [loading, setLoading] = useState(false);

  const handlePersonalize = async () => {
    setLoading(true);
    try {
      const token = localStorage.getItem('token');
      const user = JSON.parse(localStorage.getItem('user') || '{}');

      const response = await axios.post(
        `${API_URL}/api/personalize`,
        {
          content,
          user_background: {
            software_background: user.software_background,
            hardware_background: user.hardware_background,
            programming_experience: user.programming_experience,
            robotics_experience: user.robotics_experience,
          },
        },
        {
          headers: { Authorization: `Bearer ${token}` },
        }
      );

      onPersonalized(response.data.personalized_content);
    } catch (error) {
      console.error('Personalization error:', error);
      alert('Failed to personalize content. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const token = localStorage.getItem('token');
  if (!token) return null;

  return (
    <button
      className="personalize-button"
      onClick={handlePersonalize}
      disabled={loading}
    >
      {loading ? '⏳ Personalizing...' : '✨ Personalize for Me'}
    </button>
  );
}
