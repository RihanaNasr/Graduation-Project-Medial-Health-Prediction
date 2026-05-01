import { useState, useEffect } from 'react';
import { 
  MessageSquare, User, Bot, Search, Clock, 
  Hash, Settings, MoreVertical, Paperclip, Send, Image
} from 'lucide-react';
import api from '../services/api';
import './ChatMonitorPage.css';

const categoryMappings = {
  'General Support': 'general',
  'Emergency Desk': 'emergency',
  'Medical Consultation': 'medical'
};

export default function ChatMonitorPage() {
  const [activeChannel, setActiveChannel] = useState('General Support');
  const [allMessages, setAllMessages] = useState([]);
  const [search, setSearch] = useState('');
  const [loading, setLoading] = useState(true);
  const [inputMessage, setInputMessage] = useState("");

  const fetchMessages = async () => {
    setLoading(true);
    try {
      const data = await api.getChatMessages();
      if (data) setAllMessages(data);
    } catch (err) {
      console.error("Failed to load chats:", err);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchMessages();
    // Auto-refresh every 5 seconds to sync with the mobile app
    const interval = setInterval(fetchMessages, 5000);
    return () => clearInterval(interval);
  }, []);

  // Filter messages based on search and logical "category"
  const filteredMessages = allMessages.filter(msg => {
    const matchesSearch = (msg.message + (msg.user_name || '')).toLowerCase().includes(search.toLowerCase());
    
    // Categorize based on content keywords
    const text = (msg.message || '').toLowerCase();
    const isEmergency = text.includes('help') || text.includes('emergency') || text.includes('sos') || text.includes('bleed') || text.includes('pain');
    const isMedical = text.includes('doctor') || text.includes('medicine') || text.includes('consult') || text.includes('symptom') || text.includes('vitals') || text.includes('heart');

    if (activeChannel === 'Emergency Desk') return matchesSearch && isEmergency;
    if (activeChannel === 'Medical Consultation') return matchesSearch && isMedical;
    
    // General Support shows everything else or all
    return matchesSearch;
  });

  const handleSendMessage = () => {
    if (!inputMessage.trim()) return;
    const newMsg = {
      id: Date.now(),
      user_name: "Admin",
      message: inputMessage,
      timestamp: new Date().toISOString(),
      userId: 'admin'
    };
    setAllMessages([...allMessages, newMsg]);
    setInputMessage("");
  };

  return (
    <div className="chat-monitor-page animate-fade-in">
      <div className="page-header">
        <h1>Global Chat Monitor</h1>
        <p>Monitor real-time interactions and maintain community standards.</p>
      </div>

      <div className="chat-container glass-card">
        <div className="chat-layout">
          {/* Channels Sidebar */}
          <div className="chat-sidebar">
            <div className="search-box">
              <Search size={16} />
              <input 
                type="text" 
                placeholder="Search conversations..." 
                value={search}
                onChange={(e) => setSearch(e.target.value)}
              />
            </div>
            <div className="channels-list">
              {['General Support', 'Emergency Desk', 'Medical Consultation'].map(channel => (
                <div 
                  key={channel}
                  className={`channel-item ${activeChannel === channel ? 'active' : ''}`}
                  onClick={() => setActiveChannel(channel)}
                >
                  <Hash size={18} />
                  <span>{channel}</span>
                  {channel === 'General Support' && <span className="unread-count">!</span>}
                </div>
              ))}
            </div>
          </div>

          {/* Main Chat Area */}
          <div className="chat-main">
            <div className="chat-header">
              <div className="chat-info">
                <Hash size={20} />
                <h3>{activeChannel}</h3>
              </div>
              <div className="chat-actions">
                <button className="btn-icon" title="Channel Settings"><Settings size={18} /></button>
                <button className="btn-icon" title="More Options"><MoreVertical size={18} /></button>
              </div>
            </div>

            <div className="chat-messages">
              {loading ? (
                <div className="chat-loading">Loading conversations...</div>
              ) : filteredMessages.length === 0 ? (
                <div className="chat-empty">No messages found for this channel.</div>
              ) : (
                filteredMessages.map((msg) => (
                  <div key={msg.id} className={`message-wrapper ${msg.userId === 'bot' || !msg.userId ? 'bot-message' : ''}`}>
                    <div className="message-avatar-box">
                      {msg.user_name?.charAt(0) || 'A'}
                    </div>
                    <div className="message-content">
                      <div className="message-header">
                        <span className="message-username">{msg.user_name || 'System'}</span>
                        <span className="message-time">{new Date(msg.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}</span>
                      </div>
                      <div className="message-text">
                        {msg.message}
                        {msg.response && (
                          <div className="bot-response">
                            <Bot size={14} style={{ marginRight: '6px' }} />
                            {msg.response}
                          </div>
                        )}
                      </div>
                    </div>
                  </div>
                ))
              )}
            </div>

            <div className="chat-input-area">
              <button className="btn-icon"><Paperclip size={20} /></button>
              <input 
                type="text" 
                placeholder="Type a message..." 
                value={inputMessage}
                onChange={(e) => setInputMessage(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
              />
              <button className="btn-send" onClick={handleSendMessage} disabled={!inputMessage.trim()}>
                <Send size={20} />
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
