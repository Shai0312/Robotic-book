// Chatbot Integration Script for Docusaurus Book Site
// This script should be injected into the Docusaurus site to enable the RAG chatbot

(function() {
  'use strict';

  // Configuration
  const CONFIG = {
    apiUrl: '/api', // Base API URL - adjust as needed for your deployment
    botName: 'Book Assistant',
    initialMessage: 'Hello! I\'m your book assistant. Ask me anything about the content you\'re reading!'
  };

  // Chatbot state
  let chatbotState = {
    isOpen: false,
    messages: [],
    isLoading: false,
    selectedText: null
  };

  // DOM elements
  let chatbotContainer = null;
  let chatbotButton = null;
  let chatbotPanel = null;
  let messagesContainer = null;
  let inputForm = null;
  let inputField = null;

  // Initialize the chatbot
  function initChatbot() {
    createChatbotUI();
    setupEventListeners();
    loadInitialState();
  }

  // Create the chatbot UI elements
  function createChatbotUI() {
    // Create main container
    chatbotContainer = document.createElement('div');
    chatbotContainer.className = 'chatbot-container';
    chatbotContainer.style.cssText = `
      position: fixed;
      bottom: 20px;
      right: 20px;
      z-index: 10000;
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    `;

    // Create floating button
    chatbotButton = document.createElement('button');
    chatbotButton.className = 'chatbot-button';
    chatbotButton.innerHTML = `
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.35L2 22L7.65 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2Z" fill="white"/>
        <path d="M9 12C9 11.45 9.45 11 10 11H14C14.55 11 15 11.45 15 12C15 12.55 14.55 13 14 13H10C9.45 13 9 12.55 9 12ZM10 9C10 8.45 10.45 8 11 8H13C13.55 8 14 8.45 14 9C14 9.55 13.55 10 13 10H11C10.45 10 10 9.55 10 9Z" fill="#4F46E5"/>
      </svg>
    `;
    chatbotButton.style.cssText = `
      width: 60px;
      height: 60px;
      border-radius: 50%;
      background-color: #4F46E5;
      color: white;
      border: none;
      cursor: pointer;
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
      display: flex;
      align-items: center;
      justify-content: center;
      transition: all 0.3s ease;
    `;

    // Create chat panel
    chatbotPanel = document.createElement('div');
    chatbotPanel.className = 'chatbot-panel';
    chatbotPanel.style.cssText = `
      width: 380px;
      height: 500px;
      background: white;
      border-radius: 12px;
      box-shadow: 0 10px 25px rgba(0, 0, 0, 0.15);
      display: none;
      flex-direction: column;
      overflow: hidden;
      animation: slideIn 0.3s ease;
    `;

    // Create panel content
    chatbotPanel.innerHTML = `
      <div class="chatbot-header" style="
        background: #4F46E5;
        color: white;
        padding: 16px;
        display: flex;
        justify-content: space-between;
        align-items: center;
      ">
        <h3 style="margin: 0; font-size: 16px; font-weight: 600;">Book Assistant</h3>
        <button class="close-button" style="
          background: none;
          border: none;
          color: white;
          font-size: 24px;
          cursor: pointer;
          width: 30px;
          height: 30px;
          display: flex;
          align-items: center;
          justify-content: center;
          border-radius: 50%;
        ">×</button>
      </div>

      <div class="selected-text-notice" style="
        background: #FEF3C7;
        border-left: 4px solid #F59E0B;
        padding: 12px;
        margin: 8px;
        font-size: 12px;
        display: none;
      ">
        <p><strong>Selected text:</strong> "<span id="selected-text-content"></span>"</p>
        <button id="ask-about-selection-btn" style="
          background: #F59E0B;
          color: white;
          border: none;
          padding: 6px 12px;
          border-radius: 6px;
          font-size: 12px;
          cursor: pointer;
        ">Ask about this</button>
      </div>

      <div class="chatbot-messages" style="
        flex: 1;
        overflow-y: auto;
        padding: 16px;
        display: flex;
        flex-direction: column;
        gap: 12px;
        background: #F9FAFB;
      ">
        <div class="message bot" style="
          background: white;
          color: #374151;
          align-self: flex-start;
          border-bottom-left-radius: 4px;
          box-shadow: 0 1px 2px rgba(0, 0, 0, 0.05);
          max-width: 80%;
          padding: 10px 14px;
          border-radius: 18px;
          font-size: 14px;
          line-height: 1.4;
        ">
          <div class="message-content" style="margin: 0;">
            ${CONFIG.initialMessage}
          </div>
        </div>
      </div>

      <form class="chatbot-input-form" style="
        display: flex;
        padding: 12px;
        background: white;
        border-top: 1px solid #E5E7EB;
      ">
        <input type="text" placeholder="Ask about the book content..." style="
          flex: 1;
          padding: 10px 12px;
          border: 1px solid #D1D5DB;
          border-radius: 24px;
          font-size: 14px;
          outline: none;
        " id="chatbot-input">
        <button type="submit" style="
          margin-left: 8px;
          padding: 10px 16px;
          background: #4F46E5;
          color: white;
          border: none;
          border-radius: 24px;
          cursor: pointer;
          font-size: 14px;
          font-weight: 500;
        ">Send</button>
      </form>
    `;

    // Add to DOM
    chatbotContainer.appendChild(chatbotButton);
    chatbotContainer.appendChild(chatbotPanel);
    document.body.appendChild(chatbotContainer);

    // Store references to form elements
    messagesContainer = chatbotPanel.querySelector('.chatbot-messages');
    inputForm = chatbotPanel.querySelector('.chatbot-input-form');
    inputField = chatbotPanel.querySelector('#chatbot-input');
  }

  // Set up event listeners
  function setupEventListeners() {
    // Toggle chatbot
    chatbotButton.addEventListener('click', toggleChatbot);

    // Close button
    chatbotPanel.querySelector('.close-button').addEventListener('click', toggleChatbot);

    // Form submission
    inputForm.addEventListener('submit', handleFormSubmit);

    // Text selection
    document.addEventListener('mouseup', handleTextSelection);

    // Ask about selection button
    chatbotPanel.querySelector('#ask-about-selection-btn').addEventListener('click', handleAskAboutSelection);
  }

  // Load initial state
  function loadInitialState() {
    // Add initial message
    addMessage(CONFIG.initialMessage, 'bot');
  }

  // Toggle chatbot visibility
  function toggleChatbot() {
    chatbotState.isOpen = !chatbotState.isOpen;

    if (chatbotState.isOpen) {
      chatbotButton.style.display = 'none';
      chatbotPanel.style.display = 'flex';
    } else {
      chatbotButton.style.display = 'flex';
      chatbotPanel.style.display = 'none';
    }
  }

  // Handle text selection
  function handleTextSelection() {
    const selection = window.getSelection();
    const selectedText = selection.toString().trim();

    if (selectedText.length > 10) { // Only consider substantial selections
      chatbotState.selectedText = selectedText;

      // Show the selected text notice
      const notice = chatbotPanel.querySelector('.selected-text-notice');
      const contentSpan = chatbotPanel.querySelector('#selected-text-content');

      contentSpan.textContent = selectedText.substring(0, 60) + (selectedText.length > 60 ? '...' : '');
      notice.style.display = 'block';
    } else {
      chatbotState.selectedText = null;
      chatbotPanel.querySelector('.selected-text-notice').style.display = 'none';
    }
  }

  // Handle asking about selected text
  async function handleAskAboutSelection() {
    if (!chatbotState.selectedText) return;

    const notice = chatbotPanel.querySelector('.selected-text-notice');
    const button = chatbotPanel.querySelector('#ask-about-selection-btn');

    button.textContent = 'Asking...';
    button.disabled = true;

    try {
      addMessage(`Asking about: "${chatbotState.selectedText.substring(0, 50)}..."`, 'user');

      const response = await fetch(\`\${CONFIG.apiUrl}/query-selected-text\`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          selected_text: chatbotState.selectedText,
          query: \`Explain this selected text: \${chatbotState.selectedText}\`
        })
      });

      const data = await response.json();
      addMessage(data.answer, 'bot');

      // Clear selection and hide notice
      window.getSelection().removeAllRanges();
      notice.style.display = 'none';
      chatbotState.selectedText = null;
    } catch (error) {
      addMessage('Sorry, I encountered an error processing your request.', 'bot');
    } finally {
      button.textContent = 'Ask about this';
      button.disabled = false;
    }
  }

  // Handle form submission
  async function handleFormSubmit(e) {
    e.preventDefault();

    const query = inputField.value.trim();
    if (!query || chatbotState.isLoading) return;

    addMessage(query, 'user');
    inputField.value = '';
    chatbotState.isLoading = true;

    try {
      // Get current page context
      const currentUrl = window.location.href;
      const currentTitle = document.title;

      const requestBody = {
        query: query,
        top_k: 5,
        source_url: currentUrl || null  // Filter by current page if available
      };

      const response = await fetch(\`\${CONFIG.apiUrl}/query\`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      const data = await response.json();
      addMessage(data.answer, 'bot');
    } catch (error) {
      addMessage('Sorry, I encountered an error processing your request.', 'bot');
    } finally {
      chatbotState.isLoading = false;
    }
  }

  // Add a message to the chat
  function addMessage(content, type) {
    const messageDiv = document.createElement('div');
    messageDiv.className = \`message \${type}\`;
    messageDiv.style.cssText = \`
      max-width: 80%;
      padding: 10px 14px;
      border-radius: 18px;
      font-size: 14px;
      line-height: 1.4;
    \`;

    if (type === 'user') {
      messageDiv.style.cssText += \`
        background: #4F46E5;
        color: white;
        align-self: flex-end;
        border-bottom-right-radius: 4px;
      \`;
    } else {
      messageDiv.style.cssText += \`
        background: white;
        color: #374151;
        align-self: flex-start;
        border-bottom-left-radius: 4px;
        box-shadow: 0 1px 2px rgba(0, 0, 0, 0.05);
      \`;
    }

    messageDiv.innerHTML = \`<div class="message-content" style="margin: 0;">\${content}</div>\`;
    messagesContainer.appendChild(messageDiv);

    // Scroll to bottom
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  // Initialize when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initChatbot);
  } else {
    initChatbot();
  }

})();