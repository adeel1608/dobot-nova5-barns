/**
 * Configuration Module
 * Centralizes all application configuration settings
 */

// API Configuration
export const API_CONFIG = {
  //  ---- PRODUCTION ----
  // API_BASE: '/api',
  // WEBSOCKET_BASE: '/ws',
  // VIDEO_STREAM: 'http://localhost:8001' // Video stream direct connection

  // ---- Development ----
  API_BASE: process.env.NODE_ENV === 'development' ? 'http://localhost:8000/api' : '/api',
  WEBSOCKET_BASE: process.env.NODE_ENV === 'development' ? 'ws://localhost:8000/ws' : '/ws',
  VIDEO_STREAM: (window.env && window.env.VIDEO_STREAM_URL) || 'http://localhost:30001'
};

// WebSocket Configuration
export const WS_CONFIG = {
  RECONNECT_INTERVAL: 5000,
  MAX_RECONNECT_ATTEMPTS: 10,
  HEARTBEAT_INTERVAL: 30000
};

// UI Configuration
export const UI_CONFIG = {
  // Refresh intervals (in milliseconds)
  HEALTH_CHECK_INTERVAL: 120000, // 2 minutes
  ORDER_REFRESH_INTERVAL: 30000,  // 30 seconds
  ALERT_REFRESH_INTERVAL: 60000,  // 1 minute

  // Timeouts
  API_TIMEOUT: 5000, // 5 seconds for faster failure

  // Pagination
  MAX_LOGS: 1000,
  ORDERS_PER_PAGE: 10,

  // Notifications
  NOTIFICATION_DURATION: 5000,

  // Drag and drop
  DRAG_THRESHOLD: 5
};

// Theme Configuration
export const THEME_CONFIG = {
  COLORS: {
    PRIMARY: '#004029',
    PRIMARY_LIGHT: '#00754a',
    PRIMARY_DARK: '#002817',
    SUCCESS: '#10b981',
    WARNING: '#f59e0b',
    ERROR: '#ef4444',
    INFO: '#3b82f6'
  },
  GRADIENTS: {
    HEADER: 'linear-gradient(135deg, #004029 0%, #00754a 50%, #008552 100%)',
    BACKGROUND: 'linear-gradient(135deg, #f0fdf4 0%, #ecfdf5 100%)'
  }
};

// Environment-specific settings
export const ENV_CONFIG = {
  isDevelopment: process.env.NODE_ENV === 'development',
  isProduction: process.env.NODE_ENV === 'production',

  // Feature flags
  FEATURES: {
    ENABLE_DEBUG_LOGS: process.env.NODE_ENV === 'development',
    ENABLE_MOCK_DATA: false,
    ENABLE_PERFORMANCE_MONITORING: true,
    ENABLE_ERROR_REPORTING: true
  }
}; 