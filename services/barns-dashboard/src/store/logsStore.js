/**
 * Logs Store
 * Centralized logging system for the entire application
 */

import { create } from 'zustand';
import { UI_CONFIG } from '../utils/config';

export const useLogsStore = create((set, get) => ({
  // State
  systemLogs: [],
  
  // Actions
  addLog: (service, level, message, details = null) => {
    const logEntry = {
      id: Date.now() + Math.random(), // Ensure unique IDs
      timestamp: new Date().toISOString(),
      service,
      level, // 'info', 'warning', 'error', 'success'
      message,
      details
    };
    
    set(state => ({
      systemLogs: [logEntry, ...state.systemLogs].slice(0, UI_CONFIG.MAX_LOGS)
    }));

    // Also log to console in development
    if (process.env.NODE_ENV === 'development') {
      const logMethod = level === 'error' ? 'error' : level === 'warning' ? 'warn' : 'log';
      console[logMethod](`[${service}] ${message}`, details);
    }
  },

  clearLogs: () => {
    set({ systemLogs: [] });
  },

  // Filter logs by service
  getLogsByService: (service) => {
    return get().systemLogs.filter(log => log.service === service);
  },

  // Filter logs by level
  getLogsByLevel: (level) => {
    return get().systemLogs.filter(log => log.level === level);
  },

  // Get recent logs (last N entries)
  getRecentLogs: (count = 50) => {
    return get().systemLogs.slice(0, count);
  },

  // Get logs within a time range
  getLogsByTimeRange: (startTime, endTime) => {
    return get().systemLogs.filter(log => {
      const logTime = new Date(log.timestamp);
      return logTime >= startTime && logTime <= endTime;
    });
  }
}));

// Export addLog function for use in other stores
export const addLog = (service, level, message, details) => {
  useLogsStore.getState().addLog(service, level, message, details);
}; 