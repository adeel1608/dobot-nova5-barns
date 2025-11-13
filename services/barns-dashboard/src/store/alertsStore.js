/**
 * Alerts Store
 * Manages alert state and operations
 */

import { create } from 'zustand';
import { alertsAPI } from '../api';
import { addLog } from './logsStore';

export const useAlertsStore = create((set, get) => ({
  // State
  alerts: [],
  acknowledgedAlerts: [],
  isLoading: false,
  errors: {
    alerts: null,
    acknowledge: null
  },

  // Actions
  clearError: (component) => {
    if (component) {
      set(state => ({
        errors: {
          ...state.errors,
          [component]: null
        }
      }));
    } else {
      set({
        errors: {
          alerts: null,
          acknowledge: null
        }
      });
    }
  },

  // Fetch active alerts
  fetchAlerts: async (options = {}) => {
    const startTime = performance.now();
    console.log('[AlertsStore] Starting fetchAlerts...', options.noCache ? '(noCache)' : '');
    
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, alerts: null }
    }));

    const result = await alertsAPI.fetchActiveAlerts(options);
    const endTime = performance.now();
    const duration = (endTime - startTime).toFixed(2);
    
    console.log(`[AlertsStore] fetchAlerts completed in ${duration}ms`);
    
    if (result.success) {
      set(state => ({ 
        alerts: result.data, 
        isLoading: false
      }));
      console.log(`[AlertsStore] Updated state with ${result.data?.length || 0} alerts`);
      // Removed verbose INFO log - only log errors
    } else {
      set(state => ({ 
        alerts: [],
        isLoading: false,
        errors: { ...state.errors, alerts: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  },

  // Fetch acknowledged alerts
  fetchAcknowledgedAlerts: async () => {
    const result = await alertsAPI.fetchAcknowledgedAlerts();
    
    if (result.success) {
      set(state => ({ 
        acknowledgedAlerts: result.data
      }));
      // Removed verbose INFO log - only log errors
    } else {
      set(state => ({ 
        acknowledgedAlerts: [],
        errors: { ...state.errors, acknowledgedAlerts: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  },

  // Acknowledge an alert
  acknowledgeAlert: async (alertId) => {
    const result = await alertsAPI.acknowledgeAlert(alertId);
    
    if (result.success) {
      // Removed verbose INFO log - only log errors
      
      // Remove the alert from active alerts immediately for better UX
      set(state => ({
        alerts: state.alerts.filter(alert => alert.id !== alertId)
      }));
      
      // Refresh both active and acknowledged alerts to sync with server
      await get().fetchAlerts();
      await get().fetchAcknowledgedAlerts();
      
      return true;
    } else {
      addLog('API', 'error', result.error, result.details);
      set(state => ({
        errors: { ...state.errors, acknowledge: result.error }
      }));
      return false;
    }
  },

  // Create a new alert (admin function)
  createAlert: async (alertData) => {
    const result = await alertsAPI.createAlert(alertData);
    
    if (result.success) {
      // Removed verbose INFO log - only log errors
      await get().fetchAlerts(); // Refresh alerts
    } else {
      addLog('API', 'error', result.error, result.details);
    }

    return result.success;
  },

  // Get alerts by severity
  getAlertsBySeverity: (severity) => {
    return get().alerts.filter(alert => alert.severity === severity);
  },

  // Get alert count by type
  getAlertStats: () => {
    const alerts = get().alerts;
    return {
      total: alerts.length,
      critical: alerts.filter(a => a.severity === 'critical').length,
      warning: alerts.filter(a => a.severity === 'warning').length,
      info: alerts.filter(a => a.severity === 'info').length
    };
  }
})); 