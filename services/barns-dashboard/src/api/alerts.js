/**
 * Alerts API - Optimized  
 * Uses base API client for DRY code
 */

import { apiClient } from './base';

export const alertsAPI = {
  // Fetch active alerts
  fetchActiveAlerts: (options = {}) =>
    apiClient.getList('/alerts/active', {}, { 
      itemName: 'active alerts',
      ...options // Allow passing noCache option to bypass cache
    })
      .then(result => ({
        ...result,
        data: result.data?.alerts || result.data || []
      })),

  // Fetch acknowledged alerts
  fetchAcknowledgedAlerts: () =>
    apiClient.getList('/alerts/acknowledged', {}, 'acknowledged alerts')
      .then(result => ({
        ...result,
        data: result.data?.alerts || result.data || []
      })),

  // Acknowledge alert
  acknowledgeAlert: (alertId) =>
    apiClient.post(`/alerts/${alertId}/acknowledge`, {}, {
      successMessage: `Alert ${alertId} acknowledged successfully`
    }).then(result => {
      // Handle special case where API returns success flag in data
      if (result.success && result.data?.success === false) {
        return {
          success: false,
          error: result.data.error || 'Failed to acknowledge alert',
          details: {
            operation: 'acknowledge_alert',
            alert_id: alertId
          }
        };
      }
      return result;
    }),

  // Create new alert
  createAlert: (alertData) =>
    apiClient.create('/alerts', alertData, 'Alert')
}; 