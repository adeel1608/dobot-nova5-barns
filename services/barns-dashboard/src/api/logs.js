/**
 * Logs API - Optimized
 * Uses base API client for DRY code
 */

import { apiClient } from './base';

export const logsAPI = {
  // Fetch system logs with filters
  fetchSystemLogs: (filters = {}) => {
    const params = new URLSearchParams(filters);
    return apiClient.getList(`/logs?${params}`, {}, 'log entries')
      .then(result => ({
        ...result,
        data: result.data?.logs || result.data || []
      }));
  },

  // Clear system logs
  clearLogs: () =>
    apiClient.delete('/logs', {
      successMessage: 'System logs cleared successfully'
    }),

  // Export logs
  exportLogs: (format = 'csv') =>
    apiClient.get('/logs/export', { format }, {
      responseType: 'blob',
      successMessage: `Logs exported successfully as ${format}`
    })
}; 