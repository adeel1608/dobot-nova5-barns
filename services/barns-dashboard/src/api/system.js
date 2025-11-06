/**
 * System API - Optimized
 * Uses base API client for DRY code
 */

import { apiClient, videoClient } from './base';

const DEFAULT_HEALTH_STATUS = {
  oms: 'offline',
  scheduler: 'offline',
  routine: 'offline',
  validation: 'offline',
  automation: 'offline',
  videoStream: 'offline'
};

export const systemAPI = {
  // Check system health status
  checkSystemHealth: async () => {
    const result = await apiClient.get('/system/status', {}, {
      timeout: 10000,
      successMessage: 'System health check completed'
    });

    if (result.success) {
      // Transform services data to health status
      const healthStatus = {};
      if (result.data?.services) {
        Object.keys(result.data.services).forEach(serviceName => {
          const serviceData = result.data.services[serviceName];
          healthStatus[serviceName] = serviceData.status === 'healthy' ? 'online' : 'offline';
        });
      }

      // Check video stream separately (longer timeout for RTSP camera init)
      try {
        const videoResult = await videoClient.get('/status', {}, { timeout: 10000 });
        healthStatus.videoStream = videoResult.success ? 'online' : 'offline';
      } catch (error) {
        console.warn('Video stream health check failed:', error.message);
        healthStatus.videoStream = 'offline';
      }

      return {
        ...result,
        data: healthStatus
      };
    } else {
      return {
        ...result,
        data: DEFAULT_HEALTH_STATUS
      };
    }
  },

  // Get system status
  getSystemStatus: () =>
    apiClient.get('/system/status', {}, {
      successMessage: 'System status fetched successfully'
    }),

  // Stop system
  stopSystem: (reason = '') =>
    apiClient.post('/system/stop', { reason }, {
      successMessage: 'System stopped successfully'
    }).then(result => ({
      ...result,
      message: result.data?.message || result.message
    })),

  // Resume system operations
  resumeOperation: () =>
    apiClient.post('/system/resume', {}, {
      successMessage: 'System operations resumed successfully'
    }).then(result => ({
      ...result,
      message: result.data?.message || result.message
    }))
}; 