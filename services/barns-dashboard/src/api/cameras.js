/**
 * Cameras API - Optimized
 * Uses base API client for DRY code
 */

import { apiClient, videoClient } from './base';

export const camerasAPI = {
  // Get video stream status
  getStreamStatus: () =>
    videoClient.get('/status', {}, {
      timeout: 5000,
      successMessage: 'Video stream status retrieved successfully'
    }).then(result => result.success ? result : {
      success: false,
      data: { status: 'offline' },
      error: result.error,
      details: result.details
    }),

  // Get list of available cameras
  getCameras: () =>
    apiClient.getList('/cameras', {}, 'cameras')
      .then(result => ({
        ...result,
        data: result.data?.cameras || result.data || []
      })),

  // Start video recording
  startRecording: (cameraId) =>
    apiClient.post(`/cameras/${cameraId}/record/start`, {}, {
      successMessage: `Recording started for camera ${cameraId}`
    }),

  // Stop video recording
  stopRecording: (cameraId) =>
    apiClient.post(`/cameras/${cameraId}/record/stop`, {}, {
      successMessage: `Recording stopped for camera ${cameraId}`
    }),

  // Get camera recordings
  getRecordings: (cameraId) =>
    apiClient.getList(`/cameras/${cameraId}/recordings`, {}, `recordings for camera ${cameraId}`)
      .then(result => ({
        ...result,
        data: result.data?.recordings || result.data || []
      }))
}; 