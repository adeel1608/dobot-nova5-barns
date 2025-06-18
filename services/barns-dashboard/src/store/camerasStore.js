/**
 * Cameras Store
 * Manages camera and video stream state and operations
 */

import { create } from 'zustand';
import { camerasAPI } from '../api';
import { addLog } from './logsStore';

export const useCamerasStore = create((set, get) => ({
  // State
  cameras: [],
  streamStatus: 'unknown',
  recordings: [],
  activeCamera: null,
  isLoading: false,
  errors: {
    cameras: null,
    stream: null,
    recording: null
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
          cameras: null,
          stream: null,
          recording: null
        }
      });
    }
  },

  // Get video stream status
  getStreamStatus: async () => {
    const result = await camerasAPI.getStreamStatus();
    
    if (result.success) {
      set(state => ({ 
        streamStatus: result.data.status || 'online',
        errors: { ...state.errors, stream: null }
      }));
      addLog('API', 'info', result.message);
    } else {
      set(state => ({ 
        streamStatus: 'offline',
        errors: { ...state.errors, stream: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.success;
  },

  // Fetch cameras list
  getCameras: async () => {
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, cameras: null }
    }));

    const result = await camerasAPI.getCameras();
    
    if (result.success) {
      set(state => ({ 
        cameras: result.data, 
        isLoading: false
      }));
      addLog('API', 'info', result.message);
    } else {
      set(state => ({ 
        cameras: [],
        isLoading: false,
        errors: { ...state.errors, cameras: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  },

  // Start recording
  startRecording: async (cameraId) => {
    const result = await camerasAPI.startRecording(cameraId);
    
    if (result.success) {
      addLog('API', 'info', result.message);
      // Update camera status
      set(state => ({
        cameras: state.cameras.map(cam => 
          cam.id === cameraId 
            ? { ...cam, recording: true }
            : cam
        )
      }));
    } else {
      addLog('API', 'error', result.error, result.details);
      set(state => ({
        errors: { ...state.errors, recording: result.error }
      }));
    }

    return result.success;
  },

  // Stop recording
  stopRecording: async (cameraId) => {
    const result = await camerasAPI.stopRecording(cameraId);
    
    if (result.success) {
      addLog('API', 'info', result.message);
      // Update camera status
      set(state => ({
        cameras: state.cameras.map(cam => 
          cam.id === cameraId 
            ? { ...cam, recording: false }
            : cam
        )
      }));
    } else {
      addLog('API', 'error', result.error, result.details);
      set(state => ({
        errors: { ...state.errors, recording: result.error }
      }));
    }

    return result.success;
  },

  // Get recordings for a camera
  getRecordings: async (cameraId) => {
    const result = await camerasAPI.getRecordings(cameraId);
    
    if (result.success) {
      set({ recordings: result.data });
      addLog('API', 'info', result.message);
    } else {
      set(state => ({
        recordings: [],
        errors: { ...state.errors, recording: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  },

  // Set active camera
  setActiveCamera: (cameraId) => {
    set({ activeCamera: cameraId });
  },

  // Get camera by ID
  getCameraById: (cameraId) => {
    const { cameras } = get();
    return cameras.find(cam => cam.id === cameraId);
  },

  // Check if any camera is recording
  isAnyRecording: () => {
    const { cameras } = get();
    return cameras.some(cam => cam.recording);
  }
})); 