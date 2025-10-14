/**
 * Main Store
 * Combines all modular stores and provides unified interface
 */

import { create } from 'zustand';
import { useDashboardStore } from './dashboardStore';
import { useAlertsStore } from './alertsStore';
import { useInventoryStore } from './inventoryStore';
import { useCamerasStore } from './camerasStore';
import { useLogsStore, addLog } from './logsStore';
import { UI_CONFIG } from '../utils/config';
import { wsManager } from '../utils/websocket';

// Enhanced WebSocket management store using the centralized WebSocket manager
export const useWebSocketStore = create((set, get) => ({
  // Connection status tracking
  connectionStatus: {
    websocket: 'disconnected'
  },

  // Connect to the main WebSocket endpoint that handles all events
  connectWebSocket: () => {
    wsManager.connect('main', '', {  // Empty endpoint because /ws is the default
      onOpen: () => {
        set(state => ({
          connectionStatus: { ...state.connectionStatus, websocket: 'connected' }
        }));
        addLog('WebSocket', 'info', 'Main WebSocket connected - real-time updates enabled');
      },
      onMessage: (data) => {
        // Handle different types of events from the API bridge
        if (data.type === 'order_update') {
          addLog('WebSocket', 'info', 'Order update received', data);
          // Handle detailed scheduler events
          if (data.event === 'scheduler.plan_built') {
            const payload = data.data || {};
            const plan = payload.plan || {};
            const orderId = payload.order_id;
            useDashboardStore.getState().setSchedulerPlan(orderId, plan);
          } else if (data.event === 'scheduler.order_completed') {
            // Order completed: freeze task state and refresh orders
            addLog('WebSocket', 'info', 'Order completed - freezing task state');
            useDashboardStore.getState().freezeSchedulerState();
            useDashboardStore.getState().fetchOrders();
          } else if (data.event === 'scheduler.order_failed') {
            // Order-level failure: mark remaining tasks as cancelled and freeze
            const payload = data.data || {};
            const reason = payload.error || 'Order failed';
            addLog('WebSocket', 'info', `Order failed: ${reason} - freezing task state`);
            useDashboardStore.getState().finalizeSchedulerAsFailed(reason);
            useDashboardStore.getState().freezeSchedulerState();
            useDashboardStore.getState().fetchOrders();
          } else if (data.event === 'scheduler.order_stopped') {
            // Order stopped: freeze state
            addLog('WebSocket', 'info', 'Order stopped - freezing task state');
            useDashboardStore.getState().freezeSchedulerState();
            useDashboardStore.getState().fetchOrders();
          } else if (data.event === 'scheduler.feedback_processed') {
            const payload = data.data || {};
            useDashboardStore.getState().updateSchedulerTask({
              cup_id: payload.cup_id,
              action: payload.action,
              success: payload.success,
              message: payload.message
            });
          } else if (data.event === 'scheduler.status_update') {
            const payload = data.data || {};
            useDashboardStore.getState().setSchedulerStatusMessage(payload.message, payload.status);
          } else {
            // Fallback: refresh orders on generic updates
            useDashboardStore.getState().fetchOrders();
          }
        } else if (data.type === 'inventory_update') {
          addLog('WebSocket', 'info', 'Inventory update received', data);
          // Refresh inventory when we get updates
          useInventoryStore.getState().fetchInventoryStatus();
        } else if (data.type === 'connection') {
          addLog('WebSocket', 'info', data.message || 'WebSocket connection established');
        } else {
          addLog('WebSocket', 'info', 'WebSocket message received', data);
        }
      },
      onClose: () => {
        set(state => ({
          connectionStatus: { ...state.connectionStatus, websocket: 'disconnected' }
        }));
        addLog('WebSocket', 'warning', 'Main WebSocket disconnected');
      },
      onError: (error) => {
        addLog('WebSocket', 'error', 'WebSocket connection error', error);
      }
    });
  },

  // Legacy functions for backward compatibility
  connectOrderWS: () => {
    // Redirect to main WebSocket connection
    get().connectWebSocket();
  },

  connectAlertWS: () => {
    // Redirect to main WebSocket connection (alerts come through main connection)
    get().connectWebSocket();
  },

  // Disconnect all WebSockets
  disconnectAll: () => {
    wsManager.disconnectAll();
    set({
      connectionStatus: {
        websocket: 'disconnected'
      }
    });
  },

  // Get connection status
  getConnectionStatus: (name) => {
    if (name === 'orders' || name === 'alerts') {
      return get().connectionStatus.websocket;
    }
    return wsManager.getStatus(name);
  }
}));

// Main application store that combines everything
export const useMainStore = create((set, get) => ({
  // Application state
  isInitialized: false,
  
  // Initialize the application
  initialize: async () => {
    try {
      addLog('App', 'info', 'Initializing BARNS Dashboard...');
      
      // Initialize all stores
      const dashboardStore = useDashboardStore.getState();
      const alertsStore = useAlertsStore.getState();
      const inventoryStore = useInventoryStore.getState();
      const camerasStore = useCamerasStore.getState();
      const webSocketStore = useWebSocketStore.getState();
      
      // Initial data fetching
      await Promise.all([
        dashboardStore.fetchOrders(),
        dashboardStore.fetchRecipes(),
        alertsStore.fetchAlerts(),
        alertsStore.fetchAcknowledgedAlerts(),
        dashboardStore.fetchSchedulerStatus(),
        inventoryStore.fetchInventoryStatus(),
        dashboardStore.checkSystemHealth(),
        camerasStore.getStreamStatus()
      ]);
      
      // Set up WebSocket connections
      webSocketStore.connectWebSocket();
      
      set({ isInitialized: true });
      addLog('App', 'info', 'BARNS Dashboard initialized successfully');
      
    } catch (error) {
      addLog('App', 'error', 'Failed to initialize BARNS Dashboard', error);
      set({ isInitialized: false });
    }
  },

  // Set up periodic tasks
  setupPeriodicTasks: () => {
    const dashboardStore = useDashboardStore.getState();
    const camerasStore = useCamerasStore.getState();
    
    // Set up periodic health checks every 2 minutes
    const healthCheckInterval = setInterval(() => {
      dashboardStore.checkSystemHealth();
      camerasStore.getStreamStatus();
    }, UI_CONFIG.HEALTH_CHECK_INTERVAL);

    // Return cleanup function
    return () => {
      clearInterval(healthCheckInterval);
      useWebSocketStore.getState().disconnectAll();
    };
  },

  // Check if there are any errors across all stores
  hasErrors: () => {
    const dashboardErrors = useDashboardStore.getState().errors;
    const alertErrors = useAlertsStore.getState().errors;
    const inventoryErrors = useInventoryStore.getState().errors;
    const camerasErrors = useCamerasStore.getState().errors;
    
    return Object.values({
      ...dashboardErrors,
      ...alertErrors,
      ...inventoryErrors,
      ...camerasErrors
    }).some(err => err !== null);
  }
}));

// Export individual stores for direct usage
export { useDashboardStore, useAlertsStore, useInventoryStore, useCamerasStore, useLogsStore }; 