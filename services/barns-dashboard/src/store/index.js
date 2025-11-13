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
import { apiClient } from '../api/base';
import { alertsAPI } from '../api/alerts';

// Enhanced WebSocket management store using the centralized WebSocket manager
export const useWebSocketStore = create((set, get) => ({
  // Connection status tracking
  connectionStatus: {
    websocket: 'disconnected'
  },
  
  // Debounce tracking for status updates
  _lastStatusUpdateTime: 0,

  // Connect to the main WebSocket endpoint that handles all events
  connectWebSocket: () => {
    wsManager.connect('main', '', {  // Connect to /ws endpoint via API Bridge
      onOpen: () => {
        set(state => ({
          connectionStatus: { ...state.connectionStatus, websocket: 'connected' }
        }));
        // Removed verbose INFO log - only log errors
      },
      onMessage: (data) => {
        // Log all WebSocket messages for debugging
        console.log('[WebSocket] Message received:', {
          type: data.type,
          event: data.event,
          timestamp: new Date().toISOString()
        });
        
        // Handle different types of events from the API bridge
        if (data.type === 'order_update') {
          // Removed verbose INFO log - only log errors
          // Handle detailed scheduler events
          if (data.event === 'scheduler.plan_built') {
            const payload = data.data || {};
            const plan = payload.plan || {};
            const orderId = payload.order_id;
            console.log(`[WebSocket] Received plan_built for order ${orderId}`, plan);
            useDashboardStore.getState().setSchedulerPlan(orderId, plan);
        } else if (data.event === 'scheduler.order_completed') {
          // Order completed: freeze task state and keep it visible until new order or deletion
          const payload = data.data || {};
          const orderId = payload.order_id;
          console.log(`[WebSocket] Order ${orderId} completed - freezing task state`);
          // Removed verbose INFO log - only log errors
          useDashboardStore.getState().freezeSchedulerState();
          useDashboardStore.getState().fetchOrders();
        } else if (data.event === 'scheduler.order_failed') {
          // Order-level failure: mark remaining tasks as cancelled and freeze
          const payload = data.data || {};
          const reason = payload.error || 'Order failed';
          // Removed verbose INFO log - only log errors
          useDashboardStore.getState().finalizeSchedulerAsFailed(reason);
          useDashboardStore.getState().freezeSchedulerState();
          useDashboardStore.getState().fetchOrders();
        } else if (data.event === 'order_stopping' || data.event === 'scheduler.order_stopping') {
          // Order is stopping: refresh orders to show STOPPING status
          const payload = data.data || {};
          const orderId = payload.order_id || payload.order;
          console.log(`[WebSocket] Order ${orderId} stopping - refreshing orders`);
          // Removed verbose INFO log - only log errors
          useDashboardStore.getState().fetchOrders();
        } else if (data.event === 'scheduler.order_stopped' || data.event === 'order_stopped') {
          // Order stopped: freeze state and keep it visible until new order or deletion
          // Handle both scheduler event (immediate) and OMS event (after DB update)
          const payload = data.data || {};
          const orderId = payload.order_id || payload.order;
          console.log(`[WebSocket] Order ${orderId} stopped - freezing and refreshing`);
          // Removed verbose INFO log - only log errors
          useDashboardStore.getState().freezeSchedulerState();
          useDashboardStore.getState().fetchOrders();
        } else if (data.event === 'order_resumed') {
          // Order resumed: refresh orders to update status
          const payload = data.data || {};
          const orderId = payload.order_id || payload.order;
          console.log(`[WebSocket] Order ${orderId} resumed - refreshing orders`);
          // Removed verbose INFO log - only log errors
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
            // Debounce status message updates to prevent excessive re-renders
            if (!get()._lastStatusUpdateTime || (Date.now() - get()._lastStatusUpdateTime) > 100) {
              set({ _lastStatusUpdateTime: Date.now() });
              useDashboardStore.getState().setSchedulerStatusMessage(payload.message, payload.status);
            }
          } else {
            // Fallback: refresh orders on generic updates
            useDashboardStore.getState().fetchOrders();
          }
        } else if (data.type === 'inventory_update') {
          // Removed verbose INFO log - only log errors
          // Refresh inventory when we get updates
          useInventoryStore.getState().fetchInventoryStatus();
        } else if (data.type === 'alert' || data.event === 'validation_failed' || data.event?.includes('threshold_warning') || data.event?.includes('all_stations_occupied') || data.event?.includes('retry_status')) {
          // Alert/warning events - refresh alerts immediately
          console.log('[WebSocket] Alert event received, refreshing alerts:', data);
          
          // Log validation failures specifically for debugging
          if (data.event === 'validation_failed' || data.type === 'alert') {
            console.log('[WebSocket] Validation failure alert received:', {
              validation_function: data.validation_function,
              cup_id: data.cup_id,
              message: data.message
            });
          }
          
          // Debounce: Only refresh if we haven't refreshed recently
          const alertsStore = useAlertsStore.getState();
          const now = Date.now();
          if (!alertsStore._lastRefreshTime || (now - alertsStore._lastRefreshTime) > 2000) {
            // Clear cache to force fresh fetch (bypass 3-second cache)
            apiClient.clearCache('/alerts/active');
            
            // Mark that we're refreshing
            alertsStore._lastRefreshTime = now;
            
            // Retry logic: Try fetching alerts multiple times with increasing delays
            const alertId = data.alert_id; // Get alert_id from WebSocket message if available
            const maxRetries = 3;
            let retryCount = 0;
            
            const fetchWithRetry = () => {
              const delay = retryCount === 0 ? 1500 : 1000; // First attempt after 1.5s, retries after 1s
              
              setTimeout(() => {
                // Use the store's fetchAlerts method with noCache to bypass cache
                alertsStore.fetchAlerts({ noCache: true }).then((alerts) => {
                  const alertCount = alerts?.length || 0;
                  console.log(`[WebSocket] Alerts refreshed successfully, found ${alertCount} alerts (attempt ${retryCount + 1}/${maxRetries})`);
                  
                  // If we found alerts or this is the last retry, stop
                  if (alertCount > 0 || retryCount >= maxRetries - 1) {
                    alertsStore._lastRefreshTime = null;
                  } else {
                    // Retry if no alerts found and we haven't exceeded max retries
                    retryCount++;
                    fetchWithRetry();
                  }
                }).catch((err) => {
                  console.error('[WebSocket] Failed to refresh alerts:', err);
                  // Retry on error if we haven't exceeded max retries
                  if (retryCount < maxRetries - 1) {
                    retryCount++;
                    fetchWithRetry();
                  } else {
                    alertsStore._lastRefreshTime = null;
                  }
                });
              }, delay);
            };
            
            fetchWithRetry();
          } else {
            console.log('[WebSocket] Skipping alert refresh (debounced, last refresh was', (now - alertsStore._lastRefreshTime), 'ms ago)');
          }
        } else if (data.type === 'connection') {
          // Connection messages are already logged in onOpen, skip duplicate logging
          return;
        } else {
          // Removed verbose INFO log - only log errors
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
      // Removed verbose INFO log - only log errors
      
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
      // Removed verbose INFO log - only log errors
      
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