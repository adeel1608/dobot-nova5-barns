/**
 * Legacy Store (Backward Compatibility)
 * 
 * This file provides backward compatibility for components still using the old store interface.
 * All actual functionality has been moved to modular stores in src/store/.
 * 
 * New components should use the modular stores directly:
 * - useDashboardStore (orders, recipes, system status)
 * - useAlertsStore (alerts, acknowledgments)
 * - useInventoryStore (inventory management)
 * - useLogsStore (system logs)
 * - useWebSocketStore (WebSocket connections)
 */

import { 
  useDashboardStore, 
  useAlertsStore, 
  useInventoryStore, 
  useLogsStore,
  useWebSocketStore 
} from './store/index';

// Global navigation state
let globalSetActiveTab = null;

// Function to set the navigation handler
const setNavigationHandler = (setActiveTab) => {
  globalSetActiveTab = setActiveTab;
};

// Function to navigate to a specific tab
const navigateToTab = (tabName) => {
  if (globalSetActiveTab) {
    globalSetActiveTab(tabName);
  } else {
    // Fallback to hash navigation
    window.location.hash = `#/${tabName}`;
  }
};

/**
 * Legacy store interface - provides backward compatibility
 * @deprecated Use modular stores directly instead
 */
export default function useStore() {
  const dashboard = useDashboardStore();
  const alerts = useAlertsStore();
  const inventory = useInventoryStore();
  const logs = useLogsStore();
  const webSocket = useWebSocketStore();

  return {
    // Navigation functions
    setNavigationHandler,
    navigateToTab,
    navigate: navigateToTab, // Alias for convenience
    
    // Dashboard state and actions
    orders: dashboard.orders || [],
    ordersTotal: dashboard.ordersTotal || 0,
    ordersOffset: dashboard.ordersOffset || 0,
    ordersHasMore: dashboard.ordersHasMore || false,
    ordersPageSize: dashboard.ordersPageSize || 20,
    orderStats: dashboard.orderStats || {},
    recipes: dashboard.recipes || [],
    menuItems: dashboard.menuItems || [],
    ingredientsByCategory: dashboard.ingredientsByCategory || {},
    systemStatus: dashboard.systemStatus,
    schedulerStatus: dashboard.schedulerStatus,
    schedulerTasks: dashboard.schedulerTasks || { Arm1: [], Arm2: [] },
    schedulerTaskStatus: dashboard.schedulerTaskStatus || {},
    schedulerStatusMessage: dashboard.schedulerStatusMessage || null,
    taskTimings: dashboard.taskTimings || {},
    fetchOrders: dashboard.fetchOrders,
    fetchOrderStats: dashboard.fetchOrderStats,
    loadMoreOrders: dashboard.loadMoreOrders,
    fetchRecipes: dashboard.fetchRecipes,
    fetchMenuItems: dashboard.fetchMenuItems,
    fetchIngredientsByCategory: dashboard.fetchIngredientsByCategory,
    createOrder: dashboard.createOrder,
    processPOSOrder: dashboard.processPOSOrder,
    startOrder: dashboard.startOrder,
    stopOrder: dashboard.stopOrder,
    resumeOrder: dashboard.resumeOrder,
    deleteOrder: dashboard.deleteOrder,
    sendReorder: dashboard.reorderQueue,
    fetchSchedulerStatus: dashboard.fetchSchedulerStatus,
    setSchedulerPlan: dashboard.setSchedulerPlan,
    updateSchedulerTask: dashboard.updateSchedulerTask,
    setSchedulerStatusMessage: dashboard.setSchedulerStatusMessage,
    updateTaskTiming: dashboard.updateTaskTiming,
    checkSystemHealth: dashboard.checkSystemHealth,
    stopSystem: dashboard.stopSystem,
    resumeOperation: dashboard.resumeOperation,
    
    // Alerts state and actions
    alerts: alerts.alerts,
    acknowledgedAlerts: alerts.acknowledgedAlerts,
    fetchAlerts: alerts.fetchAlerts,
    fetchAcknowledgedAlerts: alerts.fetchAcknowledgedAlerts,
    acknowledgeAlert: alerts.acknowledgeAlert,
    
    // Inventory state and actions
    inventoryStatus: inventory.inventoryStatus,
    fetchInventoryStatus: inventory.fetchInventoryStatus,
    refillInventory: inventory.refillInventory,
    
    // Logs state and actions
    systemLogs: logs.systemLogs,
    addLog: logs.addLog,
    clearLogs: logs.clearLogs,
    
    // WebSocket connections
    connectOrderWS: webSocket.connectOrderWS,
    connectAlertWS: webSocket.connectAlertWS,
    
    // Common state
    isLoading: dashboard.isLoading || alerts.isLoading || inventory.isLoading,
    errors: {
      ...dashboard.errors,
      ...alerts.errors,
      ...inventory.errors,
      websocket: null // WebSocket errors are handled in logs
    },
    
    // Error management
    clearError: (component) => {
      dashboard.clearError(component);
      alerts.clearError(component);
      inventory.clearError(component);
    },

    // Legacy functions (deprecated - keeping for compatibility)
    extractErrorMessage: (error) => {
      console.warn('extractErrorMessage is deprecated. Use errorHandler.js directly.');
      if (error.response?.data?.detail) return error.response.data.detail;
      if (error.response?.data?.message) return error.response.data.message;
      return error.message || 'An unexpected error occurred.';
    },
    
    // Deprecated WebSocket functions (keeping for compatibility)
    connectWS: () => {
      console.warn('connectWS is deprecated. Use useWebSocketStore directly.');
      webSocket.connectOrderWS();
    },
    
    // Mock functions for backward compatibility
    haltOrder: () => Promise.resolve(false),
    handleThresholdWarning: () => {},
    handleInventoryRefilled: () => {}
  };
} 