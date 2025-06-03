import { create } from 'zustand';
import axios from 'axios';

// BARNS API Configuration
const API_ENDPOINTS = {
  OMS: 'http://localhost:8000',
  SCHEDULER: 'http://localhost:8001', 
  ROUTINE: 'http://localhost:8002',
  VALIDATION: 'http://localhost:8003',
  VIDEO_STREAM: 'http://localhost:8004'
};

const useStore = create((set, get) => ({
  // Core data
  orders: [],
  alerts: [],
  systemStatus: {
    oms: 'unknown',
    scheduler: 'unknown', 
    routine: 'unknown',
    validation: 'unknown',
    videoStream: 'unknown'
  },
  schedulerStatus: null,
  isLoading: false,
  
  // Inventory data
  inventoryStatus: {
    milk: { level: 'unknown', last_refilled: null },
    cup: { level: 'unknown', last_refilled: null },
    beans: { level: 'unknown', last_refilled: null },
    syrup: { level: 'unknown', last_refilled: null }
  },
  
  // Service-specific errors and logs
  errors: {
    orders: null,
    alerts: null,
    websocket: null,
    system: null,
    scheduler: null,
    routine: null,
    validation: null,
    videoStream: null,
    inventory: null
  },
  
  // System logs for the new logs tab
  systemLogs: [],
  
  // Add log entry
  addLog: (service, level, message, details = null) => {
    const logEntry = {
      id: Date.now(),
      timestamp: new Date().toISOString(),
      service,
      level, // 'info', 'warning', 'error'
      message,
      details
    };
    
    set(state => ({
      systemLogs: [logEntry, ...state.systemLogs].slice(0, 1000) // Keep last 1000 logs
    }));
  },

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
          orders: null,
          alerts: null,
          websocket: null,
          system: null,
          scheduler: null,
          routine: null,
          validation: null,
          videoStream: null,
          inventory: null
        }
      });
    }
  },

  clearLogs: () => {
    set({ systemLogs: [] });
  },
  
  // OMS API calls
  fetchOrders: async () => {
    const { addLog, extractErrorMessage } = get();
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, orders: null }
    }));
    
    try {
      addLog('OMS', 'info', 'Fetching orders...');
      const res = await axios.get(`${API_ENDPOINTS.OMS}/orders/`);
      const orders = res.data.orders || [];
      
      set(state => ({ 
        orders, 
        isLoading: false,
        systemStatus: { ...state.systemStatus, oms: 'online' }
      }));
      
      addLog('OMS', 'info', `Successfully fetched ${orders.length} orders`);
      return orders;
    } catch (error) {
      const userFriendlyMessage = extractErrorMessage(error);
      addLog('OMS', 'error', userFriendlyMessage, {
        operation: 'fetch_orders',
        technical_details: error.message
      });
      
      set(state => ({ 
        orders: [],
        isLoading: false,
        systemStatus: { ...state.systemStatus, oms: 'offline' },
        errors: { ...state.errors, orders: userFriendlyMessage }
      }));
      return [];
    }
  },

  fetchAlerts: async () => {
    const { addLog, extractErrorMessage } = get();
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, alerts: null }
    }));
    
    try {
      addLog('OMS', 'info', 'Fetching alerts...');
      const res = await axios.get(`${API_ENDPOINTS.OMS}/alerts/active`);
      const alerts = res.data.alerts || [];
      
      set(state => ({ 
        alerts, 
        isLoading: false,
        systemStatus: { ...state.systemStatus, oms: 'online' }
      }));
      
      addLog('OMS', 'info', `Successfully fetched ${alerts.length} active alerts`);
      return alerts;
    } catch (error) {
      const userFriendlyMessage = extractErrorMessage(error);
      addLog('OMS', 'error', userFriendlyMessage, {
        operation: 'fetch_alerts',
        technical_details: error.message
      });
      
      set(state => ({ 
        alerts: [],
        isLoading: false,
        errors: { ...state.errors, alerts: userFriendlyMessage }
      }));
      return [];
    }
  },

  // New: Fetch scheduler status
  fetchSchedulerStatus: async () => {
    const { addLog } = get();
    
    try {
      addLog('Scheduler', 'info', 'Fetching scheduler status...');
      const res = await axios.get(`${API_ENDPOINTS.SCHEDULER}/status`);
      
      set(state => ({ 
        schedulerStatus: res.data,
        systemStatus: { ...state.systemStatus, scheduler: 'online' },
        errors: { ...state.errors, scheduler: null }
      }));
      
      addLog('Scheduler', 'info', `Scheduler status: ${res.data.status || 'unknown'}`);
      return res.data;
    } catch (error) {
      const errorMsg = 'Failed to fetch scheduler status: ' + (error.response?.data?.detail || error.message);
      addLog('Scheduler', 'error', errorMsg, error);
      
      set(state => ({ 
        schedulerStatus: null,
        systemStatus: { ...state.systemStatus, scheduler: 'offline' },
        errors: { ...state.errors, scheduler: errorMsg }
      }));
      return null;
    }
  },

  // Start order processing
  startOrder: async (orderId) => {
    const { addLog, extractErrorMessage } = get();
    
    console.log('🏪 Store startOrder called with:', orderId);
    console.log('🏪 API endpoint will be:', `${API_ENDPOINTS.OMS}/orders/${orderId}/start`);
    
    try {
      addLog('OMS', 'info', `Starting order ${orderId}...`);
      console.log('🏪 Making PATCH request...');
      
      const res = await axios.patch(`${API_ENDPOINTS.OMS}/orders/${orderId}/start`);
      
      console.log('🏪 API Response:', res.data);
      addLog('OMS', 'info', `Order ${orderId} started successfully`);
      
      // Refresh orders after starting
      console.log('🏪 Refreshing orders...');
      await get().fetchOrders();
      console.log('🏪 Orders refreshed, returning true');
      return true;
    } catch (error) {
      const userFriendlyMessage = extractErrorMessage(error);
      console.error('🏪 API Error:', error);
      console.error('🏪 User-friendly message:', userFriendlyMessage);
      addLog('OMS', 'error', userFriendlyMessage, {
        operation: 'start_order',
        order_id: orderId,
        technical_details: error.message
      });
      return false;
    }
  },

  // Create new order
  createOrder: async (orderData) => {
    const { addLog } = get();
    
    try {
      addLog('OMS', 'info', 'Creating new order...', orderData);
      const res = await axios.post(`${API_ENDPOINTS.OMS}/orders/`, orderData);
      
      addLog('OMS', 'info', `Order created successfully with ID: ${res.data.order_id}`);
      
      // Refresh orders after creating
      await get().fetchOrders();
      return res.data;
    } catch (error) {
      const errorMsg = 'Failed to create order: ' + (error.response?.data?.detail || error.message);
      addLog('OMS', 'error', errorMsg, error);
      return null;
    }
  },

  // Delete order
  deleteOrder: async (orderId) => {
    const { addLog, extractErrorMessage } = get();
    
    try {
      // Get the order first to check its status
      const currentOrders = get().orders;
      const orderToDelete = currentOrders.find(o => o.id === orderId);
      const orderStatus = orderToDelete?.status?.toUpperCase() || 'UNKNOWN';
      
      if (orderStatus === 'PROCESSING') {
        addLog('OMS', 'warning', `Force deleting processing order ${orderId}...`);
      } else {
        addLog('OMS', 'info', `Deleting order ${orderId} (status: ${orderStatus})...`);
      }
      
      const res = await axios.delete(`${API_ENDPOINTS.OMS}/orders/${orderId}`);
      
      if (orderStatus === 'PROCESSING') {
        addLog('OMS', 'warning', `Processing order ${orderId} force deleted successfully`);
      } else {
        addLog('OMS', 'info', `Order ${orderId} deleted successfully`);
      }
      
      // Refresh orders after deleting
      await get().fetchOrders();
      return true;
    } catch (error) {
      const userFriendlyMessage = extractErrorMessage(error);
      addLog('OMS', 'error', userFriendlyMessage, {
        operation: 'delete_order',
        order_id: orderId,
        technical_details: error.message
      });
      return false;
    }
  },

  // Check all service health
  checkSystemHealth: async () => {
    const { addLog } = get();
    const services = [
      { name: 'oms', url: `${API_ENDPOINTS.OMS}/alerts/active` },
      { name: 'scheduler', url: `${API_ENDPOINTS.SCHEDULER}/status` },
      { name: 'routine', url: `${API_ENDPOINTS.ROUTINE}/health` },
      { name: 'validation', url: `${API_ENDPOINTS.VALIDATION}` },
      { name: 'videoStream', url: `${API_ENDPOINTS.VIDEO_STREAM}/status` }
    ];

    const healthStatus = {};
    
    for (const service of services) {
      try {
        await axios.get(service.url, { timeout: 5000 });
        healthStatus[service.name] = 'online';
        addLog(service.name.toUpperCase(), 'info', `Service is healthy`);
      } catch (error) {
        healthStatus[service.name] = 'offline';
        addLog(service.name.toUpperCase(), 'warning', `Service health check failed: ${error.message}`);
      }
    }

    set(state => ({
      systemStatus: { ...state.systemStatus, ...healthStatus }
    }));

    return healthStatus;
  },

  sendReorder: async (newOrders) => {
    const { addLog } = get();
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, orders: null }
    }));
    
    try {
      addLog('OMS', 'info', 'Reordering queue...');
      // Note: This endpoint might need to be implemented in OMS
      await axios.put(`${API_ENDPOINTS.OMS}/orders/reorder`, { 
        order_ids: newOrders.map(o => o.id) 
      });
      
      set({ orders: newOrders, isLoading: false });
      addLog('OMS', 'info', 'Queue reordered successfully');
      return true;
    } catch (error) {
      const errorMsg = 'Failed to reorder: ' + (error.response?.data?.detail || error.message);
      addLog('OMS', 'error', errorMsg, error);
      
      set(state => ({ 
        isLoading: false,
        errors: { ...state.errors, orders: errorMsg }
      }));
      return false;
    }
  },

  // Helper function to extract user-friendly error messages
  extractErrorMessage: (error) => {
    // Network/Connection errors
    if (error.code === 'ECONNREFUSED' || error.code === 'ERR_NETWORK') {
      return 'Service is offline or unreachable. Please check if the service is running.';
    }
    
    if (error.code === 'ENOTFOUND') {
      return 'Service not found. Please verify the service configuration.';
    }
    
    // HTTP status code errors
    if (error.response) {
      const status = error.response.status;
      const detail = error.response.data?.detail || error.response.data?.message;
      
      switch (status) {
        case 400:
          return `Invalid request: ${detail || 'Please check your input and try again.'}`;
        case 401:
          return 'Authentication required. Please check your credentials.';
        case 403:
          return 'Access denied. You do not have permission to perform this action.';
        case 404:
          return `Service endpoint not found: ${detail || 'The requested operation is not available.'}`;
        case 500:
          return `Server error: ${detail || 'An internal error occurred. Please try again later.'}`;
        case 503:
          return 'Service temporarily unavailable. Please try again in a moment.';
        default:
          return detail || `HTTP ${status} error occurred.`;
      }
    }
    
    // Timeout errors
    if (error.code === 'ECONNABORTED' || error.message?.includes('timeout')) {
      return 'Request timed out. The service may be busy, please try again.';
    }
    
    // Generic fallback
    return error.message || 'An unexpected error occurred.';
  },

  stopSystem: async (reason = '') => {
    const { addLog, extractErrorMessage } = get();
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, system: null }
    }));
    
    try {
      addLog('System', 'warning', `Initiating emergency stop: ${reason || 'No reason provided'}`);
      const response = await axios.post(`${API_ENDPOINTS.OMS}/system/stop`, { reason });
      
      set({ isLoading: false });
      addLog('System', 'warning', response.data.message || 'System stopped successfully');
      
      // Refresh alerts to show any new system alerts
      await get().fetchAlerts();
      return true;
    } catch (error) {
      const userFriendlyMessage = extractErrorMessage(error);
      addLog('System', 'error', userFriendlyMessage, {
        operation: 'emergency_stop',
        reason: reason,
        technical_details: error.message
      });
      
      set(state => ({ 
        isLoading: false,
        errors: { ...state.errors, system: userFriendlyMessage }
      }));
      return false;
    }
  },

  resumeOperation: async () => {
    const { addLog, extractErrorMessage } = get();
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, system: null }
    }));
    
    try {
      addLog('System', 'info', 'Attempting to resume system operations...');
      const response = await axios.post(`${API_ENDPOINTS.OMS}/system/resume`);
      
      set({ isLoading: false });
      addLog('System', 'info', response.data.message || 'System operations resumed successfully');
      
      // Refresh alerts and orders after resume
      await get().fetchAlerts();
      await get().fetchOrders();
      return true;
    } catch (error) {
      const userFriendlyMessage = extractErrorMessage(error);
      addLog('System', 'error', userFriendlyMessage, {
        operation: 'resume_operation',
        technical_details: error.message
      });
      
      set(state => ({ 
        isLoading: false,
        errors: { ...state.errors, system: userFriendlyMessage }
      }));
      return false;
    }
  },

  connectOrderWS: () => {
    const { addLog } = get();
    
    try {
      addLog('WebSocket', 'info', 'Connecting to orders WebSocket...');
      const ws = new WebSocket(`ws://localhost:8000/ws/orders`);
      
      ws.onopen = () => {
        addLog('WebSocket', 'info', 'Orders WebSocket connected successfully');
        set(state => ({ 
          errors: { ...state.errors, websocket: null }
        }));
      };
      
      ws.onmessage = e => {
        try {
          const data = JSON.parse(e.data);
          addLog('WebSocket', 'info', 'Received orders update', data);
          
          // Handle different event types
          if (data.event === 'threshold_warning') {
            get().handleThresholdWarning(data.ingredient, data.severity);
          } else if (data.event === 'inventory_refilled') {
            get().handleInventoryRefilled(data.ingredient);
          } else {
            // Refresh orders for other events
            get().fetchOrders();
          }
        } catch (error) {
          addLog('WebSocket', 'error', 'Error parsing WebSocket data', error);
        }
      };
      
      ws.onerror = (error) => {
        addLog('WebSocket', 'error', 'Orders WebSocket error', error);
        set(state => ({ 
          errors: { ...state.errors, websocket: 'Failed to connect to orders WebSocket' }
        }));
      };
      
      ws.onclose = () => {
        addLog('WebSocket', 'warning', 'Orders WebSocket connection closed, attempting reconnect...');
        setTimeout(() => {
          get().connectOrderWS();
        }, 5000);
      };
      
      return ws;
    } catch (error) {
      addLog('WebSocket', 'error', 'Failed to create WebSocket connection', error);
      set(state => ({ 
        errors: { ...state.errors, websocket: 'WebSocket connection error: ' + error.message }
      }));
      return null;
    }
  },

  connectAlertWS: () => {
    const { addLog } = get();
    
    try {
      addLog('WebSocket', 'info', 'Connecting to alerts WebSocket...');
      // Note: This might need to be implemented in the backend
      const ws = new WebSocket(`ws://localhost:8000/ws/alerts`);
      
      ws.onopen = () => {
        addLog('WebSocket', 'info', 'Alerts WebSocket connected successfully');
        set(state => ({ 
          errors: { ...state.errors, websocket: null }
        }));
      };
      
      ws.onmessage = e => {
        try {
          const data = JSON.parse(e.data);
          addLog('WebSocket', 'info', 'Received alerts update', data);
          
          // Refresh alerts when we get updates
          get().fetchAlerts();
        } catch (error) {
          addLog('WebSocket', 'error', 'Error parsing alerts WebSocket data', error);
        }
      };
      
      ws.onerror = (error) => {
        addLog('WebSocket', 'error', 'Alerts WebSocket error', error);
      };
      
      ws.onclose = () => {
        addLog('WebSocket', 'warning', 'Alerts WebSocket connection closed, attempting reconnect...');
        setTimeout(() => {
          get().connectAlertWS();
        }, 5000);
      };
      
      return ws;
    } catch (error) {
      addLog('WebSocket', 'error', 'Failed to create alerts WebSocket connection', error);
      return null;
    }
  },
  
  // Mock data for demonstration when APIs are unavailable (removed mock data usage)
  mockOrders: [],
  mockAlerts: [],

  // Resume halted order
  resumeOrder: async (orderId) => {
    const { addLog, extractErrorMessage } = get();
    
    try {
      addLog('OMS', 'info', `Resuming halted order ${orderId}...`);
      const response = await axios.post(`${API_ENDPOINTS.OMS}/orders/${orderId}/resume`);
      
      addLog('OMS', 'info', `Order ${orderId} resumed successfully`);
      
      // Refresh orders after resuming
      await get().fetchOrders();
      return true;
    } catch (error) {
      const userFriendlyMessage = extractErrorMessage(error);
      addLog('OMS', 'error', userFriendlyMessage, {
        operation: 'resume_order',
        order_id: orderId,
        technical_details: error.message
      });
      return false;
    }
  },

  // Halt order with reason
  haltOrder: async (orderId, reason) => {
    const { addLog, extractErrorMessage } = get();
    
    try {
      addLog('OMS', 'warning', `Halting order ${orderId}: ${reason}`);
      const response = await axios.post(`${API_ENDPOINTS.OMS}/orders/${orderId}/halt`, null, {
        params: { reason }
      });
      
      addLog('OMS', 'warning', `Order ${orderId} halted: ${reason}`);
      
      // Refresh orders and alerts after halting
      await get().fetchOrders();
      await get().fetchAlerts();
      return true;
    } catch (error) {
      const userFriendlyMessage = extractErrorMessage(error);
      addLog('OMS', 'error', userFriendlyMessage, {
        operation: 'halt_order',
        order_id: orderId,
        reason: reason,
        technical_details: error.message
      });
      return false;
    }
  },

  // Inventory Management Functions
  
  // Fetch inventory status
  fetchInventoryStatus: async () => {
    const { addLog, extractErrorMessage } = get();
    
    try {
      addLog('OMS', 'info', 'Fetching inventory status...');
      const res = await axios.get(`${API_ENDPOINTS.OMS}/inventory/status`);
      
      set(state => ({
        inventoryStatus: res.data.inventory || state.inventoryStatus,
        errors: { ...state.errors, inventory: null }
      }));
      
      addLog('OMS', 'info', 'Inventory status fetched successfully');
      return res.data.inventory;
    } catch (error) {
      const userFriendlyMessage = extractErrorMessage(error);
      addLog('OMS', 'error', userFriendlyMessage, {
        operation: 'fetch_inventory_status',
        technical_details: error.message
      });
      
      set(state => ({
        errors: { ...state.errors, inventory: userFriendlyMessage }
      }));
      return null;
    }
  },

  // Refill inventory for a specific ingredient
  refillInventory: async (ingredient) => {
    const { addLog, extractErrorMessage } = get();
    
    try {
      addLog('OMS', 'info', `Initiating refill for ${ingredient}...`);
      const res = await axios.post(`${API_ENDPOINTS.OMS}/inventory/refill`, {
        ingredient: ingredient.toLowerCase()
      });
      
      addLog('OMS', 'info', `Refill initiated for ${ingredient} successfully`);
      
      // Refresh inventory status after refill
      await get().fetchInventoryStatus();
      
      // Also refresh alerts in case this resolves any threshold warnings
      await get().fetchAlerts();
      
      return true;
    } catch (error) {
      const userFriendlyMessage = extractErrorMessage(error);
      addLog('OMS', 'error', userFriendlyMessage, {
        operation: 'refill_inventory',
        ingredient: ingredient,
        technical_details: error.message
      });
      return false;
    }
  },

  // Handle threshold warning (called when WebSocket receives threshold warning)
  handleThresholdWarning: (ingredient, severity) => {
    const { addLog } = get();
    
    addLog('Validation', 'warning', `Threshold warning: ${ingredient} level is ${severity}`, {
      ingredient,
      severity,
      timestamp: new Date().toISOString()
    });
    
    // Update inventory status to reflect the warning
    set(state => ({
      inventoryStatus: {
        ...state.inventoryStatus,
        [ingredient]: {
          ...state.inventoryStatus[ingredient],
          level: severity
        }
      }
    }));
    
    // Refresh alerts to show the new threshold warning
    get().fetchAlerts();
  },

  // Handle inventory refill confirmation (called when WebSocket receives refill confirmation)
  handleInventoryRefilled: (ingredient) => {
    const { addLog } = get();
    
    addLog('Validation', 'info', `Inventory refilled: ${ingredient}`, {
      ingredient,
      timestamp: new Date().toISOString()
    });
    
    // Update inventory status to reflect the refill
    set(state => ({
      inventoryStatus: {
        ...state.inventoryStatus,
        [ingredient]: {
          ...state.inventoryStatus[ingredient],
          level: 'high',
          last_refilled: new Date().toISOString()
        }
      }
    }));
    
    // Refresh inventory status and alerts
    get().fetchInventoryStatus();
    get().fetchAlerts();
  },
}));

export default useStore;
