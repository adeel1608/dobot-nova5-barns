/**
 * Dashboard Store
 * Manages dashboard-specific state including orders, recipes, and system status
 */

import { create } from 'zustand';
import { ordersAPI, systemAPI, recipesAPI } from '../api';
import { addLog } from './logsStore';

export const useDashboardStore = create((set, get) => ({
  // State
  orders: [],
  recipes: [],
  menuItems: [],
  ingredientsByCategory: {},
  systemStatus: {
    oms: 'unknown',
    scheduler: 'unknown',
    routine: 'unknown',
    validation: 'unknown',
    automation: 'unknown',
    videoStream: 'unknown'
  },
  schedulerStatus: null,
  isLoading: false,
  errors: {
    orders: null,
    recipes: null,
    system: null,
    scheduler: null
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
          orders: null,
          recipes: null,
          system: null,
          scheduler: null
        }
      });
    }
  },

  // Recipe Management
  fetchRecipes: async () => {
    set(state => ({ 
      errors: { ...state.errors, recipes: null }
    }));

    const result = await recipesAPI.fetchRecipes();
    
    if (result.success) {
      set({ recipes: result.data });
      addLog('API', 'info', result.message);
    } else {
      set(state => ({ 
        recipes: [],
        errors: { ...state.errors, recipes: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  },

  // Order Management
  fetchOrders: async () => {
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, orders: null }
    }));

    const result = await ordersAPI.fetchOrders();
    
    if (result.success) {
      set(state => ({ 
        orders: result.data, 
        isLoading: false,
        systemStatus: { ...state.systemStatus, oms: 'online' }
      }));
      addLog('API', 'info', result.message);
    } else {
      set(state => ({ 
        orders: [],
        isLoading: false,
        systemStatus: { ...state.systemStatus, oms: 'offline' },
        errors: { ...state.errors, orders: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  },

  createOrder: async (orderData) => {
    addLog('API', 'info', `Creating new order...`);
    const result = await ordersAPI.createOrder(orderData);
    
    if (result.success) {
      addLog('API', 'info', result.message, orderData);
      
      // Add a small delay to ensure backend has time to update
      setTimeout(async () => {
        addLog('API', 'info', `Refreshing orders after creating order`);
        await get().fetchOrders(); // Refresh orders
      }, 500);
    } else {
      addLog('API', 'error', result.error, result.details);
    }

    return result.success;
  },

  processPOSOrder: async (posOrderData) => {
    addLog('API', 'info', `Processing POS order...`);
    const result = await ordersAPI.processPOSOrder(posOrderData);
    
    if (result.success) {
      addLog('API', 'info', result.message, posOrderData);
      
      // Add a small delay to ensure backend has time to update
      setTimeout(async () => {
        addLog('API', 'info', `Refreshing orders after processing POS order`);
        await get().fetchOrders(); // Refresh orders
      }, 500);
    } else {
      addLog('API', 'error', result.error, result.details);
    }

    return result.success;
  },

  fetchMenuItems: async () => {
    addLog('API', 'info', `Fetching menu items...`);
    const result = await ordersAPI.fetchMenuItems();
    
    if (result.success) {
      set({ menuItems: result.data });
      addLog('API', 'info', `Loaded ${result.data.length} menu items`);
    } else {
      addLog('API', 'error', result.error);
    }
    
    return result.data;
  },

  fetchIngredientsByCategory: async () => {
    addLog('API', 'info', `Fetching ingredients...`);
    const result = await ordersAPI.fetchIngredients();
    
    if (result.success) {
      set({ ingredientsByCategory: result.data });
      const categoryCount = Object.keys(result.data).length;
      addLog('API', 'info', `Loaded ingredients from ${categoryCount} categories`);
    } else {
      addLog('API', 'error', result.error);
    }
    
    return result.data;
  },

  startOrder: async (orderId) => {
    addLog('API', 'info', `Starting order ${orderId}...`);
    const result = await ordersAPI.startOrder(orderId);
    
    if (result.success) {
      addLog('API', 'info', result.message);
      
      // Add a small delay to ensure backend has time to update
      setTimeout(async () => {
        addLog('API', 'info', `Refreshing orders after starting order ${orderId}`);
        await get().fetchOrders(); // Refresh orders
      }, 500);
    } else {
      addLog('API', 'error', result.error, result.details);
    }

    return result.success;
  },

  deleteOrder: async (orderId) => {
    addLog('API', 'info', `Deleting order ${orderId}...`);
    const result = await ordersAPI.deleteOrder(orderId);
    
    if (result.success) {
      addLog('API', 'info', result.message);
      
      // Add a small delay to ensure backend has time to update
      setTimeout(async () => {
        addLog('API', 'info', `Refreshing orders after deleting order ${orderId}`);
        await get().fetchOrders(); // Refresh orders
      }, 500);
    } else {
      addLog('API', 'error', result.error, result.details);
    }

    return result.success;
  },

  reorderQueue: async (newOrders) => {
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, orders: null }
    }));

    const orderIds = newOrders.map(o => o.id);
    const result = await ordersAPI.reorderQueue(orderIds);
    
    if (result.success) {
      set({ orders: newOrders, isLoading: false });
      addLog('API', 'info', result.message);
    } else {
      set(state => ({ 
        isLoading: false,
        errors: { ...state.errors, orders: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.success;
  },

  // System Management
  checkSystemHealth: async () => {
    const result = await systemAPI.checkSystemHealth();
    
    if (result.success) {
      set(state => ({
        systemStatus: { ...state.systemStatus, ...result.data }
      }));
      addLog('API', 'info', result.message);
    } else {
      set(state => ({
        systemStatus: { ...state.systemStatus, ...result.data }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  },

  fetchSchedulerStatus: async () => {
    const result = await systemAPI.getSystemStatus();
    
    if (result.success) {
      set(state => ({ 
        schedulerStatus: result.data,
        systemStatus: { ...state.systemStatus, scheduler: 'online' },
        errors: { ...state.errors, scheduler: null }
      }));
      addLog('API', 'info', result.message);
    } else {
      set(state => ({ 
        schedulerStatus: null,
        systemStatus: { ...state.systemStatus, scheduler: 'offline' },
        errors: { ...state.errors, scheduler: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  },

  stopSystem: async (reason = '') => {
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, system: null }
    }));

    const result = await systemAPI.stopSystem(reason);
    
    if (result.success) {
      set({ isLoading: false });
      addLog('System', 'warning', result.message);
      // Refresh alerts after system stop
      // This would typically trigger a refresh in the alerts store
    } else {
      set(state => ({ 
        isLoading: false,
        errors: { ...state.errors, system: result.error }
      }));
      addLog('System', 'error', result.error, result.details);
    }

    return result.success;
  },

  resumeOperation: async () => {
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, system: null }
    }));

    const result = await systemAPI.resumeOperation();
    
    if (result.success) {
      set({ isLoading: false });
      addLog('System', 'info', result.message);
      // Refresh orders and alerts after resume
      await get().fetchOrders();
    } else {
      set(state => ({ 
        isLoading: false,
        errors: { ...state.errors, system: result.error }
      }));
      addLog('System', 'error', result.error, result.details);
    }

    return result.success;
  }
})); 