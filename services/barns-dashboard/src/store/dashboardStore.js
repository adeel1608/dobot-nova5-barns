/**
 * Dashboard Store
 * Manages dashboard-specific state including orders, recipes, and system status
 */

import { create } from 'zustand';
import { ordersAPI, systemAPI, recipesAPI } from '../api';
import { addLog } from './logsStore';

// Local persistence helpers for scheduler task view
const STORAGE_KEY = 'barns_scheduler_state_v1';
const loadSchedulerFromStorage = () => {
  try {
    if (typeof window === 'undefined') return null;
    const raw = window.localStorage.getItem(STORAGE_KEY);
    return raw ? JSON.parse(raw) : null;
  } catch {
    return null;
  }
};
const saveSchedulerToStorage = (state) => {
  try {
    if (typeof window === 'undefined') return;
    const payload = {
      orderId: state.schedulerCurrentOrderId || null,
      schedulerTasks: state.schedulerTasks,
      schedulerTaskStatus: state.schedulerTaskStatus,
      taskTimings: state.taskTimings || {}
    };
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify(payload));
  } catch {}
};
const clearSchedulerStorage = () => {
  try {
    if (typeof window === 'undefined') return;
    window.localStorage.removeItem(STORAGE_KEY);
  } catch {}
};

export const useDashboardStore = create((set, get) => ({
  // State
  orders: [],
  ordersTotal: 0,
  ordersOffset: 0,
  ordersHasMore: false,
  ordersPageSize: 15,
  recipes: [],
  menuItems: [],
  ingredientsByCategory: {},
  // Hydrate scheduler view from storage to persist across refresh
  ...(loadSchedulerFromStorage() ? {
    schedulerCurrentOrderId: loadSchedulerFromStorage().orderId || null,
    schedulerTasks: loadSchedulerFromStorage().schedulerTasks || { Arm1: [], Arm2: [] },
    schedulerTaskStatus: loadSchedulerFromStorage().schedulerTaskStatus || {},
    taskTimings: loadSchedulerFromStorage().taskTimings || {}
  } : {
    schedulerCurrentOrderId: null,
    schedulerTasks: { Arm1: [], Arm2: [] },
    schedulerTaskStatus: {},
    taskTimings: {}
  }),
  // key: `${cup_id}:${action}` -> { status, success, message }
  schedulerStatusMessage: null,
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
  setSchedulerPlan: (orderId, plan) => {
    // plan: { Arm1: [[action, cup_id], ...], Arm2: [...] }
    const format = (arr) => (arr || []).map(([action, cup]) => ({ action, cup_id: cup, status: 'pending' }));
    set({
      schedulerCurrentOrderId: orderId,
      schedulerTasks: {
        Arm1: format(plan.Arm1),
        Arm2: format(plan.Arm2)
      },
      schedulerTaskStatus: {},
      taskTimings: {} // Clear timings when new plan starts
    });
    saveSchedulerToStorage(get());
  },

  updateSchedulerTask: ({ cup_id, action, success, message }) => {
    const key = `${cup_id}:${action}`;
    set(state => {
      // Don't update if order is already completed/failed (frozen state)
      const currentOrder = state.orders.find(o => o.id === state.schedulerCurrentOrderId);
      const isFrozenOrder = currentOrder && ['COMPLETED', 'ERROR', 'STOPPED', 'CANCELLED'].includes(currentOrder.status?.toUpperCase());
      
      if (isFrozenOrder) {
        console.log(`[Store] Ignoring task update for frozen order ${state.schedulerCurrentOrderId}`);
        return state; // Don't update frozen orders
      }
      
      const updateList = (list) => list.map(t => {
        if (t.cup_id === cup_id && t.action === action) {
          // Preserve terminal states - don't overwrite completed, failed, or cancelled
          const isTerminal = t.status === 'completed' || t.status === 'failed' || t.status === 'cancelled';
          if (isTerminal) {
            return t; // Keep the existing terminal state
          }
          // Update to new status if not terminal
          return { 
            ...t, 
            status: success === true ? 'completed' : success === false ? 'failed' : 'in_progress', 
            message 
          };
        }
        return t;
      });
      return {
        schedulerTasks: {
          Arm1: updateList(state.schedulerTasks.Arm1),
          Arm2: updateList(state.schedulerTasks.Arm2)
        },
        schedulerTaskStatus: {
          ...state.schedulerTaskStatus,
          [key]: { success, message, status: success === true ? 'completed' : success === false ? 'failed' : 'in_progress' }
        }
      };
    });
    saveSchedulerToStorage(get());
  },

  setSchedulerStatusMessage: (message, statusObj) => {
    set({ schedulerStatusMessage: message });
    saveSchedulerToStorage(get());
  },

  updateTaskTiming: (taskKey, timing) => {
    set(state => ({
      taskTimings: {
        ...state.taskTimings,
        [taskKey]: timing
      }
    }));
    saveSchedulerToStorage(get());
  },

  // Mark all non-final tasks as cancelled after an order-level failure
  finalizeSchedulerAsFailed: (reason) => {
    set(state => {
      const cancelList = (list) => list.map(t => {
        if (t.status === 'completed' || t.status === 'failed') return t;
        if (t.status === 'cancelled') return t;
        return { ...t, status: 'cancelled' };
      });
      return {
        schedulerTasks: {
          Arm1: cancelList(state.schedulerTasks.Arm1),
          Arm2: cancelList(state.schedulerTasks.Arm2)
        },
        schedulerStatusMessage: reason ? String(reason) : state.schedulerStatusMessage
      };
    });
    saveSchedulerToStorage(get());
  },

  // Freeze task state when order completes/fails/stops
  freezeSchedulerState: () => {
    const state = get();
    console.log(`[Store] Freezing scheduler state for order ${state.schedulerCurrentOrderId}`);
    // Just save current state - it's already frozen by preventing updates in updateSchedulerTask
    saveSchedulerToStorage(state);
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
  fetchOrders: async (append = false) => {
    const state = get();
    const offset = append ? state.ordersOffset : 0;
    const limit = state.ordersPageSize;
    
    set(currentState => ({ 
      isLoading: true, 
      errors: { ...currentState.errors, orders: null }
    }));

    const result = await ordersAPI.fetchOrders(limit, offset);
    
    if (result.success) {
      set(state => ({ 
        orders: append ? [...state.orders, ...result.data] : result.data,
        ordersTotal: result.total,
        ordersOffset: append ? state.ordersOffset + result.data.length : result.data.length,
        ordersHasMore: result.hasMore,
        isLoading: false,
        systemStatus: { ...state.systemStatus, oms: 'online' }
      }));
      // Keep last plan on screen even if processing stopped (error/completed).
      // Only clear when a different order starts processing.
      try {
        const processing = (result.data || []).find(o => (o.status || '').toUpperCase() === 'PROCESSING');
        const currentOrderId = processing ? processing.id : null;
        const persistedOrderId = get().schedulerCurrentOrderId || null;
        
        if (currentOrderId && persistedOrderId && currentOrderId !== persistedOrderId) {
          // A different order began processing → reset until new plan arrives
          set({
            schedulerCurrentOrderId: currentOrderId,
            schedulerTasks: { Arm1: [], Arm2: [] },
            schedulerTaskStatus: {},
            schedulerStatusMessage: null,
            taskTimings: {} // Clear task timings for new order
          });
          clearSchedulerStorage();
        } else if (!currentOrderId && persistedOrderId) {
          // No order is currently processing, but we have persisted tasks
          // Check if the persisted order is in a terminal state
          const persistedOrder = (result.data || []).find(o => o.id === persistedOrderId);
          const isTerminalState = persistedOrder && 
            ['COMPLETED', 'ERROR', 'STOPPED', 'CANCELLED'].includes(persistedOrder.status?.toUpperCase());
          
          if (isTerminalState) {
            // Order is complete/failed/stopped - clear tasks to allow new orders
            console.log(`[Store] Clearing scheduler tasks for completed order ${persistedOrderId}`);
            set({
              schedulerCurrentOrderId: null,
              schedulerTasks: { Arm1: [], Arm2: [] },
              schedulerTaskStatus: {},
              schedulerStatusMessage: null,
              taskTimings: {}
            });
            clearSchedulerStorage();
          }
        }
      } catch {}
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

  loadMoreOrders: async () => {
    const state = get();
    if (!state.ordersHasMore || state.isLoading) {
      return;
    }
    
    addLog('API', 'info', `Loading more orders (offset: ${state.ordersOffset})...`);
    await get().fetchOrders(true); // true = append mode
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
    
    // Verify no other order is currently processing
    const state = get();
    const processingOrder = state.orders.find(o => ['PROCESSING', 'STOPPING'].includes(o.status?.toUpperCase()));
    if (processingOrder && processingOrder.id !== orderId) {
      addLog('API', 'error', `Cannot start order ${orderId} - Order ${processingOrder.id} is still ${processingOrder.status}`);
      return false;
    }
    
    // Optimistic update - mark order as PROCESSING
    set(state => ({
      orders: state.orders.map(order => {
        if (order.id === orderId) {
          return { ...order, status: 'PROCESSING', started_at: new Date().toISOString() };
        }
        return order;
      })
    }));
    
    const result = await ordersAPI.startOrder(orderId);
    
    if (result.success) {
      addLog('API', 'info', result.message);
      
      // Refresh orders to get the actual state from backend
      setTimeout(async () => {
        addLog('API', 'info', `Refreshing orders after starting order ${orderId}`);
        await get().fetchOrders(); // Refresh orders
      }, 500);
    } else {
      addLog('API', 'error', result.error, result.details);
      // Revert optimistic update on failure
      await get().fetchOrders();
    }

    return result.success;
  },

  stopOrder: async (orderId) => {
    addLog('API', 'info', `Stopping order ${orderId}...`);
    
    // Optimistic update: immediately change order status to STOPPING in UI
    set(state => ({
      orders: state.orders.map(order => 
        order.id === orderId 
          ? { ...order, status: 'STOPPING' }
          : order
      )
    }));
    
    const result = await ordersAPI.stopOrder(orderId);
    
    if (result.success) {
      addLog('API', 'info', result.message);
      
      // Refresh orders to get the actual state from backend
      setTimeout(async () => {
        addLog('API', 'info', `Refreshing orders after stopping order ${orderId}`);
        await get().fetchOrders();
      }, 500);
    } else {
      addLog('API', 'error', result.error, result.details);
      // Revert optimistic update on failure
      await get().fetchOrders();
    }

    return result.success;
  },

  resumeOrder: async (orderId) => {
    addLog('API', 'info', `Resuming order ${orderId}...`);
    
    // Optimistic update: immediately change order status to PROCESSING in UI
    set(state => ({
      orders: state.orders.map(order => 
        order.id === orderId 
          ? { ...order, status: 'PROCESSING' }
          : order
      )
    }));
    
    const result = await ordersAPI.resumeOrder(orderId);
    
    if (result.success) {
      addLog('API', 'info', result.message);
      
      // Refresh orders to get the actual state from backend
      setTimeout(async () => {
        addLog('API', 'info', `Refreshing orders after resuming order ${orderId}`);
        await get().fetchOrders();
      }, 500);
    } else {
      addLog('API', 'error', result.error, result.details);
      // Revert optimistic update on failure
      await get().fetchOrders();
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