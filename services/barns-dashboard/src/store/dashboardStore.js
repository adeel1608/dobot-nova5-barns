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
    
    // Log task statuses for debugging
    const allTasks = [...(state.schedulerTasks.Arm1 || []), ...(state.schedulerTasks.Arm2 || [])];
    const statusCounts = allTasks.reduce((acc, t) => {
      acc[t.status] = (acc[t.status] || 0) + 1;
      return acc;
    }, {});
    console.log(`[Store] Saving to localStorage - Order: ${payload.orderId}, Tasks:`, statusCounts);
    
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify(payload));
  } catch {}
};
const clearSchedulerStorage = () => {
  try {
    if (typeof window === 'undefined') return;
    window.localStorage.removeItem(STORAGE_KEY);
  } catch {}
};

export const useDashboardStore = create((set, get) => {
  // Load scheduler state from storage once
  const savedSchedulerState = loadSchedulerFromStorage();
  
  return {
  // State
  orders: [],
  ordersTotal: 0,
  ordersOffset: 0,
  ordersHasMore: false,
  ordersPageSize: 20,
  orderStats: {
    total: 0,
    processing: 0,
    queued: 0,
    completed: 0,
    stopped: 0,
    error: 0,
    halted: 0,
    cancelled: 0,
    stopping: 0
  },
  recipes: [],
  menuItems: [],
  ingredientsByCategory: {},
  // Hydrate scheduler view from storage to persist across refresh
  schedulerCurrentOrderId: savedSchedulerState?.orderId || null,
  schedulerTasks: savedSchedulerState?.schedulerTasks || { Arm1: [], Arm2: [] },
  schedulerTaskStatus: savedSchedulerState?.schedulerTaskStatus || {},
  taskTimings: savedSchedulerState?.taskTimings || {},
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
    const state = get();
    
    // Don't reset plan if it's for the same order that's already being tracked
    // This prevents resetting tasks that have already been updated with completion status
    if (state.schedulerCurrentOrderId === orderId && state.schedulerTasks) {
      // Check if we have any non-pending tasks
      const allTasks = [...(state.schedulerTasks.Arm1 || []), ...(state.schedulerTasks.Arm2 || [])];
      const hasNonPendingTasks = allTasks.some(t => t.status !== 'pending');
      
      if (hasNonPendingTasks) {
        console.log(`[Store] Ignoring duplicate plan for order ${orderId} - tasks already in progress/completed`);
        return;
      }
    }
    
    // Don't reset plan if it's for an order that's already completed/frozen
    const existingOrder = state.orders.find(o => o.id === orderId);
    const isAlreadyCompleted = existingOrder && 
      ['COMPLETED', 'ERROR', 'STOPPED', 'CANCELLED'].includes(existingOrder.status?.toUpperCase());
    
    if (isAlreadyCompleted) {
      console.log(`[Store] Ignoring plan for already completed order ${orderId} (status: ${existingOrder.status})`);
      return;
    }
    
    // plan: { Arm1: [[action, cup_id], ...], Arm2: [...] }
    const format = (arr) => (arr || []).map(([action, cup]) => ({ action, cup_id: cup, status: 'pending' }));
    console.log(`[Store] Setting new plan for order ${orderId}`);
    set(state => {
      const newState = {
        ...state,
        schedulerCurrentOrderId: orderId,
        schedulerTasks: {
          Arm1: format(plan.Arm1),
          Arm2: format(plan.Arm2)
        },
        schedulerTaskStatus: {},
        taskTimings: {} // Clear timings when new plan starts
      };
      
      // Save to storage using the new state directly, not get()
      saveSchedulerToStorage(newState);
      
      return {
        schedulerCurrentOrderId: newState.schedulerCurrentOrderId,
        schedulerTasks: newState.schedulerTasks,
        schedulerTaskStatus: {},
        taskTimings: {}
      };
    });
  },

  updateSchedulerTask: ({ cup_id, action, success, message }) => {
    const key = `${cup_id}:${action}`;
    const newStatus = success === true ? 'completed' : success === false ? 'failed' : 'in_progress';
    console.log(`[Store] Updating task ${cup_id}:${action} to ${newStatus}`);
    
    set(state => {
      const updateList = (list) => list.map(t => {
        if (t.cup_id === cup_id && t.action === action) {
          // Preserve terminal states - don't overwrite completed, failed, or cancelled
          const isTerminal = t.status === 'completed' || t.status === 'failed' || t.status === 'cancelled';
          if (isTerminal) {
            console.log(`[Store] Task ${cup_id}:${action} already in terminal state: ${t.status}`);
            return t; // Keep the existing terminal state
          }
          // Update to new status if not terminal
          console.log(`[Store] Task ${cup_id}:${action} updated from ${t.status} to ${newStatus}`);
          return { 
            ...t, 
            status: newStatus, 
            message 
          };
        }
        return t;
      });
      
      const newState = {
        ...state,
        schedulerTasks: {
          Arm1: updateList(state.schedulerTasks.Arm1),
          Arm2: updateList(state.schedulerTasks.Arm2)
        },
        schedulerTaskStatus: {
          ...state.schedulerTaskStatus,
          [key]: { success, message, status: newStatus }
        }
      };
      
      // Save to storage using the new state directly, not get()
      saveSchedulerToStorage(newState);
      
      return {
        schedulerTasks: newState.schedulerTasks,
        schedulerTaskStatus: newState.schedulerTaskStatus
      };
    });
  },

  setSchedulerStatusMessage: (message, statusObj) => {
    set(state => {
      const newState = { ...state, schedulerStatusMessage: message };
      // Save to storage using the new state directly, not get()
      saveSchedulerToStorage(newState);
      return { schedulerStatusMessage: message };
    });
  },

  updateTaskTiming: (taskKey, timing) => {
    set(state => {
      const newState = {
        ...state,
        taskTimings: {
          ...state.taskTimings,
          [taskKey]: timing
        }
      };
      // Save to storage using the new state directly, not get()
      saveSchedulerToStorage(newState);
      return {
        taskTimings: {
          ...state.taskTimings,
          [taskKey]: timing
        }
      };
    });
  },

  // Mark all non-final tasks as cancelled after an order-level failure
  finalizeSchedulerAsFailed: (reason) => {
    set(state => {
      const cancelList = (list) => list.map(t => {
        if (t.status === 'completed' || t.status === 'failed') return t;
        if (t.status === 'cancelled') return t;
        return { ...t, status: 'cancelled' };
      });
      
      const newState = {
        ...state,
        schedulerTasks: {
          Arm1: cancelList(state.schedulerTasks.Arm1),
          Arm2: cancelList(state.schedulerTasks.Arm2)
        },
        schedulerStatusMessage: reason ? String(reason) : state.schedulerStatusMessage
      };
      
      // Save to storage using the new state directly, not get()
      saveSchedulerToStorage(newState);
      
      return {
        schedulerTasks: newState.schedulerTasks,
        schedulerStatusMessage: newState.schedulerStatusMessage
      };
    });
  },

  // Freeze task state when order completes/fails/stops
  freezeSchedulerState: () => {
    // Use set with a callback to safely access and save state
    set(state => {
      console.log(`[Store] Freezing scheduler state for order ${state.schedulerCurrentOrderId}`);
      // Just save current state - it's already frozen by preventing updates in updateSchedulerTask
      saveSchedulerToStorage(state);
      // Return empty object since we're not changing anything
      return {};
    });
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
  fetchOrderStats: async () => {
    const result = await ordersAPI.fetchOrderStats();
    
    if (result.success) {
      const stats = result.data || {};
      const totalOrders = stats.total || 0;
      const completedOrders = stats.completed || 0;
      set({ orderStats: result.data });
      // Removed verbose INFO log - only log errors
    } else {
      addLog('API', 'error', 'Failed to fetch order statistics', result.error);
    }
    
    return result.data;
  },

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
      // Only clear when a different order starts processing or the displayed order is deleted.
      try {
        const processing = (result.data || []).find(o => (o.status || '').toUpperCase() === 'PROCESSING');
        const currentOrderId = processing ? processing.id : null;
        const persistedOrderId = get().schedulerCurrentOrderId || null;
        
        if (currentOrderId && persistedOrderId && currentOrderId !== persistedOrderId) {
          // A different order began processing → reset until new plan arrives
          console.log(`[Store] New order ${currentOrderId} started, clearing previous order ${persistedOrderId} tasks`);
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
          // Check if the persisted order still exists (if not, it was deleted)
          const persistedOrder = (result.data || []).find(o => o.id === persistedOrderId);
          
          if (!persistedOrder) {
            // Order was deleted - clear tasks
            console.log(`[Store] Displayed order ${persistedOrderId} was deleted, clearing tasks`);
            set({
              schedulerCurrentOrderId: null,
              schedulerTasks: { Arm1: [], Arm2: [] },
              schedulerTaskStatus: {},
              schedulerStatusMessage: null,
              taskTimings: {}
            });
            clearSchedulerStorage();
          }
          // If order still exists but is completed/failed/stopped, keep the tasks visible
        }
      } catch {}
      // Removed verbose INFO log - only log errors
      
      // Fetch updated statistics whenever orders are fetched
      get().fetchOrderStats();
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
    
    // Removed verbose INFO log - only log errors
    await get().fetchOrders(true); // true = append mode
  },

  createOrder: async (orderData) => {
    // Removed verbose INFO log - only log errors
    const result = await ordersAPI.createOrder(orderData);
    
    if (result.success) {
      // Removed verbose INFO log - only log errors
      
      // Add a small delay to ensure backend has time to update
      setTimeout(async () => {
        await get().fetchOrders(); // Refresh orders
      }, 500);
    } else {
      addLog('API', 'error', result.error, result.details);
    }

    return result.success;
  },

  processPOSOrder: async (posOrderData) => {
    // Removed verbose INFO log - only log errors
    const result = await ordersAPI.processPOSOrder(posOrderData);
    
    if (result.success) {
      // Removed verbose INFO log - only log errors
      
      // Add a small delay to ensure backend has time to update
      setTimeout(async () => {
        await get().fetchOrders(); // Refresh orders
      }, 500);
    } else {
      addLog('API', 'error', result.error, result.details);
    }

    return result.success;
  },

  fetchMenuItems: async () => {
    // Removed verbose INFO log - only log errors
    const result = await ordersAPI.fetchMenuItems();
    
    if (result.success) {
      set({ menuItems: result.data });
      // Removed verbose INFO log - only log errors
    } else {
      addLog('API', 'error', result.error);
    }
    
    return result.data;
  },

  fetchIngredientsByCategory: async () => {
    // Removed verbose INFO log - only log errors
    const result = await ordersAPI.fetchIngredients();
    
    if (result.success) {
      set({ ingredientsByCategory: result.data });
      // Removed verbose INFO log - only log errors
    } else {
      addLog('API', 'error', result.error);
    }
    
    return result.data;
  },

  startOrder: async (orderId) => {
    // Removed verbose INFO log - only log errors
    
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
      // Removed verbose INFO log - only log errors
      
      // Refresh orders to get the actual state from backend
      setTimeout(async () => {
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
    // Removed verbose INFO log - only log errors
    
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
      // Removed verbose INFO log - only log errors
      
      // Refresh orders to get the actual state from backend
      setTimeout(async () => {
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
    // Removed verbose INFO log - only log errors
    
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
      // Removed verbose INFO log - only log errors
      
      // Refresh orders to get the actual state from backend
      setTimeout(async () => {
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
    // Removed verbose INFO log - only log errors
    const result = await ordersAPI.deleteOrder(orderId);
    
    if (result.success) {
      // Removed verbose INFO log - only log errors
      
      // Add a small delay to ensure backend has time to update
      setTimeout(async () => {
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
      // Removed verbose INFO log - only log errors
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
      // Removed verbose INFO log - only log errors
    } else {
      set(state => ({
        systemStatus: { ...state.systemStatus, ...result.data }
      }));
      addLog('API', 'error', `Health check failed: ${result.error}`, result.details);
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
      // Removed verbose INFO log - only log errors
    } else {
      set(state => ({ 
        schedulerStatus: null,
        systemStatus: { ...state.systemStatus, scheduler: 'offline' },
        errors: { ...state.errors, scheduler: result.error }
      }));
      addLog('API', 'error', `Scheduler status fetch failed: ${result.error}`, result.details);
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
      // Removed verbose INFO log - only log errors
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
};
}); 