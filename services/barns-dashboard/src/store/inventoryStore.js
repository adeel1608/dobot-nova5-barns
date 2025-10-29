/**
 * Inventory Store
 * Manages categorized inventory state and operations
 */

import { create } from 'zustand';
import { inventoryAPI } from '../api';
import { addLog } from './logsStore';
import { INVENTORY_CATEGORIES, getCategoryItems, ALL_INVENTORY_ITEMS, CATEGORY_INFO } from '../utils/inventoryData';

export const useInventoryStore = create((set, get) => ({
  // State - Now supports full categorized inventory
  
  FullCategoryInfo:{},
  FullStockSummary:{},
  inventoryStatus: {},
  inventoryStockLevel: {},
  categorySummary: {
    milk: { level: 'unknown', numeric: 0, last_refilled: null },
    coffee_beans: { level: 'unknown', numeric: 0, last_refilled: null },
    syrups: { level: 'unknown', numeric: 0, last_refilled: null },
    cups: { level: 'unknown', numeric: 0, last_refilled: null },
    sauces: { level: 'unknown', numeric: 0, last_refilled: null },
    premixes: { level: 'unknown', numeric: 0, last_refilled: null }
  },
  isLoading: false,
  errors: {
    inventory: null,
    refill: null
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
          inventory: null,
          refill: null
        }
      });
    }
  },

  updateInventoryData: (newInventory) => {
  set((state) => ({
    inventoryStatus: {
      ...state.inventoryStatus,
      ...newInventory
    }
  }));
},


  // Fetch full categories  info
  fetchCategoryInfoData: async () => {
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, inventory: null }
    }));

    const result = await inventoryAPI.fetchCategoryInfo();
    //console.log('🧠 Inventory full categories info:', result);
    if (result.success) {
      set(state => ({ 
        FullCategoryInfo: result.data, 
        isLoading: false
      }));
      addLog('API', 'info', result.message);

    } else {
      set(state => ({ 
        FullCategoryInfo: {},
        isLoading: false,
        errors: { ...state.errors, inventory: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  
  },
  // Fetch full Stock level info
  fetchFullStockSummaryData: async () => {
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, inventory: null }
    }));

    const result = await inventoryAPI.fetchFullStockSummary();
   
    if (result.success) {
      set(state => ({ 
        FullStockSummary: result.data.details, 
        isLoading: false
      }));
      addLog('API', 'info', result.message);

    } else {
      set(state => ({ 
        FullStockSummary: {},
        isLoading: false,
        errors: { ...state.errors, inventory: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  
  },
  // Fetch Stock level (all items)
  fetchStockLevelData: async () => {
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, inventory: null }
    }));

    const result = await inventoryAPI.fetchStocklevel();
    //console.log('🧠 Inventory stock level fetched:', result);
    if (result.success) {
      set(state => ({ 
        inventoryStockLevel: result.data, 
        isLoading: false
      }));
      addLog('API', 'info', result.message);
      
      // Also update category summary
      // await get().updateCategorySummary();
    } else {
      set(state => ({ 
        inventoryStockLevel: {},
        isLoading: false,
        errors: { ...state.errors, inventory: result.error }
      }));
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  
  },
  // Fetch full inventory status (all items)
  fetchInventoryStatus: async () => {
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, inventory: null }
    }));

    const result = await inventoryAPI.fetchInventoryStatus();
    if (result.success) {
      const itemCount = Object.keys(result.data || {}).length;
      set(state => ({ 
        inventoryStatus: result.data, 
        isLoading: false
      }));
      addLog('API', 'info', `Inventory refreshed: ${itemCount} items loaded`);
      
      // Also update category summary (non-blocking)
      get().updateCategorySummary();
    } else {
      set(state => ({ 
        inventoryStatus: {},
        isLoading: false,
        errors: { ...state.errors, inventory: result.error }
      }));
      addLog('API', 'error', `Inventory fetch failed: ${result.error}`, result.details);
    }

    return result.data;
  },

  // Update category summary for dashboard
  
  updateCategorySummary: async () => {
    const result = await inventoryAPI.getCategorySummary();
    if (result.success) {
      const lowCategories = Object.entries(result.data || {})
        .filter(([_, data]) => data.level === 'low')
        .map(([name]) => name);
        
      set(state => ({
        categorySummary: { ...state.categorySummary, ...result.data }
      }));
      
      if (lowCategories.length > 0) {
        addLog('API', 'warning', `Category summary updated - ${lowCategories.length} categories low: ${lowCategories.join(', ')}`);
      } else {
        addLog('API', 'info', 'Category summary updated - all categories OK');
      }
    } else {
      addLog('API', 'error', `Category summary update failed: ${result.error}`, result.details);
    }
    
    return result.data;
  },

  // Fetch inventory by category
  fetchCategoryInventory: async (category) => {
    const result = await inventoryAPI.fetchCategoryInventory(category);
    
    if (result.success) {
      set(state => ({
        inventoryStatus: { ...state.inventoryStatus, ...result.data }
      }));
      addLog('API', 'info', result.message);
    } else {
      addLog('API', 'error', result.error, result.details);
    }

    return result.data;
  },

  // Refill specific inventory item
  refillInventory: async (item, amount = 100) => {
    const result = await inventoryAPI.refillInventory(item, amount);
    
    if (result.success) {
      addLog('API', 'info', result.message);
      // Refresh inventory status after refill
      await get().fetchInventoryStatus();
    } else {
      addLog('API', 'error', result.error, result.details);
      set(state => ({
        errors: { ...state.errors, refill: result.error }
      }));
    }

    return result.success;
  },

  // Refill entire category
  refillCategory: async (category, amount = 100) => {
    set(state => ({ 
      isLoading: true, 
      errors: { ...state.errors, refill: null }
    }));

    const result = await inventoryAPI.refillCategory(category, amount);
    
    if (result.success) {
      addLog('API', 'info', result.message);
      // Refresh inventory status after category refill
      await get().fetchInventoryStatus();
    } else {
      addLog('API', 'error', result.error, result.details);
      set(state => ({
        errors: { ...state.errors, refill: result.error }
      }));
    }

    set({ isLoading: false });
    return result.success;
  },

  // Get inventory item details
  getInventoryItem: async (item) => {
    const result = await inventoryAPI.getInventoryItem(item);
    
    if (result.success) {
      addLog('API', 'info', result.message);
    } else {
      addLog('API', 'error', result.error, result.details);
    }

    return result.success ? result.data : null;
  },

  // Update inventory thresholds
  updateThresholds: async (item, thresholds) => {
    const result = await inventoryAPI.updateThresholds(item, thresholds);
    
    if (result.success) {
      addLog('API', 'info', result.message);
      // Refresh inventory status after threshold update
      await get().fetchInventoryStatus();
    } else {
      addLog('API', 'error', result.error, result.details);
    }

    return result.success;
  },

  // Get items by category from current state
  getItemsByCategory: (category) => {
    const { inventoryStatus } = get();
    //console.log("🧠 Getting items for category:", category);
    const categoryItems = getCategoryItems(category);
    const result = {};
    
    Object.keys(categoryItems).forEach(itemKey => {
      if (inventoryStatus[itemKey]) {
        result[itemKey] = {
          ...inventoryStatus[itemKey],
          ...categoryItems[itemKey]
        };
      }
    });
    
    return result;
  },

  // Get low inventory items
  getLowInventoryItems: () => {
    const { inventoryStatus } = get();
    return Object.entries(inventoryStatus)
      .filter(([_, data]) => data.level === 'low')
      .map(([itemKey, data]) => ({
        key: itemKey,
        name: ALL_INVENTORY_ITEMS[itemKey]?.name || itemKey,
        category: ALL_INVENTORY_ITEMS[itemKey]?.category,
        ...data
      }));
  },

  // Get inventory statistics by category
  getInventoryStatsByCategory: () => {
    const { inventoryStatus } = get();
    const stats = {};
    //console.log('🧠 Inventory stats by category:', inventoryStatus);
    Object.values(INVENTORY_CATEGORIES).forEach(category => {
      const categoryItems = getCategoryItems(category);
      const categoryStats = {
        total: Object.keys(categoryItems).length,
        low: 0,
        medium: 0,
        high: 0,
        unknown: 0
      };
      
      Object.keys(categoryItems).forEach(itemKey => {
        const itemData = inventoryStatus[itemKey];
        if (itemData) {
          categoryStats[itemData.level] = (categoryStats[itemData.level] || 0) + 1;
        } else {
          categoryStats.unknown += 1;
        }
      });
      
      stats[category] = categoryStats;
    });
    
    return stats;
  },

  // Get overall inventory statistics
  getInventoryStats: () => {
    const { inventoryStatus } = get();
    const stats = {
      total: Object.keys(ALL_INVENTORY_ITEMS).length,
      low: 0,
      medium: 0,
      high: 0,
      unknown: 0
    };

    Object.values(inventoryStatus).forEach(item => {
      stats[item.level] = (stats[item.level] || 0) + 1;
    });

    // Count unknown items
    stats.unknown = stats.total - (stats.low + stats.medium + stats.high);

    return stats;
  },

  // Check if any items need refilling
  hasLowInventory: () => {
    const { inventoryStatus } = get();
    return Object.values(inventoryStatus).some(item => item.level === 'low');
  },

  // Check if category has low inventory
  categoryHasLowInventory: (category) => {
    const items = get().getItemsByCategory(category);
    return Object.values(items).some(item => item.level === 'low');
  },

  // Get category progress (for dashboard indicators)
  getCategoryProgress: (category) => {
    const { categorySummary } = get();
    return categorySummary[category] || { level: 'unknown', numeric: 0 };
  }
})); 