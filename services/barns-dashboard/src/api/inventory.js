/**
 * Inventory API - Optimized
 * Uses base API client for DRY code with support for categorized inventory
 */

import { apiClient } from './base';

export const inventoryAPI = {


    fetchCategoryInfo: async () => {
    const result = await apiClient.get('/inventory/category-info', {}, {
      successMessage: 'Successfully fetched category information',
      timeout: 10000  // 10 second timeout for faster failure
    });

    if (result.success) {
      return {
        ...result,
        data: result.data?.inventory || result.data || {}
      };
    }

    return result;
  },

  fetchFullStockSummary: async () => {
    const result = await apiClient.get('/inventory/category-count', {}, {
      successMessage: 'Successfully fetched full stock summary',
      timeout: 10000  // 10 second timeout for faster failure
    });

    if (result.success) {
      return {
        ...result,
        data: result.data?.inventory || result.data || {}
      };
    }

    return result;
  },

  // Fetch full inventory status (all items)
  fetchStocklevel: async () => {
    const result = await apiClient.get('/inventory/stock-level', {}, {
      successMessage: 'Successfully fetched stock-level',
      timeout: 10000  // 10 second timeout for faster failure
    });
    
    if (result.success) {
      return {
        ...result,
        data: result.data?.inventory || result.data || {}
      };
    }
    
    return result;
  },


  // Fetch full inventory status (all items)
  fetchInventoryStatus: async () => {
    const result = await apiClient.get('/inventory/status', {}, {
      timeout: 10000,  // 10 second timeout for faster failure (was 30s)
      successMessage: 'Successfully fetched inventory status'
    });
    
    if (result.success) {
      return {
        ...result,
        data: result.data?.inventory || result.data || {}
      };
    }
    
    return result;
  },

  // Fetch inventory by category
  fetchCategoryInventory: async (category) => {
    const result = await inventoryAPI.fetchInventoryStatus();
    
    if (result.success) {
      // Import here to avoid circular dependency
      const { getCategoryItems } = await import('../utils/inventoryData');
      const categoryItems = getCategoryItems(category);
   
      const categoryData = {};
      
      Object.keys(categoryItems).forEach(itemKey => {
        if (result.data[itemKey]) {
          categoryData[itemKey] = result.data[itemKey];
        }
      });
      
      return {
        ...result,
        data: categoryData,
        message: `Successfully fetched ${category} inventory`
      };
    }
    
    return result;
  },

  // Get category summary (lowest level for dashboard)
  getCategorySummary: async () => {
    const result = await apiClient.get('/inventory/category-summary', {}, {
      successMessage: 'Successfully calculated category summary'
    });

    if (result.success) {
      return {
        ...result,
        data: result.data?.summary || result.data || {}
      };
    }
    
    return result;
  },

  // Refill specific inventory item
  refillInventory: (item, amount = 100) =>
    apiClient.post('/inventory/refill', { ingredient: item, amount }, {
      successMessage: `Successfully refilled ${item} to ${amount}%`
    }),

  // Refill entire category
  refillCategory: async (category, amount = 100) => {
    // Import here to avoid circular dependency
    const { getCategoryItems } = await import('../utils/inventoryData');
    const categoryItems = getCategoryItems(category);
    const results = [];
    
    for (const itemKey of Object.keys(categoryItems)) {
      const result = await inventoryAPI.refillInventory(itemKey, amount);
      results.push({ item: itemKey, result });
    }
    
    const successCount = results.filter(r => r.result.success).length;
    const totalCount = results.length;
    
    return {
      success: successCount === totalCount,
      data: results,
      message: `Refilled ${successCount}/${totalCount} items in ${category} category`
    };
  },

  // Get specific inventory item
  getInventoryItem: (item) =>
    apiClient.get(`/inventory/status/${item}`, {}, {
      successMessage: `Successfully fetched ${item} inventory status`
    }),

  // Update inventory thresholds
  updateThresholds: (item, thresholds) =>
    apiClient.put(`/inventory/${item}/thresholds`, thresholds, {
      successMessage: `Successfully updated ${item} thresholds`
    }),

  // Get low inventory items
  getLowInventoryItems: async () => {
    const result = await inventoryAPI.fetchInventoryStatus();
    
    if (result.success) {
      const lowItems = [];
      const { ALL_INVENTORY_ITEMS } = await import('../utils/inventoryData');
      
      Object.entries(result.data).forEach(([itemKey, itemData]) => {
        if (itemData.level === 'low') {
          lowItems.push({
            key: itemKey,
            name: ALL_INVENTORY_ITEMS[itemKey]?.name || itemKey,
            category: ALL_INVENTORY_ITEMS[itemKey]?.category,
            ...itemData
          });
        }
      });
      
      return {
        success: true,
        data: lowItems,
        message: `Found ${lowItems.length} low inventory items`
      };
    }
    
    return result;
  }
}; 