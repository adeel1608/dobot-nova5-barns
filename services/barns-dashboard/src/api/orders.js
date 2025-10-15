/**
 * Orders API - Optimized
 * Uses base API client for DRY code
 */

import { apiClient } from './base';

export const ordersAPI = {
  // Fetch all orders with pagination support
  fetchOrders: (limit = null, offset = 0) => {
    const params = {};
    if (limit !== null) params.limit = limit;
    if (offset > 0) params.offset = offset;
    
    return apiClient.getList('/orders', params, 'orders')
      .then(result => {
        const response = result.data || {};
        const ordersList = Array.isArray(response.orders)
          ? response.orders
          : Array.isArray(response)
            ? response
            : [];
        const total = typeof response.total === 'number' ? response.total : ordersList.length;
        const resolvedLimit = typeof response.limit === 'number' ? response.limit : limit;
        const resolvedOffset = typeof response.offset === 'number' ? response.offset : offset;
        const hasMore =
          typeof response.has_more === 'boolean'
            ? response.has_more
            : typeof response.hasMore === 'boolean'
              ? response.hasMore
              : (resolvedOffset + ordersList.length) < total;

        return {
          ...result,
          data: ordersList,
          total,
          limit: resolvedLimit,
          offset: resolvedOffset,
          hasMore
        };
      });
  },

  // Create new order  
  createOrder: (orderData) =>
    apiClient.create('/orders', orderData, 'Order')
      .then(result => ({
        ...result,
        message: result.success ? `Order created successfully with ID: ${result.data?.order_id}` : result.message
      })),

  // Start processing order
  startOrder: (orderId) =>
    apiClient.patch(`/orders/${orderId}/start`, {}, {
      successMessage: `Order ${orderId} started successfully`
    }),

  // Stop processing order
  stopOrder: (orderId) =>
    apiClient.create(`/orders/${orderId}/stop`, {}, {
      successMessage: `Order ${orderId} stopped successfully`
    }),

  // Resume stopped/halted order
  resumeOrder: (orderId) =>
    apiClient.create(`/orders/${orderId}/resume`, {}, {
      successMessage: `Order ${orderId} resumed successfully`
    }),

  // Delete order
  deleteOrder: (orderId) =>
    apiClient.remove('/orders', orderId, 'Order'),

  // Reorder queue
  reorderQueue: (orderIds) =>
    apiClient.put('/queue/reorder', { order_ids: orderIds }, {
      successMessage: 'Queue reordered successfully'
    }),

  // Process POS order (uses POS transaction format)
  processPOSOrder: (posOrderData) =>
    apiClient.create('/pos/process-order', posOrderData, 'POS Order')
      .then(result => ({
        ...result,
        message: result.success ? 'POS Order processed successfully' : result.message
      })),

  // Fetch menu items with default ingredients
  fetchMenuItems: () =>
    apiClient.getList('/pos/menu-items', {}, 'menu items')
      .then(result => ({
        ...result,
        data: result.data?.menu_items || []
      })),

  // Fetch available ingredients by category
  fetchIngredients: () =>
    apiClient.getList('/pos/ingredients', {}, 'ingredients')
      .then(result => ({
        ...result,
        data: result.data?.ingredients_by_category || {}
      })),

  // Fetch order statistics
  fetchOrderStats: () =>
    apiClient.getList('/orders/stats/summary', {}, 'order statistics')
      .then(result => ({
        ...result,
        data: result.data || {}
      }))
}; 