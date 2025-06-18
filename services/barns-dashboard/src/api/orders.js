/**
 * Orders API - Optimized
 * Uses base API client for DRY code
 */

import { apiClient } from './base';

export const ordersAPI = {
  // Fetch all orders
  fetchOrders: () => 
    apiClient.getList('/orders', {}, 'orders')
      .then(result => ({
        ...result,
        data: result.data?.orders || result.data || []
      })),

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

  // Delete order
  deleteOrder: (orderId) =>
    apiClient.remove('/orders', orderId, 'Order'),

  // Reorder queue
  reorderQueue: (orderIds) =>
    apiClient.put('/queue/reorder', { order_ids: orderIds }, {
      successMessage: 'Queue reordered successfully'
    })
}; 