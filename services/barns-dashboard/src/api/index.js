/**
 * Centralized API Index
 * Exports all optimized API modules
 */

// Export base client for direct usage if needed
export { apiClient, videoClient } from './base';

// Export all API modules
export { ordersAPI } from './orders';
export { alertsAPI } from './alerts';
export { inventoryAPI } from './inventory';
export { systemAPI } from './system';
export { camerasAPI } from './cameras';
export { logsAPI } from './logs';
export { recipesAPI } from './recipes';

// Create a unified API object for convenience
export const api = {
  orders: () => import('./orders').then(m => m.ordersAPI),
  alerts: () => import('./alerts').then(m => m.alertsAPI),
  inventory: () => import('./inventory').then(m => m.inventoryAPI),
  system: () => import('./system').then(m => m.systemAPI),
  cameras: () => import('./cameras').then(m => m.camerasAPI),
  logs: () => import('./logs').then(m => m.logsAPI),
  recipes: () => import('./recipes').then(m => m.recipesAPI)
};

// Default export for legacy compatibility
export default {
  ordersAPI: () => import('./orders').then(m => m.ordersAPI),
  alertsAPI: () => import('./alerts').then(m => m.alertsAPI),
  inventoryAPI: () => import('./inventory').then(m => m.inventoryAPI),
  systemAPI: () => import('./system').then(m => m.systemAPI),
  camerasAPI: () => import('./cameras').then(m => m.camerasAPI),
  logsAPI: () => import('./logs').then(m => m.logsAPI),
  recipesAPI: () => import('./recipes').then(m => m.recipesAPI)
}; 