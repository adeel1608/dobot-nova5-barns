/**
 * Base API Client
 * Centralized API handling with common patterns and error management
 */

import axios from 'axios';
import { API_CONFIG } from '../utils/config';
import { extractErrorMessage } from '../utils/errorHandler';

class APIClient {
  constructor(baseURL = API_CONFIG.API_BASE, defaultTimeout = 10000) {
    this.client = axios.create({
      baseURL,
      timeout: defaultTimeout,
      headers: {
        'Content-Type': 'application/json',
      }
    });

    // Request interceptor for logging
    this.client.interceptors.request.use(
      (config) => {
        console.log(`API Request: ${config.method?.toUpperCase()} ${config.url}`);
        return config;
      },
      (error) => {
        console.error('API Request Error:', error);
        return Promise.reject(error);
      }
    );

    // Response interceptor for logging
    this.client.interceptors.response.use(
      (response) => {
        console.log(`API Response: ${response.status} ${response.config.url}`);
        return response;
      },
      (error) => {
        console.error(`API Error: ${error.response?.status} ${error.config?.url}`, error);
        return Promise.reject(error);
      }
    );
  }

  /**
   * Generic API request handler with standardized response format
   */
  async request(config, operation, successMessage, defaultData = null) {
    try {
      const response = await this.client(config);
      
      return {
        success: true,
        data: response.data?.data || response.data || defaultData,
        message: successMessage || 'Operation completed successfully',
        status: response.status
      };
    } catch (error) {
      return {
        success: false,
        data: defaultData,
        error: extractErrorMessage(error),
        details: {
          operation,
          status: error.response?.status,
          technical_details: error.message,
          endpoint: config.url
        }
      };
    }
  }

  /**
   * GET request
   */
  async get(url, params = {}, options = {}) {
    const operation = `get_${url.replace(/[^a-zA-Z0-9]/g, '_')}`;
    return this.request(
      { method: 'GET', url, params, ...options },
      operation,
      options.successMessage
    );
  }

  /**
   * POST request
   */
  async post(url, data = {}, options = {}) {
    const operation = `post_${url.replace(/[^a-zA-Z0-9]/g, '_')}`;
    return this.request(
      { method: 'POST', url, data, ...options },
      operation,
      options.successMessage
    );
  }

  /**
   * PUT request
   */
  async put(url, data = {}, options = {}) {
    const operation = `put_${url.replace(/[^a-zA-Z0-9]/g, '_')}`;
    return this.request(
      { method: 'PUT', url, data, ...options },
      operation,
      options.successMessage
    );
  }

  /**
   * PATCH request
   */
  async patch(url, data = {}, options = {}) {
    const operation = `patch_${url.replace(/[^a-zA-Z0-9]/g, '_')}`;
    return this.request(
      { method: 'PATCH', url, data, ...options },
      operation,
      options.successMessage
    );
  }

  /**
   * DELETE request
   */
  async delete(url, options = {}) {
    const operation = `delete_${url.replace(/[^a-zA-Z0-9]/g, '_')}`;
    return this.request(
      { method: 'DELETE', url, ...options },
      operation,
      options.successMessage
    );
  }

  /**
   * Helper for list operations with count
   */
  async getList(url, params = {}, itemName = 'items') {
    const result = await this.get(url, params);
    if (result.success && Array.isArray(result.data)) {
      result.message = `Successfully fetched ${result.data.length} ${itemName}`;
    }
    return result;
  }

  /**
   * Helper for operations with ID
   */
  async getById(url, id, itemName = 'item') {
    return this.get(`${url}/${id}`, {}, {
      successMessage: `Successfully fetched ${itemName} ${id}`
    });
  }

  /**
   * Helper for create operations
   */
  async create(url, data, itemName = 'item') {
    return this.post(url, data, {
      successMessage: `${itemName} created successfully`
    });
  }

  /**
   * Helper for update operations
   */
  async update(url, id, data, itemName = 'item') {
    return this.put(`${url}/${id}`, data, {
      successMessage: `${itemName} ${id} updated successfully`
    });
  }

  /**
   * Helper for delete operations
   */
  async remove(url, id, itemName = 'item') {
    return this.delete(`${url}/${id}`, {
      successMessage: `${itemName} ${id} deleted successfully`
    });
  }
}

// Create singleton instances for different endpoints
export const apiClient = new APIClient();
export const videoClient = new APIClient(API_CONFIG.VIDEO_STREAM, 5000);

export default APIClient; 