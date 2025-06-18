/**
 * WebSocket Utility Module
 * Handles all WebSocket connections and messaging
 */

import { API_CONFIG, WS_CONFIG } from './config';
import { addLog } from '../store/logsStore';

export class WebSocketManager {
  constructor() {
    this.connections = new Map();
    this.reconnectAttempts = new Map();
  }

  /**
   * Connect to a WebSocket endpoint
   */
  connect(name, endpoint, handlers = {}) {
    // Close existing connection if it exists
    this.disconnect(name);

    try {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      
      // If endpoint is empty, connect directly to WEBSOCKET_BASE
      const fullEndpoint = endpoint ? `${API_CONFIG.WEBSOCKET_BASE}${endpoint}` : API_CONFIG.WEBSOCKET_BASE;
      const wsUrl = `${protocol}//${window.location.host}${fullEndpoint}`;
      
      addLog('WebSocket', 'info', `Connecting to WebSocket: ${wsUrl}`);
      const ws = new WebSocket(wsUrl);

      const connection = {
        ws,
        endpoint: fullEndpoint,
        handlers,
        pingInterval: null,
        status: 'connecting'
      };

      // Set up event handlers
      ws.onopen = () => {
        connection.status = 'connected';
        this.reconnectAttempts.set(name, 0);
        addLog('WebSocket', 'info', `${name} WebSocket connected to ${fullEndpoint}`);
        
        if (handlers.onOpen) handlers.onOpen();

        // Start ping interval
        connection.pingInterval = setInterval(() => {
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ 
              type: 'ping', 
              timestamp: new Date().toISOString() 
            }));
          }
        }, WS_CONFIG.HEARTBEAT_INTERVAL);
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          
          if (data.type === 'pong') {
            // Handle pong responses silently
            return;
          }

          addLog('WebSocket', 'info', `${name} received message: ${data.type || 'unknown'}`, data);
          
          if (handlers.onMessage) {
            handlers.onMessage(data);
          }
        } catch (error) {
          addLog('WebSocket', 'error', `Failed to parse ${name} WebSocket message`, error);
        }
      };

      ws.onclose = (event) => {
        connection.status = 'disconnected';
        addLog('WebSocket', 'warning', `${name} WebSocket disconnected (code: ${event.code}, reason: ${event.reason})`);
        
        if (connection.pingInterval) {
          clearInterval(connection.pingInterval);
        }

        if (handlers.onClose) handlers.onClose();

        // Attempt to reconnect
        this.attemptReconnect(name, endpoint, handlers);
      };

      ws.onerror = (error) => {
        addLog('WebSocket', 'error', `${name} WebSocket error`, error);
        if (handlers.onError) handlers.onError(error);
      };

      this.connections.set(name, connection);
      return ws;

    } catch (error) {
      addLog('WebSocket', 'error', `Failed to create ${name} WebSocket connection`, error);
      return null;
    }
  }

  /**
   * Disconnect a WebSocket connection
   */
  disconnect(name) {
    const connection = this.connections.get(name);
    if (connection) {
      if (connection.pingInterval) {
        clearInterval(connection.pingInterval);
      }
      if (connection.ws && connection.ws.readyState === WebSocket.OPEN) {
        connection.ws.close();
      }
      this.connections.delete(name);
    }
  }

  /**
   * Disconnect all WebSocket connections
   */
  disconnectAll() {
    for (const name of this.connections.keys()) {
      this.disconnect(name);
    }
  }

  /**
   * Send a message through a WebSocket connection
   */
  send(name, message) {
    const connection = this.connections.get(name);
    if (connection && connection.ws.readyState === WebSocket.OPEN) {
      connection.ws.send(JSON.stringify(message));
      return true;
    }
    return false;
  }

  /**
   * Get connection status
   */
  getStatus(name) {
    const connection = this.connections.get(name);
    return connection ? connection.status : 'disconnected';
  }

  /**
   * Attempt to reconnect
   */
  attemptReconnect(name, endpoint, handlers) {
    const attempts = this.reconnectAttempts.get(name) || 0;
    
    if (attempts < WS_CONFIG.MAX_RECONNECT_ATTEMPTS) {
      setTimeout(() => {
        this.reconnectAttempts.set(name, attempts + 1);
        addLog('WebSocket', 'info', `Attempting to reconnect ${name} (attempt ${attempts + 1})`);
        this.connect(name, endpoint, handlers);
      }, WS_CONFIG.RECONNECT_INTERVAL);
    } else {
      addLog('WebSocket', 'error', `Max reconnection attempts reached for ${name}`);
    }
  }
}

// Global WebSocket manager instance
export const wsManager = new WebSocketManager(); 