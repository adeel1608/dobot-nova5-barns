/**
 * Error Handler Utility
 * Provides consistent error handling and user-friendly error messages
 */

/** Message shown when service is offline; UI can match this to show translated text. */
export const SERVICE_OFFLINE_MESSAGE = 'Service is offline or unreachable. Please check if the service is running.';

/**
 * Extract user-friendly error messages from API errors
 */
export function extractErrorMessage(error) {
  // Network/Connection errors
  if (error.code === 'ECONNREFUSED' || error.code === 'ERR_NETWORK') {
    return SERVICE_OFFLINE_MESSAGE;
  }
  
  if (error.code === 'ENOTFOUND') {
    return 'Service not found. Please verify the service configuration.';
  }
  
  // HTTP status code errors
  if (error.response) {
    const status = error.response.status;
    const detail = error.response.data?.detail || error.response.data?.message;
    
    switch (status) {
      case 400:
        return `Invalid request: ${detail || 'Please check your input and try again.'}`;
      case 401:
        return 'Authentication required. Please check your credentials.';
      case 403:
        return 'Access denied. You do not have permission to perform this action.';
      case 404:
        return `Service endpoint not found: ${detail || 'The requested operation is not available.'}`;
      case 500:
        return `Server error: ${detail || 'An internal error occurred. Please try again later.'}`;
      case 503:
        return 'Service temporarily unavailable. Please try again in a moment.';
      default:
        return detail || `HTTP ${status} error occurred.`;
    }
  }
  
  // Timeout errors
  if (error.code === 'ECONNABORTED' || error.message?.includes('timeout')) {
    return 'Request timed out. The service may be busy, please try again.';
  }
  
  // Generic fallback
  return error.message || 'An unexpected error occurred.';
}

/**
 * Log error with context information
 */
export function logError(operation, error, context = {}) {
  const errorInfo = {
    operation,
    message: error.message,
    stack: error.stack,
    context,
    timestamp: new Date().toISOString()
  };
  
  console.error('BARNS Error:', errorInfo);
  
  // In production, you might want to send this to an error reporting service
  if (process.env.NODE_ENV === 'production') {
    // sendToErrorService(errorInfo);
  }
}

/**
 * Create standardized error response
 */
export function createErrorResponse(operation, error, context = {}) {
  return {
    success: false,
    error: extractErrorMessage(error),
    details: {
      operation,
      technical_details: error.message,
      ...context
    },
    timestamp: new Date().toISOString()
  };
}

/**
 * Error severity levels
 */
export const ERROR_LEVELS = {
  LOW: 'low',
  MEDIUM: 'medium',
  HIGH: 'high',
  CRITICAL: 'critical'
};

/**
 * Determine error severity based on error type
 */
export function getErrorSeverity(error) {
  // Network errors are typically high severity
  if (error.code === 'ECONNREFUSED' || error.code === 'ERR_NETWORK') {
    return ERROR_LEVELS.HIGH;
  }
  
  // HTTP errors
  if (error.response) {
    const status = error.response.status;
    if (status >= 500) return ERROR_LEVELS.CRITICAL;
    if (status >= 400) return ERROR_LEVELS.MEDIUM;
  }
  
  // Timeout errors
  if (error.code === 'ECONNABORTED' || error.message?.includes('timeout')) {
    return ERROR_LEVELS.MEDIUM;
  }
  
  return ERROR_LEVELS.LOW;
}

/**
 * Check if error is retryable
 */
export function isRetryableError(error) {
  // Network timeouts and 503 errors are typically retryable
  if (error.code === 'ECONNABORTED' || error.message?.includes('timeout')) {
    return true;
  }
  
  if (error.response && error.response.status === 503) {
    return true;
  }
  
  return false;
} 