import React, { useState, useEffect } from 'react';
import useStore from '../../../store';
import { useTranslation } from '../../../store/translationsStore';
import { SERVICE_OFFLINE_MESSAGE } from '../../../utils/errorHandler';
import { getValidationMessage, getValidationData } from '../../../constants/validationMessages.jsx';
import { getIngredientData, getSeverityMessage } from '../../../constants/ingredientMappings.jsx';

export default function AlertPanel() {
  const { t } = useTranslation('alerts');
  const { 
    alerts, 
    acknowledgedAlerts, 
    mockAlerts, 
    resumeOperation, 
    refillInventory, 
    acknowledgeAlert,
    fetchAcknowledgedAlerts,
    isLoading, 
    errors, 
    clearError 
  } = useStore();
  const [selectedAlert, setSelectedAlert] = useState(null);
  const [refilling, setRefilling] = useState(false);
  const [acknowledging, setAcknowledging] = useState(false);
  const [showAcknowledged, setShowAcknowledged] = useState(false);

  // Use mock data if there's an API error
  const displayAlerts = errors.alerts ? mockAlerts : alerts;
  const displayAcknowledgedAlerts = acknowledgedAlerts || [];

  // Fetch acknowledged alerts on component mount
  useEffect(() => {
    fetchAcknowledgedAlerts();
  }, [fetchAcknowledgedAlerts]);

  const handleAcknowledgeAlert = async (alertId) => {
    setAcknowledging(true);
    try {
      const success = await acknowledgeAlert(alertId);
      if (success) {
        // Refresh acknowledged alerts after successful acknowledgment
        await fetchAcknowledgedAlerts();
      }
    } catch (error) {
      console.error('Error acknowledging alert:', error);
    } finally {
      setAcknowledging(false);
    }
  };

  const handleAlertClick = (alert) => {
    setSelectedAlert(alert);
  };

  const closeAlertDetails = () => {
    setSelectedAlert(null);
  };

  const retryFetchAlerts = () => {
    clearError('alerts');
    useStore.getState().fetchAlerts();
  };

  const handleRefillFromAlert = async (ingredient) => {
    setRefilling(true);
    try {
      const success = await refillInventory(ingredient);
      if (success) {
        // Close alert details after successful refill
        setTimeout(() => {
          setSelectedAlert(null);
          setRefilling(false);
        }, 1000);
      } else {
        setRefilling(false);
      }
    } catch (error) {
      setRefilling(false);
    }
  };

  const getAlertIcon = (alertType) => {
    switch (alertType) {
      case 'ingredient_threshold':
        return '🥛'; // Milk icon for ingredient alerts
      case 'hardware':
        return '⚙️';
      case 'order_halted':
        return '⏸️';
      case 'emergency_stop':
        return '🛑';
      default:
        return '⚠️';
    }
  };

  const getIngredientFromAlert = (alert) => {
    // Extract ingredient from alert payload or event data
    if (alert.payload && typeof alert.payload === 'string') {
      try {
        const payload = JSON.parse(alert.payload);
        return payload.ingredient;
      } catch (e) {
        // If parsing fails, try to extract from message
      }
    }
    
    // Try to extract ingredient from message
    const message = alert.message || '';
    const ingredients = ['milk', 'cup', 'beans', 'syrup'];
    return ingredients.find(ing => message.toLowerCase().includes(ing));
  };

  const getAlertMessage = (alert) => {
    // For order_halted alerts, try to extract validation_function and map to message
    if (alert.alert_type === 'order_halted') {
      if (alert.payload && typeof alert.payload === 'string') {
        try {
          const payload = JSON.parse(alert.payload);
          // Check for validation_function and map to localized message
          if (payload.validation_function) {
            const validationKey = payload.validation_function;
            // Use the validation message mapping
            return getValidationMessage(validationKey, 'Order processing has been halted due to validation failure');
          }
        } catch (e) {
          // If parsing fails, fall through to default message
        }
      }
      // Return default message if no custom message found
      return alert.message || 'Order processing has been halted due to system issue';
    }
    
    // For other alert types, return the message or alert_type
    return alert.message || alert.alert_type;
  };

  const getValidationDataFromAlert = (alert) => {
    // For order_halted alerts, try to extract validation_function and get data
    if (alert.alert_type === 'order_halted') {
      if (alert.payload && typeof alert.payload === 'string') {
        try {
          const payload = JSON.parse(alert.payload);
          if (payload.validation_function) {
            return getValidationData(payload.validation_function);
          }
        } catch (e) {
          return { icon: null, color: "text-gray-600", message: null };
        }
      }
    }
    return { icon: null, color: "text-gray-600", message: null };
  };

  const getIngredientDataFromAlert = (alert) => {
    // For ingredient_threshold alerts, extract ingredient ID and get display data
    if (alert.alert_type === 'ingredient_threshold') {
      const ingredientId = getIngredientFromAlert(alert);
      if (ingredientId) {
        return getIngredientData(ingredientId);
      }
    }
    return { name: null, icon: null, color: "text-gray-600", category: null };
  };

  const getSeverityFromAlert = (alert) => {
    // Extract severity from payload
    if (alert.payload && typeof alert.payload === 'string') {
      try {
        const payload = JSON.parse(alert.payload);
        return payload.severity || alert.severity || 'low';
      } catch (e) {
        return alert.severity || 'low';
      }
    }
    return alert.severity || 'low';
  };

  // Helper function to get the severity badge
  const getSeverityBadge = (severity) => {
    switch (severity?.toLowerCase()) {
      case 'critical':
        return <span className="bg-red-100 text-red-800 text-xs font-medium px-2 py-0.5 rounded">{t('critical')}</span>;
      case 'high':
        return <span className="bg-orange-100 text-orange-800 text-xs font-medium px-2 py-0.5 rounded">{t('high')}</span>;
      case 'medium':
        return <span className="bg-yellow-100 text-yellow-800 text-xs font-medium px-2 py-0.5 rounded">{t('medium')}</span>;
      case 'low':
        return <span className="bg-blue-100 text-blue-800 text-xs font-medium px-2 py-0.5 rounded">{t('low')}</span>;
      default:
        return <span className="bg-gray-100 text-gray-800 text-xs font-medium px-2 py-0.5 rounded">{t('info')}</span>;
    }
  };

  return (
    <div className="bg-white rounded-lg shadow-sm border border-gray-200 mt-2">
      {/* Compact Header */}
      <div className="px-4 py-3 border-b border-gray-200">
        <div className="flex justify-between items-center mb-3">
          <div className="flex items-center space-x-3">
            <h2 className="text-lg font-semibold">{t('alertsManagement')}</h2>
            {errors.alerts && (
              <span className="inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-red-100 text-red-800">
                {t('critical')}
              </span>
            )}
          </div>
          <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
            displayAlerts.length > 0 ? 'bg-red-100 text-red-800' : 'bg-green-100 text-green-800'
          }`}>
            {displayAlerts.length > 0 ? `${displayAlerts.length} ${t('active')}` : t('allClear')}
          </span>
        </div>
        
        <div className="flex border-b border-gray-200 -mb-3">
          <button
            onClick={() => setShowAcknowledged(false)}
            className={`px-3 py-2 text-sm font-medium border-b-2 transition-colors ${
              !showAcknowledged
                ? 'border-blue-500 text-blue-600'
                : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
            }`}
          >
            {t('active')} ({displayAlerts.length})
          </button>
          <button
            onClick={() => setShowAcknowledged(true)}
            className={`px-3 py-2 text-sm font-medium border-b-2 transition-colors ${
              showAcknowledged
                ? 'border-blue-500 text-blue-600'
                : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
            }`}
          >
            {t('acknowledged')} ({displayAcknowledgedAlerts.length})
          </button>
        </div>
      </div>
      
      {/* API Error display - Compact */}
      {errors.alerts && (
        <div className="border-b border-red-200 bg-red-50 px-4 py-2 text-sm text-red-700 flex justify-between items-center">
          <div>
            <span className="font-medium">{t('unableToFetchAlerts')}</span>{' '}
            {errors.alerts === SERVICE_OFFLINE_MESSAGE ? t('serviceOfflineUnreachable') : errors.alerts}
            <p className="text-xs mt-0.5">{t('serviceOfflineUnreachable')}</p>
          </div>
          <button 
            onClick={retryFetchAlerts}
            className="px-2 py-1 bg-red-100 hover:bg-red-200 text-red-800 rounded text-xs font-medium"
          >
            Retry
          </button>
        </div>
      )}
      
      {/* Dense Alerts List */}
      <div className="overflow-y-auto p-2" style={{ maxHeight: 'calc(100vh - 200px)' }}>
        {/* Active Alerts Tab */}
        {!showAcknowledged && (
          <>
            {isLoading ? (
              <div className="flex justify-center items-center py-8">
                <svg className="animate-spin h-6 w-6 text-blue-500" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                  <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                  <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                </svg>
              </div>
            ) : displayAlerts.length === 0 ? (
              <div className="flex flex-col items-center justify-center py-6 text-gray-500">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-8 w-8 text-green-500 mb-2" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                </svg>
                <p className="text-sm">{t('noActiveAlerts')}</p>
              </div>
            ) : (
              <div className="divide-y divide-gray-100">
                {displayAlerts.map(alert => (
                  <div
                    key={alert.id}
                    onClick={() => handleAlertClick(alert)}
                    className={`px-4 py-3 cursor-pointer hover:bg-gray-50 transition-colors border-l-4 ${
                      alert.alert_type === 'ingredient_threshold'
                        ? 'border-yellow-500 bg-yellow-50/30'
                        : alert.alert_type === 'order_halted'
                        ? 'border-orange-500 bg-orange-50/30'
                        : alert.alert_type === 'emergency_stop'
                        ? 'border-red-500 bg-red-50/30'
                        : 'border-blue-500 bg-blue-50/30'
                    }`}
                  >
                    <div className="flex items-start gap-4">
                      {/* Large Icon on Left */}
                      <div className="flex-shrink-0">
                        {alert.alert_type === 'order_halted' && (() => {
                          const validationData = getValidationDataFromAlert(alert);
                          return validationData.icon ? (
                            <div className={validationData.color || 'text-gray-600'}>
                              {validationData.icon}
                            </div>
                          ) : getAlertIcon(alert.alert_type);
                        })()}
                        {alert.alert_type === 'ingredient_threshold' && (() => {
                          const ingredientData = getIngredientDataFromAlert(alert);
                          return ingredientData.icon ? (
                            <div className={ingredientData.color || 'text-gray-600'}>
                              {ingredientData.icon}
                            </div>
                          ) : getAlertIcon(alert.alert_type);
                        })()}
                        {alert.alert_type !== 'order_halted' && alert.alert_type !== 'ingredient_threshold' && (
                          <span className="text-base">{getAlertIcon(alert.alert_type)}</span>
                        )}
                      </div>
                      
                      {/* Content on Right */}
                      <div className="flex-1 min-w-0">
                        <div className="flex items-center justify-between">
                          <p className="font-semibold text-sm text-gray-900">
                            {alert.alert_type === 'ingredient_threshold' 
                              ? (() => {
                                  const ingredientData = getIngredientDataFromAlert(alert);
                                  const severity = getSeverityFromAlert(alert);
                                  const severityText = severity === 'empty' ? 'Out of Stock' : 'Low Stock';
                                  return ingredientData.name ? `${ingredientData.name} - ${severityText}` : `Low ${getIngredientFromAlert(alert) || 'ingredient'} level`;
                                })()
                              : getAlertMessage(alert)}
                          </p>
                          {alert.severity && getSeverityBadge(alert.severity)}
                        </div>
                        <p className="text-xs text-gray-500 mt-1">
                          {alert.alert_type === 'ingredient_threshold' 
                            ? getSeverityMessage(getSeverityFromAlert(alert))
                            : (alert.created_at ? new Date(alert.created_at).toLocaleString() : 'Just now')}
                        </p>
                        
                        {/* Action Buttons */}
                        <div className="flex items-center justify-end gap-2 mt-3">
                          {alert.alert_type === 'ingredient_threshold' && (
                            <button
                              onClick={(e) => {
                                e.stopPropagation();
                                const ingredient = getIngredientFromAlert(alert);
                                if (ingredient) {
                                  handleRefillFromAlert(ingredient);
                                }
                              }}
                              disabled={refilling}
                              className="bg-green-600 hover:bg-green-700 disabled:bg-green-400 text-white text-xs font-medium px-3 py-1.5 rounded-md flex items-center space-x-1"
                            >
                            {refilling ? (
                              <>
                                <svg className="animate-spin h-3 w-3" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                                  <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                                  <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                                </svg>
                                <span>{t('refilling')}</span>
                              </>
                            ) : (
                              <>
                                <svg className="w-3 h-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6v6m0 0v6m0-6h6m-6 0H6" />
                                </svg>
                                <span>{t('refill')}</span>
                              </>
                            )}
                          </button>
                          )}
                          
                          <button
                            onClick={(e) => {
                              e.stopPropagation();
                              handleAcknowledgeAlert(alert.id);
                            }}
                            className="text-xs bg-green-100 hover:bg-green-200 text-green-700 px-3 py-1.5 rounded-md font-medium disabled:bg-gray-100 disabled:text-gray-400"
                            disabled={errors.alerts || acknowledging}
                          >
                            {acknowledging ? t('acknowledging') : t('acknowledge')}
                          </button>
                        </div>
                      </div>
                    </div>
                    
                    {/* Resume operation button for order_halted alerts - moved inside the main flex */}
                    {alert.alert_type === 'order_halted' && (
                      <div className="mt-2">
                        <button
                          onClick={(e) => {
                            e.stopPropagation();
                            if (!errors.system) {
                              resumeOperation();
                            }
                          }}
                          disabled={isLoading || errors.system}
                          className="bg-green-600 hover:bg-green-700 disabled:bg-green-400 text-white text-xs font-medium px-3 py-1.5 rounded-md flex items-center space-x-1"
                        >
                          {isLoading ? (
                            <>
                              <svg className="animate-spin h-3 w-3" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                                <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                                <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 714 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                              </svg>
                              <span>Resuming...</span>
                            </>
                          ) : errors.system ? (
                            <>
                              <svg className="w-3 h-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
                              </svg>
                              <span>{t('systemError')}</span>
                            </>
                          ) : (
                            <>
                              <svg className="w-3 h-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z" />
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
                              </svg>
                              <span>{t('resumeOperation')}</span>
                            </>
                          )}
                        </button>
                      </div>
                    )}
                  </div>
                ))}
              </div>
            )}
          </>
        )}

        {/* Acknowledged Alerts Tab */}
        {showAcknowledged && (
          <>
            {displayAcknowledgedAlerts.length === 0 ? (
              <div className="flex flex-col items-center justify-center py-6 text-gray-500">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-8 w-8 text-gray-400 mb-2" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5H7a2 2 0 00-2 2v8a2 2 0 002 2h8a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2m-6 9l2 2 4-4" />
                </svg>
                <p className="text-sm">{t('noAcknowledgedAlerts')}</p>
              </div>
            ) : (
              <div className="divide-y divide-gray-100">
                {displayAcknowledgedAlerts.map(alert => (
                  <div
                    key={alert.id}
                    onClick={() => handleAlertClick(alert)}
                    className={`px-4 py-3 cursor-pointer hover:bg-gray-50 transition-colors border-l-4 opacity-75 ${
                      alert.alert_type === 'ingredient_threshold'
                        ? 'border-yellow-500 bg-yellow-50/20'
                        : alert.alert_type === 'order_halted'
                        ? 'border-orange-500 bg-orange-50/20'
                        : alert.alert_type === 'emergency_stop'
                        ? 'border-red-500 bg-red-50/20'
                        : 'border-blue-500 bg-blue-50/20'
                    }`}
                  >
                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-3 flex-1 min-w-0">
                        <span className="text-base flex-shrink-0">{getAlertIcon(alert.alert_type)}</span>
                        <div className="flex-1 min-w-0">
                          <div className="flex items-center space-x-2">
                            <p className="font-medium text-sm text-gray-900 truncate">
                              {alert.alert_type === 'ingredient_threshold' 
                                ? `Low ${getIngredientFromAlert(alert) || 'ingredient'} level`
                                : getAlertMessage(alert)}
                            </p>
                            {alert.severity && getSeverityBadge(alert.severity)}
                          </div>
                          <div className="text-xs text-gray-500 mt-0.5 space-y-0.5">
                            <p>Created: {alert.created_at ? new Date(alert.created_at).toLocaleString() : 'Unknown'}</p>
                            <p>Acknowledged: {alert.acknowledged_at ? new Date(alert.acknowledged_at).toLocaleString() : 'Unknown'}</p>
                          </div>
                        </div>
                      </div>
                      
                      <div className="flex items-center space-x-2 flex-shrink-0">
                        <span className="text-xs bg-green-100 text-green-700 px-2 py-1 rounded">
                          ✓ Acknowledged
                        </span>
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </>
        )}
      </div>
      
      {/* Alert Details Modal - Keep existing modal code */}
      {selectedAlert && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-white rounded-lg max-w-md w-full p-6">
            <div className="flex justify-between items-start mb-4">
              <h3 className="text-xl font-bold">{t('alertDetails')}</h3>
              <button 
                onClick={closeAlertDetails}
                className="text-gray-500 hover:text-gray-700"
              >
                <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                </svg>
              </button>
            </div>
            
            <div className="bg-gray-50 p-4 rounded mb-4">
              <div className="flex justify-between items-center mb-2">
                <h4 className="font-medium">{selectedAlert.message || selectedAlert.alert_type}</h4>
                {selectedAlert.severity && getSeverityBadge(selectedAlert.severity)}
              </div>
              <p className="text-sm text-gray-600 mb-2">
                <span className="font-medium">Type:</span> {selectedAlert.alert_type}
              </p>
              {selectedAlert.created_at && (
                <p className="text-sm text-gray-600 mb-2">
                  <span className="font-medium">Time:</span> {new Date(selectedAlert.created_at).toLocaleString()}
                </p>
              )}
              {selectedAlert.location && (
                <p className="text-sm text-gray-600 mb-2">
                  <span className="font-medium">Location:</span> {selectedAlert.location}
                </p>
              )}
              {selectedAlert.details && (
                <div className="mt-2">
                  <p className="text-sm font-medium text-gray-600">Details:</p>
                  <p className="text-sm text-gray-600 mt-1">{selectedAlert.details}</p>
                </div>
              )}
            </div>
            
            <div className="flex justify-end space-x-3">
              <button
                onClick={closeAlertDetails}
                className="px-4 py-2 border border-gray-300 rounded-md text-gray-700 hover:bg-gray-50"
              >
                {t('close')}
              </button>
              <button
                onClick={async () => {
                  const success = await handleAcknowledgeAlert(selectedAlert.id);
                  if (success) {
                    closeAlertDetails();
                  }
                }}
                className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 disabled:bg-blue-400"
                disabled={errors.alerts || acknowledging}
              >
                {acknowledging ? t('acknowledging') : t('acknowledge')}
              </button>
              {selectedAlert.alert_type === 'ingredient_threshold' && (
                <button
                  onClick={() => {
                    if (!errors.system) {
                      resumeOperation();
                    }
                    closeAlertDetails();
                  }}
                  disabled={isLoading || errors.system}
                  className="px-4 py-2 bg-yellow-600 text-white rounded-md hover:bg-yellow-700 disabled:bg-yellow-400"
                >
                  {isLoading ? t('refilling') : errors.system ? t('systemError') : t('resumeOperation')}
                </button>
              )}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
