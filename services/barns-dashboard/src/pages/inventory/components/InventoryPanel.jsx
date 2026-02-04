import React, { useState, useEffect } from 'react';
import useStore from '../../../store';
import { useTranslation } from '../../../store/translationsStore';
import { SERVICE_OFFLINE_MESSAGE } from '../../../utils/errorHandler';

export default function InventoryPanel() {
  const { t } = useTranslation('inventory');
  const {
    inventoryStatus,
    refillInventory,
    fetchInventoryStatus,
    errors,
    isLoading
  } = useStore();
  
  const [refilling, setRefilling] = useState({});

  useEffect(() => {
    // Fetch inventory status on component mount
    fetchInventoryStatus();
  }, [fetchInventoryStatus]);

  const handleRefill = async (ingredient) => {
    setRefilling(prev => ({ ...prev, [ingredient]: true }));
    
    try {
      const success = await refillInventory(ingredient);
      if (success) {
        // Show success feedback briefly
        setTimeout(() => {
          setRefilling(prev => ({ ...prev, [ingredient]: false }));
        }, 1500);
      } else {
        setRefilling(prev => ({ ...prev, [ingredient]: false }));
      }
    } catch (error) {
      setRefilling(prev => ({ ...prev, [ingredient]: false }));
    }
  };

  const getLevelColor = (level, numeric) => {
    if (numeric !== undefined) {
      if (numeric >= 75) return 'text-green-700 bg-green-100 border-green-200';
      if (numeric >= 50) return 'text-yellow-700 bg-yellow-100 border-yellow-200';
      if (numeric >= 25) return 'text-orange-700 bg-orange-100 border-orange-200';
      return 'text-red-700 bg-red-100 border-red-200';
    }
    
    switch (level?.toLowerCase()) {
      case 'high':
        return 'text-green-700 bg-green-100 border-green-200';
      case 'medium':
        return 'text-yellow-700 bg-yellow-100 border-yellow-200';
      case 'low':
        return 'text-red-700 bg-red-100 border-red-200';
      default:
        return 'text-gray-700 bg-gray-100 border-gray-200';
    }
  };

  const getLevelIcon = (level, numeric) => {
    if (numeric !== undefined) {
      if (numeric >= 75) return '🟢';
      if (numeric >= 50) return '🟡';
      if (numeric >= 25) return '🟠';
      return '🔴';
    }
    
    switch (level?.toLowerCase()) {
      case 'high':
        return '🟢';
      case 'medium':
        return '🟡';
      case 'low':
        return '🔴';
      default:
        return '⚪';
    }
  };

  const getIngredientIcon = (ingredient) => {
    switch (ingredient.toLowerCase()) {
      case 'milk':
        return '🥛';
      case 'cup':
        return '☕';
      case 'beans':
        return '🫘';
      case 'syrup':
        return '🍯';
      default:
        return '📦';
    }
  };

  const getLevelText = (status) => {
    // Handle numeric level (from validation service)
    if (typeof status.level === 'number') {
      return `${status.level} units`;
    }
    
    // Handle percentage format
    if (status.numeric !== undefined) {
      return `${status.numeric}%`;
    }
    
    // Handle string level
    return status.level || 'Unknown';
  };

  const getProgressWidth = (status) => {
    // Handle numeric level (from validation service)
    if (typeof status.level === 'number') {
      const percentage = Math.min(100, (status.level / 100) * 100);
      return `${percentage}%`;
    }
    
    // Handle percentage format
    if (status.numeric !== undefined) {
      return `${Math.max(0, Math.min(100, status.numeric))}%`;
    }
    
    // Handle string level
    switch (status.level?.toLowerCase()) {
      case 'high': return '85%';
      case 'medium': return '60%';
      case 'low': return '25%';
      default: return '0%';
    }
  };

  const getStatusLevel = (status) => {
    // Handle numeric level (from validation service)
    if (typeof status.level === 'number') {
      if (status.level >= (status.threshold_medium || 75)) return 'high';
      if (status.level >= (status.threshold_low || 25)) return 'medium';
      return 'low';
    }
    
    // Handle percentage format
    if (status.numeric !== undefined) {
      if (status.numeric >= 75) return 'high';
      if (status.numeric >= 25) return 'medium';
      return 'low';
    }
    
    // Handle string level
    return status.level?.toLowerCase() || 'unknown';
  };

  return (
    <div className="bg-white rounded-xl shadow-lg border border-gray-100">
      {/* Header */}
      <div className="px-6 py-4 border-b border-gray-200 bg-gradient-to-r from-blue-50 to-indigo-50">
        <div className="flex items-center justify-between">
          <h2 className="text-xl font-bold text-gray-900 flex items-center">
            <div className="p-2 bg-blue-100 rounded-lg mr-3">
              <svg className="w-5 h-5 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
              </svg>
            </div>
            Inventory Status
          </h2>
          <button
            onClick={fetchInventoryStatus}
            disabled={isLoading}
            className="px-4 py-2 bg-blue-600 hover:bg-blue-700 text-white rounded-lg text-sm font-medium disabled:opacity-50 disabled:cursor-not-allowed flex items-center transition-all duration-200 shadow-sm hover:shadow-md"
          >
            <svg className={`w-4 h-4 mr-2 ${isLoading ? 'animate-spin' : ''}`} fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0V9a8 8 0 1115.356 2m-15.356-2H9" />
            </svg>
            {isLoading ? 'Refreshing...' : 'Refresh'}
          </button>
        </div>
        {errors.inventory && (
          <div className="mt-3 p-3 bg-red-50 border border-red-200 rounded-lg">
            <div className="flex items-center">
              <svg className="w-5 h-5 text-red-500 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
              <span className="text-sm text-red-700 font-medium">{t('error')}: {errors.inventory === SERVICE_OFFLINE_MESSAGE ? t('serviceOfflineUnreachable') : errors.inventory}</span>
            </div>
          </div>
        )}
      </div>

      {/* Inventory Items */}
      <div className="p-6">
        {Object.keys(inventoryStatus).length === 0 ? (
          <div className="text-center py-8">
            <div className="text-gray-400 mb-2">
              <svg className="w-12 h-12 mx-auto" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
              </svg>
            </div>
            <p className="text-gray-500">No inventory data available</p>
            <button
              onClick={fetchInventoryStatus}
              className="mt-2 text-blue-600 hover:text-blue-700 text-sm font-medium"
            >
              Try refreshing
            </button>
          </div>
        ) : (
          <div className="space-y-4">
            {Object.entries(inventoryStatus).map(([ingredient, status]) => {
              const statusLevel = getStatusLevel(status);
              return (
                <div
                  key={ingredient}
                  className={`border-2 rounded-xl p-5 transition-all duration-200 hover:shadow-md ${getLevelColor(statusLevel, status.numeric)}`}
                >
                  <div className="flex items-center justify-between mb-4">
                    <div className="flex items-center space-x-3">
                      <div className="text-3xl">{getIngredientIcon(ingredient)}</div>
                      <div>
                        <h3 className="font-bold text-lg text-gray-900 capitalize">{ingredient}</h3>
                        <div className="flex items-center space-x-2 mt-1">
                          <span className="text-lg">{getLevelIcon(statusLevel, status.numeric)}</span>
                          <span className="text-sm font-semibold">
                            {getLevelText(status)}
                          </span>
                        </div>
                      </div>
                    </div>
                    
                    <button
                      onClick={() => handleRefill(ingredient)}
                      disabled={refilling[ingredient]}
                      className={`px-4 py-2 rounded-lg text-sm font-bold transition-all duration-200 shadow-sm hover:shadow-md disabled:cursor-not-allowed ${
                        refilling[ingredient]
                          ? 'bg-green-400 text-white'
                          : statusLevel === 'low'
                            ? 'bg-red-600 hover:bg-red-700 text-white'
                            : statusLevel === 'medium'
                              ? 'bg-yellow-600 hover:bg-yellow-700 text-white'
                              : 'bg-green-600 hover:bg-green-700 text-white'
                      }`}
                    >
                      {refilling[ingredient] ? (
                        <div className="flex items-center">
                          <svg className="animate-spin h-4 w-4 mr-2" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                            <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                            <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                          </svg>
                          Refilling...
                        </div>
                      ) : (
                        <div className="flex items-center">
                          <svg className="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6v6m0 0v6m0-6h6m-6 0H6" />
                          </svg>
                          Refill
                        </div>
                      )}
                    </button>
                  </div>

                  {/* Progress Bar */}
                  <div className="mb-3">
                    <div className="w-full bg-gray-200 rounded-full h-3">
                      <div 
                        className={`h-3 rounded-full transition-all duration-500 ${
                          statusLevel === 'low'
                            ? 'bg-red-500'
                            : statusLevel === 'medium'
                              ? 'bg-yellow-500'
                              : 'bg-green-500'
                        }`}
                        style={{ width: getProgressWidth(status) }}
                      ></div>
                    </div>
                  </div>

                  {status.last_refilled && (
                    <div className="text-xs text-gray-600 flex items-center">
                      <svg className="w-3 h-3 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                      </svg>
                      Last refilled: {new Date(status.last_refilled).toLocaleString()}
                    </div>
                  )}
                </div>
              );
            })}
          </div>
        )}

        {/* Quick Actions */}
        {Object.keys(inventoryStatus).length > 0 && (
          <div className="mt-8 p-5 bg-gradient-to-r from-gray-50 to-blue-50 rounded-xl border border-gray-200">
            <h3 className="font-bold text-gray-900 mb-4 flex items-center">
              <div className="p-1 bg-blue-100 rounded-lg mr-2">
                <svg className="w-4 h-4 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
              </div>
              Quick Actions
            </h3>
            <div className="flex flex-wrap gap-3">
              <button
                onClick={() => {
                  Object.keys(inventoryStatus).forEach(ingredient => {
                    const status = inventoryStatus[ingredient];
                    const statusLevel = getStatusLevel(status);
                    if (statusLevel === 'low') {
                      handleRefill(ingredient);
                    }
                  });
                }}
                className="px-4 py-2 bg-red-600 hover:bg-red-700 text-white rounded-lg text-sm font-bold transition-all duration-200 shadow-sm hover:shadow-md flex items-center"
              >
                <svg className="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v3m0 0v3m0-3h3m-3 0H9m12 0a9 9 0 11-18 0 9 9 0 0118 0z" />
                </svg>
                Refill All Low Items
              </button>
              <button
                onClick={() => {
                  Object.keys(inventoryStatus).forEach(ingredient => {
                    handleRefill(ingredient);
                  });
                }}
                className="px-4 py-2 bg-blue-600 hover:bg-blue-700 text-white rounded-lg text-sm font-bold transition-all duration-200 shadow-sm hover:shadow-md flex items-center"
              >
                <svg className="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0V9a8 8 0 1115.356 2m-15.356-2H9" />
                </svg>
                Refill All Items
              </button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
} 