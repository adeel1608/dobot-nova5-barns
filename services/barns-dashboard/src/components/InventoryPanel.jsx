import React, { useState, useEffect } from 'react';
import useStore from '../store';

export default function InventoryPanel() {
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
        }, 1000);
      } else {
        setRefilling(prev => ({ ...prev, [ingredient]: false }));
      }
    } catch (error) {
      setRefilling(prev => ({ ...prev, [ingredient]: false }));
    }
  };

  const getLevelColor = (level) => {
    switch (level?.toLowerCase()) {
      case 'high':
        return 'text-green-600 bg-green-100';
      case 'medium':
        return 'text-yellow-600 bg-yellow-100';
      case 'low':
        return 'text-red-600 bg-red-100';
      default:
        return 'text-gray-600 bg-gray-100';
    }
  };

  const getLevelIcon = (level) => {
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

  return (
    <div className="bg-white rounded-lg shadow-md">
      <div className="p-4 border-b border-gray-200">
        <div className="flex items-center justify-between">
          <h2 className="text-xl font-bold text-gray-900 flex items-center">
            <svg className="w-6 h-6 mr-2 text-blue-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
            </svg>
            Inventory Status
          </h2>
          <button
            onClick={fetchInventoryStatus}
            disabled={isLoading}
            className="text-sm px-3 py-1 bg-blue-500 hover:bg-blue-600 text-white rounded-lg disabled:opacity-50 flex items-center"
          >
            <svg className={`w-4 h-4 mr-1 ${isLoading ? 'animate-spin' : ''}`} fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0V9a8 8 0 1115.356 2m-15.356-2H9" />
            </svg>
            Refresh
          </button>
        </div>
        {errors.inventory && (
          <div className="mt-2 text-sm text-red-600 bg-red-50 p-2 rounded">
            Error: {errors.inventory}
          </div>
        )}
      </div>

      <div className="p-4">
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
          {Object.entries(inventoryStatus).map(([ingredient, status]) => (
            <div
              key={ingredient}
              className="border border-gray-200 rounded-lg p-4 hover:shadow-md transition-shadow"
            >
              <div className="flex items-center justify-between mb-3">
                <div className="flex items-center">
                  <span className="text-2xl mr-2">{getIngredientIcon(ingredient)}</span>
                  <div>
                    <h3 className="font-semibold text-gray-900 capitalize">{ingredient}</h3>
                    <div className="flex items-center mt-1">
                      <span className="mr-1">{getLevelIcon(status.level)}</span>
                      <span className={`text-xs px-2 py-1 rounded-full font-medium ${getLevelColor(status.level)}`}>
                        {status.level || 'Unknown'}
                      </span>
                    </div>
                  </div>
                </div>
                
                <button
                  onClick={() => handleRefill(ingredient)}
                  disabled={refilling[ingredient]}
                  className={`px-3 py-2 rounded-lg text-sm font-medium transition-colors ${
                    refilling[ingredient]
                      ? 'bg-green-300 text-white cursor-not-allowed'
                      : status.level === 'low'
                        ? 'bg-red-500 hover:bg-red-600 text-white'
                        : status.level === 'medium'
                          ? 'bg-yellow-500 hover:bg-yellow-600 text-white'
                          : 'bg-green-500 hover:bg-green-600 text-white'
                  }`}
                >
                  {refilling[ingredient] ? (
                    <div className="flex items-center">
                      <svg className="animate-spin h-4 w-4 mr-1" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                        <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                        <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                      </svg>
                      Refilling...
                    </div>
                  ) : (
                    <div className="flex items-center">
                      <svg className="w-4 h-4 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6v6m0 0v6m0-6h6m-6 0H6" />
                      </svg>
                      Refill
                    </div>
                  )}
                </button>
              </div>

              {status.last_refilled && (
                <div className="text-xs text-gray-500">
                  Last refilled: {new Date(status.last_refilled).toLocaleString()}
                </div>
              )}
            </div>
          ))}
        </div>

        {/* Quick Actions */}
        <div className="mt-6 p-4 bg-gray-50 rounded-lg">
          <h3 className="font-semibold text-gray-900 mb-3 flex items-center">
            <svg className="w-5 h-5 mr-2 text-blue-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
            </svg>
            Quick Actions
          </h3>
          <div className="flex flex-wrap gap-2">
            <button
              onClick={() => {
                Object.keys(inventoryStatus).forEach(ingredient => {
                  if (inventoryStatus[ingredient].level === 'low') {
                    handleRefill(ingredient);
                  }
                });
              }}
              className="px-4 py-2 bg-red-500 hover:bg-red-600 text-white rounded-lg text-sm font-medium"
            >
              Refill All Low Items
            </button>
            <button
              onClick={() => {
                Object.keys(inventoryStatus).forEach(ingredient => {
                  handleRefill(ingredient);
                });
              }}
              className="px-4 py-2 bg-blue-500 hover:bg-blue-600 text-white rounded-lg text-sm font-medium"
            >
              Refill All Items
            </button>
          </div>
        </div>
      </div>
    </div>
  );
} 