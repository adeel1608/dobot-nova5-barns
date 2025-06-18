/**
 * Category Inventory Card Component
 * Displays inventory items grouped by category with individual item details in responsive grid
 */

import React, { useState } from 'react';
import { useInventoryStore } from '../../../store/inventoryStore';
import { CATEGORY_INFO, getCategoryItems } from '../../../utils/inventoryData';

const CategoryInventoryCard = ({ category }) => {
  const [expanded, setExpanded] = useState(true);
  const { 
    getItemsByCategory, 
    refillInventory, 
    refillCategory, 
    isLoading,
    categoryHasLowInventory 
  } = useInventoryStore();

  const categoryInfo = CATEGORY_INFO[category];
  const items = getItemsByCategory(category);
  const hasLowItems = categoryHasLowInventory(category);
  const itemCount = Object.keys(items).length;

  const handleRefillItem = async (itemKey) => {
    await refillInventory(itemKey, 100);
  };

  const handleRefillCategory = async () => {
    await refillCategory(category, 100);
  };

  const getProgressColor = (level, numeric) => {
    if (level === 'low' || numeric < 20) return 'bg-red-500';
    if (level === 'medium' || numeric < 60) return 'bg-yellow-500';
    return 'bg-green-500';
  };

  const getLevelBadgeColor = (level) => {
    switch (level) {
      case 'low': return 'bg-red-100 text-red-800 border-red-200';
      case 'medium': return 'bg-yellow-100 text-yellow-800 border-yellow-200';
      case 'high': return 'bg-green-100 text-green-800 border-green-200';
      default: return 'bg-gray-100 text-gray-800 border-gray-200';
    }
  };

  if (!categoryInfo) return null;

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-200 overflow-hidden">
      {/* Category Header */}
      <div className="bg-gradient-to-r from-gray-50 to-gray-100 px-4 sm:px-6 py-4 border-b border-gray-200">
        <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
          <div className="flex items-center space-x-3 sm:space-x-4 flex-1 min-w-0">
            <div className="flex items-center justify-center w-10 h-10 sm:w-12 sm:h-12 rounded-lg bg-white border border-gray-200 shadow-sm flex-shrink-0">
              <span className="text-xl sm:text-2xl">{categoryInfo.icon}</span>
            </div>
            <div className="min-w-0 flex-1">
              <h3 className="text-lg sm:text-xl font-bold text-gray-900 truncate">
                {categoryInfo.title}
              </h3>
              <p className="text-xs sm:text-sm text-gray-600 mt-1 line-clamp-2">{categoryInfo.description}</p>
              <div className="flex flex-wrap items-center gap-2 sm:gap-3 mt-2">
                <span className="text-xs sm:text-sm text-gray-500 whitespace-nowrap">{itemCount} items</span>
                {hasLowItems && (
                  <span className="inline-flex items-center px-2 py-0.5 rounded-full text-xs font-medium bg-red-100 text-red-800 border border-red-200 whitespace-nowrap">
                    ⚠️ Low Stock
                  </span>
                )}
              </div>
            </div>
          </div>
          <div className="flex items-center justify-end sm:justify-start gap-2 sm:gap-3 flex-shrink-0">
            <button
              onClick={handleRefillCategory}
              disabled={isLoading}
              className="px-3 sm:px-4 py-2 bg-blue-600 text-white text-xs sm:text-sm font-medium rounded-lg hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2 disabled:opacity-50 disabled:cursor-not-allowed transition-colors duration-200 whitespace-nowrap"
            >
              {isLoading ? (
                <div className="flex items-center">
                  <svg className="animate-spin -ml-1 mr-1 h-3 w-3 sm:h-4 sm:w-4 text-white" fill="none" viewBox="0 0 24 24">
                    <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                    <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                  </svg>
                  <span className="hidden sm:inline">Refilling...</span>
                  <span className="sm:hidden">...</span>
                </div>
              ) : (
                <>
                  <span className="hidden sm:inline">Refill All</span>
                  <span className="sm:hidden">Refill</span>
                </>
              )}
            </button>
            <button
              onClick={() => setExpanded(!expanded)}
              className="p-1.5 sm:p-2 text-gray-400 hover:text-gray-600 rounded-lg hover:bg-white transition-colors duration-200 flex-shrink-0"
            >
              <svg 
                className={`w-4 h-4 sm:w-5 sm:h-5 transform transition-transform duration-200 ${expanded ? 'rotate-180' : ''}`}
                fill="none" 
                stroke="currentColor" 
                viewBox="0 0 24 24"
              >
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
              </svg>
            </button>
          </div>
        </div>
      </div>

      {/* Items Grid */}
      {expanded && (
        <div className="p-3 sm:p-4 lg:p-6">
          {Object.keys(items).length > 0 ? (
            <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-2 xl:grid-cols-3 2xl:grid-cols-4 gap-3 sm:gap-4 lg:gap-6">
              {Object.entries(items).map(([itemKey, itemData]) => (
                <div 
                  key={itemKey}
                  className="group bg-white rounded-lg p-3 sm:p-4 lg:p-5 border border-gray-200 hover:border-gray-300 hover:shadow-lg transition-all duration-200 min-w-0"
                >
                  {/* Item Header */}
                  <div className="flex items-start justify-between mb-3 sm:mb-4 gap-2">
                    <div className="flex items-center space-x-2 sm:space-x-3 flex-1 min-w-0">
                      <span className="text-lg sm:text-xl lg:text-2xl flex-shrink-0">{itemData.icon}</span>
                      <div className="min-w-0 flex-1">
                        <h4 className="font-semibold text-gray-900 text-xs sm:text-sm lg:text-base leading-tight mb-1 line-clamp-2">
                          {itemData.name}
                        </h4>
                        {/* Material and Size Info for Cups */}
                        {category === 'cups' && (
                          <div className="text-xs text-gray-500 truncate">
                            <span className="capitalize">{itemData.material}</span> • {itemData.size}
                          </div>
                        )}
                      </div>
                    </div>
                    <span className={`inline-flex items-center px-1.5 sm:px-2 lg:px-3 py-0.5 sm:py-1 rounded-md text-xs font-medium border ${getLevelBadgeColor(itemData.level)} flex-shrink-0 ml-1 sm:ml-2`}>
                      <span className="hidden sm:inline">{itemData.level}</span>
                      <span className="sm:hidden">{itemData.level.charAt(0).toUpperCase()}</span>
                    </span>
                  </div>

                  {/* Progress Bar */}
                  <div className="mb-3 sm:mb-4 lg:mb-5">
                    <div className="flex justify-between text-xs sm:text-sm text-gray-600 mb-2">
                      <span className="font-medium">Level</span>
                      <span className="font-semibold">{itemData.numeric || 0}%</span>
                    </div>
                    <div className="w-full bg-gray-200 rounded-full h-2 sm:h-3 overflow-hidden">
                      <div 
                        className={`h-2 sm:h-3 rounded-full transition-all duration-500 ease-out ${getProgressColor(itemData.level, itemData.numeric)}`}
                        style={{ width: `${Math.max(0, Math.min(100, itemData.numeric || 0))}%` }}
                      />
                    </div>
                  </div>

                  {/* Last Refilled Info */}
                  {itemData.last_refilled && (
                    <div className="text-xs text-gray-500 mb-3 sm:mb-4 flex items-center">
                      <svg className="w-3 h-3 mr-1 text-gray-400 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                      </svg>
                      <span className="truncate">
                        <span className="hidden sm:inline">Refilled: </span>
                        {new Date(itemData.last_refilled).toLocaleDateString('en-US', { 
                          month: 'short', 
                          day: 'numeric',
                          year: window.innerWidth > 640 ? 'numeric' : '2-digit'
                        })}
                      </span>
                    </div>
                  )}

                  {/* Refill Button */}
                  <button
                    onClick={() => handleRefillItem(itemKey)}
                    disabled={isLoading}
                    className={`w-full px-3 py-2 sm:py-3 text-xs sm:text-sm font-medium rounded-lg transition-all duration-200 focus:outline-none focus:ring-2 focus:ring-offset-2 disabled:opacity-50 disabled:cursor-not-allowed ${
                      itemData.level === 'low' 
                        ? 'bg-red-600 text-white hover:bg-red-700 focus:ring-red-500' 
                        : 'bg-green-600 text-white hover:bg-green-700 focus:ring-green-500'
                    }`}
                  >
                    {isLoading ? (
                      <div className="flex items-center justify-center">
                        <svg className="animate-spin -ml-1 mr-1 sm:mr-2 h-3 w-3 sm:h-4 sm:w-4 text-white" fill="none" viewBox="0 0 24 24">
                          <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                          <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                        </svg>
                        <span className="hidden sm:inline">Refilling...</span>
                        <span className="sm:hidden">...</span>
                      </div>
                    ) : (
                      <div className="flex items-center justify-center">
                        <svg className="w-3 h-3 sm:w-4 sm:h-4 inline mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
                        </svg>
                        Refill
                      </div>
                    )}
                  </button>
                </div>
              ))}
            </div>
          ) : (
            <div className="text-center py-8 sm:py-12">
              <div className="inline-flex items-center justify-center w-12 h-12 sm:w-16 sm:h-16 rounded-full bg-gray-100 mb-3 sm:mb-4">
                <span className="text-2xl sm:text-3xl text-gray-400">{categoryInfo.icon}</span>
              </div>
              <h3 className="text-base sm:text-lg font-medium text-gray-900 mb-2">No Data Available</h3>
              <p className="text-sm text-gray-500 mb-4 px-4">
                No inventory data available for {categoryInfo.title.toLowerCase()}
              </p>
              <button
                onClick={handleRefillCategory}
                className="px-4 py-2 bg-blue-600 text-white text-sm font-medium rounded-lg hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2 transition-colors duration-200"
              >
                Initialize Inventory
              </button>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default CategoryInventoryCard; 