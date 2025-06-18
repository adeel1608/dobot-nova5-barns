/**
 * Inventory Management Page
 * Enhanced with categorized inventory display and individual item management
 */

import React, { useEffect, useState } from 'react';
import { useInventoryStore } from '../../store/inventoryStore';
import { INVENTORY_CATEGORIES } from '../../utils/inventoryData';
import CategoryInventoryCard from './components/CategoryInventoryCard';
import './styles.css';

const InventoryPage = () => {
  const [refreshing, setRefreshing] = useState(false);
  const [activeTab, setActiveTab] = useState('all');
  const { 
    fetchInventoryStatus, 
    refillCategory,
    getInventoryStats,
    getInventoryStatsByCategory,
    getLowInventoryItems,
    hasLowInventory,
    isLoading,
    errors
  } = useInventoryStore();

  useEffect(() => {
    fetchInventoryStatus();
  }, [fetchInventoryStatus]);

  const handleRefresh = async () => {
    setRefreshing(true);
    await fetchInventoryStatus();
    setRefreshing(false);
  };

  const handleRefillAllLow = async () => {
    const lowItems = getLowInventoryItems();
    const categoriesWithLowItems = [...new Set(lowItems.map(item => item.category))];
    
    for (const category of categoriesWithLowItems) {
      await refillCategory(category, 100);
    }
  };

  const stats = getInventoryStats();
  const categoryStats = getInventoryStatsByCategory();
  const lowItems = getLowInventoryItems();

  const tabs = [
    { id: 'all', name: 'All Categories', count: stats.total },
    { id: 'milk', name: 'Milk Products', count: categoryStats.milk?.total || 0 },
    { id: 'beans', name: 'Coffee Beans', count: categoryStats.beans?.total || 0 },
    { id: 'syrups', name: 'Syrups', count: categoryStats.syrups?.total || 0 },
    { id: 'cups', name: 'Cups', count: categoryStats.cups?.total || 0 }
  ];

  return (
    <div className="min-h-screen bg-gray-50">
      <div className="w-full max-w-none px-4 sm:px-6 lg:px-8 py-6">
        {/* Header */}
        <div className="mb-4 sm:mb-6">
          <div className="flex flex-col gap-3 sm:gap-4">
            <div className="flex flex-col sm:flex-row sm:items-start sm:justify-between gap-3 sm:gap-4">
              <div className="flex-1 min-w-0">
                <h1 className="text-xl sm:text-2xl lg:text-3xl font-bold text-gray-900">Inventory Management</h1>
                <p className="mt-1 text-sm sm:text-base text-gray-600">
                  Monitor and manage coffee machine inventory levels
                </p>
              </div>
              <div className="flex flex-col sm:flex-row items-stretch sm:items-center gap-2 sm:gap-3 flex-shrink-0">
                {hasLowInventory() && (
                  <button
                    onClick={handleRefillAllLow}
                    disabled={isLoading}
                    className="px-3 sm:px-4 py-2 bg-red-600 text-white text-sm font-medium rounded-lg hover:bg-red-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors whitespace-nowrap order-2 sm:order-1"
                  >
                    {isLoading ? (
                      <div className="flex items-center justify-center">
                        <svg className="animate-spin -ml-1 mr-1 sm:mr-2 h-4 w-4 text-white" fill="none" viewBox="0 0 24 24">
                          <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                          <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                        </svg>
                        <span className="hidden sm:inline">Refilling...</span>
                        <span className="sm:hidden">...</span>
                      </div>
                    ) : (
                      <>
                        <span className="hidden sm:inline">Refill All Low Items</span>
                        <span className="sm:hidden">Refill Low Items</span>
                      </>
                    )}
                  </button>
                )}
                <button
                  onClick={handleRefresh}
                  disabled={refreshing}
                  className="px-3 sm:px-4 py-2 bg-blue-600 text-white text-sm font-medium rounded-lg hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center transition-colors order-1 sm:order-2"
                >
                  <svg 
                    className={`w-4 h-4 mr-1 sm:mr-2 ${refreshing ? 'animate-spin' : ''}`}
                    fill="none" 
                    stroke="currentColor" 
                    viewBox="0 0 24 24"
                  >
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
                  </svg>
                  <span className="hidden sm:inline">{refreshing ? 'Refreshing...' : 'Refresh'}</span>
                  <span className="sm:hidden">{refreshing ? '...' : 'Refresh'}</span>
                </button>
              </div>
            </div>

            {/* Statistics Overview */}
            <div className="grid grid-cols-2 lg:grid-cols-4 gap-2 sm:gap-3 lg:gap-4">
              <div className="bg-white p-3 sm:p-4 rounded-lg shadow-sm border border-gray-200">
                <div className="flex items-center">
                  <div className="flex-shrink-0">
                    <div className="w-6 h-6 sm:w-8 sm:h-8 bg-blue-100 rounded-full flex items-center justify-center">
                      <span className="text-blue-600 font-semibold text-xs sm:text-sm">{stats.total}</span>
                    </div>
                  </div>
                  <div className="ml-2 sm:ml-3 flex-1 min-w-0">
                    <p className="text-xs sm:text-sm font-medium text-gray-500 truncate">Total Items</p>
                    <p className="text-sm sm:text-lg lg:text-xl font-bold text-gray-900">{stats.total}</p>
                  </div>
                </div>
              </div>

              <div className="bg-white p-3 sm:p-4 rounded-lg shadow-sm border border-gray-200">
                <div className="flex items-center">
                  <div className="flex-shrink-0">
                    <div className="w-6 h-6 sm:w-8 sm:h-8 bg-red-100 rounded-full flex items-center justify-center">
                      <span className="text-red-600 font-semibold text-xs sm:text-sm">{stats.low}</span>
                    </div>
                  </div>
                  <div className="ml-2 sm:ml-3 flex-1 min-w-0">
                    <p className="text-xs sm:text-sm font-medium text-gray-500 truncate">Low Stock</p>
                    <p className="text-sm sm:text-lg lg:text-xl font-bold text-gray-900">{stats.low}</p>
                  </div>
                </div>
              </div>

              <div className="bg-white p-3 sm:p-4 rounded-lg shadow-sm border border-gray-200">
                <div className="flex items-center">
                  <div className="flex-shrink-0">
                    <div className="w-6 h-6 sm:w-8 sm:h-8 bg-yellow-100 rounded-full flex items-center justify-center">
                      <span className="text-yellow-600 font-semibold text-xs sm:text-sm">{stats.medium}</span>
                    </div>
                  </div>
                  <div className="ml-2 sm:ml-3 flex-1 min-w-0">
                    <p className="text-xs sm:text-sm font-medium text-gray-500 truncate">Medium Stock</p>
                    <p className="text-sm sm:text-lg lg:text-xl font-bold text-gray-900">{stats.medium}</p>
                  </div>
                </div>
              </div>

              <div className="bg-white p-3 sm:p-4 rounded-lg shadow-sm border border-gray-200">
                <div className="flex items-center">
                  <div className="flex-shrink-0">
                    <div className="w-6 h-6 sm:w-8 sm:h-8 bg-green-100 rounded-full flex items-center justify-center">
                      <span className="text-green-600 font-semibold text-xs sm:text-sm">{stats.high}</span>
                    </div>
                  </div>
                  <div className="ml-2 sm:ml-3 flex-1 min-w-0">
                    <p className="text-xs sm:text-sm font-medium text-gray-500 truncate">High Stock</p>
                    <p className="text-sm sm:text-lg lg:text-xl font-bold text-gray-900">{stats.high}</p>
                  </div>
                </div>
              </div>
            </div>

            {/* Low Stock Alert */}
            {lowItems.length > 0 && (
              <div className="bg-red-50 border border-red-200 rounded-lg p-3 sm:p-4">
                <div className="flex items-start">
                  <svg className="w-4 h-4 sm:w-5 sm:h-5 text-red-400 mt-0.5 mr-2 sm:mr-3 flex-shrink-0" fill="currentColor" viewBox="0 0 20 20">
                    <path fillRule="evenodd" d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
                  </svg>
                  <div className="flex-1">
                    <h3 className="text-sm font-medium text-red-800">Low Stock Alert</h3>
                    <div className="mt-1 sm:mt-2 text-sm text-red-700">
                      <p>{lowItems.length} items are running low:</p>
                      <ul className="mt-1 list-disc list-inside space-y-0.5">
                        {lowItems.slice(0, window.innerWidth > 640 ? 5 : 3).map(item => (
                          <li key={item.key} className="truncate">{item.name} ({item.numeric}%)</li>
                        ))}
                        {lowItems.length > (window.innerWidth > 640 ? 5 : 3) && (
                          <li>...and {lowItems.length - (window.innerWidth > 640 ? 5 : 3)} more items</li>
                        )}
                      </ul>
                    </div>
                  </div>
                </div>
              </div>
            )}
          </div>
        </div>

        {/* Category Tabs */}
        <div className="mb-4 sm:mb-6">
          <div className="border-b border-gray-200">
            <nav className="-mb-px flex space-x-2 sm:space-x-4 lg:space-x-8 overflow-x-auto scrollbar-hide">
              {tabs.map(tab => (
                <button
                  key={tab.id}
                  onClick={() => setActiveTab(tab.id)}
                  className={`whitespace-nowrap py-2 px-1 border-b-2 font-medium text-xs sm:text-sm transition-colors flex-shrink-0 ${
                    activeTab === tab.id
                      ? 'border-blue-500 text-blue-600'
                      : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
                  }`}
                >
                  <span className="hidden sm:inline">{tab.name}</span>
                  <span className="sm:hidden capitalize">{tab.id === 'all' ? 'All' : tab.id}</span>
                  <span className="ml-1 sm:ml-2 py-0.5 px-1.5 sm:px-2 rounded-full text-xs bg-gray-100 text-gray-900">
                    {tab.count}
                  </span>
                </button>
              ))}
            </nav>
          </div>
        </div>

        {/* Error Display */}
        {errors.inventory && (
          <div className="mb-6 bg-red-50 border border-red-200 rounded-lg p-4">
            <div className="text-red-800">
              <strong>Error:</strong> {errors.inventory}
            </div>
          </div>
        )}

        {/* Category Content */}
        <div className="space-y-6 pb-6">
          {activeTab === 'all' ? (
            // Show all categories
            Object.values(INVENTORY_CATEGORIES).map(category => (
              <CategoryInventoryCard 
                key={category} 
                category={category} 
              />
            ))
          ) : (
            // Show specific category
            <CategoryInventoryCard category={activeTab} />
          )}
        </div>
      </div>

      {/* Loading Overlay */}
      {isLoading && (
        <div className="fixed inset-0 bg-black bg-opacity-20 flex items-center justify-center z-50">
          <div className="bg-white p-6 rounded-lg shadow-xl">
            <div className="flex items-center space-x-3">
              <svg className="animate-spin h-5 w-5 text-blue-600" fill="none" viewBox="0 0 24 24">
                <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
              </svg>
              <span className="text-gray-900">Processing...</span>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default InventoryPage; 