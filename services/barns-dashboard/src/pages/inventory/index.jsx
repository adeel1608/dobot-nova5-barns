/**
 * Inventory Management Page
 * Enhanced with categorized inventory display and individual item management
 */

import React, { useEffect, useState } from "react";
import { useInventoryStore } from "../../store/inventoryStore";
import { INVENTORY_CATEGORIES } from "../../utils/inventoryData";
import CategoryInventoryCard from "./components/CategoryInventoryCard";
import "./styles.css";

const InventoryPage = () => {
  const [refreshing, setRefreshing] = useState(false);
  const [activeTab, setActiveTab] = useState("all");
  const {
    fetchInventoryStatus,
    refillCategory,
    getInventoryStats,
    getInventoryStatsByCategory,
    getLowInventoryItems,
    hasLowInventory,
    isLoading,
    errors,
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
    const categoriesWithLowItems = [
      ...new Set(lowItems.map((item) => item.category)),
    ];

    for (const category of categoriesWithLowItems) {
      await refillCategory(category, 100);
    }
  };

  const stats = getInventoryStats();
  const categoryStats = getInventoryStatsByCategory();
  const lowItems = getLowInventoryItems();

  const tabs = [
    { id: "all", name: "All Categories", count: stats.total },
    {
      id: "milk",
      name: "Milk Products",
      count: categoryStats.milk?.total || 0,
    },
    {
      id: "beans",
      name: "Coffee Beans",
      count: categoryStats.beans?.total || 0,
    },
    { id: "syrups", name: "Syrups", count: categoryStats.syrups?.total || 0 },
    { id: "cups", name: "Cups", count: categoryStats.cups?.total || 0 },
  ];

  return (
    <div className="min-h-screen bg-gray-50">
      <div className="w-full max-w-none px-2 sm:px-2 lg:px-2 ">
        {/* Header */}
        <div className="mb-4 sm:mb-6">
          <div className="flex flex-col ">
            <div className="mb-2 sm:mb-4">
              <div className="flex flex-col">
                {/* Header with Refresh Button */}
                {/* <div className="flex items-center justify-end mb-2">
      <button
        onClick={handleRefresh}
        disabled={refreshing}
        className="p-2 rounded-full hover:bg-[#233746]/5 transition-all duration-300 group"
        title="Refresh Data"
      >
        <svg
          className={`w-5 h-5 barns-dark-text ${refreshing ? "animate-spin" : ""}`}
          fill="none"
          stroke="currentColor"
          viewBox="0 0 24 24"
        >
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            strokeWidth={2}
            d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15"
          />
        </svg>
      </button>
    </div> */}

                {/* Statistics Grid */}
                <div className="grid grid-cols-5 gap-2 sm:gap-3 lg:gap-4">
                  {/* ... existing statistics cards ... */}
                </div>
              </div>
            </div>
            <div className="flex flex-col sm:flex-row sm:items-start sm:justify-between gap-3 sm:gap-4">
              {/* <div className="flex-1 min-w-0">
                <h3 className="text-xl sm:text-2xl lg:text-3xl font-bold barns-green-text">
                  Inventory Management
                </h3>
                <p className="mt-1 text-sm sm:text-base text-gray-600">
                  Monitor and manage coffee machine inventory levels
                </p>
              </div> */}
              <div className="flex flex-col sm:flex-row items-stretch sm:items-center gap-2 sm:gap-3 flex-shrink-0">
                {/* {hasLowInventory() && (
                  <button
                    onClick={handleRefillAllLow}
                    disabled={isLoading}
                    className="px-3 sm:px-4 py-2 bg-red-600 text-white text-sm font-medium rounded-lg hover:bg-red-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors whitespace-nowrap order-2 sm:order-1"
                  >
                    {isLoading ? (
                      <div className="flex items-center justify-center">
                        <svg
                          className="animate-spin -ml-1 mr-1 sm:mr-2 h-4 w-4 text-white"
                          fill="none"
                          viewBox="0 0 24 24"
                        >
                          <circle
                            className="opacity-25"
                            cx="12"
                            cy="12"
                            r="10"
                            stroke="currentColor"
                            strokeWidth="4"
                          ></circle>
                          <path
                            className="opacity-75"
                            fill="currentColor"
                            d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                          ></path>
                        </svg>
                        <span className="hidden sm:inline">Refilling...</span>
                        <span className="sm:hidden">...</span>
                      </div>
                    ) : (
                      <>
                        <span className="hidden sm:inline">
                          Refill All Low Items
                        </span>
                        <span className="sm:hidden">Refill Low Items</span>
                      </>
                    )}
                  </button>
                )} */}
                {/* <button
                  onClick={handleRefresh}
                  disabled={refreshing}
                  className="relative overflow-hidden p-2.5 bg-indigo-500 text-white text-sm font-medium rounded-full
  hover:bg-indigo-600 disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center gap-2
  transition-all duration-300 transform hover:scale-[1.02] active:scale-[0.98] shadow-sm hover:shadow-md
  group"
                  title="Refresh Data"
                >
                  {refreshing && (
                    <div className="absolute inset-0 bg-black/10" />
                  )}
                  <svg
                    className={`w-4 h-4 ${refreshing ? "animate-spin" : ""}`}
                    fill="none"
                    stroke="currentColor"
                    viewBox="0 0 24 24"
                  >
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth={2}
                      d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15"
                    />
                  </svg>
                  <span className="hidden sm:inline text-xs font-semibold tracking-wide group-hover:opacity-100">
                    {refreshing ? "Refreshing..." : "Refresh"}
                  </span>
                </button> */}
                {/* <button
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
                </button> */}
              </div>
            </div>

            {/* Statistics Overview */}
            <div className="grid grid-cols-5 gap-2 sm:gap-3 lg:gap-4">
              {/* Info Card - Enhanced styling */}
              <div className="col-span-1 bg-gradient-to-br from-[#00784B]/5 via-white to-[#233746]/5 p-3 sm:p-4 rounded-lg shadow-md border-2 border-[#00784B] hover:shadow-lg transition-all duration-300">
                <div className="flex items-center">
                  <div className="flex-shrink-0">
                    <div className="w-10 h-10 sm:w-12 sm:h-12 bg-[#00784B] rounded-lg flex items-center justify-center">
                      <svg
                        className="w-6 h-6 text-white"
                        fill="none"
                        stroke="currentColor"
                        viewBox="0 0 24 24"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth={2}
                          d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"
                        />
                      </svg>
                    </div>
                  </div>
                  <div className="ml-3 sm:ml-4 flex-1">
                    <p className="text-xs sm:text-sm font-medium barns-green-text truncate">
                      Inventory
                    </p>
                    <p className="text-sm sm:text-lg lg:text-xl barns-green-text font-medium text-[#233746]">
                       Management
                    </p>
                  </div>
                </div>
              </div>
              <div className="bg-gradient-to-br from-blue-50 to-white p-3 sm:p-4 rounded-lg shadow-sm border border-blue-100 hover:shadow-md transition-all duration-300">
                <div className="flex items-center">
                  <div className="flex-shrink-0">
                    <div className="w-10 h-10 sm:w-12 sm:h-12 bg-blue-100 rounded-lg flex items-center justify-center">
                      <svg
                        className="w-6 h-6 text-blue-600"
                        fill="none"
                        stroke="currentColor"
                        viewBox="0 0 24 24"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth={2}
                          d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4"
                        />
                      </svg>
                    </div>
                  </div>
                  <div className="ml-3 sm:ml-4 flex-1">
                    <p className="text-xs sm:text-sm font-medium text-blue-600 truncate">
                      Total Items
                    </p>
                    <p className="text-sm sm:text-lg lg:text-xl font-bold text-gray-900 barns-dark-text">
                      {stats.total}
                    </p>
                  </div>
                </div>
              </div>

              <div className="bg-gradient-to-br from-red-50 to-white p-3 sm:p-4 rounded-lg shadow-sm border border-red-100 hover:shadow-md transition-all duration-300">
                <div className="flex items-center">
                  <div className="flex-shrink-0">
                    <div className="w-10 h-10 sm:w-12 sm:h-12 bg-red-100 rounded-lg flex items-center justify-center">
                      <svg
                        className="w-6 h-6 text-red-600"
                        fill="none"
                        stroke="currentColor"
                        viewBox="0 0 24 24"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth={2}
                          d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z"
                        />
                      </svg>
                    </div>
                  </div>
                  <div className="ml-3 sm:ml-4 flex-1">
                    <p className="text-xs sm:text-sm font-medium text-red-600 truncate">
                      Low Stock
                    </p>
                    <p className="text-sm sm:text-lg lg:text-xl font-bold text-gray-900">
                      {stats.low}
                    </p>
                  </div>
                </div>
              </div>

              <div className="bg-gradient-to-br from-yellow-50 to-white p-3 sm:p-4 rounded-lg shadow-sm border border-yellow-100 hover:shadow-md transition-all duration-300">
                <div className="flex items-center">
                  <div className="flex-shrink-0">
                    <div className="w-10 h-10 sm:w-12 sm:h-12 bg-yellow-100 rounded-lg flex items-center justify-center">
                      <svg
                        className="w-6 h-6 text-yellow-600"
                        fill="none"
                        stroke="currentColor"
                        viewBox="0 0 24 24"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth={2}
                          d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z"
                        />
                      </svg>
                    </div>
                  </div>
                  <div className="ml-3 sm:ml-4 flex-1">
                    <p className="text-xs sm:text-sm font-medium text-yellow-600 truncate">
                      Medium Stock
                    </p>
                    <p className="text-sm sm:text-lg lg:text-xl font-bold text-gray-900">
                      {stats.medium}
                    </p>
                  </div>
                </div>
              </div>

              <div className="bg-gradient-to-br from-green-50 to-white p-3 sm:p-4 rounded-lg shadow-sm border border-green-100 hover:shadow-md transition-all duration-300">
                <div className="flex items-center">
                  <div className="flex-shrink-0">
                    <div className="w-10 h-10 sm:w-12 sm:h-12 bg-green-100 rounded-lg flex items-center justify-center">
                      <svg
                        className="w-6 h-6 text-green-600"
                        fill="none"
                        stroke="currentColor"
                        viewBox="0 0 24 24"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth={2}
                          d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z"
                        />
                      </svg>
                    </div>
                  </div>
                  <div className="ml-3 sm:ml-4 flex-1">
                    <p className="text-xs sm:text-sm font-medium text-green-600 truncate">
                      High Stock
                    </p>
                    <p className="text-sm sm:text-lg lg:text-xl font-bold text-gray-900">
                      {stats.high}
                    </p>
                  </div>
                </div>
              </div>
            </div>
            {/* <div className="grid grid-cols-2 lg:grid-cols-4 gap-2 sm:gap-3 lg:gap-4">
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
            </div> */}

            {/* Low Stock Alert */}
            {lowItems.length > 0 && (
              <div className="bg-red-50 mt-2 sm:mt-4 border border-red-200 rounded-lg p-3 sm:p-4 flex items-center justify-between shadow-sm ">
                <div className="flex items-start w-full">
                  <svg
                    className="w-4 h-4 sm:w-5 sm:h-5 text-red-400 mt-0.5 mr-2 sm:mr-3 flex-shrink-0"
                    fill="currentColor"
                    viewBox="0 0 20 20"
                  >
                    <path
                      fillRule="evenodd"
                      d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z"
                      clipRule="evenodd"
                    />
                  </svg>
                  <div className="flex-1 ">
                    <div className="flex justify-between w-full ">
                      <h3 className="text-sm font-medium  text-red-800">
                        Low Stock Alert
                      </h3>
                    
                      {hasLowInventory() && (
                        <button
                          onClick={handleRefillAllLow}
                          disabled={isLoading}
                          className="px-3 sm:px-4 py-1 bg-red-600 text-white text-sm font-small rounded-lg hover:bg-red-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors whitespace-nowrap order-2 sm:order-1"
                          
                        >
                          {isLoading ? (
                            <div className="flex items-center justify-center">
                              <svg
                                className="animate-spin -ml-1 mr-1 sm:mr-2 h-4 w-4 text-white"
                                fill="none"
                                viewBox="0 0 24 24"
                              >
                                <circle
                                  className="opacity-25"
                                  cx="12"
                                  cy="12"
                                  r="10"
                                  stroke="currentColor"
                                  strokeWidth="4"
                                ></circle>
                                <path
                                  className="opacity-75"
                                  fill="currentColor"
                                  d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                                ></path>
                              </svg>
                              <span className="hidden sm:inline  text-sm">Refilling...</span>
                              <span className="sm:hidden">...</span>
                            </div>
                          ) : (
                            <>
                              <span className="hidden sm:inline text-sm">
                                Refill All Low Items
                              </span>
                              <span className="sm:hidden">Refill Low Items</span>
                            </>
                          )}
                        </button>
                      )}
                    </div>
                    <div className="mt-1 sm:mt-2 text-sm text-red-700 ">
                      <p>{lowItems.length} items are running low:</p>
                      <div
                        className=" max-w-full overflow-x-auto custom-scrollbar "
                        style={{ width: "85rem" }}
                      >
                        <ul className="mt-1 list-inside space-y-0.5 pb-2 space-x-3 flex">
                          {lowItems
                            .slice(0, window.innerWidth > 640 ? 5 : 3)
                            .map((item) => (
                              <li
                                key={item.key}
                                className="truncate min-w-max  px-3 rounded-full border-red-300 border-2"
                              >
                                {item.name} ({item.numeric}%)
                              </li>
                            ))}
                          {lowItems.length >
                            (window.innerWidth > 640 ? 5 : 3) && (
                            <li>
                              ...and{" "}
                              {lowItems.length -
                                (window.innerWidth > 640 ? 5 : 3)}{" "}
                              more items
                            </li>
                          )}
                        </ul>
                      </div>
                    </div>
                  </div>
                </div>
               
              </div>
            )}
          </div>
        </div>

        {/* Category Tabs */}
        {/* <div className="mb-4 sm:mb-6">
          <div className="border-b border-gray-200">
            <nav className="-mb-px flex space-x-2 sm:space-x-4 lg:space-x-8 overflow-x-auto scrollbar-hide">
              {tabs.map((tab) => (
                <button
                  key={tab.id}
                  onClick={() => setActiveTab(tab.id)}
                  className={`whitespace-nowrap py-2 px-1 border-b-2 font-medium text-xs sm:text-sm transition-colors flex-shrink-0 ${
                    activeTab === tab.id
                      ? "border-blue-500 text-blue-600"
                      : "border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300"
                  }`}
                >
                  <span className="hidden sm:inline">{tab.name}</span>
                  <span className="sm:hidden capitalize">
                    {tab.id === "all" ? "All" : tab.id}
                  </span>
                  <span className="ml-1 sm:ml-2 py-0.5 px-1.5 sm:px-2 rounded-full text-xs bg-gray-100 text-gray-900">
                    {tab.count}
                  </span>
                </button>
              ))}
            </nav>
          </div>
        </div> */}
        <div className="mb-4 sm:mb-6">
          <div className="bg-white/90 backdrop-blur-sm rounded-xl p-3 shadow-md border border-[#00784B]/20">
            <nav className="flex space-x-2 sm:space-x-4 lg:space-x-6 overflow-x-auto scrollbar-hide">
              {tabs.map((tab) => (
                <button
                  key={tab.id}
                  onClick={() => setActiveTab(tab.id)}
                  className={`tab-button relative py-2.5 px-5 rounded-lg font-medium text-sm
            flex items-center gap-2 min-w-[120px] justify-center group
            ${
              activeTab === tab.id
                ? "bg-[#00784B] text-white border-2 border-[#00784B] shadow-md hover:text-black"
                : "bg-white text-[#00784B] hover:bg-[#00784B]/5 hover:text-black"
            }`}
                  data-active={activeTab === tab.id}
                >
                  <span className="hidden sm:inline">{tab.name}</span>
                  <span className="sm:hidden capitalize">
                    {tab.id === "all" ? "All" : tab.id}
                  </span>
                  <span
                    className={`py-0.5 px-2 rounded-full text-xs font-medium 
              transition-all duration-300
              ${
                activeTab === tab.id
                  ? "bg-white/20 text-white"
                  : "bg-[#00784B]/5 text-[#00784B] group-hover:bg-[#00784B]/20"
              }`}
                  >
                    {tab.count}
                  </span>
                  {activeTab === tab.id && (
                    <div className="absolute inset-0 rounded-lg ring-2 ring-[#00784B]/20 animate-pulse"></div>
                  )}
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
          {activeTab === "all" ? (
            Object.values(INVENTORY_CATEGORIES).map((category) => (
              <CategoryInventoryCard
                key={category}
                category={category}
                isAllView={true}
              />
            ))
          ) : (
            <CategoryInventoryCard category={activeTab} isAllView={false} />
          )}
        </div>
        {/* <div className="space-y-6 pb-6">
          {activeTab === "all" ? (
            // Show all categories
            Object.values(INVENTORY_CATEGORIES).map((category) => (
              <CategoryInventoryCard key={category} category={category} />
            ))
          ) : (
            // Show specific category
            <CategoryInventoryCard category={activeTab} />
          )}
        </div> */}
      </div>

      {/* Loading Overlay */}
      {isLoading && (
        <div className="fixed inset-0  flex items-center justify-center z-50">
          <div className="bg-white p-6 rounded-lg shadow-xl">
            <div className="flex items-center space-x-3">
              <svg
                className="animate-spin h-5 w-5 text-blue-600"
                fill="none"
                viewBox="0 0 24 24"
              >
                <circle
                  className="opacity-25"
                  cx="12"
                  cy="12"
                  r="10"
                  stroke="currentColor"
                  strokeWidth="4"
                ></circle>
                <path
                  className="opacity-75"
                  fill="currentColor"
                  d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                ></path>
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
