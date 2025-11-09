import React, { useEffect, useState } from 'react';
import { useInventoryStore } from '../../../store/inventoryStore';
import { CATEGORY_INFO } from '../../../utils/inventoryData';
import useStore from '../../../store';
// import socket from '../../../utils/socketConfigure'; // Removed old Socket.IO - using WebSocket now

const IngredientsIndicator = () => {
  const {
    inventoryStatus,
    fetchInventoryStatus,
  } = useInventoryStore();

  const { navigateToTab } = useStore();
  // const [isSocketConnected, setSocketConnected] = useState(socket.connected); // Removed old Socket.IO
  const [isSocketConnected, setSocketConnected] = useState(false);

  // SVG Icons for each category
  const categoryIcons = {
    milk: (
      <svg className="w-5 h-5" viewBox="0 0 24 24" fill="currentColor">
        <path d="M6 3h12l2 4v14a1 1 0 0 1-1 1H5a1 1 0 0 1-1-1V7l2-4zm0 2l-1 2v12h14V7l-1-2H6zm2 8h8v2H8v-2zm0 4h8v2H8v-2z"/>
      </svg>
    ),
    coffee_beans: (
      <svg className="w-5 h-5" viewBox="0 0 24 24" fill="currentColor">
        <path d="M8.5 2c-1.7 0-3 1.3-3 3 0 1.1.6 2.1 1.5 2.6-.9.5-1.5 1.5-1.5 2.6 0 1.7 1.3 3 3 3s3-1.3 3-3c0-1.1-.6-2.1-1.5-2.6.9-.5 1.5-1.5 1.5-2.6 0-1.7-1.3-3-3-3zm7 0c-1.7 0-3 1.3-3 3 0 1.1.6 2.1 1.5 2.6-.9.5-1.5 1.5-1.5 2.6 0 1.7 1.3 3 3 3s3-1.3 3-3c0-1.1-.6-2.1-1.5-2.6.9-.5 1.5-1.5 1.5-2.6 0-1.7-1.3-3-3-3zm-7 11c-1.7 0-3 1.3-3 3 0 1.1.6 2.1 1.5 2.6-.9.5-1.5 1.5-1.5 2.6 0 .6.5 1 1 1h4c.6 0 1-.4 1-1 0-1.1-.6-2.1-1.5-2.6.9-.5 1.5-1.5 1.5-2.6 0-1.7-1.3-3-3-3zm7 0c-1.7 0-3 1.3-3 3 0 1.1.6 2.1 1.5 2.6-.9.5-1.5 1.5-1.5 2.6 0 .6.5 1 1 1h4c.6 0 1-.4 1-1 0-1.1-.6-2.1-1.5-2.6.9-.5 1.5-1.5 1.5-2.6 0-1.7-1.3-3-3-3z"/>
      </svg>
    ),
    syrup: (
      <svg className="w-5 h-5" viewBox="0 0 24 24" fill="currentColor">
        <path d="M8 2h8v3h-8V2m0 4h8v2l2 8H6l2-8V6m2 10h4v2h-4v-2m-1 3h6v4a1 1 0 0 1-1 1h-4a1 1 0 0 1-1-1v-4z"/>
      </svg>
    ),
    cups: (
      <svg className="w-5 h-5" viewBox="0 0 24 24" fill="currentColor">
        <path d="M18 5V3H6v2H3v6c0 2.2 1.8 4 4 4v3h10v-3c2.2 0 4-1.8 4-4V5h-3zM7 13c-1.1 0-2-.9-2-2V7h2v6zm10 5H7v-3h10v3zm2-7c0 1.1-.9 2-2 2V7h2v4z"/>
      </svg>
    ),
    premixes: (
      <svg className="w-5 h-5" viewBox="0 0 24 24" fill="currentColor">
        <path d="M9 3h6l2 2v16a1 1 0 0 1-1 1H8a1 1 0 0 1-1-1V5l2-2zm0 2v14h6V5H9zm2 3h2v2h-2V8zm0 3h2v2h-2v-2zm0 3h2v2h-2v-2z"/>
      </svg>
    )
  };

  useEffect(() => {
    fetchInventoryStatus();

    // Old Socket.IO code - commented out, using WebSocket now
    // const handleConnect = () => {
    //   setSocketConnected(true);
    //   socket.on('inventory.summary', handleInventorySummary);
    // };

    // const handleDisconnect = () => {
    //   setSocketConnected(false);
    //   socket.off('inventory.summary', handleInventorySummary);
    // };

    // socket.on('connect', handleConnect);
    // socket.on('disconnect', handleDisconnect);

    // if (socket.connected) {
    //   handleConnect();
    // }

    // return () => {
    //   socket.off('connect', handleConnect);
    //   socket.off('disconnect', handleDisconnect);
    //   socket.off('inventory.summary', handleInventorySummary);
    // };
  }, []);

  const handleNavigate = () => {
    navigateToTab('inventory');
    window.location.hash = '#/inventory';
  };

  // Helper function to format ingredient names
  const formatIngredientName = (subtype, categoryKey) => {
    if (!subtype) return categoryKey;
    
    // Replace underscores with spaces
    let formatted = subtype.replace(/_/g, ' ');
    
    // Capitalize each word properly
    formatted = formatted
      .split(' ')
      .map(word => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
      .join(' ');
    
    return formatted;
  };

  // Flatten inventoryStatus to get all individual ingredients
  const flattenIngredients = () => {
    const ingredients = [];
    
    if (!inventoryStatus || Object.keys(inventoryStatus).length === 0) {
      return ingredients;
    }

    // Iterate through each category
    Object.entries(inventoryStatus).forEach(([categoryKey, subtypes]) => {
      // Get the icon for this category
      const icon = categoryIcons[categoryKey] || categoryIcons.premixes;
      
      // Iterate through each subtype/ingredient
      Object.entries(subtypes).forEach(([subtypeKey, data]) => {
        ingredients.push({
          key: `${categoryKey}_${subtypeKey}`,
          categoryKey: categoryKey,
          subtypeKey: subtypeKey,
          name: formatIngredientName(subtypeKey, categoryKey),
          percentage: data?.percentage || 0,
          status: data?.status || 'unknown',
          last_updated: data?.last_updated || null,
          icon: icon
        });
      });
    });

    return ingredients;
  };

  // Helper function to shuffle array (for randomizing items with same percentage)
  const shuffleArray = (array) => {
    const shuffled = [...array];
    for (let i = shuffled.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [shuffled[i], shuffled[j]] = [shuffled[j], shuffled[i]];
    }
    return shuffled;
  };

  // Get all ingredients, sort by percentage, randomize same values, and take top 6
  const allIngredients = flattenIngredients();
  
  // Group by percentage
  const groupedByPercentage = {};
  allIngredients.forEach(ingredient => {
    const percent = ingredient.percentage;
    if (!groupedByPercentage[percent]) {
      groupedByPercentage[percent] = [];
    }
    groupedByPercentage[percent].push(ingredient);
  });

  // Sort percentages and shuffle within each group
  const sortedIngredients = Object.keys(groupedByPercentage)
    .sort((a, b) => Number(a) - Number(b)) // Sort percentages ascending (lowest first)
    .flatMap(percent => shuffleArray(groupedByPercentage[percent])); // Shuffle items with same percentage

  // Take top 6 lowest
  const ingredients = sortedIngredients.slice(0, 6);

  return (
    <div className="bg-white rounded-lg shadow-sm p-5 border border-gray-200 flex flex-col max-h-full">
      {/* Header */}
      <div className="flex items-center justify-between mb-4 pb-3 border-b border-gray-100 flex-shrink-0">
        <div className="flex items-center gap-2">
          <h2 className="text-base md:text-lg font-semibold text-gray-900">Inventory Status</h2>
        </div>
        <div className="flex items-center gap-1.5">
          <span className="text-xs text-gray-400">Live</span>
          <div
            className={`w-2 h-2 rounded-full ${isSocketConnected ? 'bg-green-500 animate-pulse' : 'bg-red-400'}`}
            title={isSocketConnected ? 'Connected' : 'Disconnected'}
          ></div>
        </div>
      </div>

      {/* Grid Layout for Inventory Cards */}
      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-3 overflow-y-auto flex-1">
        {ingredients.map((ingredient) => {
          const { key, name, percentage: numeric, status: level, icon } = ingredient;
          const percentage = Math.max(0, Math.min(100, numeric));
          
          // Get card colors based on stock level (lighter, more subtle)
          const getCardBg = (level, numeric) => {
            if (level === 'low' || numeric < 20) return 'bg-red-50 border-red-100 hover:border-red-200';
            if (level === 'medium' || numeric < 60) return 'bg-yellow-50 border-yellow-100 hover:border-yellow-200';
            return 'bg-green-50 border-green-100 hover:border-green-200';
          };

          // Get progress bar color based on stock level
          const getProgressBarColor = (level, numeric) => {
            if (level === 'low' || numeric < 20) return 'bg-red-500';
            if (level === 'medium' || numeric < 60) return 'bg-yellow-500';
            return 'bg-green-500';
          };

          return (
            <div
              key={key}
              className={`${getCardBg(level, numeric)} rounded-lg p-3 cursor-pointer transition-all duration-200 hover:shadow-md border`}
              onClick={handleNavigate}
            >
              {/* Icon and Title Row */}
              <div className="flex items-center gap-2 mb-2">
                <div className="flex-1">
                  <h4 className="font-medium text-gray-700 text-xs">
                    {name}
                  </h4>
                </div>
              </div>

              {/* Percentage Display */}
              <div className="mb-2">
                <div className="flex items-baseline justify-between">
                  <span className="text-l font-semibold text-gray-800">{percentage}%</span>
                </div>
              </div>
              
              {/* Progress Bar */}
              <div className="relative w-full bg-gray-200 rounded-full h-2">
                <div
                  className={`h-2 rounded-full transition-all duration-300 ${getProgressBarColor(level, numeric)}`}
                  style={{ width: `${percentage}%` }}
                ></div>
              </div>
            </div>
          );
        })}
      </div>

      {/* Empty State */}
      {ingredients.length === 0 && (
        <div className="text-center py-6">
          <div className="inline-flex items-center justify-center w-12 h-12 rounded-full bg-green-100 mb-3">
            <svg className="w-6 h-6 text-green-600" fill="currentColor" viewBox="0 0 20 20">
              <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
            </svg>
          </div>
          <h4 className="text-sm font-medium text-gray-700 mb-1">All Items Stocked</h4>
          <p className="text-xs text-gray-500">All inventory items are at 100%</p>
        </div>
      )}
    </div>
  );
};

export default IngredientsIndicator;
