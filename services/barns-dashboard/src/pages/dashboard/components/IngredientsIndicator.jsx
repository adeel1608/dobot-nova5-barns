import React, { useEffect, useState } from 'react';
import { useInventoryStore } from '../../../store/inventoryStore';
import { useTranslation } from '../../../store/translationsStore';
import { CATEGORY_INFO } from '../../../utils/inventoryData';
import useStore from '../../../store';
import { useIsPosMode } from '../../../store/displayStore';
// import socket from '../../../utils/socketConfigure'; // Removed old Socket.IO - using WebSocket now

const IngredientsIndicator = () => {
  const {
    inventoryStatus,
    fetchInventoryStatus,
  } = useInventoryStore();
  const { t } = useTranslation('dashboard');
  const { navigateToTab } = useStore();
  const isPosMode = useIsPosMode();
  // const [isSocketConnected, setSocketConnected] = useState(socket.connected); // Removed old Socket.IO
  const [isSocketConnected, setSocketConnected] = useState(false);

  // SVG Icons for each category — recognizable at small sizes
  const categoryIcons = {
    milk: (
      /* Milk jug / carton */
      <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
        <path d="M6 6l2-4h8l2 4" />
        <path d="M5 6h14v4l-1 10a2 2 0 0 1-2 2H8a2 2 0 0 1-2-2L5 10V6Z" />
        <path d="M8 14c2 2 6 2 8 0" />
      </svg>
    ),
    coffee_beans: (
      /* Coffee bean */
      <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
        <path d="M12 2C8 2 4 6 4 12s4 10 8 10 8-4 8-10S16 2 12 2Z" />
        <path d="M12 2c-2 4-2 16 0 20" />
      </svg>
    ),
    syrup: (
      /* Bottle */
      <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
        <path d="M10 2h4v3a1 1 0 0 1-1 1h-2a1 1 0 0 1-1-1V2Z" />
        <path d="M8.5 7h7l1 3v10a2 2 0 0 1-2 2h-5a2 2 0 0 1-2-2V10l1-3Z" />
        <path d="M10 14h4" />
      </svg>
    ),
    cups: (
      /* Paper cup — tapered body with a rim and sleeve line */
      <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
        <path d="M7 3h10l-1.5 18H8.5L7 3Z" />
        <path d="M6.5 3h11" />
        <path d="M8.2 10h7.6" />
      </svg>
    ),
    premixes: (
      /* Flask / beaker */
      <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
        <path d="M9 3h6v5l4 9a2 2 0 0 1-1.8 2.9H6.8A2 2 0 0 1 5 17l4-9V3Z" />
        <path d="M9 3h6" />
        <path d="M7 15h10" />
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
    <div className={`bg-white rounded-lg shadow-sm border border-gray-200 flex flex-col max-h-full ${isPosMode ? 'p-2' : 'p-5'}`}>
      {/* Header */}
      <div className={`flex items-center justify-between border-b border-gray-100 flex-shrink-0 ${isPosMode ? 'mb-2 pb-1.5' : 'mb-4 pb-3'}`}>
        <div className="flex items-center gap-2">
          <h2 className="text-base md:text-lg font-semibold text-gray-900">{t('inventoryStatus')}</h2>
        </div>
        <div className="flex items-center gap-1.5">
          <span className="text-xs text-gray-400">{t('live')}</span>
          <div
            className={`w-2 h-2 rounded-full ${isSocketConnected ? 'bg-green-500 animate-pulse' : 'bg-red-400'}`}
            title={isSocketConnected ? t('connected') : t('disconnected')}
          ></div>
        </div>
      </div>

      {/* Grid Layout for Inventory Cards */}
      <div className={`grid gap-2 overflow-y-auto flex-1 ${isPosMode ? 'grid-cols-3' : 'grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-3'}`}>
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

          return isPosMode ? (
            /* POS compact card: icon + progress bar only, name/% as tooltip */
            <div
              key={key}
              className={`${getCardBg(level, numeric)} rounded-lg p-2 cursor-pointer transition-all duration-200 hover:shadow-md border flex flex-col items-center gap-1.5`}
              onClick={handleNavigate}
              title={`${name}: ${percentage}%`}
            >
              <div className={`${level === 'low' || numeric < 20 ? 'text-red-500' : level === 'medium' || numeric < 60 ? 'text-yellow-500' : 'text-green-600'}`}>
                {icon}
              </div>
              <div className="w-full bg-gray-200 rounded-full h-2">
                <div
                  className={`h-2 rounded-full transition-all duration-300 ${getProgressBarColor(level, numeric)}`}
                  style={{ width: `${percentage}%` }}
                />
              </div>
            </div>
          ) : (
            /* Default card: icon, name, percentage, progress bar */
            <div
              key={key}
              className={`${getCardBg(level, numeric)} rounded-lg p-3 cursor-pointer transition-all duration-200 hover:shadow-md border`}
              onClick={handleNavigate}
            >
              <div className="flex items-center gap-2 mb-2">
                <div className="flex-1">
                  <h4 className="font-medium text-gray-700 text-xs">{name}</h4>
                </div>
              </div>
              <div className="mb-2">
                <span className="text-l font-semibold text-gray-800">{percentage}%</span>
              </div>
              <div className="relative w-full bg-gray-200 rounded-full h-2">
                <div
                  className={`h-2 rounded-full transition-all duration-300 ${getProgressBarColor(level, numeric)}`}
                  style={{ width: `${percentage}%` }}
                />
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
          <h4 className="text-sm font-medium text-gray-700 mb-1">{t('allItemsStocked')}</h4>
          <p className="text-xs text-gray-500">{t('allInventoryFull')}</p>
        </div>
      )}
    </div>
  );
};

export default IngredientsIndicator;
