import React, { useEffect } from 'react';
import { useInventoryStore } from '../../../store/inventoryStore';
import { CATEGORY_INFO } from '../../../utils/inventoryData';

import refreshBtn from '../../../assets/restock.png';
import milk from '../../../assets/milk.png';
import beans from '../../../assets/beans.png';
import syrup from '../../../assets/syrup.png';
import cup from '../../../assets/cup.png';
import useStore from '../../../store';
const IngredientsIndicator = () => {
  const {
    categorySummary,
    updateCategorySummary,
    fetchInventoryStatus,
  } = useInventoryStore();

  useEffect(() => {
    fetchInventoryStatus();
  }, [fetchInventoryStatus]);
const { navigateToTab } = useStore();
const handleNavigate = () => {
  navigateToTab('inventory');       // Update state
  window.location.hash = '#/inventory'; // Update URL
};


  const getProgressColor = (level, numeric) => {
    if (level === 'low' || numeric < 20) return 'text-red-500';
    if (level === 'medium' || numeric < 60) return 'text-yellow-500';
    return 'text-green-500';
  };

  const getStatusBadge = (level) => {
    switch (level) {
      case 'low':
        return 'bg-red-100 text-red-800';
      case 'medium':
        return 'bg-yellow-100 text-yellow-800';
      case 'high':
        return 'bg-green-100 text-green-800';
      default:
        return 'bg-gray-100 text-gray-800';
    }
  };

  const iconMap = {
    milk,
    beans,
    syrups: syrup,
    cups: cup,
  };

  const categories = [
    { key: 'milk', title: CATEGORY_INFO.milk.title, data: categorySummary.milk || { level: 'unknown', numeric: 0 } },
    { key: 'beans', title: CATEGORY_INFO.beans.title, data: categorySummary.beans || { level: 'unknown', numeric: 0 } },
    { key: 'syrups', title: CATEGORY_INFO.syrups.title, data: categorySummary.syrups || { level: 'unknown', numeric: 0 } },
    { key: 'cups', title: CATEGORY_INFO.cups.title, data: categorySummary.cups || { level: 'unknown', numeric: 0 } },
  ];

  return (
    <div className="bg-white rounded-lg shadow-md  p-4">
      <div className="flex items-center justify-between mb-4 border-b border-gray-200">
        <h3 className="text-lg font-semibold text-gray-900">Ingredients</h3>
        <div className="flex gap-x-2">
          {/* <button
            onClick={handleNavigate}
            className="barns-dark-bg"
            style={{ padding: '0.3rem', outline: 'none' }}
          >
            <img src={refreshBtn} alt="Refresh" className="w-5 h-5 cursor-pointer" />
          </button> */}
          <div className="relative group inline-block">
          <button
            onClick={handleNavigate}
            className="barns-dark-bg"
            style={{ padding: '0.3rem', outline: 'none' }}
          >
            <img src={refreshBtn} alt="Refresh" className="w-5 h-5 cursor-pointer" />
          </button>

          {/* Tooltip */}
          <div className="absolute bottom-full left-[-50%] transform -translate-x-1/2 mb-2  
                          bg-gray-800 text-white text-xs rounded px-2 py-1 
                          opacity-0 group-hover:opacity-100 transition-opacity z-10 whitespace-nowrap">
            Click to More Details
          </div>
        </div>

        </div>
      </div>

      <div className="grid grid-cols-2 lg:grid-cols-4 gap-4">
        {categories.map(({ key, title, data }) => {
          const percentage = Math.max(0, Math.min(100, data.numeric || 0));
          const badgeClass = getStatusBadge(data.level);

          return (
            <button
              key={key}
              onClick={handleNavigate}
              className="group flex flex-col items-center p-3 rounded-lg border border-gray-200 barns-bg-hover transition-all duration-200  "
            >
              {/* Icon */}
              <img src={iconMap[key]} alt={title} className="w-8 h-8 mb-2" />

              {/* Title */}
              <span className="text-sm  text-gray-700  text-center leading-tight font-semibold">
                {title?.split(' ')[0]}
              </span>

              {/* Status + Percentage */}
              <div className="mt-1 flex flex-col items-center">
                <span className={`inline-flex items-center px-4 py-0.5 rounded text-xs font-medium ${badgeClass} font-semibold`}>
                  {data.level === 'unknown' ? 'Unknown' : data.level.charAt(0).toUpperCase() + data.level.slice(1) }
                </span>
                <span className={`text-xs font-bold mt-1 ${getProgressColor(data.level, data.numeric)}`}>
                  {percentage}%
                </span>
              </div>
            </button>
          );
        })}
      </div>
    </div>
  );
};

export default IngredientsIndicator;
