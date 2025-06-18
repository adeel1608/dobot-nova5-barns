/**
 * Ingredients Indicator Component
 * Shows progress based on the lowest level item in each category
 */

import React, { useEffect } from 'react';
import { useInventoryStore } from '../../../store/inventoryStore';
import { CATEGORY_INFO } from '../../../utils/inventoryData';

const IngredientsIndicator = () => {
  const { 
    categorySummary, 
    updateCategorySummary, 
    fetchInventoryStatus 
  } = useInventoryStore();

  // Fetch inventory data on component mount
  useEffect(() => {
    fetchInventoryStatus();
  }, [fetchInventoryStatus]);

  const handleNavigate = () => {
    // Simple navigation using window.location
    window.location.hash = '#/inventory';
  };

  const getProgressColor = (level, numeric) => {
    if (level === 'low' || numeric < 20) return 'text-red-500';
    if (level === 'medium' || numeric < 60) return 'text-yellow-500';
    return 'text-green-500';
  };

  const getProgressStroke = (level, numeric) => {
    if (level === 'low' || numeric < 20) return 'stroke-red-500';
    if (level === 'medium' || numeric < 60) return 'stroke-yellow-500';
    return 'stroke-green-500';
  };

  const categories = [
    {
      key: 'milk',
      info: CATEGORY_INFO.milk,
      data: categorySummary.milk || { level: 'unknown', numeric: 0 }
    },
    {
      key: 'beans',
      info: CATEGORY_INFO.beans,
      data: categorySummary.beans || { level: 'unknown', numeric: 0 }
    },
    {
      key: 'syrups',
      info: CATEGORY_INFO.syrups,
      data: categorySummary.syrups || { level: 'unknown', numeric: 0 }
    },
    {
      key: 'cups',
      info: CATEGORY_INFO.cups,
      data: categorySummary.cups || { level: 'unknown', numeric: 0 }
    }
  ];

  return (
    <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-4">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-semibold text-gray-900">Ingredients Status</h3>
        <button
          onClick={handleNavigate}
          className="text-sm text-blue-600 hover:text-blue-800 font-medium"
        >
          View Details →
        </button>
      </div>
      
      <div className="grid grid-cols-2 lg:grid-cols-4 gap-4">
        {categories.map(({ key, info, data }) => {
          const percentage = Math.max(0, Math.min(100, data.numeric || 0));
          const circumference = 2 * Math.PI * 16; // radius = 16
          const strokeDasharray = circumference;
          const strokeDashoffset = circumference - (percentage / 100) * circumference;

          return (
            <button
              key={key}
              onClick={handleNavigate}
              className="group flex flex-col items-center p-3 rounded-lg border border-gray-200 hover:border-blue-300 hover:bg-blue-50 transition-all duration-200"
            >
              {/* Circular Progress */}
              <div className="relative w-12 h-12 mb-2">
                <svg className="w-12 h-12 transform -rotate-90" viewBox="0 0 40 40">
                  {/* Background circle */}
                  <circle
                    cx="20"
                    cy="20"
                    r="16"
                    stroke="currentColor"
                    strokeWidth="3"
                    fill="transparent"
                    className="text-gray-200"
                  />
                  {/* Progress circle */}
                  <circle
                    cx="20"
                    cy="20"
                    r="16"
                    stroke="currentColor"
                    strokeWidth="3"
                    fill="transparent"
                    strokeDasharray={strokeDasharray}
                    strokeDashoffset={strokeDashoffset}
                    strokeLinecap="round"
                    className={`transition-all duration-500 ${getProgressStroke(data.level, data.numeric)}`}
                  />
                </svg>
                
                {/* Center icon */}
                <div className="absolute inset-0 flex items-center justify-center">
                  <span className="text-lg">{info.icon}</span>
                </div>
                
                {/* Percentage text */}
                <div className="absolute -bottom-1 left-1/2 transform -translate-x-1/2">
                  <span className={`text-xs font-semibold ${getProgressColor(data.level, data.numeric)}`}>
                    {percentage}%
                  </span>
                </div>
              </div>

              {/* Category title */}
              <span className="text-sm font-medium text-gray-700 group-hover:text-blue-600 text-center leading-tight">
                {info.title}
              </span>

              {/* Status indicator */}
              <div className="mt-1">
                {data.level === 'low' && (
                  <span className="inline-flex items-center px-1.5 py-0.5 rounded-full text-xs font-medium bg-red-100 text-red-800">
                    Low
                  </span>
                )}
                {data.level === 'medium' && (
                  <span className="inline-flex items-center px-1.5 py-0.5 rounded-full text-xs font-medium bg-yellow-100 text-yellow-800">
                    Medium
                  </span>
                )}
                {data.level === 'high' && (
                  <span className="inline-flex items-center px-1.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">
                    Good
                  </span>
                )}
                {data.level === 'unknown' && (
                  <span className="inline-flex items-center px-1.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800">
                    Unknown
                  </span>
                )}
              </div>
            </button>
          );
        })}
      </div>

      {/* Summary info */}
      <div className="mt-4 pt-3 border-t border-gray-200">
        <div className="flex items-center justify-between text-sm text-gray-500">
          <span>Levels based on lowest item per category</span>
          <button
            onClick={() => updateCategorySummary()}
            className="text-blue-600 hover:text-blue-800"
          >
            Refresh
          </button>
        </div>
      </div>
    </div>
  );
};

export default IngredientsIndicator; 