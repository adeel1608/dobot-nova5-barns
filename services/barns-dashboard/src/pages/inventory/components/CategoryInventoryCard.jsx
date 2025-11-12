import React, { useState, useEffect } from "react";
import { useInventoryStore } from "../../../store/inventoryStore";

// Direct imports for all category/item icons
import coffee_beans from "../../../assets/coffee_beans.png";
import beans from "../../../assets/beans.png";
import cups from "../../../assets/cups.png";
import cup from "../../../assets/cup.png";
import milks from "../../../assets/milks.png";
import milk from "../../../assets/milk.png";
import syrups from "../../../assets/syrups.png";
import syrup from "../../../assets/syrup.png";
import premixes from "../../../assets/premixes.png";
import defaultIcon from "../../../assets/default.png";

// Map of asset names to imported modules
const pngAssets = {
  "coffee_beans.png": coffee_beans,
  "beans.png": beans,
  "cups.png": cups,
  "cup.png": cup,
  "milks.png": milks,
  "milk.png": milk,
  "syrups.png": syrups,
  "syrup.png": syrup,
  "premixes.png": premixes,
  "default.png": defaultIcon,
};

const CategoryInventoryCard = ({ category, isAllView, count }) => {
  const [expanded, setExpanded] = useState(!isAllView);
  const [refillingCategory, setRefillingCategory] = useState(false);

  const {
    FullCategoryInfo,
    inventoryStatus,
    refillInventory,
    refillCategory,
    isLoading,
    categoryHasLowInventory,
    fetchCategoryInfoData
  } = useInventoryStore();

useEffect(() => {
  fetchCategoryInfoData();
  setExpanded(!isAllView);
}, [isAllView]);

const categoryInfo = FullCategoryInfo?.[category] || {};
const categoryInventory = inventoryStatus?.[category] || {};
const items = {};

Object.entries(categoryInfo).forEach(([itemKey, meta]) => {
  const inv = categoryInventory?.[itemKey];
  items[itemKey] = {
    ...meta,
    level: inv?.status || "unknown",
    numeric: inv?.percentage || 0,
    last_refilled: inv?.last_updated || null,
    icon: `${category}.png`, // Assign category icon
  };
});


  const hasLowItems = categoryHasLowInventory(category);

  const handleRefillItem = async (itemKey) => {
    await refillInventory(itemKey, 100);
  };

  const handleRefillCategory = async () => {
    setRefillingCategory(true);
    try {
      await refillCategory(category, 100);
    } finally {
      // Keep loading state for a brief moment to show success feedback
      setTimeout(() => {
        setRefillingCategory(false);
      }, 500);
    }
  };

  const getProgressColor = (level, numeric) => {
    if (level === "low" || numeric < 20) return "bg-red-500";
    if (level === "medium" || numeric < 60) return "bg-yellow-500";
    return "bg-green-500";
  };

  const getLevelBadgeColor = (level) => {
    switch (level) {
      case "low":
        return "bg-red-100 text-red-800 border-red-200";
      case "medium":
        return "bg-yellow-100 text-yellow-800 border-yellow-200";
      case "high":
        return "bg-green-100 text-green-800 border-green-200";
      default:
        return "bg-gray-100 text-gray-800 border-gray-200";
    }
  };

  const itemCount = Object.keys(items).length;

  // Calculate ingredient level statistics
  const ingredientStats = Object.values(items).reduce((acc, item) => {
    const level = item.level || 'unknown';
    acc[level] = (acc[level] || 0) + 1;
    return acc;
  }, {});

  return (
    <div className="bg-white rounded-2xl shadow-lg border-2 border-[#00784B]/10 overflow-hidden transform transition-all duration-300 hover:shadow-xl hover:-translate-y-1">
      {/* Header */}
      <div className="bg-gradient-to-r from-[#00784B]/5 via-white to-[#233746]/5 px-4 sm:px-6 py-2 border-b border-[#00784B]/10">
        <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
          <div 
            onClick={() => isAllView && setExpanded(!expanded)}
            className={`flex items-center space-x-3 sm:space-x-4 flex-1 min-w-0 ${isAllView ? 'cursor-pointer group' : ''}`}
          >
            <div className="flex items-center justify-center w-12 h-12 sm:w-14 sm:h-14 rounded-xl bg-gradient-to-br from-[#00784B]/10 to-white border-2 border-[#00784B]/20 shadow-inner flex-shrink-0 group-hover:border-[#00784B]/40 transition-all">
              <img
                src={pngAssets[`${category}.png`] || defaultIcon}
                alt={category}
                className="w-8 h-8 sm:w-10 sm:h-10 object-contain"
              />
            </div>
            <div className="min-w-0 flex-1">
              <div className="flex items-center gap-2">
                <h3 className="text-lg sm:text-xl font-bold text-[#233746] truncate">
                  {category.replace(/_/g, " ").replace(/\b\w/g, (c) => c.toUpperCase())}
                </h3>
                {isAllView && (
                  <svg
                    className={`w-5 h-5 text-[#00784B] transform transition-transform duration-300 flex-shrink-0 ${
                      expanded ? "rotate-180" : ""
                    }`}
                    fill="none"
                    stroke="currentColor"
                    viewBox="0 0 24 24"
                  >
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth={2}
                      d="M19 9l-7 7-7-7"
                    />
                  </svg>
                )}
              </div>

              <div className="flex flex-wrap items-center gap-2 mt-2">
                <span className="text-xs text-[#233746]/60 font-medium">
                  {itemCount} {itemCount === 1 ? 'ingredient' : 'ingredients'}:
                </span>
                {ingredientStats.high > 0 && (
                  <div className="flex items-center gap-1 px-2 py-0.5 rounded-full bg-green-50 border border-green-200">
                    <div className="w-2 h-2 rounded-full bg-green-500"></div>
                    <span className="text-xs font-semibold text-green-700">{ingredientStats.high}</span>
                  </div>
                )}
                {ingredientStats.medium > 0 && (
                  <div className="flex items-center gap-1 px-2 py-0.5 rounded-full bg-yellow-50 border border-yellow-200">
                    <div className="w-2 h-2 rounded-full bg-yellow-500"></div>
                    <span className="text-xs font-semibold text-yellow-700">{ingredientStats.medium}</span>
                  </div>
                )}
                {ingredientStats.low > 0 && (
                  <div className="flex items-center gap-1 px-2 py-0.5 rounded-full bg-red-50 border border-red-200">
                    <div className="w-2 h-2 rounded-full bg-red-500"></div>
                    <span className="text-xs font-semibold text-red-700">{ingredientStats.low}</span>
                  </div>
                )}
                {ingredientStats.empty > 0 && (
                  <div className="flex items-center gap-1 px-2 py-0.5 rounded-full bg-gray-100 border border-gray-300">
                    <div className="w-2 h-2 rounded-full bg-gray-400"></div>
                    <span className="text-xs font-semibold text-gray-700">{ingredientStats.empty}</span>
                  </div>
                )}
              </div>
            </div>
          </div>

          <div className="flex items-center justify-end sm:justify-start gap-2 sm:gap-3 flex-shrink-0">
            <button
              onClick={handleRefillCategory}
              disabled={refillingCategory || isLoading}
              className="px-4 py-2.5 bg-[#00784B] text-white text-sm font-semibold rounded-lg hover:bg-[#00784B]/90 focus:outline-none focus:ring-2 focus:ring-[#00784B]/50 focus:ring-offset-2 disabled:opacity-50 disabled:cursor-not-allowed transition-all duration-200 shadow-sm hover:shadow-md transform hover:-translate-y-0.5 active:translate-y-0"
            >
              {refillingCategory ? (
                <div className="flex items-center">
                  <svg className="animate-spin -ml-1 mr-2 h-4 w-4 text-white" fill="none" viewBox="0 0 24 24">
                    <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                    <path
                      className="opacity-75"
                      fill="currentColor"
                      d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                    ></path>
                  </svg>
                  <span>Refilling...</span>
                </div>
              ) : (
                <span>Refill All</span>
              )}
            </button>

            {isAllView && (
              <button
                onClick={() => setExpanded(!expanded)}
                className="p-2.5 text-[#233746] hover:text-[#00784B] rounded-lg hover:bg-[#00784B]/5 transition-all duration-200 flex-shrink-0"
              >
                <svg
                  className={`w-5 h-5 transform transition-transform duration-300 ${
                    expanded ? "rotate-180" : ""
                  }`}
                  fill="none"
                  stroke="currentColor"
                  viewBox="0 0 24 24"
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    strokeWidth={2}
                    d="M19 9l-7 7-7-7"
                  />
                </svg>
              </button>
            )}
          </div>
        </div>
      </div>

      {/* Items Grid */}
      {expanded && (
        <div className="p-4 sm:p-6">
          {Object.keys(items).length > 0 ? (
            <div className="h-[520px] overflow-y-auto scrollbar-thin scrollbar-track-[#233746]/5 scrollbar-thumb-[#00784B]/60 hover:scrollbar-thumb-[#00784B]/80 pr-2">
              <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-6 gap-4 sm:gap-6 auto-rows-[250px]">
                {Object.entries(items).map(([itemKey, itemData]) => (
                  <div
                    key={itemKey}
                    className="group bg-gradient-to-br from-white to-gray-50/50 rounded-xl p-4 border-2 border-[#00784B]/10 hover:border-[#00784B]/30 shadow-sm hover:shadow-md transition-all duration-300 transform hover:-translate-y-1 h-full"
                  >
                    <div className="flex items-start justify-between mb-4">
                      <div className="flex items-center space-x-3 flex-1 min-w-0">
                        <div className="flex items-center justify-center w-10 h-10 rounded-lg bg-[#00784B]/5 flex-shrink-0">
                          <img
                            src={pngAssets[itemData.icon] || defaultIcon}
                            alt={itemData.name}
                            className="w-6 h-6 object-contain"
                          />
                        </div>
                        <div className="min-w-0 flex-1">
                          <h4 className="font-semibold text-[#233746] text-sm leading-tight mb-1 line-clamp-2">
                            {/* {itemData.name} */}
                            {itemData.name.replace(/_/g, " ").replace(/\b\w/g, (c) => c.toUpperCase())}
                          </h4>
                        </div>
                      </div>
                      <span
                        className={`inline-flex items-center px-2.5 py-1 rounded-lg text-xs font-medium border ${getLevelBadgeColor(
                          itemData.level
                        )} flex-shrink-0`}
                      >
                        {itemData.level}
                      </span>
                    </div>

                    <div className="mb-4">
                      <div className="flex justify-between text-sm text-[#233746]/70 mb-2">
                        <span className="font-medium">Level</span>
                        <span className="font-semibold text-[#00784B]">
                          {itemData.numeric || 0}%
                        </span>
                      </div>
                      <div className="w-full bg-gray-100 rounded-full h-2.5 overflow-hidden">
                        <div
                          className={`h-2.5 rounded-full transition-all duration-500 ease-out ${getProgressColor(
                            itemData.level,
                            itemData.numeric
                          )}`}
                          style={{
                            width: `${Math.max(0, Math.min(100, itemData.numeric || 0))}%`
                          }}
                        />
                      </div>
                    </div>

                    {itemData.last_refilled && (
                      <div className="text-xs text-[#233746]/60 mb-4 flex items-center">
                        <svg
                          className="w-3.5 h-3.5 mr-1.5 text-[#00784B]"
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
                        <span>
                          Last refilled:{" "}
                          {new Date(itemData.last_refilled).toLocaleDateString("en-US", {
                            month: "short",
                            day: "numeric",
                            year: "numeric"
                          })}
                        </span>
                      </div>
                    )}

                    <button
                      onClick={() => handleRefillItem(itemKey)}
                      disabled={isLoading}
                      className={`w-full py-2.5 px-4 text-sm font-semibold rounded-lg transition-all duration-200 focus:outline-none focus:ring-2 focus:ring-offset-2 disabled:opacity-50 disabled:cursor-not-allowed transform hover:-translate-y-0.5 active:translate-y-0 ${
                        itemData.level === "low"
                          ? "bg-red-600 text-white hover:bg-red-700 focus:ring-red-500"
                          : "bg-[#00784B] text-white hover:bg-[#00784B]/90 focus:ring-[#00784B]/50"
                      }`}
                    >
                      Refill Item
                    </button>
                  </div>
                ))}
              </div>
            </div>
          ) : (
            <div className="text-center py-12 text-[#233746]/70">
              No inventory data available for this category.
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default CategoryInventoryCard;
