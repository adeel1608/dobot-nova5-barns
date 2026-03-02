import React, { useState, useEffect } from "react";
import { useInventoryStore } from "../../../../store/inventoryStore";

// Direct imports for all category/item icons
import coffee_beans from "../../../../assets/coffee_beans.png";
import beans from "../../../../assets/beans.png";
import cups from "../../../../assets/cups.png";
import cup from "../../../../assets/cup.png";
import milks from "../../../../assets/milks.png";
import milk from "../../../../assets/milk.png";
import syrups from "../../../../assets/syrups.png";
import syrup from "../../../../assets/syrup.png";
import premixes from "../../../../assets/premixes.png";
import defaultIcon from "../../../../assets/default.png";

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

const CompactCategoryCard = ({ category, isAllView, count }) => {
    const [expanded, setExpanded] = useState(!isAllView);
    const [refillingCategory, setRefillingCategory] = useState(false);

    const {
        FullCategoryInfo,
        inventoryStatus,
        refillInventory,
        refillCategory,
        isLoading,
        fetchCategoryInfoData
    } = useInventoryStore();

    useEffect(() => {
        fetchCategoryInfoData();
        setExpanded(!isAllView);
    }, [isAllView, fetchCategoryInfoData]);

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
            icon: `${category}.png`,
        };
    });

    const handleRefillItem = async (itemKey) => {
        await refillInventory(itemKey, 100);
    };

    const handleRefillCategory = async () => {
        setRefillingCategory(true);
        try {
            await refillCategory(category, 100);
        } finally {
            setTimeout(() => {
                setRefillingCategory(false);
            }, 500);
        }
    };

    const getProgressColor = (level, numeric) => {
        if (level === "low" || numeric < 20) return "bg-red-500";
        if (level === "medium" || numeric < 60) return "bg-orange-500";
        return "bg-green-500";
    };

    const getLevelBadgeColor = (level) => {
        switch (level) {
            case "low":
                return "bg-red-100 text-red-800 border-red-200";
            case "medium":
                return "bg-orange-100 text-orange-800 border-orange-200";
            case "high":
                return "bg-green-100 text-green-800 border-green-200";
            default:
                return "bg-gray-100 text-gray-800 border-gray-200";
        }
    };

    const itemCount = Object.keys(items).length;

    const ingredientStats = Object.values(items).reduce((acc, item) => {
        const level = item.level || 'unknown';
        acc[level] = (acc[level] || 0) + 1;
        return acc;
    }, {});

    return (
        <div className="bg-white rounded-2xl overflow-hidden mb-4 transition-all" style={{ border: '1px solid #e5e7eb' }}>
            {/* Header */}
            <div
                className="px-4 py-3 flex items-center justify-between bg-white cursor-pointer hover:bg-gray-50/50"
                onClick={() => setExpanded(!expanded)}
            >
                <div className="flex items-center gap-4">
                    <div className="flex items-center justify-center w-12 h-12 rounded-xl bg-[#00784B]/5 flex-shrink-0" style={{ border: '1px solid rgba(0, 120, 75, 0.1)' }}>
                        <img
                            src={pngAssets[`${category}.png`] || defaultIcon}
                            alt={category}
                            className="w-7 h-7 object-contain opacity-80 mix-blend-multiply"
                        />
                    </div>
                    <div>
                        <div className="flex items-center gap-2">
                            <h3 className="text-lg font-bold text-gray-900 leading-tight">
                                {category.replace(/_/g, " ").replace(/\b\w/g, (c) => c.toUpperCase())}
                            </h3>
                            <svg
                                className={`w-5 h-5 text-[#00784B] font-bold transition-transform ${expanded ? "rotate-180" : ""}`}
                                fill="none" stroke="currentColor" viewBox="0 0 24 24"
                            >
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M19 9l-7 7-7-7" />
                            </svg>
                        </div>

                        <div className="flex items-center gap-2 mt-1">
                            <span className="text-xs text-gray-500 font-medium">
                                {itemCount} {itemCount === 1 ? 'ingredient:' : 'ingredients:'}
                            </span>
                            {ingredientStats.high > 0 && (
                                <div className="flex items-center gap-1.5 px-2 py-0.5 rounded-full" style={{ border: '1px solid #bbf7d0' }}>
                                    <div className="w-1.5 h-1.5 rounded-full bg-green-500"></div>
                                    <span className="text-[10px] font-bold text-green-700">{ingredientStats.high}</span>
                                </div>
                            )}
                            {ingredientStats.medium > 0 && (
                                <div className="flex items-center gap-1.5 px-2 py-0.5 rounded-full" style={{ border: '1px solid #fed7aa' }}>
                                    <div className="w-1.5 h-1.5 rounded-full bg-orange-500"></div>
                                    <span className="text-[10px] font-bold text-orange-700">{ingredientStats.medium}</span>
                                </div>
                            )}
                            {ingredientStats.low > 0 && (
                                <div className="flex items-center gap-1.5 px-2 py-0.5 rounded-full" style={{ border: '1px solid #fecaca' }}>
                                    <div className="w-1.5 h-1.5 rounded-full bg-red-500"></div>
                                    <span className="text-[10px] font-bold text-red-700">{ingredientStats.low}</span>
                                </div>
                            )}
                        </div>
                    </div>
                </div>

                <div className="flex items-center gap-3">
                    <button
                        onClick={(e) => { e.stopPropagation(); handleRefillCategory(); }}
                        disabled={refillingCategory || isLoading}
                        className="px-5 min-h-[40px] bg-[#00784B] text-white text-sm font-bold rounded-xl hover:bg-[#00784B]/90 active:scale-95 transition-all disabled:opacity-50 whitespace-nowrap"
                        style={{ border: '1px solid #005a38' }}
                    >
                        {refillingCategory ? '...' : 'Refill All'}
                    </button>
                    <div className="w-10 h-10 rounded-xl bg-gray-50 flex items-center justify-center text-gray-500" style={{ border: '1px solid #e5e7eb' }}>
                        <svg
                            className={`w-5 h-5 transition-transform ${expanded ? "rotate-180" : ""}`}
                            fill="none" stroke="currentColor" viewBox="0 0 24 24"
                        >
                            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
                        </svg>
                    </div>
                </div>
            </div>

            {/* Items Grid (Compact) */}
            {expanded && (
                <div className="p-3 bg-gray-50/50 border-t border-gray-100">
                    {Object.keys(items).length > 0 ? (
                        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-3">
                            {Object.entries(items).map(([itemKey, itemData]) => (
                                <div
                                    key={itemKey}
                                    className="bg-white rounded-xl p-3 flex items-center justify-between gap-3 min-h-[70px] hover:border-green-200 transition-colors"
                                    style={{ border: '1px solid #e5e7eb' }}
                                >
                                    <div className="flex items-center gap-3 min-w-0 flex-1">
                                        <div className="flex items-center justify-center w-10 h-10 rounded-lg bg-gray-50 flex-shrink-0 p-1" style={{ border: '1px solid #e5e7eb' }}>
                                            <img
                                                src={pngAssets[itemData.icon] || defaultIcon}
                                                alt={itemData.name}
                                                className="w-full h-full object-contain opacity-70"
                                            />
                                        </div>
                                        <div className="min-w-0 flex-1">
                                            <h4 className="font-bold text-gray-800 text-sm leading-snug truncate mb-1">
                                                {itemData.name.replace(/_/g, " ").replace(/\b\w/g, (c) => c.toUpperCase())}
                                            </h4>
                                            <div className="flex items-center gap-2">
                                                <div className="flex-1 max-w-[100px] bg-gray-100 rounded-full h-1.5 overflow-hidden">
                                                    <div
                                                        className={`h-1.5 rounded-full transition-all duration-500 ${getProgressColor(itemData.level, itemData.numeric)}`}
                                                        style={{ width: `${Math.max(0, Math.min(100, itemData.numeric || 0))}%` }}
                                                    />
                                                </div>
                                                <span className={`text-[10px] font-bold ${itemData.numeric < 20 ? 'text-red-600' : 'text-gray-500'}`}>
                                                    {itemData.numeric || 0}%
                                                </span>
                                            </div>
                                        </div>
                                    </div>

                                    <button
                                        onClick={() => handleRefillItem(itemKey)}
                                        disabled={isLoading}
                                        className={`px-3 min-h-[32px] text-xs font-bold rounded-lg transition-all active:scale-95 disabled:opacity-50 flex-shrink-0 ${itemData.level === "low" || itemData.level === "empty"
                                            ? "bg-red-50 text-red-600 hover:bg-red-100"
                                            : "bg-gray-50 text-gray-600 hover:bg-gray-100 hover:text-green-700"
                                            }`}
                                        style={{ border: itemData.level === "low" || itemData.level === "empty" ? "1px solid #fecaca" : "1px solid #e5e7eb" }}
                                    >
                                        Refill
                                    </button>
                                </div>
                            ))}
                        </div>
                    ) : (
                        <div className="text-center py-6 text-sm text-gray-500 font-medium">
                            No inventory data available for this category.
                        </div>
                    )}
                </div>
            )}
        </div>
    );
};

export default CompactCategoryCard;
