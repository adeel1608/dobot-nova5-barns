import React, { useEffect, useState } from "react";
import { useInventoryStore } from "../../store/inventoryStore";
import { useTranslation } from "../../store/translationsStore";
import { SERVICE_OFFLINE_MESSAGE } from "../../utils/errorHandler";
import CompactCategoryCard from "./components/compact/CompactCategoryCard";

export default function InventoryCompact() {
    const { t } = useTranslation('inventory');
    const [activeTab, setActiveTab] = useState("all");

    const {
        fetchInventoryStatus,
        getInventoryStats,
        getLowInventoryItems,
        hasLowInventory,
        refillCategory,
        isLoading,
        fetchStockLevelData,
        fetchFullStockSummaryData,
        errors,
    } = useInventoryStore();

    useEffect(() => {
        const inventoryStore = useInventoryStore.getState();
        const hasInventoryData = Object.keys(inventoryStore.inventoryStatus || {}).length > 0;

        if (!hasInventoryData) {
            Promise.allSettled([
                fetchInventoryStatus(),
                fetchStockLevelData(),
                fetchFullStockSummaryData()
            ]).catch(err => console.error("Inventory loading error:", err));
        } else {
            Promise.allSettled([
                fetchStockLevelData(),
                fetchFullStockSummaryData()
            ]).catch(err => console.error("Inventory refresh error:", err));
        }
    }, [fetchInventoryStatus, fetchStockLevelData, fetchFullStockSummaryData]);

    const handleRefillAllLow = async () => {
        const lowItems = getLowInventoryItems();
        const categoriesWithLowItems = [...new Set(lowItems.map((item) => item.category))];
        for (const category of categoriesWithLowItems) {
            await refillCategory(category, 100);
        }
    };

    const stockLevelData = useInventoryStore((state) => state.inventoryStockLevel);
    const stocklevel = stockLevelData?.stock_level || { high: 0, medium: 0, low: 0, empty: 0, total: 0 };
    const lowItems = getLowInventoryItems();
    const FullStockSummary = useInventoryStore(state => state.FullStockSummary);
    const categoryDetails = FullStockSummary || {};
    const totalFullStock = Object.values(categoryDetails).reduce((sum, count) => sum + count, 0);

    const tabs = [
        { id: 'all', name: t('allCategories') || 'All', count: totalFullStock },
        ...Object.entries(categoryDetails).map(([key, count]) => ({
            id: key,
            name: key.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase()),
            count
        }))
    ];

    return (
        <div className="flex flex-col h-[calc(100vh-48px)] bg-gray-50 overflow-hidden">

            {/* ── Main Content Area ─────────────────────────────────── */}
            <div className="flex-1 flex flex-col min-w-0">

                {/* Top Header & Stats (Compact) - Matching Screenshot Style */}
                <div className="flex-shrink-0 pt-2 px-2">
                    <div className="flex items-stretch gap-3 overflow-x-auto pb-2" style={{ scrollbarWidth: 'none' }}>

                        {/* Total Items Card */}
                        <div className="flex-1 min-w-[110px] bg-blue-50/30 rounded-2xl p-2.5 flex items-center" style={{ border: '1px solid #dbeafe' }}>
                            <div className="w-10 h-10 bg-blue-100 rounded-xl flex items-center justify-center flex-shrink-0" style={{ border: '1px solid #bfdbfe' }}>
                                <svg className="w-5 h-5 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
                                </svg>
                            </div>
                            <div className="ml-3 min-w-0">
                                <p className="text-[11px] font-semibold text-blue-600 truncate">{t('totalItems') || 'Total Items'}</p>
                                <p className="text-lg font-bold text-gray-900">{stocklevel.total}</p>
                            </div>
                        </div>

                        {/* High Stock Card */}
                        <div className="flex-1 min-w-[110px] bg-green-50/30 rounded-2xl p-2.5 flex items-center" style={{ border: '1px solid #dcfce7' }}>
                            <div className="w-10 h-10 bg-green-100 rounded-xl flex items-center justify-center flex-shrink-0" style={{ border: '1px solid #bbf7d0' }}>
                                <svg className="w-5 h-5 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                                </svg>
                            </div>
                            <div className="ml-3 min-w-0">
                                <p className="text-[11px] font-semibold text-green-600 truncate">{t('highStock') || 'High Stock'}</p>
                                <p className="text-lg font-bold text-gray-900">{stocklevel.high}</p>
                            </div>
                        </div>

                        {/* Medium Stock Card */}
                        <div className="flex-1 min-w-[110px] bg-yellow-50/30 rounded-2xl p-2.5 flex items-center" style={{ border: '1px solid #fef08a' }}>
                            <div className="w-10 h-10 bg-yellow-100 rounded-xl flex items-center justify-center flex-shrink-0" style={{ border: '1px solid #fde047' }}>
                                <svg className="w-5 h-5 text-yellow-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                                </svg>
                            </div>
                            <div className="ml-3 min-w-0">
                                <p className="text-[11px] font-semibold text-yellow-600 truncate">{t('mediumStock') || 'Medium Stock'}</p>
                                <p className="text-lg font-bold text-gray-900">{stocklevel.medium}</p>
                            </div>
                        </div>

                        {/* Low Stock Card */}
                        <div className="flex-1 min-w-[110px] bg-orange-50/30 rounded-2xl p-2.5 flex items-center" style={{ border: '1px solid #ffedd5' }}>
                            <div className="w-10 h-10 bg-orange-100 rounded-xl flex items-center justify-center flex-shrink-0" style={{ border: '1px solid #fed7aa' }}>
                                <svg className="w-5 h-5 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
                                </svg>
                            </div>
                            <div className="ml-3 min-w-0">
                                <p className="text-[11px] font-semibold text-orange-600 truncate">{t('lowStock') || 'Low Stock'}</p>
                                <p className="text-lg font-bold text-gray-900">{stocklevel.low}</p>
                            </div>
                        </div>

                        {/* Empty Stock Card */}
                        <div className="flex-1 min-w-[110px] bg-red-50/30 rounded-2xl p-2.5 flex items-center" style={{ border: '1px solid #fee2e2' }}>
                            <div className="w-10 h-10 bg-red-400 rounded-xl flex items-center justify-center flex-shrink-0" style={{ border: '1px solid #f87171' }}>
                                <svg className="w-5 h-5 text-red-100" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                                </svg>
                            </div>
                            <div className="ml-3 min-w-0">
                                <p className="text-[11px] font-semibold text-red-600 truncate">{t('emptyStock') || 'Empty Stock'}</p>
                                <p className="text-lg font-bold text-gray-900">{stocklevel.empty}</p>
                            </div>
                        </div>

                    </div>


                    {/* Low Stock Alert Compact Banner */}
                    {lowItems.length > 0 && (
                        <div className="bg-red-50 rounded-xl p-2.5 flex items-center justify-between" style={{ border: '1px solid #fecaca' }}>
                            <div className="flex items-center gap-2 flex-1 min-w-0">
                                <div className="w-8 h-8 rounded-full bg-red-100 flex items-center justify-center flex-shrink-0" style={{ border: '1px solid #fca5a5' }}>
                                    <svg className="w-4 h-4 text-red-500" fill="currentColor" viewBox="0 0 20 20">
                                        <path fillRule="evenodd" d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
                                    </svg>
                                </div>
                                <div className="min-w-0">
                                    <h3 className="text-sm font-bold text-red-800 line-clamp-1">
                                        {lowItems.length} {t('itemsRunningLow') || 'Items Running Low'}
                                    </h3>
                                    <p className="text-xs font-semibold text-red-600/80 truncate">
                                        {lowItems.slice(0, 4).map(i => i.name).join(', ')}{lowItems.length > 4 ? '...' : ''}
                                    </p>
                                </div>
                            </div>

                            {hasLowInventory() && (
                                <button
                                    onClick={handleRefillAllLow}
                                    disabled={isLoading}
                                    className="px-4 min-h-[40px] bg-red-600 text-white text-sm font-bold rounded-xl hover:bg-red-700 active:scale-95 disabled:opacity-50 transition-all flex-shrink-0 ml-3"
                                >
                                    {isLoading ? '...' : t('refillAll') || 'Refill All Low'}
                                </button>
                            )}
                        </div>
                    )}
                </div>

                {/* Categories / Grid Area */}
                <div className="flex-1 flex flex-col overflow-hidden mt-1">

                    {/* Tabs - Outline styled like screenshot */}
                    <div className="flex flex-shrink-0 overflow-x-auto pb-3 gap-3 px-3 mt-2" style={{ scrollbarWidth: 'none' }}>
                        {tabs.map((tab) => (
                            <button
                                key={tab.id}
                                onClick={() => setActiveTab(tab.id)}
                                className={`flex items-center gap-2 whitespace-nowrap px-4 py-2 min-h-[48px] rounded-xl font-bold transition-all flex-shrink-0 active:scale-95 ${activeTab === tab.id
                                    ? "bg-[#00784B] text-white"
                                    : "bg-white text-[#00784B] hover:bg-green-50/50"
                                    }`}
                                style={{ border: activeTab === tab.id ? '2px solid #00784B' : '2px solid rgba(187, 247, 208, 0.6)' }}
                            >
                                <span className="text-sm">{tab.name}</span>
                                <span className={`px-2 py-0.5 rounded-full text-xs font-bold ${activeTab === tab.id ? 'bg-white/20 text-white' : 'bg-gray-100 text-gray-400'}`}>
                                    {tab.count}
                                </span>
                            </button>
                        ))}
                    </div>

                    {/* Cards List */}
                    <div className="flex-1 overflow-y-auto px-3 pb-4" style={{ scrollbarWidth: 'thin', scrollbarColor: '#bbf7d0 transparent' }}>
                        {errors.inventory && (
                            <div className="mb-4 bg-red-50 rounded-xl p-3 text-sm text-red-800 font-bold" style={{ border: '1px solid #fecaca' }}>
                                {t('error') || 'Error'}: {errors.inventory === SERVICE_OFFLINE_MESSAGE ? t('serviceOfflineUnreachable') : errors.inventory}
                            </div>
                        )}

                        {activeTab === "all" ? (
                            Object.entries(categoryDetails).map(([category, count]) => (
                                <CompactCategoryCard key={category} category={category} count={count} isAllView={true} />
                            ))
                        ) : (
                            <CompactCategoryCard category={activeTab} count={categoryDetails[activeTab] || 0} isAllView={false} />
                        )}
                    </div>
                </div>

            </div>

            {/* Loading Overlay */}
            {isLoading && (
                <div className="absolute inset-0 bg-white/40 backdrop-blur-sm flex items-center justify-center z-50 rounded-2xl m-2">
                    <div className="bg-white p-4 px-6 rounded-2xl shadow-xl flex flex-col items-center gap-3 border border-gray-100">
                        <svg className="animate-spin h-8 w-8 text-green-600" fill="none" viewBox="0 0 24 24">
                            <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                            <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                        </svg>
                        <span className="text-gray-900 font-bold">{t('processing') || 'Processing...'}</span>
                    </div>
                </div>
            )}
        </div>
    );
}
