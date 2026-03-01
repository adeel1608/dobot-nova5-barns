/**
 * CompactOrderCart
 * Touch-screen optimized order cart layout for POS.
 * Replaces the default OrderCart with a sleeker, space-saving design.
 */

import React from 'react';
import CompactItemCustomization from './CompactItemCustomization';

/* ─── Cart item ─────────────────────────────────────────────────── */
function CompactCartItem({
    item,
    removeFromCart,
    updateQuantity,
    updateCartItem,
    addKitchenNote,
    updateKitchenNote,
    removeKitchenNote,
    addIngredientReplacement,
    addIngredientAddon,
    removeIngredientModification,
    getIngredientNameById,
    getIngredientDetailsById,
    ingredientsByCategory,
    t
}) {
    const modCount =
        (item.kitchen_notes || []).filter(n => n.qty > 0).length +
        (item.item_ingredients || []).filter(
            m => !m.isIceModification && !m.isMilkAmountModification && !m.isWaterAmountModification
        ).length;

    return (
        <div
            className={`rounded-2xl p-3.5 mb-2.5 transition-all duration-150 relative ${item.capacityExceeded
                ? 'bg-red-50 hover:shadow-sm'
                : 'bg-white hover:shadow-sm'
                }`}
            style={{ border: item.capacityExceeded ? '2px solid #fca5a5' : '2px solid #d1d5db' }}
        >
            {/* Row 1: name (left) — size (right) */}
            <div className="flex items-baseline justify-between gap-2 mb-2">
                <p className="text-[14px] font-bold text-gray-900 truncate leading-tight min-w-0" style={{ fontSize: '14px' }}>{item.selectedDrinkName}</p>
                <span className="text-[11px] font-semibold text-gray-400 uppercase tracking-wide flex-shrink-0" style={{ fontSize: '11px' }}>{item.selectedSize}</span>
            </div>

            {/* Row 2: stepper left — edit+delete right */}
            <div className="flex items-center gap-2">
                {/* Quantity stepper — no border, bigger buttons */}
                <div className="flex items-center gap-1 flex-shrink-0">
                    <button
                        type="button"
                        onClick={() => updateQuantity(item.id, item.quantity - 1)}
                        style={{ padding: 0 }}
                        className="w-8 h-8 flex items-center justify-center rounded-[10px] text-[16px] font-bold text-green-700 bg-green-100 hover:bg-green-200 active:scale-95 transition-all leading-none"
                    >
                        −
                    </button>
                    <span className="text-sm font-bold text-gray-800 min-w-[22px] text-center select-none">
                        {item.quantity}
                    </span>
                    <button
                        type="button"
                        onClick={() => updateQuantity(item.id, item.quantity + 1)}
                        style={{ padding: 0 }}
                        className="w-8 h-8 flex items-center justify-center rounded-[10px] text-[16px] font-bold text-white bg-green-600 hover:bg-green-700 active:scale-95 transition-all leading-none"
                    >
                        +
                    </button>
                </div>

                {/* Spacer */}
                <div className="flex-1" />

                {/* Mod count badge */}
                {modCount > 0 && (
                    <span className="font-bold bg-green-100 text-green-800 rounded-lg px-1.5 h-8 flex items-center justify-center flex-shrink-0 border border-green-200" style={{ fontSize: '9px' }}>
                        {modCount}
                    </span>
                )}

                {/* Edit */}
                <button
                    type="button"
                    onClick={() => updateCartItem(item.id, { isCustomizeOpen: !item.isCustomizeOpen })}
                    style={{ padding: 0, border: item.isCustomizeOpen ? '2px solid #22c55e' : '2px solid #9ca3af' }}
                    className={`w-12 h-8 flex items-center justify-center rounded-xl transition-all flex-shrink-0 ${item.isCustomizeOpen
                        ? 'bg-green-50 text-green-800'
                        : 'bg-gray-50 text-gray-500 hover:bg-white hover:text-green-700'
                        }`}
                >
                    <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="2">
                        <path strokeLinecap="round" strokeLinejoin="round" d="M11 5H6a2 2 0 00-2 2v11a2 2 0 002 2h11a2 2 0 002-2v-5m-1.414-9.414a2 2 0 112.828 2.828L11.828 15H9v-2.828l8.586-8.586z" />
                    </svg>
                </button>

                {/* Delete */}
                <button
                    type="button"
                    onClick={() => removeFromCart(item.id)}
                    style={{ padding: 0, border: '2px solid #f87171' }}
                    className="w-12 h-8 flex flex-shrink-0 items-center justify-center rounded-xl bg-red-50 text-red-500 hover:bg-red-100 active:scale-95 transition-all"
                    aria-label={t('removeItem')}
                >
                    <svg className="w-4 h-4 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="2">
                        <path strokeLinecap="round" strokeLinejoin="round" d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16" />
                    </svg>
                </button>
            </div>

            {item.capacityExceeded && (
                <span className="text-[9px] font-bold bg-red-100 text-red-700 rounded-lg px-2 mt-1.5 inline-flex items-center border border-red-200" style={{ fontSize: '9px' }}>
                    Over limit
                </span>
            )}

            {/* Customisation panel */}
            {item.isCustomizeOpen && (
                <div className="mt-3 pt-3 border-t-2 border-gray-100 animation-fade-in">
                    <CompactItemCustomization
                        item={item}
                        itemId={item.id}
                        addKitchenNote={addKitchenNote}
                        updateKitchenNote={updateKitchenNote}
                        removeKitchenNote={removeKitchenNote}
                        addIngredientReplacement={addIngredientReplacement}
                        addIngredientAddon={addIngredientAddon}
                        removeIngredientModification={removeIngredientModification}
                        getIngredientNameById={getIngredientNameById}
                        getIngredientDetailsById={getIngredientDetailsById}
                        ingredientsByCategory={ingredientsByCategory}
                        updateCartItem={updateCartItem}
                    />
                </div>
            )}
        </div>
    );
}

/* ─── Main Cart Component ───────────────────────────────────────── */
export default function CompactOrderCart({
    cartItems = [],
    totalItems = 0,
    hasCapacityExceeded = false,
    removeFromCart,
    updateQuantity,
    updateCartItem,
    addKitchenNote,
    updateKitchenNote,
    removeKitchenNote,
    addIngredientReplacement,
    addIngredientAddon,
    removeIngredientModification,
    getIngredientNameById,
    getIngredientDetailsById,
    ingredientsByCategory,
    handleSubmitOrder,
    handleCancel,
    isLoading,
    t
}) {
    return (
        <div className="w-[340px] bg-white rounded-2xl shadow-sm border border-gray-100 flex flex-col overflow-hidden flex-shrink-0 h-full">
            {/* Header */}
            <div className="flex items-center justify-between px-4 py-3.5 border-b-2 border-gray-50 flex-shrink-0 bg-white z-10">
                <span className="text-[17px] font-bold text-gray-900 tracking-tight">{t('yourOrder')}</span>
                <span
                    className={`text-[11px] font-bold px-3 py-1.5 rounded-full transition-colors uppercase tracking-wider ${totalItems > 0 ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-500'
                        }`}
                >
                    {totalItems} {t('totalItems')}
                </span>
            </div>

            {/* Items list */}
            <div
                className="flex-1 overflow-y-auto px-3 pt-3.5 pb-2 bg-gray-50/50"
                style={{ scrollbarWidth: 'thin', scrollbarColor: '#bbf7d0 transparent' }}
            >
                {cartItems.length === 0 ? (
                    <div className="flex flex-col items-center justify-center h-full gap-4 pb-10">
                        <div className="w-16 h-16 rounded-full bg-gray-100 flex items-center justify-center">
                            <svg className="w-8 h-8 text-gray-300" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="2">
                                <path strokeLinecap="round" strokeLinejoin="round" d="M16 11V7a4 4 0 00-8 0v4M5 9h14l1 12H4L5 9z" />
                            </svg>
                        </div>
                        <div className="text-center">
                            <p className="text-[15px] font-bold text-gray-500">{t('cartEmpty')}</p>
                            <p className="text-xs font-medium text-gray-400 mt-1">{t('addItemsToStart')}</p>
                        </div>
                    </div>
                ) : (
                    cartItems.map(item => (
                        <CompactCartItem
                            key={item.id}
                            item={item}
                            removeFromCart={removeFromCart}
                            updateQuantity={updateQuantity}
                            updateCartItem={updateCartItem}
                            addKitchenNote={addKitchenNote}
                            updateKitchenNote={updateKitchenNote}
                            removeKitchenNote={removeKitchenNote}
                            addIngredientReplacement={addIngredientReplacement}
                            addIngredientAddon={addIngredientAddon}
                            removeIngredientModification={removeIngredientModification}
                            getIngredientNameById={getIngredientNameById}
                            getIngredientDetailsById={getIngredientDetailsById}
                            ingredientsByCategory={ingredientsByCategory}
                            t={t}
                        />
                    ))
                )}
            </div>

            {/* Footer / Checkout Actions */}
            <div className="px-3 pb-3 pt-3 border-t-2 border-gray-50 bg-white flex flex-col gap-2 flex-shrink-0 z-10 shadow-[0_-4px_20px_-10px_rgba(0,0,0,0.05)]">
                {/* Buttons row — side by side */}
                <div className="flex gap-2">
                    {/* Cancel Button */}
                    <button
                        type="button"
                        onClick={handleCancel}
                        disabled={isLoading || cartItems.length === 0}
                        className="flex-1 py-3 rounded-xl text-sm font-bold text-gray-500 bg-gray-100 hover:bg-red-50 hover:text-red-500 active:bg-red-100 transition-all disabled:opacity-40"
                        style={{ border: '2px solid #9ca3af' }}
                    >
                        {t('cancel')}
                    </button>

                    {/* Process Order Button */}
                    <button
                        type="button"
                        onClick={handleSubmitOrder}
                        disabled={isLoading || cartItems.length === 0 || hasCapacityExceeded}
                        className={`flex-[2] py-3 rounded-xl text-sm font-bold flex items-center justify-center gap-2 transition-all active:scale-[0.98] ${isLoading
                            ? 'bg-blue-500 text-white cursor-not-allowed opacity-90'
                            : hasCapacityExceeded
                                ? 'bg-amber-500 text-white cursor-not-allowed opacity-90'
                                : cartItems.length === 0
                                    ? 'bg-gray-100 text-gray-400 cursor-not-allowed'
                                    : 'bg-green-600 hover:bg-green-700 active:bg-green-800 text-white shadow-[0_4px_12px_rgba(22,163,74,0.3)]'
                            }`}
                    >
                        {isLoading ? (
                            <>
                                <svg className="animate-spin w-4 h-4" fill="none" viewBox="0 0 24 24">
                                    <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
                                    <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" />
                                </svg>
                                {t('processing')}
                            </>
                        ) : hasCapacityExceeded ? (
                            <>
                                <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="2.5">
                                    <path strokeLinecap="round" strokeLinejoin="round" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
                                </svg>
                                Exceeded
                            </>
                        ) : (
                            <>
                                {t('processOrder')} ({totalItems})
                            </>
                        )}
                    </button>
                </div>
            </div>
        </div>
    );
}
