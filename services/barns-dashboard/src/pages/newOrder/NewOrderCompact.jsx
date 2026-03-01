/**
 * NewOrderCompact
 * POS touch-screen layout for New Order (1024x768 optimised).
 * All interactive elements meet 44px minimum touch target.
 */

import React from 'react';
import { useNewOrderState } from './useNewOrderState';
import CompactMenuGrid from './components/compact/CompactMenuGrid';
import CompactOrderCart from './components/compact/CompactOrderCart';

/* ─── Page ──────────────────────────────────────────────────────── */
export default function NewOrderCompact() {
  const {
    t,
    menuItems,
    ingredientsByCategory,
    cartItems,
    isLoading,
    dataLoading,
    searchTerm,
    setSearchTerm,
    selectedCategory,
    setSelectedCategory,
    categories,
    filteredDrinkNames,
    addToCart,
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
    handleSubmitOrder,
    handleCancel
  } = useNewOrderState();

  const totalItems = cartItems.reduce((sum, item) => sum + item.quantity, 0);
  const hasCapacityExceeded = cartItems.some(item => item.capacityExceeded === true);

  if (dataLoading) {
    return (
      <div className="flex h-[calc(100vh-48px)] items-center justify-center bg-gray-50">
        <div className="flex flex-col items-center gap-3">
          <svg className="animate-spin h-9 w-9 text-green-600" fill="none" viewBox="0 0 24 24">
            <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
            <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" />
          </svg>
          <span className="text-sm text-gray-500 font-medium">{t('loadingMenu') || 'Loading menu...'}</span>
        </div>
      </div>
    );
  }

  return (
    <div className="flex h-[calc(100vh-48px)] bg-gray-100 gap-2 p-2 overflow-hidden">

      {/* ── Left: Menu grid ───────────────────────────────────── */}
      <div className="flex-1 bg-white rounded-2xl shadow-sm overflow-hidden min-w-0">
        <CompactMenuGrid
          drinkNames={filteredDrinkNames}
          menuItems={menuItems}
          searchTerm={searchTerm}
          setSearchTerm={setSearchTerm}
          selectedCategory={selectedCategory}
          setSelectedCategory={setSelectedCategory}
          categories={categories}
          addToCart={addToCart}
        />
      </div>

      <CompactOrderCart
        cartItems={cartItems}
        totalItems={totalItems}
        hasCapacityExceeded={hasCapacityExceeded}
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
        handleSubmitOrder={handleSubmitOrder}
        handleCancel={handleCancel}
        isLoading={isLoading}
        t={t}
      />
    </div>
  );
}
