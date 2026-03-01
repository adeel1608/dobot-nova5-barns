/**
 * New Order Page
 * Default (full-screen) layout for creating POS orders.
 * Compact POS variant is handled by NewOrderCompact.
 */

import React from 'react';
import { useNewOrderState } from './useNewOrderState';
import MenuGrid from './components/MenuGrid';
import OrderCart from './components/OrderCart';
import './styles.css';

export default function NewOrderPage() {
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

  if (dataLoading) {
    return (
      <div className="new-order-page">
        <div className="flex items-center justify-center h-full">
          <div className="text-center">
            <svg className="animate-spin h-12 w-12 text-green-600 mx-auto mb-4" fill="none" viewBox="0 0 24 24">
              <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
              <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" />
            </svg>
            <p className="text-gray-600">{t('loadingMenu') || 'Loading menu...'}</p>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="new-order-page">
      <div className="new-order-content">
        <div className="menu-panel">
          <MenuGrid
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
        <div className="cart-panel">
          <OrderCart
            cartItems={cartItems}
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
          />
        </div>
      </div>
    </div>
  );
}
