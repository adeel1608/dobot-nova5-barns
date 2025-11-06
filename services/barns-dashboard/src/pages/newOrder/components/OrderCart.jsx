/**
 * OrderCart Component
 * Displays selected items and checkout interface
 */

import React from 'react';
import ItemCustomization from './ItemCustomization';

export default function OrderCart({
  cartItems,
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
  isLoading
}) {
  const totalItems = cartItems.reduce((sum, item) => sum + item.quantity, 0);

  return (
    <div className="order-cart">
      {/* Cart Header */}
      <div className="cart-header">
        <h2 className="cart-title">Your Order</h2>
        <div className="cart-count">
          {totalItems} item{totalItems !== 1 ? 's' : ''}
        </div>
      </div>

      {/* Cart Items */}
      <div className="cart-items">
        {cartItems.length === 0 ? (
          <div className="empty-cart">
            <svg className="empty-cart-icon" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M16 11V7a4 4 0 00-8 0v4M5 9h14l1 12H4L5 9z" />
            </svg>
            <p className="empty-cart-text">Your cart is empty</p>
            <p className="empty-cart-subtext">Add items from the menu to get started</p>
          </div>
        ) : (
          cartItems.map((item) => (
            <div key={item.id} className="cart-item">
              {/* Item Header */}
              <div className="cart-item-header">
                <div className="cart-item-info">
                  <h3 className="cart-item-name">{item.selectedDrinkName}</h3>
                  <p className="cart-item-size">{item.selectedSize}</p>
                </div>
                <button
                  onClick={() => removeFromCart(item.id)}
                  className="remove-item-btn"
                  title="Remove item"
                >
                  <svg className="icon" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16" />
                  </svg>
                </button>
              </div>

              {/* Quantity Controls */}
              <div className="quantity-controls">
                <label className="quantity-label">Quantity:</label>
                <div className="quantity-buttons">
                  <button
                    onClick={() => updateQuantity(item.id, item.quantity - 1)}
                    className="quantity-btn"
                  >
                    −
                  </button>
                  <span className="quantity-value">{item.quantity}</span>
                  <button
                    onClick={() => updateQuantity(item.id, item.quantity + 1)}
                    className="quantity-btn"
                  >
                    +
                  </button>
                </div>
              </div>

              {/* Customization Toggle */}
              <button
                onClick={() => updateCartItem(item.id, { isCustomizeOpen: !item.isCustomizeOpen })}
                className="customize-toggle"
              >
                <svg className="icon" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4" />
                </svg>
                Customize
                <svg 
                  className={`chevron ${item.isCustomizeOpen ? 'open' : ''}`} 
                  fill="none" 
                  stroke="currentColor" 
                  viewBox="0 0 24 24"
                >
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
                </svg>
              </button>

              {/* Customization Panel */}
              {item.isCustomizeOpen && (
                <ItemCustomization
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
              )}

              {/* Modifications Summary */}
              {(item.kitchen_notes.length > 0 || item.item_ingredients.length > 0) && (
                <div className="modifications-summary">
                  {item.kitchen_notes.filter(n => n.qty > 0).length > 0 && (
                    <span className="mod-badge">
                      {item.kitchen_notes.filter(n => n.qty > 0).length} note{item.kitchen_notes.filter(n => n.qty > 0).length !== 1 ? 's' : ''}
                    </span>
                  )}
                  {item.item_ingredients.length > 0 && (
                    <span className="mod-badge">
                      {item.item_ingredients.length} modification{item.item_ingredients.length !== 1 ? 's' : ''}
                    </span>
                  )}
                </div>
              )}
            </div>
          ))
        )}
      </div>

      {/* Cart Footer / Checkout */}
      <div className="cart-footer">
        {/* Order Summary */}
        {cartItems.length > 0 && (
          <div className="order-summary">
            <div className="summary-row">
              <span className="summary-label">Total Items:</span>
              <span className="summary-value">{totalItems}</span>
            </div>
          </div>
        )}

        {/* Action Buttons */}
        <div className="cart-actions">
          <button
            onClick={handleCancel}
            disabled={isLoading}
            className="btn-cancel"
          >
            Cancel
          </button>
          <button
            onClick={handleSubmitOrder}
            disabled={isLoading || cartItems.length === 0}
            className="btn-process"
          >
            {isLoading ? (
              <>
                <svg className="spinner" fill="none" viewBox="0 0 24 24">
                  <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                  <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                </svg>
                Processing...
              </>
            ) : (
              <>
                <svg className="icon" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
                </svg>
                Process Order
              </>
            )}
          </button>
        </div>
      </div>
    </div>
  );
}

