/**
 * ItemCustomization Component
 * Ingredient modifications and special instructions - Improved UX
 */

import React from 'react';

export default function ItemCustomization({
  item,
  itemId,
  addKitchenNote,
  updateKitchenNote,
  removeKitchenNote,
  addIngredientReplacement,
  addIngredientAddon,
  removeIngredientModification,
  getIngredientNameById,
  getIngredientDetailsById,
  ingredientsByCategory,
  updateCartItem
}) {
  // Categories to hide completely
  const hiddenCategories = ['premixes', 'sachets', 'cups', 'position'];
  
  // Categories that can only have one selection (replaceable)
  const replaceableCategories = ['espresso', 'milk', 'temperature', 'ice'];
  
  // Categories that can have multiple with quantity (syrups, etc)
  const additiveCategories = ['syrups', 'toppings', 'extras'];

  // Get current selection for a replaceable category
  const getCurrentSelection = (category) => {
    const defaultIng = item.selectedMenuItem?.default_ingredients?.find(
      di => di.category === category
    );
    
    if (!defaultIng) return null;
    
    const replacement = (item.item_ingredients || []).find(
      m => !m.isAddon && m.isModified && String(m.initialItemId) === String(defaultIng.ingredient_id)
    );
    
    return replacement || { itemId: defaultIng.ingredient_id, isDefault: true };
  };

  // Handle replaceable category selection (only one at a time)
  const handleReplaceableSelection = (category, newIngredientId) => {
    const defaultIng = item.selectedMenuItem?.default_ingredients?.find(
      di => di.category === category
    );
    
    if (!defaultIng) return;

    // Remove any existing replacement for this category
    const existingIndex = (item.item_ingredients || []).findIndex(
      m => !m.isAddon && m.isModified && String(m.initialItemId) === String(defaultIng.ingredient_id)
    );
    
    if (existingIndex >= 0) {
      removeIngredientModification(itemId, existingIndex);
    }

    // If not selecting the default, add new replacement
    if (newIngredientId !== defaultIng.ingredient_id) {
      addIngredientReplacement(itemId, defaultIng.ingredient_id, newIngredientId, 1, category);
    }
  };

  // Get addons for a category with their quantities
  const getAddonsForCategory = (category) => {
    return (item.item_ingredients || [])
      .map((mod, idx) => ({ ...mod, originalIndex: idx }))
      .filter(mod => {
        if (!mod.isAddon) return false;
        const details = getIngredientDetailsById(mod.itemId);
        return details && details.category === category;
      });
  };

  // Update addon quantity
  const updateAddonQuantity = (modIndex, newQty) => {
    if (newQty < 1) {
      removeIngredientModification(itemId, modIndex);
    } else {
      // Update the quantity
      const updatedIngredients = [...(item.item_ingredients || [])];
      updatedIngredients[modIndex] = { ...updatedIngredients[modIndex], qty: newQty };
      updateCartItem(itemId, { item_ingredients: updatedIngredients });
    }
  };

  return (
    <div className="item-customization">
      {/* Replaceable Ingredients (Espresso, Milk, Temperature, Ice) */}
      {item.selectedMenuItem && ingredientsByCategory && typeof ingredientsByCategory === 'object' && (
        <>
          {Object.entries(ingredientsByCategory).map(([category, ingredients]) => {
            // Skip hidden and non-replaceable categories
            if (hiddenCategories.includes(category.toLowerCase())) return null;
            if (!replaceableCategories.includes(category.toLowerCase())) return null;

            const currentSelection = getCurrentSelection(category);
            if (!currentSelection) return null;

            return (
              <div key={category} className="ingredient-category replaceable">
                <label className="category-label">{category.replace('_', ' ')}</label>
                
                <div className="replaceable-options">
                  {ingredients.map(ing => (
                    <button
                      key={ing.ingredient_id}
                      onClick={() => handleReplaceableSelection(category, ing.ingredient_id)}
                      className={`option-btn ${
                        String(currentSelection.itemId) === String(ing.ingredient_id) ? 'active' : ''
                      }`}
                    >
                      {ing.name || ing.type}
                      {currentSelection.isDefault && String(currentSelection.itemId) === String(ing.ingredient_id) && (
                        <span className="default-badge">default</span>
                      )}
                    </button>
                  ))}
                </div>
              </div>
            );
          })}

          {/* Additive Ingredients (Syrups with quantity control) */}
          {Object.entries(ingredientsByCategory).map(([category, ingredients]) => {
            // Skip hidden, replaceable, and non-additive categories
            if (hiddenCategories.includes(category.toLowerCase())) return null;
            if (replaceableCategories.includes(category.toLowerCase())) return null;

            const addons = getAddonsForCategory(category);
            const defaults = (item.selectedMenuItem?.default_ingredients || []).filter(
              di => di.category === category
            );

            return (
              <div key={category} className="ingredient-category additive">
                <label className="category-label">{category.replace('_', ' ')}</label>
                
                {/* Dropdown to add new */}
                <select
                  className="ingredient-select-compact"
                  onChange={(e) => {
                    if (e.target.value) {
                      // Check if already exists
                      const existing = addons.find(a => String(a.itemId) === String(e.target.value));
                      if (existing) {
                        // Increase quantity
                        updateAddonQuantity(existing.originalIndex, existing.qty + 1);
                      } else {
                        // Add new
                        addIngredientAddon(itemId, e.target.value, 1);
                      }
                      e.target.value = '';
                    }
                  }}
                >
                  <option value="">+ Add {category}...</option>
                  {ingredients.map(ing => (
                    <option key={ing.ingredient_id} value={ing.ingredient_id}>
                      {ing.name}
                    </option>
                  ))}
                </select>

                {/* Show defaults and addons */}
                {(defaults.length > 0 || addons.length > 0) && (
                  <div className="addon-list">
                    {/* Default ingredients (read-only display) */}
                    {defaults.map((def, idx) => (
                      <div key={`def-${idx}`} className="addon-item default-item">
                        <span className="addon-name">
                          {def.type} <span className="default-badge-small">default {def.quantity}×{def.unit_amount}</span>
                        </span>
                      </div>
                    ))}

                    {/* User-added addons with quantity controls */}
                    {addons.map(addon => {
                      const details = getIngredientDetailsById(addon.itemId);
                      return (
                        <div key={addon.originalIndex} className="addon-item">
                          <span className="addon-name">
                            {details?.name || getIngredientNameById(addon.itemId)}
                          </span>
                          <div className="addon-controls">
                            <button
                              onClick={() => updateAddonQuantity(addon.originalIndex, addon.qty - 1)}
                              className="qty-btn minus"
                              title="Decrease quantity"
                            >
                              −
                            </button>
                            <span className="addon-qty">{addon.qty}×</span>
                            <button
                              onClick={() => updateAddonQuantity(addon.originalIndex, addon.qty + 1)}
                              className="qty-btn plus"
                              title="Increase quantity"
                            >
                              +
                            </button>
                            <button
                              onClick={() => removeIngredientModification(itemId, addon.originalIndex)}
                              className="addon-remove"
                              title="Remove"
                            >
                              ×
                            </button>
                          </div>
                        </div>
                      );
                    })}
                  </div>
                )}
              </div>
            );
          })}
        </>
      )}

      {/* Manual Kitchen Notes */}
      <div className="manual-notes-section">
        <div className="section-header">
          <h4 className="section-title">Special Instructions</h4>
          <button
            onClick={() => addKitchenNote(itemId)}
            className="add-note-btn-small"
          >
            + Add Note
          </button>
        </div>

        {item.kitchen_notes.filter(n => n.qty > 0).length > 0 ? (
          <div className="notes-list-compact">
            {item.kitchen_notes.filter(n => n.qty > 0).map((note, idx) => {
              const actualIndex = item.kitchen_notes.findIndex(n => n === note);
              return (
                <div key={idx} className="note-item-compact">
                  <div className="note-inputs">
                    <input
                      type="text"
                      value={note.type}
                      onChange={(e) => updateKitchenNote(itemId, actualIndex, 'type', e.target.value)}
                      className="note-input-compact"
                      placeholder="e.g., Sugar"
                    />
                    <input
                      type="number"
                      min="1"
                      value={note.qty}
                      onChange={(e) => updateKitchenNote(itemId, actualIndex, 'qty', parseInt(e.target.value) || 1)}
                      className="note-qty-compact"
                      placeholder="Qty"
                    />
                  </div>
                  <input
                    type="text"
                    value={note.detail}
                    onChange={(e) => updateKitchenNote(itemId, actualIndex, 'detail', e.target.value)}
                    className="note-detail-compact"
                    placeholder="Detail (optional)"
                  />
                  <button
                    onClick={() => removeKitchenNote(itemId, actualIndex)}
                    className="note-remove-compact"
                    title="Remove note"
                  >
                    ×
                  </button>
                </div>
              );
            })}
          </div>
        ) : (
          <p className="no-notes-compact">No special instructions</p>
        )}
      </div>
    </div>
  );
}
