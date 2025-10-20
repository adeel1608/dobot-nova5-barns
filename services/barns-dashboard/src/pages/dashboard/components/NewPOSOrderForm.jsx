import React from 'react';

function NewPOSOrderForm({
  posOrderData,
  setPosOrderData,
  menuItems,
  ingredientsByCategory,
  isLoading,
  uniqueDrinkNames,
  handleSubmitPOSOrder,
  addPOSItem,
  removePOSItem,
  updatePOSItem,
  addKitchenNote,
  updateKitchenNote,
  removeKitchenNote,
  addIngredientReplacement,
  addIngredientAddon,
  removeIngredientModification,
  getIngredientDetailsById,
  getIngredientNameById,
  onCancel
}) {
  // Sorted drink names for better UX
  const sortedDrinkNames = React.useMemo(() => {
    return Array.isArray(uniqueDrinkNames)
      ? [...uniqueDrinkNames].sort((a, b) => String(a).localeCompare(String(b)))
      : [];
  }, [uniqueDrinkNames]);

  // Duplicate an item at index
  const duplicateItem = (itemIndex) => {
    setPosOrderData(prev => {
      const item = prev.items[itemIndex];
      const cloned = {
        ...item,
        isExpanded: true
      };
      const items = [...prev.items.slice(0, itemIndex + 1), cloned, ...prev.items.slice(itemIndex + 1)];
      return { items };
    });
  };

  // Reset an item to defaults
  const resetItem = (itemIndex) => {
    setPosOrderData(prev => ({
      items: prev.items.map((itm, i) => i === itemIndex ? {
        item_id: '',
        quantity: 1,
        kitchen_notes: [],
        item_ingredients: [],
        selectedDrinkName: '',
        selectedSize: '',
        isExpanded: true,
        isCustomizeOpen: false
      } : itm)
    }));
  };

  // Set preparation preferences (Temperature / Ice / Foam)
  const setKitchenPreference = (itemIndex, type, value) => {
    if (value === undefined || value === null) return;
    setPosOrderData(prev => ({
      items: prev.items.map((itm, i) => i === itemIndex ? {
        ...itm,
        kitchen_notes: [
          ...itm.kitchen_notes.filter(n => n.type !== type),
          { type, qty: 0, detail: value }
        ]
      } : itm)
    }));
  };

  return (
    /* New Order Form */
    <div className="space-y-6">
      {/* Custom Order Form */}
      <div>
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-semibold text-gray-900">New POS Order</h3>
          <div className="flex items-center space-x-2">
            <span className="text-sm text-gray-600">
              {posOrderData.items.length} item{posOrderData.items.length !== 1 ? 's' : ''}
            </span>
          </div>
        </div>
        
        {/* POS Order Form - Create new order*/}
        <form onSubmit={handleSubmitPOSOrder} className="space-y-4">
          {/* Add/Remove Items and Submit */}
          <div className="flex justify-between items-center sticky top-0 z-10 bg-gradient-to-b from-white/70 to-transparent backdrop-blur-sm">
            <button
              type="button"
              onClick={addPOSItem}
              className="px-4 py-2 barns-dark-bg text-white rounded-lg cursor-pointer text-sm"
              style={{outline:'none', color:'white'}}
            >
              Add Item
            </button>
            
            <div className="flex items-center gap-2">
              <button
                type="button"
                disabled={isLoading}
                onClick={() => {
                  onCancel?.();
                }}
                className="px-4 py-2 bg-red-600 text-white rounded-lg hover:bg-red-700 disabled:opacity-50 disabled:cursor-not-allowed text-sm"
              >
                Cancel
              </button>
              <button
                type="submit"
                disabled={isLoading || posOrderData.items.some(item => !item.item_id || item.item_id.trim() === '')}
                className="px-6 py-2 bg-green-600 text-white rounded-lg hover:bg-green-700 disabled:opacity-50 disabled:cursor-not-allowed"
              >
                {isLoading ? 'Processing...' : `Process Order (${posOrderData.items.length} item${posOrderData.items.length !== 1 ? 's' : ''})`}
              </button>
            </div>
          </div>
          
          {/* Multiple Items */}
          {posOrderData.items.map((item, itemIndex) => (
            <div key={itemIndex} className="border border-gray-200 rounded-lg p-3 bg-gray-50 space-y-3">
              <div className="flex items-center justify-between">
                <button
                  type="button"
                  className="flex items-center gap-2 text-left"
                  onClick={() => setPosOrderData(prev => ({
                    items: prev.items.map((itm, i) => i === itemIndex ? { ...itm, isExpanded: !itm.isExpanded } : itm)
                  }))}
                >
                  <h4 className="text-sm font-medium text-gray-900">Item {itemIndex + 1}</h4>
                  <span className="text-gray-500 text-xs">{item.selectedDrinkName && item.selectedSize ? `${item.selectedDrinkName} • ${item.selectedSize}` : ''}</span>
                </button>
                {posOrderData.items.length > 1 && (
                  <button
                    type="button"
                    onClick={() => removePOSItem(itemIndex)}
                    className="text-red-500 hover:text-red-700 text-sm"
                  >
                    Remove
                  </button>
                )}
              </div>
              
              {/* Menu Item Selection (two-step) and Quantity */}
              <div className={`grid ${item.isExpanded ? 'grid-cols-3' : 'grid-cols-3'} gap-3`}>
                {/* Drink */}
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-1">Drink</label>
                  <select
                    value={item.selectedDrinkName}
                    onChange={(e) => updatePOSItem(itemIndex, 'selectedDrinkName', e.target.value)}
                    className="w-full px-3 py-2 border border-gray-300 rounded-lg text-sm"
                  >
                    <option value="">Select a drink...</option>
                    {sortedDrinkNames.length === 0 ? (
                      <option value="">Loading drinks...</option>
                    ) : (
                      sortedDrinkNames.map(name => (
                        <option key={name} value={name}>{name}</option>
                      ))
                    )}
                  </select>
                </div>
                
                {/* Size */}
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-1">Size</label>
                  <select
                    value={item.selectedSize}
                    onChange={(e) => updatePOSItem(itemIndex, 'selectedSize', e.target.value)}
                    className={`w-full px-3 py-2 border rounded-lg text-sm ${
                      !item.item_id || item.item_id.trim() === '' 
                        ? 'border-red-300 bg-red-50' 
                        : 'border-gray-300'
                    }`}
                    disabled={!item.selectedDrinkName}
                  >
                    {!item.selectedDrinkName ? (
                      <option value="">Select drink first</option>
                    ) : (
                      <>
                        <option value="">Select a size...</option>
                        {menuItems
                          .filter(m => m.name === item.selectedDrinkName)
                          .map(mi => mi.size)
                          .filter((v, i, a) => a.indexOf(v) === i)
                          .map(size => (
                            <option key={size} value={size}>{size}</option>
                          ))}
                      </>
                    )}
                  </select>
                  {(!item.item_id || item.item_id.trim() === '') && (
                    <p className="text-xs text-red-600 mt-1">Select size to continue</p>
                  )}
                </div>
                
                {/* Quantity */}
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-1">Quantity</label>
                  <input
                    type="number"
                    min="1"
                    value={item.quantity}
                    onChange={(e) => updatePOSItem(itemIndex, 'quantity', parseInt(e.target.value) || 1)}
                    className="w-full px-3 py-2 border border-gray-300 rounded-lg text-sm"
                  />
                </div>
              </div>
              
              
              {/* Manual Kitchen Notes (qty > 0) */}
              <div className="border-t pt-3">
                <div className="flex items-center justify-between mb-2">
                  <label className="block text-sm font-medium text-gray-700">Manual Notes (for barista)</label>
                  <button
                    type="button"
                    onClick={() => addKitchenNote(itemIndex)}
                    className="text-xs text-blue-600 hover:text-blue-800"
                  >
                    + Add Manual Note
                  </button>
                </div>
                {item.kitchen_notes.filter(n => n.qty > 0).map((note, noteIndex) => {
                  const actualIndex = item.kitchen_notes.findIndex(n => n === note);
                  return (
                    <div key={noteIndex} className="grid grid-cols-3 gap-2 mb-2">
                      <input
                        type="text"
                        value={note.type}
                        onChange={(e) => updateKitchenNote(itemIndex, actualIndex, 'type', e.target.value)}
                        className="px-2 py-1 border border-gray-300 rounded text-xs"
                        placeholder="Type (e.g., Sugar)"
                      />
                      <input
                        type="number"
                        min="1"
                        value={note.qty}
                        onChange={(e) => updateKitchenNote(itemIndex, actualIndex, 'qty', parseInt(e.target.value) || 1)}
                        className="px-2 py-1 border border-gray-300 rounded text-xs"
                        placeholder="Qty"
                      />
                      <div className="flex gap-1">
                        <input
                          type="text"
                          value={note.detail}
                          onChange={(e) => updateKitchenNote(itemIndex, actualIndex, 'detail', e.target.value)}
                          className="flex-1 px-2 py-1 border border-gray-300 rounded text-xs"
                          placeholder="Detail (e.g., Brown)"
                        />
                        <button
                          type="button"
                          onClick={() => removeKitchenNote(itemIndex, actualIndex)}
                          className="text-red-500 hover:text-red-700 text-xs px-2"
                        >
                          ×
                        </button>
                      </div>
                    </div>
                  );
                })}
              </div>
              
              {/* Item Ingredients - Organized by Category (collapsible) */}
              {item.isExpanded && item.selectedMenuItem && Object.keys(ingredientsByCategory).length > 0 && (
                <div className="border-t pt-3">
                <div className="flex items-center justify-between">
                    <h5 className="text-sm font-medium text-gray-700">Customize Ingredients</h5>
                  <div className="flex items-center gap-2">
                    <button
                      type="button"
                      className="text-xs text-blue-600 hover:text-blue-800"
                      onClick={() => setPosOrderData(prev => ({
                        items: prev.items.map((itm, i) => i === itemIndex ? { ...itm, isCustomizeOpen: !itm.isCustomizeOpen } : itm)
                      }))}
                    >
                      {item.isCustomizeOpen ? 'Hide' : 'Show'}
                    </button>
                    <button
                      type="button"
                      onClick={() => duplicateItem(itemIndex)}
                      className="text-xs text-gray-600 hover:text-gray-800"
                      title="Duplicate item"
                    >
                      Duplicate
                    </button>
                    <button
                      type="button"
                      onClick={() => resetItem(itemIndex)}
                      className="text-xs text-gray-600 hover:text-gray-800"
                      title="Reset item"
                    >
                      Reset
                    </button>
                    {posOrderData.items.length > 1 && (
                      <button
                        type="button"
                        onClick={() => removePOSItem(itemIndex)}
                        className="text-xs text-red-600 hover:text-red-700"
                        title="Remove item"
                      >
                        Remove
                      </button>
                    )}
                  </div>
                  </div>
                  {item.isCustomizeOpen && (
                    <div className="space-y-3 mt-2">
                      {/* Show ingredient categories with dropdowns */}
                      {Object.entries(ingredientsByCategory).map(([category, ingredients]) => (
                        // Skip cups category from modification in removal/replacement, but still show current cup
                        category === 'cups' ? (
                          <div key={category} className="bg-gray-50 p-3 rounded">
                            <label className="block text-xs font-medium text-gray-700 mb-1 capitalize">cups</label>
                            {/* Show current cup (from defaults) as read-only */}
                            <div className="text-xs bg-white px-2 py-1 rounded">
                              {(item.selectedMenuItem.default_ingredients || []).filter(di => di.category === 'cups').map((di, idx) => (
                                <div key={`cup-${idx}`}>{di.type} (fixed)</div>
                              ))}
                            </div>
                          </div>
                        ) : (
                          <div key={category} className="bg-gray-50 p-3 rounded">
                            <label className="block text-xs font-medium text-gray-700 mb-1 capitalize">
                              {category.replace('_', ' ')}
                            </label>
                            <div className="flex gap-2">
                              <select
                                className="flex-1 px-2 py-1 border border-gray-300 rounded text-xs"
                                onChange={(e) => {
                                  if (e.target.value) {
                                    const defaultIng = item.selectedMenuItem.default_ingredients.find(
                                      di => di.category === category
                                    );
                                    if (defaultIng) {
                                      // Replacement: persist category as modifierGroupId
                                      addIngredientReplacement(
                                        itemIndex,
                                        defaultIng.ingredient_id,
                                        e.target.value,
                                        1,
                                        category
                                      );
                                    } else {
                                      // Add-on
                                      addIngredientAddon(itemIndex, e.target.value, 1);
                                    }
                                    e.target.value = '';
                                  }
                                }}
                              >
                                <option value="">Add {category}...</option>
                                {ingredients.map(ing => (
                                  <option key={ing.ingredient_id} value={ing.ingredient_id}>
                                    {ing.name} ({ing.type})
                                  </option>
                                ))}
                              </select>
                            </div>
                            
                            {/* Current items in this category: defaults (with ability to hide via zero-qty replacement) + add-ons */}
                            <div className="mt-2 space-y-1">
                              {/* Defaults for this category */}
                              {(item.selectedMenuItem.default_ingredients || []).filter(di => di.category === category).map((di, dIdx) => {
                                const replacementIndex = (item.item_ingredients || []).findIndex(m => !m.isAddon && m.isModified && String(m.initialItemId) === String(di.ingredient_id));
                                const replacement = replacementIndex >= 0 ? item.item_ingredients[replacementIndex] : null;
                                return (
                                  <div key={`def-${category}-${dIdx}`} className="flex items-center justify-between bg-white px-2 py-1 rounded text-xs">
                                    <span className="truncate">
                                      {di.type} <span className="text-gray-500">(default {di.quantity} x {di.unit_amount})</span>
                                      {replacement && (
                                        <span className="ml-2 text-amber-700">→ {getIngredientNameById(replacement.itemId)} x{replacement.qty}</span>
                                      )}
                                    </span>
                                    <div className="flex items-center gap-1">
                                      {replacement && (
                                        <button
                                          type="button"
                                          onClick={() => removeIngredientModification(itemIndex, replacementIndex)}
                                          className="text-red-500 hover:text-red-700"
                                          title="Remove replacement"
                                        >
                                          ×
                                        </button>
                                      )}
                                      {/* Hide default by adding zero-qty replacement (interpreted as removed) */}
                                      {!replacement && (
                                        <button
                                          type="button"
                                          className="text-gray-500 hover:text-gray-700"
                                          title="Exclude default"
                                          onClick={() => addIngredientReplacement(itemIndex, di.ingredient_id, di.ingredient_id, 0, category)}
                                        >
                                          ×
                                        </button>
                                      )}
                                    </div>
                                  </div>
                                );
                              })}
                              
                              {/* Add-ons for this category */}
                              {(item.item_ingredients || []).map((mod, modIdx) => {
                                if (!mod.isAddon) return null;
                                const details = getIngredientDetailsById(mod.itemId);
                                if (!details || details.category !== category) return null;
                                return (
                                  <div key={`addon-${category}-${modIdx}`} className="flex items-center justify-between bg-white px-2 py-1 rounded text-xs">
                                    <span className="truncate">{details.name || getIngredientNameById(mod.itemId)} x{mod.qty} <span className="text-gray-500">(add-on)</span></span>
                                    <button
                                      type="button"
                                      onClick={() => removeIngredientModification(itemIndex, modIdx)}
                                      className="text-red-500 hover:text-red-700"
                                      title="Remove add-on"
                                    >
                                      ×
                                    </button>
                                  </div>
                                );
                              })}
                            </div>
                          </div>
                        )
                      ))}
                    </div>
                  )}
                  {/* Global modifications list removed to avoid duplication; shown per category above */}
                </div>
              )}
            </div>
          ))}
        </form>
        
      </div>
    </div>
  );
}

export default NewPOSOrderForm;


