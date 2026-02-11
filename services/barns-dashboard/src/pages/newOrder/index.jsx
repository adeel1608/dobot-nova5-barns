/**
 * New Order Page
 * Dedicated page for creating POS orders with improved UX
 */

import React, { useState, useEffect } from 'react';
import useStore from '../../store';
import { useTranslation } from '../../store/translationsStore';
import MenuGrid from './components/MenuGrid';
import OrderCart from './components/OrderCart';
import Swal from 'sweetalert2';
import './styles.css';

export default function NewOrderPage() {
  const { t } = useTranslation('newOrder');
  const {
    menuItems = [],
    ingredientsByCategory = {},
    fetchMenuItems,
    fetchIngredientsByCategory,
    processPOSOrder,
    navigate
  } = useStore() || {};

  // Order state
  const [cartItems, setCartItems] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [dataLoading, setDataLoading] = useState(true);
  const [searchTerm, setSearchTerm] = useState('');
  const [selectedCategory, setSelectedCategory] = useState('all');

  // Get unique drink names - MUST be before any conditional returns
  const uniqueDrinkNames = React.useMemo(() => {
    if (!menuItems || !Array.isArray(menuItems)) return [];
    const names = [...new Set(menuItems.map(item => item.name))];
    return names.sort((a, b) => String(a).localeCompare(String(b)));
  }, [menuItems]);

  // Get categories from menu items - MUST be before any conditional returns
  const categories = React.useMemo(() => {
    if (!menuItems || !Array.isArray(menuItems)) return ['all'];
    const cats = [...new Set(menuItems.map(item => item.category || 'Other'))];
    return ['all', ...cats.filter(c => c).sort()];
  }, [menuItems]);

  // Filter menu items by category and search - MUST be before any conditional returns
  const filteredDrinkNames = React.useMemo(() => {
    if (!menuItems || !Array.isArray(menuItems)) return [];
    let drinks = uniqueDrinkNames;
    
    if (selectedCategory !== 'all') {
      drinks = drinks.filter(name => {
        const menuItem = menuItems.find(m => m.name === name);
        return menuItem && menuItem.category === selectedCategory;
      });
    }
    
    if (searchTerm) {
      drinks = drinks.filter(name => 
        name.toLowerCase().includes(searchTerm.toLowerCase())
      );
    }
    
    return drinks;
  }, [uniqueDrinkNames, selectedCategory, searchTerm, menuItems]);

  // Load menu data on mount
  useEffect(() => {
    const loadMenuData = async () => {
      setDataLoading(true);
      try {
        await Promise.all([
          fetchMenuItems?.(),
          fetchIngredientsByCategory?.()
        ]);
      } catch (error) {
        console.error('Failed to load menu data:', error);
      } finally {
        setDataLoading(false);
      }
    };
    loadMenuData();
  }, [fetchMenuItems, fetchIngredientsByCategory]);

  // Show loading state while data is being fetched
  if (dataLoading) {
    return (
      <div className="new-order-page">
        <div className="flex items-center justify-center h-full">
          <div className="text-center">
            <svg className="animate-spin h-12 w-12 text-green-600 mx-auto mb-4" fill="none" viewBox="0 0 24 24">
              <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
              <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
            </svg>
            <p className="text-gray-600">Loading menu...</p>
          </div>
        </div>
      </div>
    );
  }

  // Add item to cart
  const addToCart = (drinkName, size) => {
    if (!menuItems || !Array.isArray(menuItems)) return;
    const menuItem = menuItems.find(m => m.name === drinkName && m.size === size);
    if (!menuItem) return;

    const newItem = {
      id: Date.now() + Math.random(), // Unique ID for cart item
      item_id: menuItem.item_id,
      quantity: 1,
      kitchen_notes: [],
      item_ingredients: [],
      selectedDrinkName: drinkName,
      selectedSize: size,
      selectedMenuItem: menuItem,
      isCustomizeOpen: false
    };

    setCartItems(prev => [...prev, newItem]);
  };

  // Remove item from cart
  const removeFromCart = (itemId) => {
    setCartItems(prev => prev.filter(item => item.id !== itemId));
  };

  // Update item quantity
  const updateQuantity = (itemId, newQuantity) => {
    if (newQuantity < 1) {
      removeFromCart(itemId);
      return;
    }
    setCartItems(prev => prev.map(item => 
      item.id === itemId ? { ...item, quantity: newQuantity } : item
    ));
  };

  // Update cart item
  const updateCartItem = (itemId, updates) => {
    setCartItems(prev => prev.map(item => 
      item.id === itemId ? { ...item, ...updates } : item
    ));
  };

  // Kitchen notes management
  const addKitchenNote = (itemId) => {
    setCartItems(prev => prev.map(item => 
      item.id === itemId ? {
        ...item,
        kitchen_notes: [...item.kitchen_notes, { type: '', qty: 1, detail: '' }]
      } : item
    ));
  };

  const updateKitchenNote = (itemId, noteIndex, field, value) => {
    setCartItems(prev => prev.map(item => 
      item.id === itemId ? {
        ...item,
        kitchen_notes: item.kitchen_notes.map((note, idx) => 
          idx === noteIndex ? { ...note, [field]: value } : note
        )
      } : item
    ));
  };

  const removeKitchenNote = (itemId, noteIndex) => {
    setCartItems(prev => prev.map(item => 
      item.id === itemId ? {
        ...item,
        kitchen_notes: item.kitchen_notes.filter((_, idx) => idx !== noteIndex)
      } : item
    ));
  };

  // Ingredient modifications
  const addIngredientReplacement = (itemId, originalIngredientId, newIngredientId, quantity, modifierGroupId) => {
    setCartItems(prev => prev.map(item => 
      item.id === itemId ? {
        ...item,
        item_ingredients: [...item.item_ingredients, {
          itemId: newIngredientId,
          qty: quantity || 1,
          isAddon: false,
          modifierGroupId: modifierGroupId || 'replacement',
          isModified: true,
          initialItemId: originalIngredientId
        }]
      } : item
    ));
  };

  const addIngredientAddon = (itemId, ingredientId, quantity) => {
    setCartItems(prev => prev.map(item => 
      item.id === itemId ? {
        ...item,
        item_ingredients: [...item.item_ingredients, {
          itemId: ingredientId,
          qty: quantity || 1,
          isAddon: true,
          modifierGroupId: 'AddOns'
        }]
      } : item
    ));
  };

  const removeIngredientModification = (itemId, ingIndex) => {
    setCartItems(prev => prev.map(item => 
      item.id === itemId ? {
        ...item,
        item_ingredients: item.item_ingredients.filter((_, idx) => idx !== ingIndex)
      } : item
    ));
  };

  // Helper functions
  const getIngredientNameById = (id) => {
    if (!ingredientsByCategory || typeof ingredientsByCategory !== 'object') {
      return String(id);
    }
    for (const arr of Object.values(ingredientsByCategory)) {
      const found = arr.find(x => String(x.ingredient_id) === String(id));
      if (found) return found.name || found.type || String(id);
    }
    return String(id);
  };

  const getIngredientDetailsById = (id) => {
    if (!ingredientsByCategory || typeof ingredientsByCategory !== 'object') {
      return null;
    }
    for (const [category, arr] of Object.entries(ingredientsByCategory)) {
      const found = arr.find(x => String(x.ingredient_id) === String(id));
      if (found) return { ...found, category };
    }
    return null;
  };

  // Submit order
  const handleSubmitOrder = async () => {
    if (cartItems.length === 0) {
      Swal.fire({
        title: t('cartEmpty'),
        text: t('pleaseAddItems'),
        icon: 'warning',
        timer: 3000,
        timerProgressBar: true,
        showConfirmButton: false
      });
      return;
    }

    setIsLoading(true);

    try {
      // Generate transaction_id
      const transactionId = `TXN${Date.now()}`;
      const now = new Date();
      const date = now.toISOString().split('T')[0];
      const time = now.toTimeString().split(' ')[0];

      // Build POS order format
      const posOrder = {
        transaction_id: transactionId,
        date: date,
        time: time,
        store_number: 1,
        pos_reg_id: 1,
        customer_id: null,
        items: cartItems.map(item => ({
          item_id: item.item_id,
          quantity: item.quantity,
          kitchen_notes: item.kitchen_notes,
          item_ingredients: item.item_ingredients
        }))
      };

      const success = await processPOSOrder(posOrder);
      
      if (success) {
        setCartItems([]);
        
        await Swal.fire({
          title: 'Success!',
          text: 'Order processed!',
          icon: 'success',
          timer: 1000,
          timerProgressBar: true,
          showConfirmButton: false
        });

        // Navigate back to dashboard after success alert closes
        navigate?.('dashboard');
        window.location.hash = '#/dashboard';
      } else {
        Swal.fire({
          title: t('failed'),
          text: t('failedToProcess'),
          icon: 'error',
          timer: 3000,
          timerProgressBar: true,
          showConfirmButton: false
        });
      }
    } catch (error) {
      console.error('Error processing order:', error);
      Swal.fire({
        title: t('failed'),
        text: t('unexpectedError'),
        icon: 'error',
        timer: 3000,
        timerProgressBar: true,
        showConfirmButton: false
      });
    } finally {
      setIsLoading(false);
    }
  };

  // Cancel and go back
  const handleCancel = () => {
    if (cartItems.length > 0) {
      Swal.fire({
        title: t('discardOrder'),
        text: t('cartHasItems'),
        icon: 'warning',
        showCancelButton: true,
        confirmButtonText: t('yesDiscard'),
        cancelButtonText: t('noKeepEditing')
      }).then((result) => {
        if (result.isConfirmed) {
          setCartItems([]);
          navigate?.('dashboard');
          window.location.hash = '#/dashboard';
        }
      });
    } else {
      navigate?.('dashboard');
      window.location.hash = '#/dashboard';
    }
  };

  return (
    <div className="new-order-page">


      {/* Split View Layout */}
      <div className="new-order-content">
        {/* Left Panel - Menu Grid */}
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

        {/* Right Panel - Order Cart */}
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

