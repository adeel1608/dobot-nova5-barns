/**
 * useNewOrderState
 * Shared state and action logic for both the default and compact New Order layouts.
 * Keeps all cart management, data loading, and submission in one place.
 */

import React, { useState, useEffect } from 'react';
import useStore from '../../store';
import { useTranslation } from '../../store/translationsStore';
import Swal from 'sweetalert2';

export function useNewOrderState() {
  const { t } = useTranslation('newOrder');
  const {
    menuItems = [],
    ingredientsByCategory = {},
    fetchMenuItems,
    fetchIngredientsByCategory,
    processPOSOrder,
    navigate
  } = useStore() || {};

  const [cartItems, setCartItems] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [dataLoading, setDataLoading] = useState(true);
  const [searchTerm, setSearchTerm] = useState('');
  const [selectedCategory, setSelectedCategory] = useState('all');

  const uniqueDrinkNames = React.useMemo(() => {
    if (!menuItems || !Array.isArray(menuItems)) return [];
    const names = [...new Set(menuItems.map(item => item.name))];
    return names.sort((a, b) => String(a).localeCompare(String(b)));
  }, [menuItems]);

  const categories = React.useMemo(() => {
    if (!menuItems || !Array.isArray(menuItems)) return ['all'];
    const cats = [...new Set(menuItems.map(item => item.category || 'Other'))];
    return ['all', ...cats.filter(c => c).sort()];
  }, [menuItems]);

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

  const addToCart = (drinkName, size) => {
    if (!menuItems || !Array.isArray(menuItems)) return;
    const menuItem = menuItems.find(m => m.name === drinkName && m.size === size);
    if (!menuItem) return;
    setCartItems(prev => [...prev, {
      id: Date.now() + Math.random(),
      item_id: menuItem.item_id,
      quantity: 1,
      kitchen_notes: [],
      item_ingredients: [],
      selectedDrinkName: drinkName,
      selectedSize: size,
      selectedMenuItem: menuItem,
      isCustomizeOpen: false
    }]);
  };

  const removeFromCart = (itemId) => {
    setCartItems(prev => prev.filter(item => item.id !== itemId));
  };

  const updateQuantity = (itemId, newQuantity) => {
    if (newQuantity < 1) { removeFromCart(itemId); return; }
    setCartItems(prev => prev.map(item =>
      item.id === itemId ? { ...item, quantity: newQuantity } : item
    ));
  };

  const updateCartItem = (itemId, updates) => {
    setCartItems(prev => prev.map(item =>
      item.id === itemId ? { ...item, ...updates } : item
    ));
  };

  const addKitchenNote = (itemId) => {
    setCartItems(prev => prev.map(item =>
      item.id === itemId
        ? { ...item, kitchen_notes: [...item.kitchen_notes, { type: '', qty: 1, detail: '' }] }
        : item
    ));
  };

  const updateKitchenNote = (itemId, noteIndex, field, value) => {
    setCartItems(prev => prev.map(item =>
      item.id === itemId
        ? {
            ...item,
            kitchen_notes: item.kitchen_notes.map((note, idx) =>
              idx === noteIndex ? { ...note, [field]: value } : note
            )
          }
        : item
    ));
  };

  const removeKitchenNote = (itemId, noteIndex) => {
    setCartItems(prev => prev.map(item =>
      item.id === itemId
        ? { ...item, kitchen_notes: item.kitchen_notes.filter((_, idx) => idx !== noteIndex) }
        : item
    ));
  };

  const addIngredientReplacement = (itemId, originalIngredientId, newIngredientId, quantity, modifierGroupId) => {
    setCartItems(prev => prev.map(item =>
      item.id === itemId
        ? {
            ...item,
            item_ingredients: [...item.item_ingredients, {
              itemId: newIngredientId,
              qty: quantity || 1,
              isAddon: false,
              modifierGroupId: modifierGroupId || 'replacement',
              isModified: true,
              initialItemId: originalIngredientId
            }]
          }
        : item
    ));
  };

  const addIngredientAddon = (itemId, ingredientId, quantity) => {
    setCartItems(prev => prev.map(item =>
      item.id === itemId
        ? {
            ...item,
            item_ingredients: [...item.item_ingredients, {
              itemId: ingredientId,
              qty: quantity || 1,
              isAddon: true,
              modifierGroupId: 'AddOns'
            }]
          }
        : item
    ));
  };

  const removeIngredientModification = (itemId, ingIndex) => {
    setCartItems(prev => prev.map(item =>
      item.id === itemId
        ? { ...item, item_ingredients: item.item_ingredients.filter((_, idx) => idx !== ingIndex) }
        : item
    ));
  };

  const getIngredientNameById = (id) => {
    if (!ingredientsByCategory || typeof ingredientsByCategory !== 'object') return String(id);
    for (const arr of Object.values(ingredientsByCategory)) {
      const found = arr.find(x => String(x.ingredient_id) === String(id));
      if (found) return found.name || found.type || String(id);
    }
    return String(id);
  };

  const getIngredientDetailsById = (id) => {
    if (!ingredientsByCategory || typeof ingredientsByCategory !== 'object') return null;
    for (const [category, arr] of Object.entries(ingredientsByCategory)) {
      const found = arr.find(x => String(x.ingredient_id) === String(id));
      if (found) return { ...found, category };
    }
    return null;
  };

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
      const transactionId = `TXN${Date.now()}`;
      const now = new Date();
      const posOrder = {
        transaction_id: transactionId,
        date: now.toISOString().split('T')[0],
        time: now.toTimeString().split(' ')[0],
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

  return {
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
    uniqueDrinkNames,
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
  };
}
