/**
 * ItemCustomization Component
 * Ingredient modifications and special instructions - Improved UX
 * With cup capacity management
 */

import React, { useState, useEffect } from 'react';
import { TEMPERATURE_OPTIONS, DEFAULT_TEMPERATURE, INGREDIENT_DENSITIES, MAX_MILK_SUBSTITUTION_PERCENT, CUP_VOLUMES, ESPRESSO_SHOT_WEIGHTS, isIcedCup } from '../../../constants/cupCapacityConfig';
import { 
  calculateAdjustedMilk, 
  getCupSizeCode, 
  calculateRecipeVolume,
  getIngredientWeight 
} from '../utils/cupCapacityCalculator';

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
  // Categories to hide completely (including temperature, ice, milk, water - handled separately)
  const hiddenCategories = ['premixes', 'sachets', 'cups', 'position', 'temperature', 'ice', 'water'];
  
  // Categories that can only have one selection (replaceable)
  const replaceableCategories = ['espresso', 'milk'];
  
  // Categories that can have multiple with quantity (syrups, sauces, etc)
  const additiveCategories = ['syrups', 'sauce', 'toppings', 'extras'];
  
  // Temperature state (managed separately from replaceable categories)
  const [selectedTemperature, setSelectedTemperature] = useState(DEFAULT_TEMPERATURE);
  
  // Ice amount state (in grams)
  const [iceAmount, setIceAmount] = useState(() => {
    const iceIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'ice'
    );
    if (!iceIngredient) return 0;
    const calculatedAmount = (iceIngredient.unit_amount || 0) * (iceIngredient.quantity || 1);
    return Math.min(calculatedAmount, 300);
  });
  
  // Milk amount state (in grams)
  const [milkAmount, setMilkAmount] = useState(() => {
    const milkIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'milk'
    );
    if (!milkIngredient) return 0;
    const calculatedAmount = (milkIngredient.unit_amount || 0) * (milkIngredient.quantity || 1);
    return Math.min(calculatedAmount, 500);
  });
  
  // Water amount state (in grams)
  const [waterAmount, setWaterAmount] = useState(() => {
    const waterIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'water'
    );
    if (!waterIngredient) return 0;
    const calculatedAmount = (waterIngredient.unit_amount || 0) * (waterIngredient.quantity || 1);
    return Math.min(calculatedAmount, 500);
  });
  
  // Update ice amount when item changes
  useEffect(() => {
    const iceIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'ice'
    );
    if (!iceIngredient) {
      setIceAmount(0);
      return;
    }
    const calculatedAmount = (iceIngredient.unit_amount || 0) * (iceIngredient.quantity || 1);
    setIceAmount(Math.min(calculatedAmount, 300));
  }, [item.selectedMenuItem]);
  
  // Update milk amount when item changes
  useEffect(() => {
    const milkIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'milk'
    );
    if (!milkIngredient) {
      setMilkAmount(0);
      return;
    }
    const calculatedAmount = (milkIngredient.unit_amount || 0) * (milkIngredient.quantity || 1);
    setMilkAmount(Math.min(calculatedAmount, 500));
  }, [item.selectedMenuItem]);
  
  // Update water amount when item changes
  useEffect(() => {
    const waterIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'water'
    );
    if (!waterIngredient) {
      setWaterAmount(0);
      return;
    }
    const calculatedAmount = (waterIngredient.unit_amount || 0) * (waterIngredient.quantity || 1);
    setWaterAmount(Math.min(calculatedAmount, 500));
  }, [item.selectedMenuItem]);
  
  // Initialize default temperature in kitchen_notes when item is first added
  // Using a ref to track if we've already initialized to prevent infinite loops
  const temperatureInitialized = React.useRef(false);
  
  useEffect(() => {
    if (!item.selectedMenuItem) return;
    if (temperatureInitialized.current) return;
    
    // Check if temperature note already exists
    const existingTempNote = (item.kitchen_notes || []).find(
      note => note.type?.toLowerCase().includes('temperature')
    );
    
    // If no temperature note exists, add the default
    if (!existingTempNote) {
      temperatureInitialized.current = true;
      const tempNote = {
        type: 'Temperature',
        detail: DEFAULT_TEMPERATURE,  // 'standard' - 10% foam
        qty: 0,  // 0 indicates automation modifier
      };
      updateCartItem(itemId, {
        kitchen_notes: [...(item.kitchen_notes || []), tempNote]
      });
    } else {
      temperatureInitialized.current = true;
    }
  }, [item.selectedMenuItem, item.kitchen_notes, itemId, updateCartItem]);
  
  // Initialize ice amount in item_ingredients when item is first added
  const iceInitialized = React.useRef(false);
  
  useEffect(() => {
    if (!item.selectedMenuItem) return;
    if (iceInitialized.current) return;
    
    const iceIngredient = item.selectedMenuItem.default_ingredients?.find(
      ing => ing.category === 'ice'
    );
    
    if (!iceIngredient) {
      iceInitialized.current = true;
      return;
    }
    
    // Check if ice modification already exists in item_ingredients
    const existingIceMod = (item.item_ingredients || []).find(
      mod => mod.isIceModification || mod.category === 'ice'
    );
    
    // If no ice modification exists, add the default
    if (!existingIceMod) {
      iceInitialized.current = true;
      const defaultIceAmount = Math.min(
        (iceIngredient.unit_amount || 0) * (iceIngredient.quantity || 1), 
        300
      );
      
      const iceModification = {
        itemId: iceIngredient.ingredient_id,
        initialItemId: iceIngredient.ingredient_id,
        category: 'ice',
        isModified: true,
        isAddon: false,
        isIceModification: true,
        iceAmountGrams: defaultIceAmount,
        qty: defaultIceAmount,
      };
      
      updateCartItem(itemId, {
        item_ingredients: [...(item.item_ingredients || []), iceModification]
      });
    } else {
      iceInitialized.current = true;
    }
  }, [item.selectedMenuItem, item.item_ingredients, itemId, updateCartItem]);
  
  // Initialize milk amount in item_ingredients when item is first added
  const milkInitialized = React.useRef(false);
  
  useEffect(() => {
    if (!item.selectedMenuItem) return;
    if (milkInitialized.current) return;
    
    const milkIngredient = item.selectedMenuItem.default_ingredients?.find(
      ing => ing.category === 'milk'
    );
    
    if (!milkIngredient) {
      milkInitialized.current = true;
      return;
    }
    
    const existingMilkMod = (item.item_ingredients || []).find(
      mod => mod.isMilkAmountModification || (mod.category === 'milk' && mod.isAmountModification)
    );
    
    if (!existingMilkMod) {
      milkInitialized.current = true;
      const defaultMilkAmount = Math.min(
        (milkIngredient.unit_amount || 0) * (milkIngredient.quantity || 1),
        500
      );
      
      const milkModification = {
        itemId: milkIngredient.ingredient_id,
        initialItemId: milkIngredient.ingredient_id,
        category: 'milk',
        isModified: true,
        isAddon: false,
        isMilkAmountModification: true,
        isAmountModification: true,
        amountGrams: defaultMilkAmount,
        qty: defaultMilkAmount,
      };
      
      updateCartItem(itemId, {
        item_ingredients: [...(item.item_ingredients || []), milkModification]
      });
    } else {
      milkInitialized.current = true;
    }
  }, [item.selectedMenuItem, item.item_ingredients, itemId, updateCartItem]);
  
  // Initialize water amount in item_ingredients when item is first added
  const waterInitialized = React.useRef(false);
  
  useEffect(() => {
    if (!item.selectedMenuItem) return;
    if (waterInitialized.current) return;
    
    const waterIngredient = item.selectedMenuItem.default_ingredients?.find(
      ing => ing.category === 'water'
    );
    
    if (!waterIngredient) {
      waterInitialized.current = true;
      return;
    }
    
    const existingWaterMod = (item.item_ingredients || []).find(
      mod => mod.isWaterAmountModification || (mod.category === 'water' && mod.isAmountModification)
    );
    
    if (!existingWaterMod) {
      waterInitialized.current = true;
      const defaultWaterAmount = Math.min(
        (waterIngredient.unit_amount || 0) * (waterIngredient.quantity || 1),
        500
      );
      
      const waterModification = {
        itemId: waterIngredient.ingredient_id,
        initialItemId: waterIngredient.ingredient_id,
        category: 'water',
        isModified: true,
        isAddon: false,
        isWaterAmountModification: true,
        isAmountModification: true,
        amountGrams: defaultWaterAmount,
        qty: defaultWaterAmount,
      };
      
      updateCartItem(itemId, {
        item_ingredients: [...(item.item_ingredients || []), waterModification]
      });
    } else {
      waterInitialized.current = true;
    }
  }, [item.selectedMenuItem, item.item_ingredients, itemId, updateCartItem]);
  
  // Capacity calculation state
  const [capacityInfo, setCapacityInfo] = useState(null);
  
  // Legend collapse state - start collapsed
  const [isLegendCollapsed, setIsLegendCollapsed] = useState(true);
  
  // Track current espresso type for real-time updates
  const getCurrentEspressoType = () => {
    const defaultEspresso = item.selectedMenuItem?.default_ingredients?.find(ing => ing.category === 'espresso');
    if (!defaultEspresso) return 'double_shot';
    
    // Check if user has replaced the espresso
    const espressoReplacement = (item.item_ingredients || []).find(
      mod => !mod.isAddon && mod.isModified && 
      String(mod.initialItemId) === String(defaultEspresso.ingredient_id)
    );
    
    if (espressoReplacement) {
      const replacementDetails = getIngredientDetailsById(espressoReplacement.itemId);
      // Check type, name, or use the itemId itself to determine shot type
      const espressoType = replacementDetails?.type || replacementDetails?.name || espressoReplacement.itemId || '';
      
      // Normalize the type string
      const normalizedType = espressoType.toString().toLowerCase().replace(/\s+/g, '_');
      
      // Check if it matches known shot types
      if (normalizedType.includes('single')) return 'single_shot';
      if (normalizedType.includes('double')) return 'double_shot';
      if (normalizedType.includes('triple') || normalizedType.includes('tripple')) return 'tripple_shot';
      
      return normalizedType || defaultEspresso.type || 'double_shot';
    }
    
    return defaultEspresso.type || 'double_shot';
  };
  
  const currentEspressoType = getCurrentEspressoType();
  
  // Calculate capacity whenever temperature, addons, ice amount, or ingredient changes
  useEffect(() => {
    if (!item.selectedMenuItem) return;
    
    // Get base milk amount from default ingredients
    const milkIngredient = item.selectedMenuItem.default_ingredients?.find(
      ing => ing.category === 'milk'
    );
    
    // Calculate milk volume - convert from grams to ml if needed
    let baseMilkAmount = 0;
    if (milkIngredient) {
      const milkWeight = (milkIngredient.unit_amount || 0) * (milkIngredient.quantity || 1);
      const milkBaseUnits = (milkIngredient.base_units || '').toLowerCase();
      
      if (milkBaseUnits === 'ml') {
        baseMilkAmount = milkWeight;
      } else {
        // Convert grams to ml using milk density
        baseMilkAmount = milkWeight / INGREDIENT_DENSITIES.milk;
      }
    }
    
    // Get cup size
    const cupIngredient = item.selectedMenuItem.default_ingredients?.find(
      ing => ing.category === 'cups'
    );
    const cupSize = cupIngredient?.type || 'H9';
    
    // Calculate total recipe volume (excluding non-volume categories)
    // Categories to skip: 
    // - milk: handled separately in capacity calculation
    // - espresso: handled separately (added based on shot type)
    // - ice: handled separately (added from iceAmount state)
    // - cups: not a consumable volume
    // - position: not a consumable volume
    // - temperature: not a consumable volume (it's a setting, not an ingredient)
    const skipCategories = ['milk', 'espresso', 'ice', 'cups', 'position', 'temperature'];
    
    let recipeVolume = 0;
    for (const ing of (item.selectedMenuItem.default_ingredients || [])) {
      const amount = ing.unit_amount || 0;
      const quantity = ing.quantity || 1;
      const baseUnits = (ing.base_units || '').toLowerCase();
      const category = (ing.category || '').toLowerCase();
      
      // Skip non-volume categories
      if (skipCategories.includes(category)) {
        continue;
      }
      
      if (baseUnits === 'ml') {
        // Already in milliliters
        recipeVolume += amount * quantity;
      } else {
        // Default: treat as grams and convert to volume using density
        // This handles 'grams', 'g', 'pumps', undefined, etc.
        const density = INGREDIENT_DENSITIES[category] || 1.0;
        recipeVolume += (amount * quantity) / density;
      }
    }
    
    // Calculate espresso volume only if drink has espresso
    const hasEspresso = item.selectedMenuItem.default_ingredients?.some(
      ing => ing.category === 'espresso'
    );
    
    let espressoVolume = 0;
    if (hasEspresso) {
      const espressoTypeKey = (currentEspressoType || 'double_shot').toLowerCase().replace(/\s+/g, '_');
      const shotWeight = ESPRESSO_SHOT_WEIGHTS[espressoTypeKey] || ESPRESSO_SHOT_WEIGHTS.double_shot;
      espressoVolume = shotWeight / INGREDIENT_DENSITIES.espresso;
      recipeVolume += espressoVolume;
    }
    
    // Add ice volume to recipe (convert grams to ml using density)
    if (iceAmount > 0) {
      const iceVolume = iceAmount / INGREDIENT_DENSITIES.ice;
      recipeVolume += iceVolume;
    }
    
    // Get addons with their categories (exclude ice - handled separately above)
    const addons = (item.item_ingredients || [])
      .filter(mod => mod.isAddon)
      .map(mod => {
        const details = getIngredientDetailsById(mod.itemId);
        return {
          category: details?.category || 'extras',
          quantity: mod.qty || 1,
          weight: getIngredientWeight(details?.category || 'extras', mod.qty || 1),
        };
      })
      .filter(addon => addon.category !== 'ice'); // Exclude ice from addon calculations
    
    // Calculate adjusted milk or just capacity for non-milk drinks
    let result;
    const cupVolume = CUP_VOLUMES[cupSize] || 266;
    
    if (baseMilkAmount > 0) {
      result = calculateAdjustedMilk(
        cupSize,
        baseMilkAmount,
        selectedTemperature,
        addons,
        recipeVolume
      );
      
      // Calculate actual free space for milk drinks:
      // cupVolume - adjustedMilk - foam - espresso - ice - recipeIngredients - addons
      // Note: recipeVolume already includes espresso and ice from calculations above
      const actualUsedVolume = result.adjustedMilkAmount + result.foamReductionAmount + recipeVolume + (result.totalAddonVolume || 0);
      result.actualFreeSpace = cupVolume - actualUsedVolume;
    } else {
      // For non-milk drinks, calculate capacity including addons
      
      // Calculate total addon volume
      let totalAddonVolume = 0;
      const addonDetails = [];
      
      for (const addon of addons) {
        const density = INGREDIENT_DENSITIES[addon.category] || 1.0;
        const volume = addon.weight / density;
        totalAddonVolume += volume;
        addonDetails.push({
          category: addon.category,
          quantity: addon.quantity,
          weight: addon.weight,
          volume: volume,
        });
      }
      
      // Free space = cup capacity - default recipe - addons
      const freeSpace = cupVolume - recipeVolume - totalAddonVolume;
      
      result = {
        baseMilkAmount: 0,
        foamReducedMilk: 0,
        foamReductionAmount: 0,
        foamReductionPercent: 0,
        freeSpaceRemaining: freeSpace,
        actualFreeSpace: freeSpace,  // For non-milk drinks, these are the same
        milkSubstitutionAmount: 0,
        milkSubstitutionPercent: 0,
        adjustedMilkAmount: 0,
        canAddMore: freeSpace > 0,
        remainingSubstitutionCapacity: 0,
        remainingSubstitutionPercent: 0,
        addonDetails: addonDetails,
        totalAddonVolume: totalAddonVolume,
        temperature: selectedTemperature,
      };
    }
    
    setCapacityInfo(result);
    
    // Update cart item with capacity exceeded status
    // Use actualFreeSpace for the visual/physical capacity check
    const isCapacityExceeded = result.milkSubstitutionPercent > MAX_MILK_SUBSTITUTION_PERCENT || result.actualFreeSpace < 0;
    if (item.capacityExceeded !== isCapacityExceeded) {
      updateCartItem(itemId, { capacityExceeded: isCapacityExceeded });
    }
  }, [item, selectedTemperature, iceAmount, milkAmount, waterAmount, currentEspressoType, getIngredientDetailsById, itemId, updateCartItem]);
  
  // Handle temperature selection with capacity validation
  const handleTemperatureChange = async (newTemperature) => {
    // Don't validate if no milk ingredient
    const milkIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'milk'
    );
    
    if (!milkIngredient) {
      setSelectedTemperature(newTemperature);
      return;
    }
    
    // Check if there are any add-ons
    const hasAddons = (item.item_ingredients || []).some(mod => mod.isAddon);
    
    if (hasAddons) {
      // Calculate what the capacity would be with the new temperature
      const milkWeight = (milkIngredient.unit_amount || 0) * (milkIngredient.quantity || 1);
      const milkBaseUnits = (milkIngredient.base_units || '').toLowerCase();
      const baseMilkAmount = milkBaseUnits === 'ml' ? milkWeight : milkWeight / INGREDIENT_DENSITIES.milk;
      
      const cupIngredient = item.selectedMenuItem.default_ingredients?.find(
        ing => ing.category === 'cups'
      );
      const cupSize = cupIngredient?.type || 'H9';
      const recipeVolume = calculateRecipeVolume(item.selectedMenuItem.default_ingredients || []);
      
      const addons = (item.item_ingredients || [])
        .filter(mod => mod.isAddon)
        .map(mod => {
          const details = getIngredientDetailsById(mod.itemId);
          return {
            category: details?.category || 'extras',
            quantity: mod.qty || 1,
            weight: getIngredientWeight(details?.category || 'extras', mod.qty || 1),
          };
        })
        .filter(addon => addon.category !== 'ice'); // Exclude ice from capacity calculations
      
      // Simulate what would happen with new temperature
      const simulatedResult = calculateAdjustedMilk(
        cupSize,
        baseMilkAmount,
        newTemperature,
        addons,
        recipeVolume
      );
      
      // Check if new temperature would exceed 30% limit
      if (simulatedResult.milkSubstitutionPercent > 30) {
        const currentTempOption = TEMPERATURE_OPTIONS.find(t => t.id === selectedTemperature);
        const newTempOption = TEMPERATURE_OPTIONS.find(t => t.id === newTemperature);
        
        const willExceed = confirm(
          `Changing temperature from ${currentTempOption?.label} to ${newTempOption?.label} ` +
          `will exceed the capacity limit (${simulatedResult.milkSubstitutionPercent.toFixed(1)}% of 30%).\n\n` +
          `You have ${addons.length} add-on(s) that need ${simulatedResult.milkSubstitutionAmount.toFixed(1)}ml of space.\n\n` +
          `Options:\n` +
          `• Click OK to keep current temperature (${currentTempOption?.label})\n` +
          `• Click Cancel to change temperature and remove some add-ons`
        );
        
        if (willExceed) {
          // User chose to keep current temperature
          return;
        } else {
          // User chose to remove add-ons - calculate how many to remove
          const overage = simulatedResult.milkSubstitutionAmount - (simulatedResult.foamReducedMilk * 0.30);
          
          alert(
            `To change to ${newTempOption?.label}, you need to free up ${overage.toFixed(1)}ml.\n\n` +
            `Please remove some add-ons manually before changing temperature.`
          );
          return;
        }
      }
    }
    
    // Temperature change is safe, proceed
    setSelectedTemperature(newTemperature);
    
    // Add temperature as a kitchen note (qty: 0 for automation modifier)
    const existingTempIndex = item.kitchen_notes.findIndex(
      note => note.type.toLowerCase().includes('temperature')
    );
    
    if (existingTempIndex >= 0) {
      updateKitchenNote(itemId, existingTempIndex, 'detail', newTemperature);
    } else {
      // Add new temperature note
      const tempNote = {
        type: 'Temperature',
        detail: newTemperature,
        qty: 0, // 0 indicates automation modifier
      };
      updateCartItem(itemId, {
        kitchen_notes: [...(item.kitchen_notes || []), tempNote]
      });
    }
  };

  // Handle ice amount adjustment
  const handleIceAdjustment = (change) => {
    // Get cup volume for capacity check
    const cupIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'cups'
    );
    const cupSize = cupIngredient?.type || 'H9';
    const cupVolume = CUP_VOLUMES[cupSize] || 266;
    
    // Calculate new ice amount with constraints
    let newAmount = iceAmount + change;
    
    // Don't go below 0
    if (newAmount < 0) newAmount = 0;
    
    // Cap at maximum 300g
    if (newAmount > 300) {
      newAmount = 300;
    }
    
    // If increasing, check if capacity would be exceeded
    if (change > 0) {
      // Calculate current non-ice recipe volume
      let baseRecipeVolume = calculateRecipeVolume(item.selectedMenuItem.default_ingredients || []);
      
      // Add new ice volume
      const newIceVolume = newAmount / INGREDIENT_DENSITIES.ice;
      const totalVolume = baseRecipeVolume + newIceVolume;
      
      // Check if this would exceed cup capacity
      if (totalVolume > cupVolume) {
        // Don't increase if already at or over capacity
        if (capacityInfo && capacityInfo.freeSpaceRemaining <= 0) {
          alert('Cannot add more ice: Cup capacity exceeded!');
          return;
        }
      }
    }
    
    setIceAmount(newAmount);
    
    // Get ice ingredient details
    const iceIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'ice'
    );
    
    if (!iceIngredient) return;
    
    // Update ice as an ingredient modification in item_ingredients (not kitchen_notes)
    // This ensures the actual gram amount is sent in the order
    const existingIceModIndex = (item.item_ingredients || []).findIndex(
      mod => mod.category === 'ice' && mod.isIceModification
    );
    
    if (existingIceModIndex >= 0) {
      // Update existing ice modification
      const updatedIngredients = [...(item.item_ingredients || [])];
      updatedIngredients[existingIceModIndex] = {
        ...updatedIngredients[existingIceModIndex],
        iceAmountGrams: newAmount,
        qty: newAmount,  // qty in grams for ice
      };
      updateCartItem(itemId, { item_ingredients: updatedIngredients });
    } else {
      // Add new ice modification
      const iceModification = {
        itemId: iceIngredient.ingredient_id,
        initialItemId: iceIngredient.ingredient_id,
        category: 'ice',
        isModified: true,
        isAddon: false,
        isIceModification: true,  // Flag to identify ice modifications
        iceAmountGrams: newAmount,
        qty: newAmount,  // qty in grams for ice
      };
      updateCartItem(itemId, {
        item_ingredients: [...(item.item_ingredients || []), iceModification]
      });
    }
  };

  // Handle milk amount adjustment
  const handleMilkAdjustment = (change) => {
    const cupIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'cups'
    );
    const cupSize = cupIngredient?.type || 'H9';
    const cupVolume = CUP_VOLUMES[cupSize] || 266;
    
    let newAmount = milkAmount + change;
    if (newAmount < 0) newAmount = 0;
    if (newAmount > 500) newAmount = 500;
    
    if (change > 0) {
      let baseRecipeVolume = calculateRecipeVolume(item.selectedMenuItem.default_ingredients || []);
      const newMilkVolume = newAmount / INGREDIENT_DENSITIES.milk;
      const totalVolume = baseRecipeVolume + newMilkVolume;
      
      if (totalVolume > cupVolume) {
        if (capacityInfo && capacityInfo.freeSpaceRemaining <= 0) {
          alert('Cannot add more milk: Cup capacity exceeded!');
          return;
        }
      }
    }
    
    setMilkAmount(newAmount);
    
    const milkIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'milk'
    );
    if (!milkIngredient) return;
    
    const existingMilkModIndex = (item.item_ingredients || []).findIndex(
      mod => mod.category === 'milk' && mod.isMilkAmountModification
    );
    
    if (existingMilkModIndex >= 0) {
      const updatedIngredients = [...(item.item_ingredients || [])];
      updatedIngredients[existingMilkModIndex] = {
        ...updatedIngredients[existingMilkModIndex],
        amountGrams: newAmount,
        qty: newAmount,
      };
      updateCartItem(itemId, { item_ingredients: updatedIngredients });
    } else {
      const milkModification = {
        itemId: milkIngredient.ingredient_id,
        initialItemId: milkIngredient.ingredient_id,
        category: 'milk',
        isModified: true,
        isAddon: false,
        isMilkAmountModification: true,
        isAmountModification: true,
        amountGrams: newAmount,
        qty: newAmount,
      };
      updateCartItem(itemId, {
        item_ingredients: [...(item.item_ingredients || []), milkModification]
      });
    }
  };

  // Handle water amount adjustment
  const handleWaterAdjustment = (change) => {
    const cupIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'cups'
    );
    const cupSize = cupIngredient?.type || 'H9';
    const cupVolume = CUP_VOLUMES[cupSize] || 266;
    
    let newAmount = waterAmount + change;
    if (newAmount < 0) newAmount = 0;
    if (newAmount > 500) newAmount = 500;
    
    if (change > 0) {
      let baseRecipeVolume = calculateRecipeVolume(item.selectedMenuItem.default_ingredients || []);
      const newWaterVolume = newAmount / INGREDIENT_DENSITIES.water;
      const totalVolume = baseRecipeVolume + newWaterVolume;
      
      if (totalVolume > cupVolume) {
        if (capacityInfo && capacityInfo.freeSpaceRemaining <= 0) {
          alert('Cannot add more water: Cup capacity exceeded!');
          return;
        }
      }
    }
    
    setWaterAmount(newAmount);
    
    const waterIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'water'
    );
    if (!waterIngredient) return;
    
    const existingWaterModIndex = (item.item_ingredients || []).findIndex(
      mod => mod.category === 'water' && mod.isWaterAmountModification
    );
    
    if (existingWaterModIndex >= 0) {
      const updatedIngredients = [...(item.item_ingredients || [])];
      updatedIngredients[existingWaterModIndex] = {
        ...updatedIngredients[existingWaterModIndex],
        amountGrams: newAmount,
        qty: newAmount,
      };
      updateCartItem(itemId, { item_ingredients: updatedIngredients });
    } else {
      const waterModification = {
        itemId: waterIngredient.ingredient_id,
        initialItemId: waterIngredient.ingredient_id,
        category: 'water',
        isModified: true,
        isAddon: false,
        isWaterAmountModification: true,
        isAmountModification: true,
        amountGrams: newAmount,
        qty: newAmount,
      };
      updateCartItem(itemId, {
        item_ingredients: [...(item.item_ingredients || []), waterModification]
      });
    }
  };

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

  // Update addon quantity with capacity check
  const updateAddonQuantity = (modIndex, newQty) => {
    if (newQty < 1) {
      removeIngredientModification(itemId, modIndex);
    } else {
      // Check if adding this would exceed capacity
      const updatedIngredients = [...(item.item_ingredients || [])];
      const currentQty = updatedIngredients[modIndex].qty || 1;
      const qtyDiff = newQty - currentQty;
      
      // If increasing quantity, check capacity
      if (qtyDiff > 0 && capacityInfo && !capacityInfo.canAddMore) {
        alert('Cannot add more: Cup capacity limit reached (30% milk substitution maximum)');
        return;
      }
      
      // Update the quantity
      updatedIngredients[modIndex] = { ...updatedIngredients[modIndex], qty: newQty };
      updateCartItem(itemId, { item_ingredients: updatedIngredients });
    }
  };
  
  // Check if we can add more addons
  const canAddMoreAddons = () => {
    if (!capacityInfo) return true;
    return capacityInfo.canAddMore;
  };
  
  // Check if changing to a specific temperature would exceed capacity
  const wouldTemperatureExceedCapacity = (newTemperature) => {
    if (!capacityInfo || newTemperature === selectedTemperature) return false;
    
    const milkIngredient = item.selectedMenuItem?.default_ingredients?.find(
      ing => ing.category === 'milk'
    );
    if (!milkIngredient) return false;
    
    const hasAddons = (item.item_ingredients || []).some(mod => mod.isAddon);
    if (!hasAddons) return false;
    
    const milkWeight = (milkIngredient.unit_amount || 0) * (milkIngredient.quantity || 1);
    const milkBaseUnits = (milkIngredient.base_units || '').toLowerCase();
    const baseMilkAmount = milkBaseUnits === 'ml' ? milkWeight : milkWeight / INGREDIENT_DENSITIES.milk;
    
    const cupIngredient = item.selectedMenuItem.default_ingredients?.find(
      ing => ing.category === 'cups'
    );
    const cupSize = cupIngredient?.type || 'H9';
    const recipeVolume = calculateRecipeVolume(item.selectedMenuItem.default_ingredients || []);
    
    const addons = (item.item_ingredients || [])
      .filter(mod => mod.isAddon)
      .map(mod => {
        const details = getIngredientDetailsById(mod.itemId);
        return {
          category: details?.category || 'extras',
          quantity: mod.qty || 1,
          weight: getIngredientWeight(details?.category || 'extras', mod.qty || 1),
        };
      })
      .filter(addon => addon.category !== 'ice'); // Exclude ice from capacity calculations
    
    const simulatedResult = calculateAdjustedMilk(
      cupSize,
      baseMilkAmount,
      newTemperature,
      addons,
      recipeVolume
    );
    
    return simulatedResult.milkSubstitutionPercent > 30;
  };

  return (
    <div className="item-customization">
      {/* Capacity Indicator at Top - Visual Cup Fill */}
      {capacityInfo && (
        <div className="capacity-indicator-floating">
          <div className="capacity-header">
            <span className="capacity-label">Cup Capacity</span>
            <button 
              className="legend-toggle"
              onClick={() => setIsLegendCollapsed(!isLegendCollapsed)}
              title={isLegendCollapsed ? 'Show legend' : 'Hide legend'}
            >
              {isLegendCollapsed ? '▼' : '▲'}
            </button>
          </div>
          
          {/* Stacked Bar Chart */}
          <div className="capacity-bar-container">
            {/* Calculate percentages for each component */}
            {(() => {
              const cupIngredient = item.selectedMenuItem.default_ingredients?.find(ing => ing.category === 'cups');
              const cupSize = cupIngredient?.type || 'H9';
              const cupVolume = CUP_VOLUMES[cupSize] || 266;
              
              // Check if this is an iced drink (cold cup = no foam)
              const isIced = isIcedCup(cupSize);
              
              // Calculate espresso volume using current selection (from component state)
              let espressoVolume = 0;
              const defaultEspresso = item.selectedMenuItem.default_ingredients?.find(ing => ing.category === 'espresso');
              if (defaultEspresso) {
                const espressoTypeKey = (currentEspressoType || 'double_shot').toLowerCase().replace(/\s+/g, '_');
                const shotWeight = ESPRESSO_SHOT_WEIGHTS[espressoTypeKey] || ESPRESSO_SHOT_WEIGHTS.double_shot;
                espressoVolume = shotWeight / INGREDIENT_DENSITIES.espresso;
              }
              
              // Get default recipe ingredients (sauces, syrups, water, etc.)
              const defaultIngredientsByCategory = {};
              item.selectedMenuItem.default_ingredients?.forEach(ing => {
                const category = (ing.category || '').toLowerCase();
                // Skip non-volume ingredients: milk (handled separately), espresso (handled separately),
                // cups, position, ice (shown separately), temperature (not a volume)
                if (['milk', 'espresso', 'cups', 'position', 'ice', 'temperature'].includes(category)) return;
                
                const qty = ing.quantity || 1;
                const amount = ing.unit_amount || 0;
                const baseUnits = (ing.base_units || '').toLowerCase();
                
                let volume = 0;
                if (baseUnits === 'ml') {
                  // Already in milliliters
                  volume = qty * amount;
                } else {
                  // Default: treat as grams and convert to volume using density
                  // This handles 'grams', 'g', 'pumps', undefined, etc.
                  const density = INGREDIENT_DENSITIES[category] || 1.0;
                  volume = (qty * amount) / density;
                }
                
                if (volume > 0) {
                  if (!defaultIngredientsByCategory[category]) {
                    defaultIngredientsByCategory[category] = 0;
                  }
                  defaultIngredientsByCategory[category] += volume;
                }
              });
              
              // Calculate addon volumes by category (user-added)
              const addonsByCategory = {};
              capacityInfo.addonDetails?.forEach(addon => {
                const category = addon.category;
                if (!addonsByCategory[category]) {
                  addonsByCategory[category] = 0;
                }
                addonsByCategory[category] += addon.volume;
              });
              
              const milkVolume = capacityInfo.adjustedMilkAmount || 0;
              const foamVolume = capacityInfo.foamReductionAmount || 0;
              
              const milkPercent = (milkVolume / cupVolume) * 100;
              const foamPercent = (foamVolume / cupVolume) * 100;
              const espressoPercent = (espressoVolume / cupVolume) * 100;
              
              // Calculate ice volume (convert grams to ml)
              const iceVolume = iceAmount > 0 ? iceAmount / INGREDIENT_DENSITIES.ice : 0;
              const icePercent = (iceVolume / cupVolume) * 100;
              
              // Combine default ingredients and add-ons by category
              const combinedIngredients = {};
              
              // Add default ingredients
              Object.entries(defaultIngredientsByCategory).forEach(([category, volume]) => {
                combinedIngredients[category] = {
                  default: volume,
                  addon: 0,
                  total: volume
                };
              });
              
              // Add user add-ons
              Object.entries(addonsByCategory).forEach(([category, volume]) => {
                if (combinedIngredients[category]) {
                  combinedIngredients[category].addon = volume;
                  combinedIngredients[category].total += volume;
                } else {
                  combinedIngredients[category] = {
                    default: 0,
                    addon: volume,
                    total: volume
                  };
                }
              });
              
              // Convert to array with percentages
              const ingredientPercentages = Object.entries(combinedIngredients).map(([category, volumes]) => ({
                category,
                percent: (volumes.total / cupVolume) * 100,
                defaultVolume: volumes.default,
                addonVolume: volumes.addon,
                totalVolume: volumes.total
              }));
              
              // Calculate total volume of other ingredients (syrups, sauces, water, etc.)
              const otherIngredientsVolume = ingredientPercentages.reduce((sum, ing) => sum + ing.totalVolume, 0);
              
              // Calculate ACTUAL free space: cup volume minus all ingredients
              // This is the true remaining space in the cup
              const totalUsedVolume = milkVolume + foamVolume + espressoVolume + iceVolume + otherIngredientsVolume;
              const actualFreeSpace = Math.max(0, cupVolume - totalUsedVolume);
              const freeSpacePercent = (actualFreeSpace / cupVolume) * 100;
              
              return (
                <>
                  <div className="capacity-bar-stacked">
                    {/* Milk */}
                    {milkPercent > 0 && (
                      <div 
                        className="capacity-segment milk"
                        style={{ width: `${milkPercent}%` }}
                        title={`Milk: ${capacityInfo.adjustedMilkAmount.toFixed(0)}ml`}
                      />
                    )}
                    
                    {/* Foam */}
                    {foamPercent > 0 && (
                      <div 
                        className="capacity-segment foam"
                        style={{ width: `${foamPercent}%` }}
                        title={`Foam: ${capacityInfo.foamReductionAmount.toFixed(0)}ml`}
                      />
                    )}
                    
                    {/* Espresso */}
                    {espressoPercent > 0 && (
                      <div 
                        className="capacity-segment espresso"
                        style={{ width: `${espressoPercent}%` }}
                        title={`Espresso: ${espressoVolume.toFixed(0)}ml`}
                      />
                    )}
                    
                    {/* Ice */}
                    {icePercent > 0 && (
                      <div 
                        className="capacity-segment ice"
                        style={{ width: `${icePercent}%` }}
                        title={`Ice: ${iceAmount}g (~${iceVolume.toFixed(0)}ml)`}
                      />
                    )}
                    
                    {/* Default ingredients and Add-ons by category */}
                    {ingredientPercentages.map((ingredient, idx) => (
                      <div
                        key={`${ingredient.category}-${idx}`}
                        className={`capacity-segment addon-${ingredient.category}`}
                        style={{ width: `${ingredient.percent}%` }}
                        title={`${ingredient.category}: ${ingredient.totalVolume.toFixed(0)}ml${ingredient.defaultVolume > 0 ? ` (${ingredient.defaultVolume.toFixed(0)}ml default` : ''}${ingredient.addonVolume > 0 ? ` + ${ingredient.addonVolume.toFixed(0)}ml added)` : ')'}`}
                      />
                    ))}
                    
                    {/* Free Space */}
                    {freeSpacePercent > 0 && (
                      <div 
                        className="capacity-segment free-space"
                        style={{ width: `${freeSpacePercent}%` }}
                        title={`Free space: ${actualFreeSpace.toFixed(0)}ml`}
                      />
                    )}
                  </div>
                  
                  {/* Collapsible Legend */}
                  {!isLegendCollapsed && (
                    <div className="capacity-legend">
                      <div className="legend-item">
                        <span className="legend-color milk"></span>
                        <span className="legend-label">Milk: {capacityInfo.adjustedMilkAmount.toFixed(0)}ml</span>
                      </div>
                      {foamPercent > 0 && (
                        <div className="legend-item">
                          <span className="legend-color foam"></span>
                          <span className="legend-label">Foam: {capacityInfo.foamReductionAmount.toFixed(0)}ml</span>
                        </div>
                      )}
                      {espressoPercent > 0 && (
                        <div className="legend-item">
                          <span className="legend-color espresso"></span>
                          <span className="legend-label">Espresso: {espressoVolume.toFixed(0)}ml</span>
                        </div>
                      )}
                      {icePercent > 0 && (
                        <div className="legend-item">
                          <span className="legend-color ice"></span>
                          <span className="legend-label">Ice: {iceAmount}g (~{iceVolume.toFixed(0)}ml)</span>
                        </div>
                      )}
                      {ingredientPercentages.map((ingredient, idx) => (
                        <div key={`legend-${ingredient.category}-${idx}`} className="legend-item">
                          <span className={`legend-color addon-${ingredient.category}`}></span>
                          <span className="legend-label">
                            {ingredient.category}: {ingredient.totalVolume.toFixed(0)}ml
                            {ingredient.defaultVolume > 0 && ingredient.addonVolume > 0 && 
                              ` (${ingredient.defaultVolume.toFixed(0)}+${ingredient.addonVolume.toFixed(0)})`
                            }
                          </span>
                        </div>
                      ))}
                      {actualFreeSpace > 0 && (
                        <div className="legend-item">
                          <span className="legend-color free-space"></span>
                          <span className="legend-label">Free: {actualFreeSpace.toFixed(0)}ml</span>
                        </div>
                      )}
                    </div>
                  )}
                  
                  {/* Warning if at capacity or exceeds */}
                  {(capacityInfo.milkSubstitutionPercent > MAX_MILK_SUBSTITUTION_PERCENT || actualFreeSpace < 0) && (
                    <div className="capacity-warning capacity-critical">
                      {actualFreeSpace < 0 ? (
                        <>
                          Critical: Default recipe exceeds cup capacity by {Math.abs(actualFreeSpace).toFixed(0)}ml! 
                          Select a larger cup size or reduce ingredients.
                        </>
                      ) : (
                        <>
                          Critical: Recipe exceeds cup capacity by {(capacityInfo.milkSubstitutionPercent - MAX_MILK_SUBSTITUTION_PERCENT).toFixed(1)}%! 
                          Remove add-ons or select a larger cup size.
                        </>
                      )}
                    </div>
                  )}
                  {!capacityInfo.canAddMore && capacityInfo.milkSubstitutionPercent <= MAX_MILK_SUBSTITUTION_PERCENT && actualFreeSpace >= 0 && (
                    <div className="capacity-warning">
                      Maximum capacity reached (30% limit)
                    </div>
                  )}
                </>
              );
            })()}
          </div>
        </div>
      )}
      
      {/* Temperature Selection (Compact) - Only for hot drinks with milk */}
      {item.selectedMenuItem && 
       item.selectedMenuItem.default_ingredients?.some(ing => ing.category === 'milk') && 
       !isIcedCup(item.selectedMenuItem.default_ingredients?.find(ing => ing.category === 'cups')?.type) && (
        <div className="ingredient-category replaceable">
          <label className="category-label">Temperature</label>
          <div className="replaceable-options">
            {TEMPERATURE_OPTIONS.map(tempOption => {
              const wouldExceed = wouldTemperatureExceedCapacity(tempOption.id);
              return (
                <button
                  key={tempOption.id}
                  onClick={() => handleTemperatureChange(tempOption.id)}
                  className={`option-btn ${selectedTemperature === tempOption.id ? 'active' : ''} ${wouldExceed ? 'would-exceed' : ''}`}
                  title={`${tempOption.label} (${tempOption.description})${tempOption.foamPercent > 0 ? ` - ${tempOption.foamPercent}% foam` : ''}${wouldExceed ? ' - Would exceed capacity' : ''}`}
                >
                  {tempOption.label}
                  {wouldExceed && <span className="temp-warning-icon-compact"> ⚠</span>}
                </button>
              );
            })}
          </div>
        </div>
      )}
      
      {/* Ice Adjustment (Grams with +/- controls) */}
      {item.selectedMenuItem && item.selectedMenuItem.default_ingredients?.some(ing => ing.category === 'ice') && (
        <div className="ingredient-category ice-adjustment">
          <label className="category-label">Ice</label>
          <div className="ice-control">
            <button
              onClick={() => handleIceAdjustment(-20)}
              className="ice-btn minus"
              disabled={iceAmount === 0}
              title="Reduce ice by 20g"
            >
              −
            </button>
            <div className="ice-display">
              <span className="ice-amount">{iceAmount}g</span>
              <span className="ice-level">
                {(() => {
                  if (iceAmount === 0) return 'No Ice';
                  
                  const iceIngredient = item.selectedMenuItem.default_ingredients.find(ing => ing.category === 'ice');
                  const defaultAmount = iceIngredient ? Math.min((iceIngredient.unit_amount || 0) * (iceIngredient.quantity || 1), 300) : 100;
                  
                  if (iceAmount < defaultAmount * 0.7) return 'Light';
                  if (iceAmount > defaultAmount * 1.3) return 'Extra';
                  return 'Normal';
                })()}
              </span>
            </div>
            <button
              onClick={() => handleIceAdjustment(10)}
              className="ice-btn plus"
              disabled={iceAmount >= 300 || (capacityInfo && capacityInfo.freeSpaceRemaining <= 0)}
              title={iceAmount >= 300 ? 'Maximum ice reached (300g)' : capacityInfo?.freeSpaceRemaining <= 0 ? 'Cup capacity exceeded' : 'Add ice by 10g'}
            >
              +
            </button>
          </div>
          {capacityInfo && capacityInfo.freeSpaceRemaining < 0 && (
            <div className="ice-warning">
              Reduce ice to fit cup capacity
            </div>
          )}
        </div>
      )}
      
      {/* Milk Amount Adjustment */}
      {item.selectedMenuItem && item.selectedMenuItem.default_ingredients?.some(ing => ing.category === 'milk') && (
        <div className="ingredient-category amount-adjustment milk-adjustment">
          <label className="category-label">Milk</label>
          <div className="amount-control">
            <button
              onClick={() => handleMilkAdjustment(-10)}
              className="amount-btn minus"
              disabled={milkAmount === 0}
              title="Reduce milk by 10g"
            >
              −
            </button>
            <div className="amount-display">
              <span className="amount-value">{milkAmount}g</span>
              <span className="amount-level">
                {(() => {
                  if (milkAmount === 0) return 'No Milk';
                  const milkIngredient = item.selectedMenuItem.default_ingredients.find(ing => ing.category === 'milk');
                  const defaultAmount = milkIngredient ? Math.min((milkIngredient.unit_amount || 0) * (milkIngredient.quantity || 1), 500) : 100;
                  if (milkAmount < defaultAmount * 0.7) return 'Light';
                  if (milkAmount > defaultAmount * 1.3) return 'Extra';
                  return 'Normal';
                })()}
              </span>
            </div>
            <button
              onClick={() => handleMilkAdjustment(10)}
              className="amount-btn plus"
              disabled={milkAmount >= 500 || (capacityInfo && capacityInfo.freeSpaceRemaining <= 0)}
              title={milkAmount >= 500 ? 'Maximum milk reached (500g)' : capacityInfo?.freeSpaceRemaining <= 0 ? 'Cup capacity exceeded' : 'Add milk by 10g'}
            >
              +
            </button>
          </div>
          {capacityInfo && capacityInfo.freeSpaceRemaining < 0 && (
            <div className="amount-warning">
              Reduce milk to fit cup capacity
            </div>
          )}
        </div>
      )}
      
      {/* Water Amount Adjustment */}
      {item.selectedMenuItem && item.selectedMenuItem.default_ingredients?.some(ing => ing.category === 'water') && (
        <div className="ingredient-category amount-adjustment water-adjustment">
          <label className="category-label">Water</label>
          <div className="amount-control">
            <button
              onClick={() => handleWaterAdjustment(-10)}
              className="amount-btn minus"
              disabled={waterAmount === 0}
              title="Reduce water by 10g"
            >
              −
            </button>
            <div className="amount-display">
              <span className="amount-value">{waterAmount}g</span>
              <span className="amount-level">
                {(() => {
                  if (waterAmount === 0) return 'No Water';
                  const waterIngredient = item.selectedMenuItem.default_ingredients.find(ing => ing.category === 'water');
                  const defaultAmount = waterIngredient ? Math.min((waterIngredient.unit_amount || 0) * (waterIngredient.quantity || 1), 500) : 100;
                  if (waterAmount < defaultAmount * 0.7) return 'Light';
                  if (waterAmount > defaultAmount * 1.3) return 'Extra';
                  return 'Normal';
                })()}
              </span>
            </div>
            <button
              onClick={() => handleWaterAdjustment(10)}
              className="amount-btn plus"
              disabled={waterAmount >= 500 || (capacityInfo && capacityInfo.freeSpaceRemaining <= 0)}
              title={waterAmount >= 500 ? 'Maximum water reached (500g)' : capacityInfo?.freeSpaceRemaining <= 0 ? 'Cup capacity exceeded' : 'Add water by 10g'}
            >
              +
            </button>
          </div>
          {capacityInfo && capacityInfo.freeSpaceRemaining < 0 && (
            <div className="amount-warning">
              Reduce water to fit cup capacity
            </div>
          )}
        </div>
      )}
      
      {/* Replaceable Ingredients (Espresso, Milk) */}
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
            
            // Only show category if there are ingredients available to add
            if (!ingredients || ingredients.length === 0) return null;

            const addons = getAddonsForCategory(category);
            const defaults = (item.selectedMenuItem?.default_ingredients || []).filter(
              di => di.category === category
            );
            
            // Only show addon category if drink has default ingredients in that category
            // This hides water, syrup, sauce options if the drink doesn't have them by default
            if (defaults.length === 0 && addons.length === 0) return null;

            return (
              <div key={category} className="ingredient-category additive">
                <label className="category-label">{category.replace('_', ' ')}</label>
                
                {/* Dropdown to add new */}
                <select
                  className="ingredient-select-compact"
                  disabled={!canAddMoreAddons()}
                  onChange={(e) => {
                    if (e.target.value) {
                      // Check capacity before adding
                      if (!canAddMoreAddons()) {
                        alert('Cannot add more: Cup capacity limit reached (30% milk substitution maximum)');
                        e.target.value = '';
                        return;
                      }
                      
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
                  <option value="">
                    {canAddMoreAddons() ? `+ Add ${category}...` : 'Capacity limit reached'}
                  </option>
                  {canAddMoreAddons() && ingredients.map(ing => (
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
                              disabled={!canAddMoreAddons()}
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
