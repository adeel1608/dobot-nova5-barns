/**
 * Cup Capacity Calculator Utility
 * Implements three-phase capacity management:
 * Phase 1: Temperature/foam reduction
 * Phase 2: Fill free space in cup
 * Phase 3: Milk substitution (max 30%)
 */

import {
  CUP_VOLUMES,
  INGREDIENT_DENSITIES,
  TEMPERATURE_FOAM_PERCENTAGES,
  MAX_MILK_SUBSTITUTION_PERCENT,
  DEFAULT_SYRUP_WEIGHT_PER_PUMP,
  DEFAULT_SAUCE_WEIGHT_PER_PUMP,
  ESPRESSO_SHOT_WEIGHTS,
} from '../../../constants/cupCapacityConfig';

/**
 * Phase 1: Apply foam reduction based on temperature
 * @param {number} baseMilkAmount - Original milk amount in ml
 * @param {string} temperature - Temperature setting (kids, standard, extra_hot)
 * @returns {number} Milk amount after foam reduction
 */
export function applyFoamReduction(baseMilkAmount, temperature = 'standard') {
  const foamPercent = TEMPERATURE_FOAM_PERCENTAGES[temperature] || TEMPERATURE_FOAM_PERCENTAGES.standard;
  const reduction = baseMilkAmount * (foamPercent / 100);
  return baseMilkAmount - reduction;
}

/**
 * Calculate free space in cup
 * @param {string} cupSize - Cup size code (H7, H9, H12, C7, C9, C12, C16)
 * @param {number} recipeVolume - Total volume of recipe ingredients in ml
 * @returns {number} Free space remaining in ml
 */
export function calculateFreeSpace(cupSize, recipeVolume) {
  const cupVolume = CUP_VOLUMES[cupSize] || 266; // Default to H9
  const freeSpace = cupVolume - recipeVolume;
  return Math.max(0, freeSpace); // Never negative
}

/**
 * Convert weight to volume using density
 * @param {number} weight - Weight in grams
 * @param {string} ingredientCategory - Category of ingredient (syrups, sauce, etc.)
 * @returns {number} Volume in ml
 */
export function getVolumeEquivalent(weight, ingredientCategory) {
  const density = INGREDIENT_DENSITIES[ingredientCategory] || INGREDIENT_DENSITIES.syrups;
  return weight / density;
}

/**
 * Allocate addon volume to free space and overflow
 * @param {number} addonVolume - Volume of addon in ml
 * @param {number} freeSpace - Available free space in ml
 * @returns {object} { allocatedToFreeSpace, overflow }
 */
export function allocateToFreeSpace(addonVolume, freeSpace) {
  if (freeSpace >= addonVolume) {
    return {
      allocatedToFreeSpace: addonVolume,
      overflow: 0,
    };
  } else {
    return {
      allocatedToFreeSpace: freeSpace,
      overflow: addonVolume - freeSpace,
    };
  }
}

/**
 * Calculate milk substitution from overflow volume
 * @param {number} overflowVolume - Volume overflowing into milk in ml
 * @param {number} foamReducedMilk - Milk amount after foam reduction in ml
 * @returns {object} { substitutionAmount, substitutionPercent }
 */
export function calculateMilkSubstitution(overflowVolume, foamReducedMilk) {
  const substitutionPercent = (overflowVolume / foamReducedMilk) * 100;
  return {
    substitutionAmount: overflowVolume,
    substitutionPercent: substitutionPercent,
  };
}

/**
 * Check if more ingredients can be added
 * @param {number} currentSubstitutionPercent - Current milk substitution percentage
 * @param {number} foamReducedMilk - Milk amount after foam reduction in ml
 * @returns {object} { canAddMore, remainingCapacity, remainingPercent }
 */
export function canAddMoreIngredients(currentSubstitutionPercent, foamReducedMilk) {
  const remainingPercent = MAX_MILK_SUBSTITUTION_PERCENT - currentSubstitutionPercent;
  const canAddMore = remainingPercent > 0.1; // Allow small tolerance
  const remainingCapacity = (remainingPercent / 100) * foamReducedMilk;
  
  return {
    canAddMore,
    remainingCapacity: Math.max(0, remainingCapacity),
    remainingPercent: Math.max(0, remainingPercent),
  };
}

/**
 * Get default weight for ingredient based on category
 * @param {string} category - Ingredient category
 * @param {number} quantity - Quantity/pumps
 * @returns {number} Weight in grams
 */
export function getIngredientWeight(category, quantity = 1) {
  const weights = {
    syrups: DEFAULT_SYRUP_WEIGHT_PER_PUMP * quantity,
    sauce: DEFAULT_SAUCE_WEIGHT_PER_PUMP * quantity,
    toppings: 5 * quantity, // Estimate
    extras: 10 * quantity, // Estimate
  };
  return weights[category] || 10 * quantity;
}

/**
 * Main function: Calculate adjusted milk with three-phase logic
 * @param {string} cupSize - Cup size code (H7, H9, H12, etc.)
 * @param {number} baseMilkAmount - Original milk amount in ml
 * @param {string} temperature - Temperature setting
 * @param {Array} addons - Array of addon objects with { category, quantity, weight }
 * @param {number} recipeVolume - Total recipe volume in ml
 * @returns {object} Complete capacity calculation results
 */
export function calculateAdjustedMilk(
  cupSize,
  baseMilkAmount,
  temperature,
  addons = [],
  recipeVolume = 0
) {
  // Phase 1: Apply foam reduction
  const foamReducedMilk = applyFoamReduction(baseMilkAmount, temperature);
  const foamReductionAmount = baseMilkAmount - foamReducedMilk;
  const foamReductionPercent = (foamReductionAmount / baseMilkAmount) * 100;
  
  // Phase 2 & 3: Calculate addon impact
  let freeSpaceRemaining = calculateFreeSpace(cupSize, recipeVolume);
  let totalOverflow = 0;
  const addonDetails = [];
  
  for (const addon of addons) {
    // Get weight and convert to volume
    const weight = addon.weight || getIngredientWeight(addon.category, addon.quantity);
    const volume = getVolumeEquivalent(weight, addon.category);
    
    // Allocate to free space first, then overflow
    const allocation = allocateToFreeSpace(volume, freeSpaceRemaining);
    freeSpaceRemaining -= allocation.allocatedToFreeSpace;
    totalOverflow += allocation.overflow;
    
    addonDetails.push({
      ...addon,
      weight,
      volume,
      allocatedToFreeSpace: allocation.allocatedToFreeSpace,
      overflow: allocation.overflow,
    });
  }
  
  // Calculate milk substitution
  const { substitutionAmount, substitutionPercent } = calculateMilkSubstitution(
    totalOverflow,
    foamReducedMilk
  );
  
  // Final adjusted milk
  const adjustedMilkAmount = foamReducedMilk - substitutionAmount;
  
  // Check capacity limits
  const { canAddMore, remainingCapacity, remainingPercent } = canAddMoreIngredients(
    substitutionPercent,
    foamReducedMilk
  );
  
  return {
    // Phase 1 results
    baseMilkAmount,
    foamReducedMilk,
    foamReductionAmount,
    foamReductionPercent,
    
    // Phase 2 results
    freeSpaceRemaining,
    
    // Phase 3 results
    milkSubstitutionAmount: substitutionAmount,
    milkSubstitutionPercent: substitutionPercent,
    adjustedMilkAmount,
    
    // Capacity check
    canAddMore,
    remainingSubstitutionCapacity: remainingCapacity,
    remainingSubstitutionPercent: remainingPercent,
    
    // Details
    addonDetails,
    temperature,
  };
}

/**
 * Helper: Get cup size from menu item size string
 * @param {string} size - Size string like "7oz", "9oz", "12oz"
 * @param {string} cupType - "hot" or "cold" (default: hot)
 * @returns {string} Cup size code like "H7", "C9"
 */
export function getCupSizeCode(size, cupType = 'hot') {
  const sizeNum = size.replace(/[^0-9]/g, '');
  const prefix = cupType === 'cold' ? 'C' : 'H';
  return `${prefix}${sizeNum}`;
}

/**
 * Helper: Calculate total recipe volume from ingredients
 * @param {Array} ingredients - Array of default ingredients with unit_amount
 * @returns {number} Total volume in ml
 */
export function calculateRecipeVolume(ingredients) {
  let total = 0;
  for (const ing of ingredients) {
    const amount = ing.unit_amount || 0;
    const quantity = ing.quantity || 1;
    const baseUnits = (ing.base_units || '').toLowerCase();
    const category = (ing.category || '').toLowerCase();
    const type = (ing.type || '').toLowerCase().replace(/\s+/g, '_');
    
    // Skip ice - it's measured in grams and doesn't affect liquid volume
    if (category === 'ice') {
      continue;
    }
    
    // Espresso shots - use actual weights converted to volume
    if (baseUnits === 'shots' || category === 'espresso') {
      // Get espresso weight based on type (single, double, triple)
      const shotWeight = ESPRESSO_SHOT_WEIGHTS[type] || ESPRESSO_SHOT_WEIGHTS.double_shot;
      // Convert weight to volume using espresso density
      const espressoVolume = (shotWeight * quantity) / INGREDIENT_DENSITIES.espresso;
      total += espressoVolume;
    }
    // Volumes in ml - use directly
    else if (baseUnits === 'ml') {
      total += amount * quantity;
    }
    // Default: treat as grams and convert to volume using density
    // This handles 'grams', 'g', 'pumps', undefined, etc.
    else {
      const density = INGREDIENT_DENSITIES[category] || 1.0;
      const volume = (amount * quantity) / density;
      total += volume;
    }
  }
  return total;
}
