/**
 * Ingredient mappings for inventory threshold alerts
 * 
 * Maps ingredient IDs from validation service to user-friendly names and icons
 * Format from validation: "{category}_{subtype}" (e.g., "milk_whole_fat_milk")
 */

import React from 'react';

// SVG Icon Components (reusing from validationMessages)
const IngredientIcons = {
  milk: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path strokeLinecap="round" strokeLinejoin="round" d="M6 3h12l1 3v13a2 2 0 01-2 2H7a2 2 0 01-2-2V6l1-3z" />
      <path strokeLinecap="round" strokeLinejoin="round" d="M9 3v3h6V3M7 10h10" />
      <circle cx="12" cy="15" r="1.5" fill="currentColor" />
    </svg>
  ),
  coffee_beans: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <ellipse cx="9" cy="10" rx="4" ry="5" strokeLinecap="round" />
      <ellipse cx="15" cy="14" rx="3.5" ry="4.5" strokeLinecap="round" />
      <path strokeLinecap="round" d="M8 10c0-1 .5-2 1.5-2.5M14 14c0-1 .5-1.5 1-2" />
    </svg>
  ),
  cups: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path strokeLinecap="round" strokeLinejoin="round" d="M3 9h18M5 9v10a2 2 0 002 2h10a2 2 0 002-2V9M9 3v2m6-2v2" />
      <circle cx="12" cy="14" r="2" fill="currentColor" />
    </svg>
  ),
  syrups: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path strokeLinecap="round" strokeLinejoin="round" d="M9 3h6M9 3v2m6-2v2M8 5h8a1 1 0 011 1v3H7V6a1 1 0 011-1z" />
      <path strokeLinecap="round" strokeLinejoin="round" d="M7 9v1a5 5 0 0010 0V9M12 14v4" />
    </svg>
  ),
  premixes: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path strokeLinecap="round" strokeLinejoin="round" d="M19 11H5m14 0a2 2 0 012 2v6a2 2 0 01-2 2H5a2 2 0 01-2-2v-6a2 2 0 012-2m14 0V9a2 2 0 00-2-2M5 11V9a2 2 0 012-2m0 0V5a2 2 0 012-2h6a2 2 0 012 2v2M7 7h10" />
    </svg>
  ),
  water: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path strokeLinecap="round" strokeLinejoin="round" d="M12 2.69l5.66 5.66a8 8 0 11-11.31 0z" />
      <path strokeLinecap="round" d="M12 2.69v18.62" opacity="0.3" />
    </svg>
  )
};

// Category-level configuration
export const INGREDIENT_CATEGORIES = {
  milk: {
    iconKey: "milk",
    color: "text-green-600",
    displayName: "Milk"
  },
  coffee_beans: {
    iconKey: "coffee_beans",
    color: "text-green-700",
    displayName: "Coffee Beans"
  },
  cups: {
    iconKey: "cups",
    color: "text-green-600",
    displayName: "Cups"
  },
  syrups: {
    iconKey: "syrups",
    color: "text-green-600",
    displayName: "Syrups"
  },
  premixes: {
    iconKey: "premixes",
    color: "text-green-700",
    displayName: "Premixes"
  },
  water: {
    iconKey: "water",
    color: "text-green-500",
    displayName: "Water"
  }
};

// Ingredient-specific mappings (based on validation_schema.sql)
export const INGREDIENT_MAPPINGS = {
  // Coffee Beans (IDs: 1-2)
  "coffee_beans_regular": { name: "Regular Coffee Beans", category: "coffee_beans" },
  "coffee_beans_decaf": { name: "Decaf Coffee Beans", category: "coffee_beans" },
  
  // Cups - Hot (IDs: 3-5)
  "cups_cup_H7": { name: "Hot Cup 7oz (H7)", category: "cups" },
  "cups_cup_H9": { name: "Hot Cup 9oz (H9)", category: "cups" },
  "cups_cup_H12": { name: "Hot Cup 12oz (H12)", category: "cups" },
  
  // Cups - Cold (IDs: 6-9)
  "cups_cup_C7": { name: "Cold Cup 7oz (C7)", category: "cups" },
  "cups_cup_C9": { name: "Cold Cup 9oz (C9)", category: "cups" },
  "cups_cup_C12": { name: "Cold Cup 12oz (C12)", category: "cups" },
  "cups_cup_C16": { name: "Cold Cup 16oz (C16)", category: "cups" },
  
  // Milk varieties (IDs: 10-13)
  "milk_whole_fat_milk": { name: "Whole Fat Milk", category: "milk" },
  "milk_low_fat_milk": { name: "Low Fat Milk", category: "milk" },
  "milk_lactose_free_milk": { name: "Lactose Free Milk", category: "milk" },
  "milk_almond_milk": { name: "Almond Milk", category: "milk" },
  
  // Syrups (IDs: 14-19)
  "syrups_vanilla_syrup": { name: "Vanilla Syrup", category: "syrups" },
  "syrups_caramel_syrup": { name: "Caramel Syrup", category: "syrups" },
  "syrups_hazelnut_syrup": { name: "Hazelnut Syrup", category: "syrups" },
  "syrups_peached_iced_syrup": { name: "Peach Iced Syrup", category: "syrups" },
  "syrups_passion_fruit_iced_syrup": { name: "Passion Fruit Iced Syrup", category: "syrups" },
  "syrups_ice_tea_syrup": { name: "Ice Tea Syrup", category: "syrups" },
  
  // Sauces (IDs: 20-22)
  "syrups_white_chocolate_sauce": { name: "White Chocolate Sauce", category: "syrups" },
  "syrups_caramel_sauce": { name: "Caramel Sauce", category: "syrups" },
  "syrups_condense_milk_sauce": { name: "Condensed Milk Sauce", category: "syrups" },
  
  // Premixes (IDs: 23-25)
  "premixes_mocha_frappe": { name: "Mocha Frappé", category: "premixes" },
  "premixes_chocolate_frappe": { name: "Chocolate Frappé", category: "premixes" },
  "premixes_half_and_half": { name: "Half & Half", category: "premixes" }
};

/**
 * Parse ingredient ID into category and subtype
 * @param {string} ingredientId - Format: "category_subtype" (e.g., "milk_whole_fat_milk")
 * @returns {{category: string, subtype: string}}
 */
export const parseIngredientId = (ingredientId) => {
  if (!ingredientId || typeof ingredientId !== 'string') {
    return { category: 'unknown', subtype: 'unknown' };
  }
  
  const parts = ingredientId.split('_');
  if (parts.length < 2) {
    return { category: ingredientId, subtype: '' };
  }
  
  // First part is category, rest is subtype
  const category = parts[0];
  const subtype = parts.slice(1).join('_');
  
  return { category, subtype };
};

/**
 * Format subtype name for display (fallback when no mapping exists)
 * @param {string} subtype - e.g., "whole_fat_milk"
 * @returns {string} - e.g., "Whole Fat Milk"
 */
const formatSubtypeName = (subtype) => {
  if (!subtype) return '';
  
  return subtype
    .split('_')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1))
    .join(' ');
};

/**
 * Get complete ingredient data (name, icon, color) for display
 * @param {string} ingredientId - Format: "category_subtype"
 * @returns {{name: string, icon: JSX.Element|null, color: string, category: string}}
 */
export const getIngredientData = (ingredientId) => {
  // Check if we have a specific mapping for this ingredient
  const mapping = INGREDIENT_MAPPINGS[ingredientId];
  
  if (mapping) {
    // We have a specific mapping - use it
    const categoryConfig = INGREDIENT_CATEGORIES[mapping.category];
    const IconComponent = categoryConfig ? IngredientIcons[categoryConfig.iconKey] : null;
    
    return {
      name: mapping.name,
      icon: IconComponent ? <IconComponent /> : null,
      color: categoryConfig ? categoryConfig.color : 'text-gray-600',
      category: mapping.category
    };
  }
  
  // No specific mapping - use fallback with parsed category
  const { category, subtype } = parseIngredientId(ingredientId);
  const categoryConfig = INGREDIENT_CATEGORIES[category];
  const IconComponent = categoryConfig ? IngredientIcons[categoryConfig.iconKey] : null;
  
  return {
    name: formatSubtypeName(subtype) || categoryConfig?.displayName || category,
    icon: IconComponent ? <IconComponent /> : null,
    color: categoryConfig ? categoryConfig.color : 'text-gray-600',
    category: category
  };
};

/**
 * Get category icon component
 * @param {string} category - Category name (e.g., "milk", "coffee_beans")
 * @returns {JSX.Element|null}
 */
export const getCategoryIcon = (category) => {
  const categoryConfig = INGREDIENT_CATEGORIES[category];
  if (!categoryConfig) return null;
  
  const IconComponent = IngredientIcons[categoryConfig.iconKey];
  return IconComponent ? <IconComponent /> : null;
};

/**
 * Get severity message for ingredient alerts
 * @param {string} severity - "low", "empty", "critical"
 * @returns {string}
 */
export const getSeverityMessage = (severity) => {
  const messages = {
    empty: "Out of stock - please refill immediately",
    low: "Low stock - please refill to continue operations",
    critical: "Critical level - refill required",
    medium: "Stock level acceptable",
    high: "Stock level good"
  };
  
  return messages[severity] || "Please check inventory levels";
};

