/**
 * Validation message mappings for dashboard display
 * 
 * This file maps validation function keys from the backend to user-friendly messages.
 * Structure is prepared for future i18n support - when adding languages, convert to:
 * { "key": { "en": "English message", "ar": "Arabic message" } }
 */

import React from 'react';

// SVG Icon Components for validation messages - using functions to avoid JSX parsing issues
const ValidationIcons = {
  cup: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path strokeLinecap="round" strokeLinejoin="round" d="M3 9h18M5 9v10a2 2 0 002 2h10a2 2 0 002-2V9M9 3v2m6-2v2" />
      <circle cx="12" cy="14" r="2" fill="currentColor" />
    </svg>
  ),
  beans: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <ellipse cx="9" cy="10" rx="4" ry="5" strokeLinecap="round" />
      <ellipse cx="15" cy="14" rx="3.5" ry="4.5" strokeLinecap="round" />
      <path strokeLinecap="round" d="M8 10c0-1 .5-2 1.5-2.5M14 14c0-1 .5-1.5 1-2" />
    </svg>
  ),
  ingredients: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path strokeLinecap="round" strokeLinejoin="round" d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
    </svg>
  ),
  syrup: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path strokeLinecap="round" strokeLinejoin="round" d="M9 3h6M9 3v2m6-2v2M8 5h8a1 1 0 011 1v3H7V6a1 1 0 011-1z" />
      <path strokeLinecap="round" strokeLinejoin="round" d="M7 9v1a5 5 0 0010 0V9M12 14v4" />
    </svg>
  ),
  milk: () => (
    <svg className="w-12 h-12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path strokeLinecap="round" strokeLinejoin="round" d="M6 3h12l1 3v13a2 2 0 01-2 2H7a2 2 0 01-2-2V6l1-3z" />
      <path strokeLinecap="round" strokeLinejoin="round" d="M9 3v3h6V3M7 10h10" />
      <circle cx="12" cy="15" r="1.5" fill="currentColor" />
    </svg>
  )
};

export const VALIDATION_MESSAGES = {
  "cup_detection": {
    message: "Please clear the cup station to proceed.",
    iconKey: "cup",
    color: "text-green-600"
  },
  "check_coffee_beans": {
    message: "Please refill coffee beans.",
    iconKey: "beans",
    color: "text-green-700"
  },
  "validate_ingredients": {
    message: "Please refill the ingredients.",
    iconKey: "ingredients",
    color: "text-green-600"
  },
  "sauce_cup_detection_absent": {
    message: "Remove the cup from the syrup dispenser to proceed.",
    iconKey: "syrup",
    color: "text-green-600"
  },
  "sauce_cup_detection_present": {
    message: "Place the cup in the syrup dispenser to proceed.",
    iconKey: "syrup",
    color: "text-green-600"
  },
  "milk_cup_detection_absent": {
    message: "Remove the cup from the milk dispenser to proceed.",
    iconKey: "milk",
    color: "text-green-600"
  },
  "milk_cup_detection_present": {
    message: "Place the cup in the milk dispenser to proceed.",
    iconKey: "milk",
    color: "text-green-600"
  }
};

/**
 * Get the display message for a validation function key
 * @param {string} key - Validation function key (e.g., "cup_detection")
 * @param {string|null} defaultMessage - Optional fallback message
 * @returns {string} The display message
 */
export const getValidationMessage = (key, defaultMessage = null) => {
  const validation = VALIDATION_MESSAGES[key];
  if (!validation) return defaultMessage || `Validation failed: ${key}`;
  return validation.message;
};

/**
 * Get the icon for a validation function key
 * @param {string} key - Validation function key (e.g., "cup_detection")
 * @returns {JSX.Element|null} The SVG icon component or null if not found
 */
export const getValidationIcon = (key) => {
  const validation = VALIDATION_MESSAGES[key];
  if (!validation || !validation.iconKey) return null;
  const IconComponent = ValidationIcons[validation.iconKey];
  return IconComponent ? <IconComponent /> : null;
};

/**
 * Get complete validation data (message, icon, color) for a validation function key
 * @param {string} key - Validation function key
 * @param {string|null} defaultMessage - Optional fallback message
 * @returns {{message: string, icon: JSX.Element|null, color: string}} Object with message, icon, and color
 */
export const getValidationData = (key, defaultMessage = null) => {
  const validation = VALIDATION_MESSAGES[key];
  if (!validation) {
    return {
      message: defaultMessage || `Validation failed: ${key}`,
      icon: null,
      color: "text-gray-600"
    };
  }
  
  // Get the icon component and render it
  const IconComponent = validation.iconKey ? ValidationIcons[validation.iconKey] : null;
  
  return {
    message: validation.message,
    icon: IconComponent ? <IconComponent /> : null,
    color: validation.color
  };
};

