/**
 * Cup Capacity Configuration
 * Constants for cup volumes, ingredient densities, and temperature foam percentages
 */

// Cup volumes in ml (1oz = ~29.57ml)
export const CUP_VOLUMES = {
  H7: 207,   // 7oz hot cup
  H9: 266,   // 9oz hot cup
  H12: 355,  // 12oz hot cup
  C7: 207,   // 7oz cold cup
  C9: 266,   // 9oz cold cup
  C12: 355,  // 12oz cold cup
  C16: 473,  // 16oz cold cup
};

// Ingredient densities (g/ml) - milk is baseline at ~1.03
export const INGREDIENT_DENSITIES = {
  milk: 1.03,        // Baseline for milk
  syrups: 1.12,      // Heavier than milk (sugar content)
  sauce: 1.35,       // Sauces are denser
  espresso: 1.02,    // Similar to water
  ice: 0.92,         // Ice floats (less dense)
  toppings: 1.20,    // Average for toppings
  extras: 1.15,      // Average for extras
  water: 1.00,       // Water is 1.00 g/ml
};

// Temperature foam percentages (percentage of milk replaced by foam)
export const TEMPERATURE_FOAM_PERCENTAGES = {
  kids: 0,           // No extra foam for kids temp (54C)
  standard: 10,      // 10% foam for standard (71.5C)
  extra_hot: 15,     // 15% foam for extra hot (83.5C)
  regular: 10,       // Alias for standard
  normal: 10,        // Alias for standard
};

// Maximum milk substitution percentage (after free space is filled)
export const MAX_MILK_SUBSTITUTION_PERCENT = 30;

// Temperature display names and descriptions
export const TEMPERATURE_OPTIONS = [
  {
    id: 'kids',
    label: 'Kids',
    description: '54°C',
    foamPercent: 0,
  },
  {
    id: 'standard',
    label: 'Regular',
    description: '71.5°C',
    foamPercent: 10,
  },
  {
    id: 'extra_hot',
    label: 'Extra Hot',
    description: '83.5°C',
    foamPercent: 15,
  },
];

// Default values
export const DEFAULT_TEMPERATURE = 'standard';
export const DEFAULT_SYRUP_WEIGHT_PER_PUMP = 10; // grams per pump
export const DEFAULT_SAUCE_WEIGHT_PER_PUMP = 10; // grams per pump

// Helper to check if a cup is for cold/iced drinks (no foam)
export const isIcedCup = (cupSize) => {
  if (!cupSize) return false;
  // Cold cups start with 'C', hot cups start with 'H'
  return cupSize.toString().toUpperCase().startsWith('C');
};

// Espresso shot weights (in grams)
export const ESPRESSO_SHOT_WEIGHTS = {
  single_shot: 30,    // Single shot = 18g
  double_shot: 60,    // Double shot = 36g
  tripple_shot: 90,   // Triple shot = 54g
  single: 30,         // Alias
  double: 60,         // Alias
  triple: 90,         // Alias
  tripple: 90,        // Typo alias (common in DBs)
};
