/**
 * Inventory Data Structure
 * Defines all inventory items organized by categories
 */
import milk from '../assets/milk.png';
import beans from '../assets/beans.png';
import syrups from '../assets/syrup.png';
import cups from '../assets/cup.png';
import premixes from '../assets/cup.png';
export const INVENTORY_CATEGORIES = {
  MILK: 'milk',
  BEANS: 'beans', 
  SYRUPS: 'syrups',
  CUPS: 'cups',
  Premixes: 'premixes'
};

export const INVENTORY_ITEMS = {
  // 8 Types of Milk
  milk: {
    whole_milk: { name: 'Whole Milk', icon: '🥛', category: 'milk' },
    skim_milk: { name: 'Skim Milk', icon: '🥛', category: 'milk' },
    almond_milk: { name: 'Almond Milk', icon: '🥛', category: 'milk' },
    soy_milk: { name: 'Soy Milk', icon: '🥛', category: 'milk' },
    oat_milk: { name: 'Oat Milk', icon: '🥛', category: 'milk' },
    coconut_milk: { name: 'Coconut Milk', icon: '🥛', category: 'milk' },
    rice_milk: { name: 'Rice Milk', icon: '🥛', category: 'milk' },
    heavy_cream: { name: 'Heavy Cream', icon: '🥛', category: 'milk' }
  },

  // 1 Type of Coffee Bean
  coffee_beans: {
    coffee_beans: { name: 'Coffee Beans', icon: '☕', category: 'beans' }
  },

  // 12 Types of Syrups
  syrups: {
    vanilla_syrup: { name: 'Vanilla Syrup', icon: '🍯', category: 'syrups' },
    caramel_syrup: { name: 'Caramel Syrup', icon: '🍯', category: 'syrups' },
    chocolate_syrup: { name: 'Chocolate Syrup', icon: '🍯', category: 'syrups' },
    hazelnut_syrup: { name: 'Hazelnut Syrup', icon: '🍯', category: 'syrups' },
    cinnamon_syrup: { name: 'Cinnamon Syrup', icon: '🍯', category: 'syrups' },
    peppermint_syrup: { name: 'Peppermint Syrup', icon: '🍯', category: 'syrups' },
    irish_cream_syrup: { name: 'Irish Cream Syrup', icon: '🍯', category: 'syrups' },
    amaretto_syrup: { name: 'Amaretto Syrup', icon: '🍯', category: 'syrups' },
    coconut_syrup: { name: 'Coconut Syrup', icon: '🍯', category: 'syrups' },
    raspberry_syrup: { name: 'Raspberry Syrup', icon: '🍯', category: 'syrups' },
    lavender_syrup: { name: 'Lavender Syrup', icon: '🍯', category: 'syrups' },
    maple_syrup: { name: 'Maple Syrup', icon: '🍯', category: 'syrups' }
  },

  // 7 Types of Cups (Paper: 7,9,12 oz + Plastic: 7,9,12,16 oz)
  cups: {
    paper_cup_7oz: { name: '7oz Paper Cup', icon: '🥤', category: 'cups', size: '7oz', material: 'paper' },
    paper_cup_9oz: { name: '9oz Paper Cup', icon: '🥤', category: 'cups', size: '9oz', material: 'paper' },
    paper_cup_12oz: { name: '12oz Paper Cup', icon: '🥤', category: 'cups', size: '12oz', material: 'paper' },
    plastic_cup_7oz: { name: '7oz Plastic Cup', icon: '🥤', category: 'cups', size: '7oz', material: 'plastic' },
    plastic_cup_9oz: { name: '9oz Plastic Cup', icon: '🥤', category: 'cups', size: '9oz', material: 'plastic' },
    plastic_cup_12oz: { name: '12oz Plastic Cup', icon: '🥤', category: 'cups', size: '12oz', material: 'plastic' },
    plastic_cup_16oz: { name: '16oz Plastic Cup', icon: '🥤', category: 'cups', size: '16oz', material: 'plastic' }
  },

  premixes: {
    mocha_frappe: { name: 'mocha frappe', icon: '🥤', category: 'premixes' },
    chocolate_frappe: { name: 'chocolate frappe', icon: '🥤', category: 'premixes'},
    half_and_half: { name: 'half and half', icon: '🥤', category: 'premixes'}
  }
};

// Flatten all items for easy access
export const ALL_INVENTORY_ITEMS = {
  ...INVENTORY_ITEMS.milk,
  ...INVENTORY_ITEMS.beans,
  ...INVENTORY_ITEMS.syrups,
  ...INVENTORY_ITEMS.cups,
  ...INVENTORY_ITEMS.premixes
};

// Category display information
export const CATEGORY_INFO = {
  milk: {
    title: 'Milk',
    icon: '🥛',
    description: 'Various types of milk and dairy products',
    avatar: milk
  },
  coffee_beans: {
    title: 'Coffee',
    icon: '☕',
    description: 'Coffee bean inventory',
    avatar: beans
  },
  syrups: {
    title: 'Syrups',
    icon: '🍯',
    description: 'Flavor syrups and additives',
    avatar: syrups
  },
  premixes: {
    title: 'Premixes',
    icon: '🍯',
    description: 'Premixes for Sluches',
    avatar: premixes
  },
  cups: {
    title: 'Cups',
    icon: '🥤',
    description: 'Paper and plastic cups in various sizes',
    avatar: cups
  }
};

// Helper functions
export const getItemsByCategory = (category) => {
  return Object.entries(ALL_INVENTORY_ITEMS)
    .filter(([_, item]) => item.category === category)
    .reduce((acc, [key, item]) => ({ ...acc, [key]: item }), {});
};

// export const getCategoryItems = (category) => {
//   return INVENTORY_ITEMS[category] || {};
// };
export const getCategoryItems = (category) => {
  
  const items = INVENTORY_ITEMS[category];
  //console.log("🔍 Requested items:", items);
  // If no exact match, try fuzzy match (like partial includes)
  if (!items) {
    const fallbackKey = Object.keys(INVENTORY_ITEMS).find(key =>
      category.includes(key) || key.includes(category)
    );
    return INVENTORY_ITEMS[fallbackKey] || {};
  }

  return items;
};
export const getItemDetails = (itemKey) => {
  return ALL_INVENTORY_ITEMS[itemKey] || null;
};

// Generate mock inventory levels for development
export const generateMockInventoryData = () => {
  const data = {};
  
  Object.keys(ALL_INVENTORY_ITEMS).forEach(itemKey => {
    const randomLevel = Math.floor(Math.random() * 100);
    let level = 'medium';
    
    if (randomLevel < 20) level = 'low';
    else if (randomLevel < 50) level = 'medium';
    else level = 'high';
    
    data[itemKey] = {
      level,
      numeric: randomLevel,
      last_refilled: new Date(Date.now() - Math.random() * 7 * 24 * 60 * 60 * 1000).toISOString()
    };
  });
  
  return data;
}; 