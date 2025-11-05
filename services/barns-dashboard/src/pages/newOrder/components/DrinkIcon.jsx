/**
 * DrinkIcon Component
 * Unique SVG icons for each drink type - Optimized for clean rendering
 */

import React from 'react';

const DrinkIcons = {
  // Hot espresso-based drinks
  'Americano': (
    <svg viewBox="0 0 64 64" fill="none">
      <rect x="17" y="25" width="30" height="26" rx="2" fill="rgba(139, 69, 19, 0.25)" stroke="currentColor" strokeWidth="2"/>
      <path d="M17 30 L47 30" stroke="currentColor" strokeWidth="2"/>
      <path d="M21 21 Q25 14 32 14 Q39 14 43 21" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
      <circle cx="32" cy="40" r="2" fill="currentColor"/>
    </svg>
  ),
  
  'Cappuccino': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup body */}
      <path d="M17 31 L47 31 L47 49 Q47 53 43 53 L21 53 Q17 53 17 49 Z" fill="rgba(139, 69, 19, 0.3)" stroke="currentColor" strokeWidth="2"/>
      {/* Foam base */}
      <ellipse cx="32" cy="31" rx="15" ry="4" fill="rgba(255, 248, 220, 0.9)" stroke="currentColor" strokeWidth="2"/>
      {/* Foam top */}
      <ellipse cx="32" cy="29" rx="13" ry="3" fill="white"/>
      {/* Steam */}
      <path d="M24 21 Q26 15 24 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.5"/>
      <path d="M32 19 Q34 13 32 9" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.5"/>
      <path d="M40 21 Q38 15 40 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.5"/>
    </svg>
  ),
  
  'Caramel Macchiato': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <rect x="17" y="25" width="30" height="26" rx="2" fill="rgba(210, 180, 140, 0.35)" stroke="currentColor" strokeWidth="2"/>
      {/* Caramel drizzle layers */}
      <path d="M20 29 L44 29" stroke="#D2691E" strokeWidth="1.5" opacity="0.7"/>
      <path d="M22 33 L42 33" stroke="#D2691E" strokeWidth="1.5" opacity="0.6"/>
      <path d="M24 37 L40 37" stroke="#D2691E" strokeWidth="1.5" opacity="0.5"/>
      {/* Steam */}
      <path d="M21 21 Q25 14 32 14 Q39 14 43 21" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
      <circle cx="32" cy="44" r="2.5" fill="#D2691E" opacity="0.6"/>
    </svg>
  ),
  
  'Chocolate Frappe': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M23 25 L23 49 Q23 53 27 53 L37 53 Q41 53 41 49 L41 25 Z" fill="rgba(101, 67, 33, 0.5)" stroke="currentColor" strokeWidth="2"/>
      {/* Whipped cream */}
      <path d="M23 25 Q27 20 32 22 Q37 20 41 25" fill="rgba(255, 248, 220, 0.95)" stroke="currentColor" strokeWidth="1.5"/>
      <circle cx="28" cy="22" r="2" fill="white"/>
      <circle cx="36" cy="22" r="2" fill="white"/>
      <circle cx="32" cy="19" r="2.5" fill="white"/>
      {/* Chocolate drizzle */}
      <path d="M27 27 Q30 29 32 27 Q34 29 37 27" stroke="#8B4513" strokeWidth="1.5" opacity="0.8"/>
      {/* Straw */}
      <rect x="37" y="15" width="2" height="14" rx="1" fill="currentColor" opacity="0.5"/>
    </svg>
  ),
  
  'Cortado': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Glass */}
      <path d="M25 29 L25 47 L39 47 L39 29 Z" fill="rgba(160, 82, 45, 0.3)" stroke="currentColor" strokeWidth="2"/>
      {/* Milk layer */}
      <rect x="25" y="38" width="14" height="9" fill="rgba(255, 248, 220, 0.5)"/>
      {/* Saucer */}
      <path d="M21 25 L43 25" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
      <circle cx="32" cy="33" r="1.5" fill="currentColor"/>
    </svg>
  ),
  
  'Espresso': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Small cup */}
      <path d="M27 31 L27 45 Q27 47 29 47 L35 47 Q37 47 37 45 L37 31 Z" fill="rgba(101, 67, 33, 0.5)" stroke="currentColor" strokeWidth="2.5"/>
      {/* Crema */}
      <ellipse cx="32" cy="31" rx="5" ry="2" fill="rgba(139, 69, 19, 0.7)" stroke="currentColor" strokeWidth="1.5"/>
      {/* Steam */}
      <path d="M28 27 L32 21 L36 27" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
      {/* Saucer */}
      <rect x="25" y="47" width="14" height="2" rx="1" fill="currentColor"/>
    </svg>
  ),
  
  'Flat White': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M19 27 L45 27 L45 49 Q45 53 41 53 L23 53 Q19 53 19 49 Z" fill="rgba(139, 69, 19, 0.25)" stroke="currentColor" strokeWidth="2"/>
      {/* Microfoam */}
      <ellipse cx="32" cy="27" rx="13" ry="2.5" fill="rgba(255, 248, 220, 0.95)" stroke="currentColor" strokeWidth="1"/>
      {/* Latte art - heart */}
      <path d="M29 29 Q32 32 35 29 Q37 27 35 25 Q33 27 32 27 Q31 27 29 25 Q27 27 29 29" fill="white"/>
      {/* Steam */}
      <path d="M25 19 Q27 15 25 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.4"/>
      <path d="M39 19 Q37 15 39 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.4"/>
    </svg>
  ),
  
  'Latte': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M17 27 L47 27 L47 49 Q47 53 43 53 L21 53 Q17 53 17 49 Z" fill="rgba(245, 222, 179, 0.4)" stroke="currentColor" strokeWidth="2"/>
      {/* Coffee layer */}
      <rect x="17" y="41" width="30" height="12" fill="rgba(160, 82, 45, 0.4)"/>
      {/* Milk foam */}
      <ellipse cx="32" cy="27" rx="13" ry="2" fill="rgba(255, 248, 220, 0.7)"/>
      {/* Latte art */}
      <path d="M32 29 L32 35" stroke="white" strokeWidth="1.5" opacity="0.9"/>
      <path d="M29 31 Q32 33 35 31" stroke="white" strokeWidth="1.5" opacity="0.9" strokeLinecap="round"/>
      <path d="M29 35 Q32 37 35 35" stroke="white" strokeWidth="1.5" opacity="0.9" strokeLinecap="round"/>
      {/* Handle */}
      <path d="M49 33 Q53 33 53 37 L53 41 Q53 45 49 45" stroke="currentColor" strokeWidth="2"/>
      {/* Steam */}
      <path d="M25 19 Q27 15 25 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.4"/>
      <path d="M39 19 Q37 15 39 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.4"/>
    </svg>
  ),
  
  'Macchiato': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M25 29 L25 47 Q25 51 29 51 L35 51 Q39 51 39 47 L39 29 Z" fill="rgba(101, 67, 33, 0.35)" stroke="currentColor" strokeWidth="2"/>
      {/* Milk dot */}
      <ellipse cx="32" cy="29" rx="7" ry="2" fill="white" stroke="currentColor" strokeWidth="1.5"/>
      <circle cx="32" cy="33" r="2" fill="white" opacity="0.8"/>
      {/* Steam */}
      <path d="M27 25 L32 19 L37 25" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
  ),
  
  'Spanish Latte': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M17 27 L47 27 L47 49 Q47 53 43 53 L21 53 Q17 53 17 49 Z" fill="rgba(222, 184, 135, 0.4)" stroke="currentColor" strokeWidth="2"/>
      {/* Condensed milk layer */}
      <rect x="18" y="41" width="28" height="10" fill="rgba(255, 228, 181, 0.85)"/>
      {/* Coffee layer */}
      <rect x="17" y="33" width="30" height="10" fill="rgba(139, 69, 19, 0.4)"/>
      {/* Caramel swirl */}
      <path d="M21 38 Q28 40 35 38 Q42 37 46 38" stroke="#FFD700" strokeWidth="1.5" opacity="0.7"/>
      {/* Handle */}
      <path d="M49 33 Q53 33 53 37 L53 41 Q53 45 49 45" stroke="currentColor" strokeWidth="2"/>
      {/* Steam */}
      <path d="M25 19 Q27 15 25 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.4"/>
      <path d="M39 19 Q37 15 39 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.4"/>
    </svg>
  ),
  
  'Mocha Frappe': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M23 25 L23 49 Q23 53 27 53 L37 53 Q41 53 41 49 L41 25 Z" fill="rgba(80, 40, 20, 0.6)" stroke="currentColor" strokeWidth="2"/>
      {/* Whipped cream */}
      <path d="M23 25 Q27 19 32 21 Q37 19 41 25" fill="rgba(255, 248, 220, 0.95)" stroke="currentColor" strokeWidth="1.5"/>
      <circle cx="28" cy="21" r="2" fill="white"/>
      <circle cx="36" cy="21" r="2" fill="white"/>
      <circle cx="32" cy="19" r="2.5" fill="white"/>
      {/* Chocolate drizzle */}
      <path d="M27 27 Q30 29 32 27 Q34 29 37 27" stroke="#3E2723" strokeWidth="1.5" opacity="0.9"/>
      {/* Straw */}
      <rect x="37" y="13" width="2" height="17" rx="1" fill="currentColor" opacity="0.5"/>
    </svg>
  ),
  
  'White Mocha': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M17 27 L47 27 L47 49 Q47 53 43 53 L21 53 Q17 53 17 49 Z" fill="rgba(139, 69, 19, 0.25)" stroke="currentColor" strokeWidth="2"/>
      {/* White chocolate layer */}
      <rect x="18" y="37" width="28" height="8" fill="rgba(255, 250, 250, 0.9)"/>
      {/* Coffee layer */}
      <rect x="17" y="45" width="30" height="8" fill="rgba(101, 67, 33, 0.35)"/>
      {/* White chocolate chips */}
      <circle cx="25" cy="33" r="1.5" fill="white" stroke="#F5F5DC" strokeWidth="0.5"/>
      <circle cx="32" cy="31" r="2" fill="white" stroke="#F5F5DC" strokeWidth="0.5"/>
      <circle cx="39" cy="33" r="1.5" fill="white" stroke="#F5F5DC" strokeWidth="0.5"/>
      {/* Handle */}
      <path d="M49 33 Q53 33 53 37 L53 41 Q53 45 49 45" stroke="currentColor" strokeWidth="2"/>
      {/* Steam */}
      <path d="M25 19 Q27 15 25 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.4"/>
      <path d="M39 19 Q37 15 39 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" opacity="0.4"/>
    </svg>
  ),
  
  // Iced drinks
  'Iced Americano': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M21 25 L21 51 L43 51 L43 25 L21 25" fill="rgba(101, 67, 33, 0.2)" stroke="currentColor" strokeWidth="2"/>
      <path d="M21 25 L25 21 L39 21 L43 25" stroke="currentColor" strokeWidth="2"/>
      {/* Ice cubes */}
      <rect x="25" y="29" width="5" height="5" rx="1" fill="rgba(173, 216, 230, 0.85)" stroke="currentColor" strokeWidth="0.5"/>
      <rect x="33" y="27" width="5" height="5" rx="1" fill="rgba(173, 216, 230, 0.85)" stroke="currentColor" strokeWidth="0.5"/>
      <rect x="27" y="37" width="5" height="5" rx="1" fill="rgba(173, 216, 230, 0.85)" stroke="currentColor" strokeWidth="0.5"/>
      <rect x="35" y="39" width="5" height="5" rx="1" fill="rgba(173, 216, 230, 0.85)" stroke="currentColor" strokeWidth="0.5"/>
      {/* Straw */}
      <rect x="37" y="13" width="2" height="21" rx="1" fill="currentColor" opacity="0.5"/>
      <circle cx="38" cy="13" r="1.5" fill="currentColor" opacity="0.3"/>
    </svg>
  ),
  
  'Iced Latte': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M21 25 L21 51 L43 51 L43 25 L21 25" fill="rgba(245, 222, 179, 0.3)" stroke="currentColor" strokeWidth="2"/>
      <path d="M21 25 L25 21 L39 21 L43 25" stroke="currentColor" strokeWidth="2"/>
      {/* Milk layer */}
      <rect x="21" y="25" width="22" height="11" fill="rgba(255, 248, 220, 0.6)"/>
      {/* Coffee layer */}
      <rect x="21" y="36" width="22" height="15" fill="rgba(139, 69, 19, 0.4)"/>
      {/* Ice cubes */}
      <rect x="27" y="27" width="4" height="5" rx="1" fill="rgba(173, 216, 230, 0.8)" stroke="currentColor" strokeWidth="0.5"/>
      <rect x="34" y="31" width="4" height="5" rx="1" fill="rgba(173, 216, 230, 0.8)" stroke="currentColor" strokeWidth="0.5"/>
      <rect x="29" y="41" width="3" height="4" rx="1" fill="rgba(173, 216, 230, 0.7)" stroke="currentColor" strokeWidth="0.5"/>
      {/* Straw */}
      <rect x="37" y="15" width="2" height="19" rx="1" fill="currentColor" opacity="0.5"/>
    </svg>
  ),
  
  'Iced Spanish Latte': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M21 25 L21 51 L43 51 L43 25 L21 25" fill="rgba(245, 222, 179, 0.3)" stroke="currentColor" strokeWidth="2"/>
      <path d="M21 25 L25 21 L39 21 L43 25" stroke="currentColor" strokeWidth="2"/>
      {/* Condensed milk layer */}
      <rect x="22" y="45" width="20" height="5" fill="rgba(255, 228, 181, 0.9)"/>
      {/* Coffee layer */}
      <rect x="21" y="37" width="22" height="8" fill="rgba(139, 69, 19, 0.45)"/>
      {/* Ice cubes */}
      <rect x="27" y="27" width="4" height="5" rx="1" fill="rgba(173, 216, 230, 0.8)" stroke="currentColor" strokeWidth="0.5"/>
      <rect x="35" y="29" width="4" height="5" rx="1" fill="rgba(173, 216, 230, 0.8)" stroke="currentColor" strokeWidth="0.5"/>
      {/* Caramel swirl */}
      <path d="M23 41 Q32 43 41 41" stroke="#FFD700" strokeWidth="1.5" opacity="0.7"/>
      {/* Straw */}
      <rect x="37" y="15" width="2" height="19" rx="1" fill="currentColor" opacity="0.5"/>
    </svg>
  ),
  
  // Iced Teas
  'Passion Fruit Iced Tea': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M21 25 L21 51 L43 51 L43 25 L21 25" fill="rgba(255, 105, 180, 0.25)" stroke="currentColor" strokeWidth="2"/>
      <path d="M21 25 L25 21 L39 21 L43 25" stroke="currentColor" strokeWidth="2"/>
      {/* Ice cubes */}
      <rect x="27" y="29" width="4" height="5" rx="1" fill="rgba(173, 216, 230, 0.75)" stroke="currentColor" strokeWidth="0.5"/>
      <rect x="35" y="33" width="4" height="5" rx="1" fill="rgba(173, 216, 230, 0.75)" stroke="currentColor" strokeWidth="0.5"/>
      {/* Passion fruit seeds */}
      <circle cx="27" cy="39" r="1" fill="#FFD700"/>
      <circle cx="31" cy="41" r="1" fill="#FFD700"/>
      <circle cx="35" cy="43" r="1" fill="#FFD700"/>
      <circle cx="37" cy="41" r="1" fill="#FFD700"/>
      {/* Fruit slice */}
      <circle cx="32" cy="45" r="4" fill="rgba(255, 69, 142, 0.5)" stroke="#FF1493" strokeWidth="1"/>
      <path d="M32 41 L32 49 M28 45 L36 45" stroke="#FF1493" strokeWidth="0.5"/>
      {/* Straw */}
      <rect x="37" y="15" width="2" height="19" rx="1" fill="#FF69B4" opacity="0.6"/>
    </svg>
  ),
  
  'Peach Iced Tea': (
    <svg viewBox="0 0 64 64" fill="none">
      {/* Cup */}
      <path d="M21 25 L21 51 L43 51 L43 25 L21 25" fill="rgba(255, 218, 185, 0.5)" stroke="currentColor" strokeWidth="2"/>
      <path d="M21 25 L25 21 L39 21 L43 25" stroke="currentColor" strokeWidth="2"/>
      {/* Ice cubes */}
      <rect x="27" y="29" width="4" height="5" rx="1" fill="rgba(173, 216, 230, 0.75)" stroke="currentColor" strokeWidth="0.5"/>
      <rect x="35" y="33" width="4" height="5" rx="1" fill="rgba(173, 216, 230, 0.75)" stroke="currentColor" strokeWidth="0.5"/>
      {/* Peach slice */}
      <circle cx="32" cy="43" r="5" fill="rgba(255, 160, 122, 0.7)" stroke="#FF8C00" strokeWidth="1.5"/>
      <path d="M32 38 L32 48" stroke="#FF8C00" strokeWidth="1" opacity="0.7"/>
      <ellipse cx="32" cy="43" rx="2" ry="4" fill="rgba(255, 218, 185, 0.6)"/>
      {/* Leaf */}
      <path d="M37 23 Q39 19 41 23 Q39 21 37 23" fill="rgba(34, 139, 34, 0.7)" stroke="#228B22" strokeWidth="0.5"/>
      {/* Straw */}
      <rect x="36" y="15" width="2" height="19" rx="1" fill="#FFA500" opacity="0.6"/>
    </svg>
  ),
  
  // Default for any unmatched drink
  'default': (
    <svg viewBox="0 0 64 64" fill="none">
      <path d="M17 27 L47 27 L47 49 Q47 53 43 53 L21 53 Q17 53 17 49 Z" fill="rgba(139, 69, 19, 0.25)" stroke="currentColor" strokeWidth="2"/>
      <path d="M49 33 Q53 33 53 37 L53 41 Q53 45 49 45" stroke="currentColor" strokeWidth="2"/>
      <path d="M21 21 Q25 14 32 14 Q39 14 43 21" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
      <circle cx="32" cy="40" r="3" fill="currentColor" opacity="0.3"/>
    </svg>
  )
};

export default function DrinkIcon({ drinkName, className = "icon" }) {
  // Get the icon for this drink, or use default
  const icon = DrinkIcons[drinkName] || DrinkIcons['default'];
  
  return (
    <div className={className}>
      {icon}
    </div>
  );
}
