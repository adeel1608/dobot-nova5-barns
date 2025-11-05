/**
 * MenuGrid Component
 * Displays menu items in a card-based grid layout with filtering
 */

import React from 'react';

export default function MenuGrid({
  drinkNames = [],
  menuItems = [],
  searchTerm = '',
  setSearchTerm = () => {},
  selectedCategory = 'all',
  setSelectedCategory = () => {},
  categories = ['all'],
  addToCart = () => {}
}) {
  // Get available sizes for a drink
  const getSizesForDrink = (drinkName) => {
    if (!menuItems || !Array.isArray(menuItems)) return [];
    return menuItems
      .filter(item => item.name === drinkName)
      .map(item => item.size)
      .filter((size, index, self) => self.indexOf(size) === index);
  };

  // Get first menu item for drink (for display purposes)
  const getMenuItemForDrink = (drinkName) => {
    if (!menuItems || !Array.isArray(menuItems)) return null;
    return menuItems.find(item => item.name === drinkName);
  };

  return (
    <div className="menu-grid-container">
      {/* Search and Filter Bar */}
      <div className="menu-controls">
        {/* Search Bar */}
        <div className="search-bar">
          <svg 
            className="search-icon" 
            fill="none" 
            stroke="currentColor" 
            viewBox="0 0 24 24"
          >
            <path 
              strokeLinecap="round" 
              strokeLinejoin="round" 
              strokeWidth={2} 
              d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" 
            />
          </svg>
          <input
            type="text"
            placeholder="Search drinks..."
            value={searchTerm}
            onChange={(e) => setSearchTerm(e.target.value)}
            className="search-input"
          />
          {searchTerm && (
            <button
              onClick={() => setSearchTerm('')}
              className="search-clear"
            >
              ×
            </button>
          )}
        </div>

        {/* Category Filters */}
        <div className="category-filters">
          {categories.map(category => (
            <button
              key={category}
              onClick={() => setSelectedCategory(category)}
              className={`category-btn ${selectedCategory === category ? 'active' : ''}`}
            >
              {category === 'all' ? 'All Drinks' : category}
            </button>
          ))}
        </div>
      </div>

      {/* Menu Grid */}
      <div className="menu-grid">
        {drinkNames.length === 0 ? (
          <div className="empty-state">
            <svg className="empty-icon" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.172 16.172a4 4 0 015.656 0M9 10h.01M15 10h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
            <p className="empty-text">No drinks found</p>
            <p className="empty-subtext">Try adjusting your search or category filter</p>
          </div>
        ) : (
          drinkNames.map(drinkName => {
            const sizes = getSizesForDrink(drinkName);
            const menuItem = getMenuItemForDrink(drinkName);
            const category = menuItem?.category || 'Other';

            return (
              <div key={drinkName} className="drink-card">
                {/* Drink Icon/Image Placeholder */}
                <div className="drink-icon">
                  {/* Coffee Cup Icon */}
                  <svg 
                    className="icon" 
                    fill="none" 
                    stroke="currentColor" 
                    viewBox="0 0 24 24"
                  >
                    <path 
                      strokeLinecap="round" 
                      strokeLinejoin="round" 
                      strokeWidth={2} 
                      d="M20.354 15.354A9 9 0 018.646 3.646 9.003 9.003 0 0012 21a9.003 9.003 0 008.354-5.646z" 
                    />
                  </svg>
                </div>

                {/* Drink Info */}
                <div className="drink-info">
                  <h3 className="drink-name">{drinkName}</h3>
                  <p className="drink-category">{category}</p>
                </div>

                {/* Size Selection */}
                <div className="size-selection">
                  <label className="size-label">Select Size:</label>
                  <div className="size-buttons">
                    {sizes.map(size => (
                      <button
                        key={size}
                        onClick={() => addToCart(drinkName, size)}
                        className="size-btn"
                        title={`Add ${drinkName} - ${size}`}
                      >
                        {size}
                      </button>
                    ))}
                  </div>
                </div>
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}

