/**
 * CompactMenuGrid
 * POS touch-screen drink selection grid.
 * All interactive targets meet 44px minimum touch area.
 */

import React from 'react';
import { useTranslation } from '../../../../store/translationsStore';
import DrinkIcon from '../DrinkIcon';

function CompactDrinkCard({ drinkName, sizes, addToCart }) {
  return (
    <div className="bg-white border-2 border-gray-100 rounded-2xl p-3.5 flex flex-col gap-3 hover:border-green-400 hover:shadow-sm active:scale-[0.98] transition-all duration-150 h-full">
      {/* Icon + name */}
      <div className="flex items-center gap-3 min-w-0 flex-1">
        <DrinkIcon drinkName={drinkName} className="w-10 h-10 text-green-700 flex-shrink-0" />
        <span className="text-lg font-bold text-gray-900 leading-snug line-clamp-2 min-w-0 mt-0.5">
          {drinkName}
        </span>
      </div>

      {/* Size buttons — min 44px tall for touch */}
      <div className="flex gap-2 overflow-x-auto pb-1 pt-px px-px" style={{ scrollbarWidth: 'none' }}>
        {sizes.map(size => (
          <button
            key={size}
            type="button"
            onClick={() => addToCart(drinkName, size)}
            className="flex-1 min-w-[56px] min-h-[44px] text-[15px] font-bold rounded-xl bg-green-50 text-green-800 hover:bg-green-600 hover:text-white active:bg-green-700 active:scale-95 transition-all duration-120 select-none flex items-center justify-center flex-shrink-0 shadow-sm"
            style={{ border: '2px solid #22c55e' }}
            onMouseEnter={e => e.currentTarget.style.border = '2px solid #16a34a'}
            onMouseLeave={e => e.currentTarget.style.border = '2px solid #22c55e'}
          >
            {size}
          </button>
        ))}
      </div>
    </div>
  );
}

export default function CompactMenuGrid({
  drinkNames = [],
  menuItems = [],
  searchTerm = '',
  setSearchTerm = () => { },
  selectedCategory = 'all',
  setSelectedCategory = () => { },
  categories = ['all'],
  addToCart = () => { }
}) {
  const { t } = useTranslation('newOrder');

  const getSizesForDrink = (name) => {
    if (!menuItems || !Array.isArray(menuItems)) return [];
    return [...new Set(menuItems.filter(i => i.name === name).map(i => i.size))];
  };

  return (
    <div className="flex flex-col h-full overflow-hidden">

      {/* Controls: search + category chips */}
      <div className="flex items-center gap-2 px-3 py-2.5 border-b border-gray-100 flex-shrink-0 bg-white">
        {/* Search — 44px tall */}
        <div className="relative flex-1 min-w-0">
          <svg
            className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-400 pointer-events-none"
            fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="2"
          >
            <path strokeLinecap="round" strokeLinejoin="round" d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
          </svg>
          <input
            type="text"
            placeholder={t('searchDrinks')}
            value={searchTerm}
            onChange={(e) => setSearchTerm(e.target.value)}
            className="w-full pl-9 pr-8 py-2.5 text-sm border-2 border-gray-200 rounded-xl focus:outline-none focus:border-green-500 focus:ring-2 focus:ring-green-100 bg-gray-50 transition-colors"
          />
          {searchTerm && (
            <button
              type="button"
              onClick={() => setSearchTerm('')}
              className="absolute right-2.5 top-1/2 -translate-y-1/2 w-6 h-6 rounded-full bg-gray-300 text-white flex items-center justify-center hover:bg-gray-500 transition-colors text-sm leading-none"
              aria-label="Clear search"
            >
              ×
            </button>
          )}
        </div>

        {/* Category chips — 40px tall */}
        <div
          className="flex items-center gap-1.5 overflow-x-auto flex-shrink-0 max-w-[45%]"
          style={{ scrollbarWidth: 'none' }}
        >
          {categories.map(cat => (
            <button
              key={cat}
              type="button"
              onClick={() => setSelectedCategory(cat)}
              className={`flex-shrink-0 text-xs font-bold px-3.5 py-2 rounded-xl border-2 transition-all duration-150 whitespace-nowrap min-h-[40px] focus:outline-none active:scale-95 ${selectedCategory === cat
                ? 'bg-green-600 text-white border-green-600 shadow-sm'
                : 'bg-white text-gray-600 border-gray-200 hover:border-green-400 hover:text-green-700'
                }`}
            >
              {cat === 'all' ? t('allDrinks') : cat}
            </button>
          ))}
        </div>
      </div>

      {/* Drink grid */}
      <div
        className="flex-1 overflow-y-auto p-2.5"
        style={{ scrollbarWidth: 'thin', scrollbarColor: '#bbf7d0 transparent' }}
      >
        {drinkNames.length === 0 ? (
          <div className="flex flex-col items-center justify-center h-full gap-3">
            <svg className="w-12 h-12 text-gray-200" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="1.5">
              <path strokeLinecap="round" strokeLinejoin="round" d="M9.172 16.172a4 4 0 015.656 0M9 10h.01M15 10h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
            <p className="text-sm font-medium text-gray-400">{t('noDrinksFound')}</p>
            {searchTerm && (
              <button
                type="button"
                onClick={() => setSearchTerm('')}
                className="text-xs text-green-600 font-semibold py-2 px-4 rounded-xl border border-green-300 hover:bg-green-50"
              >
                Clear search
              </button>
            )}
          </div>
        ) : (
          <div
            className="grid gap-2"
            style={{ gridTemplateColumns: 'repeat(auto-fill, minmax(220px, 1fr))' }}
          >
            {drinkNames.map(drinkName => (
              <CompactDrinkCard
                key={drinkName}
                drinkName={drinkName}
                sizes={getSizesForDrink(drinkName)}
                addToCart={addToCart}
              />
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
