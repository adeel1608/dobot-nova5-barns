import React, { useState, useEffect } from 'react';
import { inventoryAPI } from '../../../api';
import { apiClient } from '../../../api/base';
import { useTranslation } from '../../../store/translationsStore';

export default function IngredientSettings() {
  const { t } = useTranslation('settings');
  const [ingredientData, setIngredientData] = useState({});
  const [editedValues, setEditedValues] = useState({});
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [selectedCategory, setSelectedCategory] = useState('all');
  const [searchTerm, setSearchTerm] = useState('');
  const [hasChanges, setHasChanges] = useState(false);
  const [notification, setNotification] = useState(null);

  // Load ingredient data with max capacity info
  useEffect(() => {
    loadIngredientData();
  }, []);

  const showNotification = (message, type = 'success') => {
    setNotification({ message, type });
    // Use longer timeout for longer messages
    const timeout = message.length > 80 ? 5000 : 3000;
    setTimeout(() => setNotification(null), timeout);
  };

  const loadIngredientData = async () => {
    setLoading(true);
    try {
      const result = await inventoryAPI.fetchInventoryStatus();
      
      if (result.success) {
        setIngredientData(result.data || {});
      } else {
        console.error('Failed to load ingredient data:', result.error);
        showNotification('Failed to load ingredient data', 'error');
      }
    } catch (error) {
      console.error('Error loading ingredient data:', error);
      showNotification('Error loading ingredient data', 'error');
    } finally {
      setLoading(false);
    }
  };

  const handleValueChange = (category, subtype, field, value) => {
    const key = `${category}.${subtype}.${field}`;
    const numValue = parseFloat(value);
    
    if (isNaN(numValue) || numValue < 0) {
      return;
    }

    setEditedValues(prev => ({
      ...prev,
      [key]: numValue
    }));
    setHasChanges(true);
  };

  const handleSave = async () => {
    if (!hasChanges) return;

    setSaving(true);
    try {
      const updates = [];
      
      // Process all edited values
      Object.entries(editedValues).forEach(([key, value]) => {
        const [category, subtype, field] = key.split('.');
        updates.push({ category, subtype, field, value });
      });

      // Call API to update values using the API client
      const result = await apiClient.post('/inventory/update-limits', { updates }, {
        successMessage: 'Ingredient limits updated successfully'
      });

      if (result.success) {
        // Check if any current amounts were adjusted
        const adjustedItems = result.data?.details?.updated_items?.filter(item => item.current_amount_adjusted) || [];
        
        if (adjustedItems.length > 0) {
          const itemNames = adjustedItems.map(item => `${item.category}:${item.subtype}`).join(', ');
          showNotification(
            `Limits updated! Note: Current amounts were automatically adjusted for: ${itemNames} (to match new max capacity)`, 
            'success'
          );
        } else {
          showNotification('Ingredient limits updated successfully', 'success');
        }
        
        setEditedValues({});
        setHasChanges(false);
        await loadIngredientData();
      } else {
        showNotification(result.error || 'Failed to update ingredient limits', 'error');
      }
    } catch (error) {
      console.error('Error saving changes:', error);
      showNotification('Error saving changes: ' + (error.message || 'Unknown error'), 'error');
    } finally {
      setSaving(false);
    }
  };

  const handleReset = () => {
    setEditedValues({});
    setHasChanges(false);
  };

  const getDisplayValue = (category, subtype, field) => {
    const key = `${category}.${subtype}.${field}`;
    if (editedValues[key] !== undefined) {
      return editedValues[key];
    }
    return ingredientData[category]?.[subtype]?.[field] || 0;
  };

  const getUnit = (category) => {
    if (category === 'cups') return 'pcs';
    if (category === 'coffee_beans') return 'g';
    return 'ml';
  };

  // Filter categories based on selection and search
  const getFilteredData = () => {
    let filtered = { ...ingredientData };

    if (selectedCategory !== 'all') {
      filtered = { [selectedCategory]: ingredientData[selectedCategory] };
    }

    if (searchTerm) {
      const search = searchTerm.toLowerCase();
      Object.keys(filtered).forEach(category => {
        const subtypes = filtered[category];
        const matchingSubtypes = {};
        
        Object.keys(subtypes).forEach(subtype => {
          if (
            subtype.toLowerCase().includes(search) ||
            category.toLowerCase().includes(search)
          ) {
            matchingSubtypes[subtype] = subtypes[subtype];
          }
        });

        if (Object.keys(matchingSubtypes).length > 0) {
          filtered[category] = matchingSubtypes;
        } else {
          delete filtered[category];
        }
      });
    }

    return filtered;
  };

  const categories = [
    { id: 'all', nameKey: 'allCategories' },
    { id: 'coffee_beans', nameKey: 'coffeeBeans' },
    { id: 'cups', nameKey: 'cups' },
    { id: 'milk', nameKey: 'milk' },
    { id: 'syrups', nameKey: 'syrups' },
    { id: 'premixes', nameKey: 'premixes' }
  ];

  const formatName = (str) => {
    return str
      .replace(/_/g, ' ')
      .replace(/\b\w/g, l => l.toUpperCase());
  };

  const getCategoryIcon = (category) => {
    const icons = {
      coffee_beans: '☕',
      cups: '🥤',
      milk: '🥛',
      syrups: '🍯',
      premixes: '🧋'
    };
    return icons[category] || '📦';
  };

  const filteredData = getFilteredData();

  if (loading) {
    return (
      <div className="p-6 bg-white rounded-xl shadow-sm border border-gray-200">
        <div className="flex items-center justify-center py-12">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-600"></div>
          <span className="ml-3 text-gray-600">{t('ingredientsLoading')}</span>
        </div>
      </div>
    );
  }

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-200 flex flex-col" style={{ height: 'calc(100vh - 270px)' }}>
      {/* Notification Toast */}
      {notification && (
        <div className={`fixed top-4 right-4 z-50 px-6 py-3 rounded-lg shadow-lg max-w-md ${
          notification.type === 'success' 
            ? 'bg-green-500 text-white' 
            : 'bg-red-500 text-white'
        }`}>
          <div className="flex items-start space-x-2">
            <div className="flex-shrink-0 mt-0.5">
              {notification.type === 'success' ? (
                <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
                  <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
                </svg>
              ) : (
                <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
                  <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clipRule="evenodd" />
                </svg>
              )}
            </div>
            <span className="font-medium text-sm leading-relaxed">{notification.message}</span>
          </div>
        </div>
      )}
      
      {/* Header - Fixed */}
      <div className="p-6 pb-0 flex-shrink-0">
        <div className="flex items-center justify-between mb-4">
          <div>
            <h2 className="text-xl font-bold text-gray-900">{t('ingredientsTitle')}</h2>
            <p className="text-sm text-gray-600 mt-1">
              {t('ingredientsSubtitle')}
            </p>
          </div>
          
          {hasChanges && (
            <div className="flex items-center space-x-3">
              <button
                onClick={handleReset}
                disabled={saving}
                className="px-4 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-lg hover:bg-gray-50 transition-colors disabled:opacity-50"
              >
                {t('reset')}
              </button>
              <button
                onClick={handleSave}
                disabled={saving}
                className="px-4 py-2 text-sm font-medium text-white bg-blue-600 rounded-lg hover:bg-blue-700 transition-colors disabled:opacity-50 flex items-center"
              >
                {saving ? (
                  <>
                    <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-white mr-2"></div>
                    {t('saving')}
                  </>
                ) : (
                  <>
                    <svg className="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
                    </svg>
                    {t('saveChanges')}
                  </>
                )}
              </button>
            </div>
          )}
        </div>

        {/* Filters */}
        <div className="flex flex-col lg:flex-row gap-3">
          <div className="relative flex-1">
            <input
              type="text"
              placeholder={t('searchPlaceholder')}
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="w-full pl-10 pr-4 py-2 bg-white border border-gray-300 rounded-lg text-gray-900 placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
            />
            <svg className="w-5 h-5 absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
            </svg>
          </div>
          
          <select
            value={selectedCategory}
            onChange={(e) => setSelectedCategory(e.target.value)}
            className="px-4 py-2 bg-white border border-gray-300 rounded-lg text-gray-900 focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            {categories.map(cat => (
              <option key={cat.id} value={cat.id}>{t(cat.nameKey)}</option>
            ))}
          </select>
        </div>
      </div>

      {/* Ingredient Table - Scrollable */}
      <div className="flex-1 overflow-y-auto px-6 pb-6" style={{ scrollbarWidth: 'thin', scrollbarColor: '#cbd5e1 #f1f5f9', marginTop: '10px'}}>
        {Object.keys(filteredData).length === 0 ? (
          <div className="text-center py-12 text-gray-500">
            <svg className="w-16 h-16 mx-auto mb-4 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 13V6a2 2 0 00-2-2H6a2 2 0 00-2 2v7m16 0v5a2 2 0 01-2 2H6a2 2 0 01-2-2v-5m16 0h-2.586a1 1 0 00-.707.293l-2.414 2.414a1 1 0 01-.707.293h-3.172a1 1 0 01-.707-.293l-2.414-2.414A1 1 0 006.586 13H4" />
            </svg>
            <p className="text-lg font-medium mb-2">{t('noIngredientsFound')}</p>
            <p className="text-sm">{t('tryCriteria')}</p>
          </div>
        ) : (
            <div className="space-y-6">
            {Object.entries(filteredData).map(([category, subtypes]) => {
              // Add null check for subtypes
              if (!subtypes || typeof subtypes !== 'object') {
                return null;
              }
              
              return (
              <div key={category} className="border border-gray-200 rounded-lg overflow-hidden">
                {/* Category Header */}
                <div className="bg-gradient-to-r from-gray-50 to-gray-100 px-4 py-3 border-b border-gray-200">
                  <div className="flex items-center">
                    <span className="text-2xl mr-3">{getCategoryIcon(category)}</span>
                    <div>
                      <h3 className="text-lg font-semibold text-gray-900">{formatName(category)}</h3>
                      <p className="text-xs text-gray-600">{Object.keys(subtypes).length} {t('itemsCount')}</p>
                    </div>
                  </div>
                </div>

                {/* Items Table */}
                <div className="overflow-x-auto">
                  <table className="w-full">
                    <thead className="bg-gray-50 border-b border-gray-200">
                      <tr>
                        <th className="px-4 py-3 text-left text-xs font-medium text-gray-600 uppercase tracking-wider">
                          {t('ingredient')}
                        </th>
                        <th className="px-4 py-3 text-left text-xs font-medium text-gray-600 uppercase tracking-wider">
                          {t('currentAmount')}
                        </th>
                        <th className="px-4 py-3 text-left text-xs font-medium text-gray-600 uppercase tracking-wider">
                          {t('maxCapacity')}
                        </th>
                        <th className="px-4 py-3 text-left text-xs font-medium text-gray-600 uppercase tracking-wider">
                          {t('warningThreshold')}
                        </th>
                        <th className="px-4 py-3 text-left text-xs font-medium text-gray-600 uppercase tracking-wider">
                          {t('criticalThreshold')}
                        </th>
                      </tr>
                    </thead>
                    <tbody className="bg-white divide-y divide-gray-200">
                      {Object.entries(subtypes).map(([subtype, data]) => {
                        // Add null check for data
                        if (!data || typeof data !== 'object') {
                          return null;
                        }
                        const unit = getUnit(category);
                        const isEdited = editedValues[`${category}.${subtype}.max_capacity`] !== undefined;
                        
                        return (
                          <tr key={subtype} className={`hover:bg-gray-50 transition-colors ${isEdited ? 'bg-blue-50' : ''}`}>
                            <td className="px-4 py-3 whitespace-nowrap">
                              <div className="text-sm font-medium text-gray-900">
                                {formatName(subtype)}
                              </div>
                            </td>
                            <td className="px-4 py-3 whitespace-nowrap">
                              <div className="text-sm text-gray-600">
                                {data.amount?.toFixed(2) || 0} {unit}
                              </div>
                              <div className="text-xs text-gray-500">
                                {data.percentage || 0}%
                              </div>
                            </td>
                            <td className="px-4 py-3 whitespace-nowrap">
                              <div className="flex items-center space-x-2">
                                <input
                                  type="number"
                                  min="0"
                                  step="1"
                                  value={getDisplayValue(category, subtype, 'max_capacity')}
                                  onChange={(e) => handleValueChange(category, subtype, 'max_capacity', e.target.value)}
                                  className="w-28 px-3 py-1.5 text-sm border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                                />
                                <span className="text-sm text-gray-600">{unit}</span>
                              </div>
                            </td>
                            <td className="px-4 py-3 whitespace-nowrap">
                              <div className="flex items-center space-x-2">
                                <input
                                  type="number"
                                  min="0"
                                  step="1"
                                  value={getDisplayValue(category, subtype, 'warning_threshold')}
                                  onChange={(e) => handleValueChange(category, subtype, 'warning_threshold', e.target.value)}
                                  className="w-28 px-3 py-1.5 text-sm border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                                />
                                <span className="text-sm text-gray-600">{unit}</span>
                              </div>
                            </td>
                            <td className="px-4 py-3 whitespace-nowrap">
                              <div className="flex items-center space-x-2">
                                <input
                                  type="number"
                                  min="0"
                                  step="1"
                                  value={getDisplayValue(category, subtype, 'critical_threshold')}
                                  onChange={(e) => handleValueChange(category, subtype, 'critical_threshold', e.target.value)}
                                  className="w-28 px-3 py-1.5 text-sm border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                                />
                                <span className="text-sm text-gray-600">{unit}</span>
                              </div>
                            </td>
                          </tr>
                        );
                      })}
                    </tbody>
                  </table>
                </div>
              </div>
            );
            })}
          </div>
        )}
      </div>


    </div>
  );
}

