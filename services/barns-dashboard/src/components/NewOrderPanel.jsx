import React, { useState } from 'react';
import useStore from '../store';

export default function NewOrderPanel() {
  const { createOrder, isLoading } = useStore();
  const [orderData, setOrderData] = useState({
    cups: [
      {
        type: 'Latte',
        size: 'regular',
        addons: []
      }
    ]
  });
  const [showForm, setShowForm] = useState(false);

  const drinkTypes = ['Latte', 'Americano', 'Cappuccino', 'Espresso', 'Mocha'];
  const sizes = ['small', 'regular', 'large'];
  const addons = ['extra_shot', 'oat_milk', 'almond_milk', 'sugar_free', 'decaf'];

  const handleAddCup = () => {
    setOrderData(prev => ({
      ...prev,
      cups: [...prev.cups, { type: 'Latte', size: 'regular', addons: [] }]
    }));
  };

  const handleRemoveCup = (index) => {
    setOrderData(prev => ({
      ...prev,
      cups: prev.cups.filter((_, i) => i !== index)
    }));
  };

  const handleCupChange = (index, field, value) => {
    setOrderData(prev => ({
      ...prev,
      cups: prev.cups.map((cup, i) => 
        i === index ? { ...cup, [field]: value } : cup
      )
    }));
  };

  const handleAddonToggle = (cupIndex, addon) => {
    setOrderData(prev => ({
      ...prev,
      cups: prev.cups.map((cup, i) => {
        if (i === cupIndex) {
          const currentAddons = cup.addons || [];
          return {
            ...cup,
            addons: currentAddons.includes(addon)
              ? currentAddons.filter(a => a !== addon)
              : [...currentAddons, addon]
          };
        }
        return cup;
      })
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    if (orderData.cups.length === 0) {
      alert('Please add at least one drink to the order.');
      return;
    }

    const success = await createOrder(orderData);
    if (success) {
      // Reset form
      setOrderData({
        cups: [{ type: 'Latte', size: 'regular', addons: [] }]
      });
      setShowForm(false);
      alert('Order created successfully!');
    } else {
      alert('Failed to create order. Please check the connection and try again.');
    }
  };

  return (
    <div className="bg-white rounded shadow">
      <div className="flex items-center justify-between p-4 border-b border-gray-200">
        <h2 className="text-xl font-bold">New Order</h2>
        <button
          onClick={() => setShowForm(!showForm)}
          className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 text-sm font-medium"
        >
          {showForm ? 'Cancel' : 'Create Order'}
        </button>
      </div>

      {showForm && (
        <div className="p-4">
          <form onSubmit={handleSubmit} className="space-y-4">
            {/* Drinks */}
            <div>
              <div className="flex items-center justify-between mb-3">
                <h3 className="text-lg font-medium">Drinks</h3>
                <button
                  type="button"
                  onClick={handleAddCup}
                  className="px-3 py-1 bg-green-100 text-green-800 rounded text-sm hover:bg-green-200"
                >
                  + Add Drink
                </button>
              </div>

              <div className="space-y-3">
                {orderData.cups.map((cup, index) => (
                  <div key={index} className="p-3 border border-gray-200 rounded bg-gray-50">
                    <div className="flex items-center justify-between mb-2">
                      <h4 className="font-medium">Drink {index + 1}</h4>
                      {orderData.cups.length > 1 && (
                        <button
                          type="button"
                          onClick={() => handleRemoveCup(index)}
                          className="text-red-600 hover:text-red-800 text-sm"
                        >
                          Remove
                        </button>
                      )}
                    </div>

                    <div className="grid grid-cols-1 md:grid-cols-2 gap-3 mb-3">
                      {/* Drink Type */}
                      <div>
                        <label className="block text-sm font-medium text-gray-700 mb-1">
                          Drink Type
                        </label>
                        <select
                          value={cup.type}
                          onChange={(e) => handleCupChange(index, 'type', e.target.value)}
                          className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm"
                        >
                          {drinkTypes.map(type => (
                            <option key={type} value={type}>{type}</option>
                          ))}
                        </select>
                      </div>

                      {/* Size */}
                      <div>
                        <label className="block text-sm font-medium text-gray-700 mb-1">
                          Size
                        </label>
                        <select
                          value={cup.size}
                          onChange={(e) => handleCupChange(index, 'size', e.target.value)}
                          className="w-full px-3 py-2 border border-gray-300 rounded-md text-sm"
                        >
                          {sizes.map(size => (
                            <option key={size} value={size}>
                              {size.charAt(0).toUpperCase() + size.slice(1)}
                            </option>
                          ))}
                        </select>
                      </div>
                    </div>

                    {/* Add-ons */}
                    <div>
                      <label className="block text-sm font-medium text-gray-700 mb-2">
                        Add-ons
                      </label>
                      <div className="flex flex-wrap gap-2">
                        {addons.map(addon => (
                          <label key={addon} className="flex items-center">
                            <input
                              type="checkbox"
                              checked={(cup.addons || []).includes(addon)}
                              onChange={() => handleAddonToggle(index, addon)}
                              className="mr-1"
                            />
                            <span className="text-sm">{addon.replace('_', ' ')}</span>
                          </label>
                        ))}
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            </div>

            {/* Submit Button */}
            <div className="flex justify-end space-x-3 pt-4 border-t border-gray-200">
              <button
                type="button"
                onClick={() => setShowForm(false)}
                className="px-4 py-2 border border-gray-300 rounded-md text-gray-700 hover:bg-gray-50"
              >
                Cancel
              </button>
              <button
                type="submit"
                disabled={isLoading}
                className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed"
              >
                {isLoading ? 'Creating...' : 'Create Order'}
              </button>
            </div>
          </form>
        </div>
      )}

      {!showForm && (
        <div className="p-8 text-center text-gray-500">
          <svg className="mx-auto h-12 w-12 text-gray-400 mb-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6v6m0 0v6m0-6h6m-6 0H6" />
          </svg>
          <p className="text-lg font-medium mb-2">Create New Order</p>
          <p className="text-sm">Click "Create Order" to add a new order to the queue.</p>
        </div>
      )}
    </div>
  );
} 