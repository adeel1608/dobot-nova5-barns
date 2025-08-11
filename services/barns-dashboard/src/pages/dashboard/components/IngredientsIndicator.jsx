import React, { useEffect, useState } from 'react';
import { useInventoryStore } from '../../../store/inventoryStore';
import { CATEGORY_INFO } from '../../../utils/inventoryData';
import socket from '../../../utils/socketConfigure';

import milk from '../../../assets/milk.png';
import coffee_beans from '../../../assets/beans.png';
import syrup from '../../../assets/syrup.png';
import cup from '../../../assets/cup.png';
import defaultImg from '../../../assets/default.png';
import sauces from '../../../assets/sauce.png';
import premixes from '../../../assets/premixes.png';
import useStore from '../../../store';

const IngredientsIndicator = () => {
  const {
    categorySummary,
    updateCategorySummary,
    fetchInventoryStatus,
  } = useInventoryStore();

  const { navigateToTab } = useStore();
  const [isSocketConnected, setSocketConnected] = useState(socket.connected);

  useEffect(() => {
    fetchInventoryStatus();

    const handleInventorySummary = (data) => {
      if (!data || typeof data !== 'object' || !data.summary) {
        console.warn('Invalid inventory.summary event payload:', data);
        return;
      }

      // console.log('✅ Received inventory summary:', data);
      const mapped = transformSummary(data.summary);

      //  console.log('✅maped data from socketio:', mapped);
      updateSummaryFromSocket(data.summary);
    };

    const handleConnect = () => {
      // console.log('🟢 Socket connected');
      setSocketConnected(true);
      socket.on('inventory.summary', handleInventorySummary);
    };

    const handleDisconnect = () => {
      // console.log('🔴 Socket disconnected');
      setSocketConnected(false);
      socket.off('inventory.summary', handleInventorySummary);
    };

    socket.on('connect', handleConnect);
    socket.on('disconnect', handleDisconnect);

    if (socket.connected) {
      handleConnect();
    }

    return () => {
      socket.off('connect', handleConnect);
      socket.off('disconnect', handleDisconnect);
      socket.off('inventory.summary', handleInventorySummary);
    };
  }, []);

  const transformSummary = (summary) => {
    return {
      milk: {
        level: summary.milk?.status || 'unknown',
        numeric: summary.milk?.percentage || 0,
        last_refilled: summary.milk?.last_updated || null,
        lowest_subtype: summary.milk?.lowest_subtype || ''
      },
      coffee_beans: {
        level: summary.coffee_beans?.status || 'unknown',
        numeric: summary.coffee_beans?.percentage || 0,
        last_refilled: summary.coffee_beans?.last_updated || null,
        lowest_subtype: summary.coffee_beans?.lowest_subtype || ''
      },
      syrups: {
        level: summary.syrups?.status || 'unknown',
        numeric: summary.syrup?.percentage || 0,
        last_refilled: summary.syrup?.last_updated || null,
        lowest_subtype: summary.syrup?.lowest_subtype || ''
      },
      premixes: {
        level: summary.premixes?.status || 'unknown',
        numeric: summary.premixes?.percentage || 0,
        last_refilled: summary.premixes?.last_updated || null,
        lowest_subtype: summary.premixes?.lowest_subtype || ''
      },
      sauces: {
        level: summary.sauces?.status || 'unknown',
        numeric: summary.sauces?.percentage || 0,
        last_refilled: summary.sauces?.last_updated || null,
        lowest_subtype: summary.sauces?.lowest_subtype || ''
      },
      cups: {
        level: summary.cups?.status || 'unknown',
        numeric: summary.cups?.percentage || 0,
        last_refilled: summary.cups?.last_updated || null,
        lowest_subtype: summary.cups?.lowest_subtype || ''
      }
    };
  };

  const updateSummaryFromSocket = (updatedData) => {
    console.log('🧠 Updating category summary state:', updatedData);
    useInventoryStore.setState((state) => ({
      categorySummary: {
        ...state.categorySummary,
        ...updatedData
      }
    }));
  };

  const handleNavigate = () => {
    navigateToTab('inventory');
    window.location.hash = '#/inventory';
  };

  const getProgressColor = (level, numeric) => {
    if (level === 'low' || numeric < 20) return 'text-red-500';
    if (level === 'medium' || numeric < 60) return 'text-yellow-500';
    return 'text-green-500';
  };

  const getStatusBadge = (level) => {
    switch (level) {
      case 'low': return 'bg-red-100 text-red-800';
      case 'medium': return 'bg-yellow-100 text-yellow-800';
      case 'high': return 'bg-green-100 text-green-800';
      default: return 'bg-gray-100 text-gray-800';
    }
  };

  const iconMap = { milk, coffee_beans, syrup: syrup, cups: cup, sauces, premixes };

  const categories = [
    { key: 'milk', title: CATEGORY_INFO.milk.title, data: categorySummary.milk },
    { key: 'coffee_beans', title: CATEGORY_INFO.coffee_beans.title, data: categorySummary.coffee_beans },
    { key: 'syrup', title: CATEGORY_INFO.syrups.title, data: categorySummary.syrups },
    { key: 'cups', title: CATEGORY_INFO.cups.title, data: categorySummary.cups },
    { key: 'sauces', title: CATEGORY_INFO.sauces.title, data: categorySummary.sauces },
    { key: 'premixes', title: CATEGORY_INFO.premixes.title, data: categorySummary.premixes },
  ];

  return (
    <div className="bg-white rounded-lg shadow-md p-4">
      <div className="flex items-center justify-between mb-4 border-b border-gray-200">
        <h3 className="text-lg font-semibold text-gray-900 flex items-center gap-2">
          Inventory
        </h3>
        <span
          className={`w-3 h-3 rounded-full ${isSocketConnected ? 'bg-green-500' : 'bg-red-500'}`}
          title={isSocketConnected ? 'Connected' : 'Disconnected'}
        ></span>
      </div>

      {/* Inventory Items with Progress Bars - Vertical List */}
      <div className="space-y-4">
        {categories.map(({ key, title, data }) => {
          const level = data?.status || 'unknown';
          const numeric = data?.percentage || 0;
          const percentage = Math.max(0, Math.min(100, numeric));
          
          // Get progress bar color based on stock level
          const getProgressBarColor = (level, numeric) => {
            if (level === 'low' || numeric < 20) return 'bg-red-500';
            if (level === 'medium' || numeric < 60) return 'bg-yellow-500';
            return 'bg-green-800';
          };

          return (
            <div
              key={key}
              className="cursor-pointer"
              onClick={handleNavigate}
            >
              {/* Item Row with Percentage */}
              <div className="flex items-center justify-between mb-2">
                <span className="text-sm font-medium text-gray-700">
                  {data?.lowest_subtype
                    ? data.lowest_subtype.charAt(0).toUpperCase() + data.lowest_subtype.slice(1)
                    : title}
                </span>
                <span className="text-sm font-semibold text-gray-700">
                  {percentage}%
                </span>
              </div>
              
              {/* Progress Bar Below Item */}
              <div className="w-full bg-gray-200 rounded-full h-2">
                <div
                  className={`h-2 rounded-full transition-all duration-300 ${getProgressBarColor(level, numeric)}`}
                  style={{ width: `${percentage}%` }}
                ></div>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
};

export default IngredientsIndicator;
