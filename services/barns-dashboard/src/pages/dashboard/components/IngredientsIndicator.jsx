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

      <div className="grid grid-cols-2 lg:grid-cols-3 gap-4">
        {categories.map(({ key, title, data }) => {
          const level = data?.status || 'unknown';
          const numeric = data?.percentage || 0;
          const percentage = Math.max(0, Math.min(100, numeric));
          const badgeClass = getStatusBadge(level);
          // console.log("special",data)
          return (
            <button
              key={key}
              onClick={handleNavigate}
              className="group flex flex-col items-center p-3 rounded-lg border border-gray-200 barns-bg-hover transition-all duration-200"
            >
              <img
                src={iconMap[key] || defaultImg}
                alt={title}
                className="w-8 h-8 mb-2"
                onError={(e) => {
                  e.target.onerror = null;
                  e.target.src = defaultImg;
                }}
              />

              {/* Lowest Subtype (e.g., almond, arabica) */}
              <span className="text-sm text-gray-700 text-center leading-tight font-semibold">
                {data?.lowest_subtype
                  ? data.lowest_subtype.charAt(0).toUpperCase() + data.lowest_subtype.slice(1)
                  : '—'}
              </span>

              <div className="mt-1 flex flex-col items-center">
                <span
                  className={`inline-flex items-center px-4 py-0.5 rounded text-xs font-medium ${badgeClass} font-semibold`}
                >
                  {level === 'unknown' ? 'Unknown' : level.charAt(0).toUpperCase() + level.slice(1)}
                </span>
                <span className={`text-xs font-bold mt-1 ${getProgressColor(level, numeric)}`}>
                  {percentage}%
                </span>
              </div>
            </button>
          );
        })}
      </div>
    </div>
  );
};

export default IngredientsIndicator;
