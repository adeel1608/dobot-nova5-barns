import React from 'react';
import useStore from '../../../store';

export default function OrderDetails() {
  const { orders } = useStore();

  // Find the currently processing order
  const processingOrder = orders?.find(order => 
    order.status?.toUpperCase() === 'PROCESSING'
  );

  if (!processingOrder) {
    return (
      <div className="bg-white rounded-lg shadow-sm border border-gray-200 flex flex-col h-full">
        {/* Header - Responsive */}
        <div className="p-2 md:p-3 border-b border-gray-200 flex-shrink-0">
          <h2 className="text-base md:text-lg font-semibold text-gray-900">Current Order</h2>
        </div>

        {/* No Processing Order - Responsive */}
        <div className="flex-1 flex items-center justify-center text-gray-500 p-3 md:p-4">
          <div className="text-center">
            <svg className="w-8 md:w-12 h-8 md:h-12 text-gray-300 mx-auto mb-2 md:mb-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5H7a2 2 0 00-2 2v10a2 2 0 002 2h8a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2" />
            </svg>
            <p className="text-sm font-medium">No Order Processing</p>
            <p className="text-xs text-gray-400 mt-1">System is idle</p>
          </div>
        </div>
      </div>
    );
  }

  // Format order data for display
  const formatOrderForDisplay = (order) => {
    return {
      ...order,
      itemName: order.cups && order.cups.length > 0 
        ? order.cups.map(cup => `${cup.drink_type || cup.type} (${cup.cup_size || cup.size})`).join(', ')
        : 'Unknown Item',
      createdAt: order.created_at ? new Date(order.created_at).toLocaleString() : 'N/A',
      startedAt: order.started_at ? new Date(order.started_at).toLocaleString() : 'N/A'
    };
  };

  const displayOrder = formatOrderForDisplay(processingOrder);

  return (
    <div className="bg-white rounded-lg shadow-sm border border-gray-200 flex flex-col h-full">
      {/* Header - Responsive */}
      <div className="p-2 md:p-3 border-b border-gray-200 flex-shrink-0">
        <div className="flex flex-col sm:flex-row sm:items-center justify-between space-y-2 sm:space-y-0">
          <h2 className="text-base md:text-lg font-semibold text-gray-900">Current Order</h2>
          <div className="flex items-center space-x-2">
            <div className="w-2 h-2 bg-yellow-400 rounded-full animate-pulse"></div>
            <span className="text-xs font-medium text-yellow-600">Processing</span>
          </div>
        </div>
      </div>

      {/* Order Content - Responsive */}
      <div className="flex-1 p-2 md:p-3 space-y-2 md:space-y-3">
        {/* Order Header */}
        <div className="bg-yellow-50 p-2 md:p-3 rounded-lg border border-yellow-200">
          <div className="flex flex-col sm:flex-row sm:items-center justify-between mb-2 space-y-1 sm:space-y-0">
            <span className="text-sm font-semibold text-gray-900">Order #{displayOrder.id}</span>
            <span className="text-xs text-yellow-600 font-medium">�� Processing</span>
          </div>
          <h3 className="text-sm md:text-base font-medium text-gray-900">{displayOrder.itemName}</h3>
        </div>

        {/* Order Details - Responsive */}
        <div className="space-y-1 md:space-y-2">
          <div className="flex justify-between text-xs md:text-sm">
            <span className="text-gray-600">Started:</span>
            <span className="font-medium">{displayOrder.startedAt}</span>
          </div>
          <div className="flex justify-between text-xs md:text-sm">
            <span className="text-gray-600">Created:</span>
            <span className="font-medium">{displayOrder.createdAt}</span>
          </div>
        </div>

        {/* Cups Details - Responsive */}
        {displayOrder.cups && displayOrder.cups.length > 0 && (
          <div>
            <h4 className="text-xs md:text-sm font-medium text-gray-700 mb-2">Order Items:</h4>
            <div className="space-y-1 md:space-y-2">
              {displayOrder.cups.map((cup, index) => (
                <div key={index} className="bg-gray-50 p-1.5 md:p-2 rounded border">
                  <div className="flex justify-between items-center">
                    <span className="text-xs md:text-sm font-medium">
                      {cup.drink_type || cup.type} ({cup.cup_size || cup.size})
                    </span>
                  </div>
                  {cup.addons && cup.addons.length > 0 && (
                    <div className="mt-1">
                      <span className="text-xs text-gray-600">Add-ons: </span>
                      <span className="text-xs text-gray-800">
                        {cup.addons.join(', ').replace(/_/g, ' ')}
                      </span>
                    </div>
                  )}
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Manual Steps Alert - Responsive */}
        {displayOrder.cups?.some(cup => 
          cup.addons && cup.addons.includes('manual_required')
        ) && (
          <div className="bg-orange-50 border border-orange-200 p-2 rounded">
            <div className="flex items-center space-x-2">
              <svg className="w-3 md:w-4 h-3 md:h-4 text-orange-500" fill="currentColor" viewBox="0 0 20 20">
                <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7 4a1 1 0 11-2 0 1 1 0 012 0zm-1-9a1 1 0 00-1 1v4a1 1 0 102 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
              </svg>
              <span className="text-xs font-medium text-orange-800">Manual step required</span>
            </div>
          </div>
        )}
      </div>

      {/* Progress Footer - Responsive */}
      <div className="p-2 md:p-3 border-t border-gray-200 flex-shrink-0">
        <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between text-xs text-gray-600 space-y-1 sm:space-y-0">
          <span>Progress: In Progress</span>
          <span>Est. completion: 3-5 min</span>
        </div>
        <div className="w-full bg-gray-200 rounded-full h-1 mt-2">
          <div className="bg-yellow-500 h-1 rounded-full w-1/3 transition-all duration-500"></div>
        </div>
      </div>
    </div>
  );
} 