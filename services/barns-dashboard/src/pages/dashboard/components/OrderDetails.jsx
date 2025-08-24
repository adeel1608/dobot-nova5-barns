import React, { useState } from 'react';
import useStore from '../../../store';
import stop from '../../../assets/stop.png';
import coffee from '../../../assets/coffee.png';
import lighting from '../../../assets/lighting.png';
import dots from '../../../assets/dots.png';
import progressing from '../../../assets/progressing.png';

export default function OrderDetails() {
  const { orders } = useStore();
  const [showTaskInterface, setShowTaskInterface] = useState(true); // State to control showing task interface

  // Find the currently processing order
  const processingOrder = orders?.find(order => 
    order.status?.toUpperCase() === 'PROCESSING'
  );

  // Show task interface if enabled, otherwise show idle message
  if (!showTaskInterface && !processingOrder) {
    return (
      <div className=" rounded-lg border border-gray-200 flex flex-col h-full ">
        {/* Header - Responsive */}
        <div className="p-2 md:p-3  flex-shrink-0 flex  justify-between  p-4">
          <h2 className="text-base md:text-lg font-semibold text-gray-900">Current Order</h2>
          <div className="flex items-center space-x-2">
            <button 
              onClick={() => setShowTaskInterface(true)}
              className="px-3 py-1 bg-green-600 text-white text-xs rounded hover:bg-green-700 transition-colors"
            >
              Show Tasks
            </button>
            <button className="flex text-red bg-red-300 hover:bg-red-600 text-white" style={{ padding: '0.3rem', outline: 'none', }}>
              {/* Stop */}
              <img src={stop} alt="Refresh" className="w-6 h-6 cursor-pointer " />
           </button>
          </div>
        </div>

        {/* No Processing Order - Responsive */}
        <div className="flex-1 flex items-center justify-center text-gray-500 p-3 md:p-4">
          <div className="text-center">
            <img src={coffee} alt="Coffee" className="w-20 h-20 mx-auto mb-2 md:mb-3" />
            <p className="text-sm font-medium">No Order Processing</p>
            <p className="text-xs text-gray-400 mt-1">System is idle</p>
          </div>
        </div>
      </div>
    );
  }

  // Format order data for display (only if we have a processing order)
  const formatOrderForDisplay = (order) => {
    if (!order) return null;
    
    return {
      ...order,
      itemName: order.cups && order.cups.length > 0 
        ? order.cups.map(cup => `${cup.drink_type || cup.type} (${cup.cup_size || cup.size})`).join(', ')
        : 'Unknown Item',
      createdAt: order.created_at ? new Date(order.created_at).toLocaleString() : 'N/A',
      startedAt: order.started_at ? new Date(order.started_at).toLocaleString() : 'N/A'
    };
  };

  const displayOrder = processingOrder ? formatOrderForDisplay(processingOrder) : null;

  return (
    <div className="bg-white rounded-lg shadow-sm border border-gray-200 flex flex-col h-full">
      {/* Header */}
      <div className="p-3 border-b border-gray-200 flex-shrink-0">
        <div className="flex items-center justify-between">
          <h2 className="text-lg font-semibold text-gray-900">Current Order</h2>
          <div className="flex items-center space-x-2">
            <button 
              onClick={() => setShowTaskInterface(false)}
              className="px-3 py-1 bg-gray-600 text-white text-xs rounded hover:bg-gray-700 transition-colors"
            >
              Show Idle
            </button>
            <div className="w-2 h-2 bg-yellow-400 rounded-full animate-pulse"></div>
            <span className="text-xs font-medium text-yellow-600">Processing</span>
          </div>
        </div>
      </div>

      {/* Task Management Interface */}
      <div className="flex-1 p-4">
        {/* Order Header */}
        <div className="bg-yellow-50 p-3 rounded-lg border border-yellow-200 mb-4">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-semibold text-gray-900">Order #16412</span>
            <span className="text-xs text-yellow-600 font-medium">Processing</span>
          </div>
          <h3 className="text-base font-medium text-gray-900">Vanilla Latte (Large)</h3>
        </div>

        {/* Task Columns */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
                     {/* Left Column - Automated Tasks */}
           <div className="space-y-4">
             <h3 className="text-sm font-semibold text-gray-900 mb-3">Automated Tasks</h3>
             
             {/* Task Steps */}
             <div className="relative max-h-96 overflow-y-auto pr-2">
              {/* Vertical Line */}
              <div className="absolute left-4 top-0 bottom-0 w-0.5 bg-green-600"></div>
              
              {/* Task 1 - Completed */}
              <div className="relative flex items-start space-x-3 mb-8">
                <div className="w-8 h-8 bg-green-600 rounded-full flex items-center justify-center flex-shrink-0">
                  <img src={lighting} alt="Lighting" className="w-4 h-4" />
                </div>
                <div className="flex-1">
                  <div className="flex items-center space-x-2">
                    <span className="text-xs font-medium text-gray-600">STEP 1</span>
                    <span className="text-sm font-medium text-gray-900">Robot Task</span>
                    <span className="text-xs font-medium bg-green-100 text-green-800 rounded-lg px-2 ">Completed</span>
                  </div>
                 
                </div>
              </div>

              {/* Task 2 - Completed */}
              <div className="relative flex items-start space-x-3 mb-8">
                <div className="w-8 h-8 bg-green-600 rounded-full flex items-center justify-center flex-shrink-0">
                  <img src={lighting} alt="Lighting" className="w-4 h-4" />
                </div>
                <div className="flex-1">
                  <div className="flex items-center space-x-2">
                    <span className="text-xs font-medium text-gray-600">STEP 2</span>
                    <span className="text-sm font-medium text-gray-900">Robot Task</span>
                    <span className="text-xs font-medium bg-green-100 text-green-800 rounded-lg px-2 ">Completed</span>
                  </div>
                  
                </div>
              </div>

              {/* Task 3 - Completed */}
              <div className="relative flex items-start space-x-3 mb-8">
                <div className="w-8 h-8 bg-green-600 rounded-full flex items-center justify-center flex-shrink-0">
                  <img src={lighting} alt="Lighting" className="w-4 h-4" />
                </div>
                <div className="flex-1">
                  <div className="flex items-center space-x-2">
                    <span className="text-xs font-medium text-gray-600">STEP 3</span>
                    <span className="text-sm font-medium text-gray-900">Robot Task</span>
                    <span className="text-xs font-medium bg-green-100 text-green-800 rounded-lg px-2 ">Completed</span>
                   
                  </div>
                 
                </div>
              </div>

              {/* Task 4 - Completed */}
              <div className="relative flex items-start space-x-3 mb-8">
                <div className="w-8 h-8 bg-green-600 rounded-full flex items-center justify-center flex-shrink-0">
                  <img src={lighting} alt="Lighting" className="w-4 h-4" />
                </div>
                <div className="flex-1">
                  <div className="flex items-center space-x-2">
                    <span className="text-xs font-medium text-gray-600">STEP 4</span>
                    <span className="text-sm font-medium text-gray-900">Robot Task</span>
                    <span className="text-xs font-medium bg-green-100 text-green-800 rounded-lg px-2 ">Completed</span>
                   
                  </div>
                 
                </div>
              </div>

              {/* Task 5 - In Progress */}
              <div className="relative flex items-start space-x-3">
                <div className="w-8 h-8 bg-orange-500 rounded-full flex items-center justify-center flex-shrink-0">
                  <img src={progressing} alt="Progressing" className="w-4 h-4" />
                </div>
                <div className="flex-1">
                  <div className="flex items-center space-x-2">
                    <span className="text-xs font-medium text-gray-600">STEP 5</span>
                    <span className="text-sm font-medium text-gray-900">Robot Task</span>
                    <span className="text-xs font-medium bg-orange-100 text-orange-800 rounded-lg px-2 "> In Progress</span>
                  </div>
               
                </div>
              </div>
            </div>
          </div>

                     {/* Right Column - Manual Tasks */}
           <div className="space-y-4">
             <h3 className="text-sm font-semibold text-gray-900 mb-3">Manual (Barista) Tasks</h3>
             
             {/* Task Steps */}
             <div className="relative max-h-96 overflow-y-auto pr-2">
              {/* Vertical Line */}
              <div className="absolute left-4 top-0 bottom-0 w-0.5 bg-green-600"></div>
              
              {/* Task 1 - Tap to Complete */}
              <div className="relative flex items-start space-x-3 mb-8">
                <div className="w-8 h-8 bg-gray-400 rounded-full flex items-center justify-center flex-shrink-0">
                  <img src={dots} alt="Dots" className="w-4 h-4" />
                </div>
                <div className="flex-1">
                  <div className="flex items-center space-x-2">
                    <span className="text-xs font-medium text-gray-600">STEP 1</span>
                    <span className="text-sm font-medium text-gray-900">Barista Task</span>
                  </div>
                  <div className="mt-1">
                    <button className="inline-flex items-center px-2 py-1 rounded text-xs font-medium bg-gray-100 text-gray-800 hover:bg-gray-200 transition-colors">
                      Tap to Complete
                    </button>
                  </div>
                </div>
              </div>

              {/* Task 2 - Completed */}
              <div className="relative flex items-start space-x-3 mb-8">
                <div className="w-8 h-8 bg-green-600 rounded-full flex items-center justify-center flex-shrink-0">
                  <img src={dots} alt="Dots" className="w-4 h-4" />
                </div>
                <div className="flex-1">
                  <div className="flex items-center space-x-2">
                    <span className="text-xs font-medium text-gray-600">STEP 2</span>
                    <span className="text-sm font-medium text-gray-900">Barista Task</span>
                  </div>
                  <div className="mt-1">
                    <span className="inline-flex items-center px-2 py-1 rounded text-xs font-medium bg-green-100 text-green-800">
                      Completed
                    </span>
                  </div>
                </div>
              </div>

              {/* Task 3 - Completed */}
              <div className="relative flex items-start space-x-3 mb-8">
                <div className="w-8 h-8 bg-green-600 rounded-full flex items-center justify-center flex-shrink-0">
                  <img src={dots} alt="Dots" className="w-4 h-4" />
                </div>
                <div className="flex-1">
                  <div className="flex items-center space-x-2">
                    <span className="text-xs font-medium text-gray-600">STEP 3</span>
                    <span className="text-sm font-medium text-gray-900">Barista Task</span>
                  </div>
                  <div className="mt-1">
                    <span className="inline-flex items-center px-2 py-1 rounded text-xs font-medium bg-green-100 text-green-800">
                      Completed
                    </span>
                  </div>
                </div>
              </div>

              {/* Task 4 - Completed */}
              <div className="relative flex items-start space-x-3">
                <div className="w-8 h-8 bg-green-600 rounded-full flex items-center justify-center flex-shrink-0">
                  <img src={dots} alt="Dots" className="w-4 h-4" />
                </div>
                <div className="flex-1">
                  <div className="flex items-center space-x-2">
                    <span className="text-xs font-medium text-gray-600">STEP 4</span>
                    <span className="text-sm font-medium text-gray-900">Barista Task</span>
                  </div>
                  <div className="mt-1">
                    <span className="inline-flex items-center px-2 py-1 rounded text-xs font-medium bg-green-100 text-green-800">
                      Completed
                    </span>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
} 