/**
 * Dashboard Page
 * Main dashboard view with orders, system controls, and status
 */

import React from 'react';
import { useWebSocketStore } from '../../store/index';
import OrderQueue from './components/OrderQueue';
import SystemPanel from './components/SystemPanel';
import AlertsPanel from './components/AlertsPanel';
import IngredientsIndicator from './components/IngredientsIndicator';
import OrderDetails from './components/OrderDetails';

export default function Dashboard() {
  const { connectionStatus } = useWebSocketStore();
  const isWebSocketConnected = connectionStatus?.websocket === 'connected';

  return (
    <div className="min-h-screen bg-gray-50">
      {/* Page Header - Responsive */}
      <div className="   px-1  py-1 md:py-2 ">
        <div className="flex flex-col sm:flex-row sm:items-center justify-between space-y-2 sm:space-y-0  px-1 py-1 rounded">
          <h2 className="text-xl md:text-2xl font-bold text-gray-900">Dashboard</h2>
          <div className="flex flex-col sm:flex-row sm:items-center space-y-2 sm:space-y-0 sm:space-x-4">
            {/* <div className="text-xs md:text-sm text-gray-500">
              Last updated: {new Date().toLocaleTimeString()}
            </div> */}
            <div className="flex items-center space-x-4">
              {/* WebSocket Connection Status */}
              <div className="flex items-center space-x-2">
                <div className={`w-2 h-2 rounded-full ${
                  isWebSocketConnected ? 'bg-green-400 animate-pulse' : 'bg-red-400'
                }`}></div>
                <span className={`text-xs md:text-sm font-medium ${
                  isWebSocketConnected ? 'text-green-600' : 'text-red-600'
                }`}>
                  {isWebSocketConnected ? 'Online' : 'Offline'}
                </span>
              </div>
           
             
            </div>
          </div>
        </div>
      </div>

      {/* Main Dashboard Grid - Responsive layout */}
      <div className="p-1 h-[calc(100vh-140px)] ">
        <div className="grid grid-cols-1 lg:grid-cols-12 gap-3 md:gap-6 h-full">
          {/* Left Column - Order Queue (full width on mobile, 5 cols on desktop) */}
          <div className="lg:col-span-5 min-h-0 order-1">
            <OrderQueue />
          </div>

          {/* Middle Column - Order Details + Ingredients (full width on mobile, 3 cols on desktop) */}
          <div className="lg:col-span-3 flex flex-col space-y-3 md:space-y-4 min-h-0 order-3 lg:order-2 overflow-hidden">
            {/* Order Details - Increased height ratio */}
            <div className="flex-[3] min-h-0 h-80 md:h-auto overflow-hidden">
              <OrderDetails />
            </div>
            
            {/* Ingredients Indicator - Reduced space, minimal padding */}
            <div className="flex-[1] min-h-0 h-32 md:h-auto overflow-hidden">
              <IngredientsIndicator />
            </div>
          </div>

          {/* Right Column - Active Alerts + System Control (full width on mobile, 4 cols on desktop) */}
          <div className="lg:col-span-4 flex flex-col space-y-3 md:space-y-6 min-h-0 order-2 lg:order-3">
            {/* Active Alerts - Responsive height */}
            <div className="flex-[1] min-h-0 h-64 md:h-80 md:flex-shrink-0">
              <AlertsPanel />
            </div>
            
            {/* System Control - Responsive space */}
            <div className="flex-[1] min-h-0 h-48 md:h-auto">
              <SystemPanel />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
} 