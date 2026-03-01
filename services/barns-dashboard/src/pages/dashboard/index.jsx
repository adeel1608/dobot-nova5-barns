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
import OrderStatusPanel from './components/OrderStatusPanel';

export default function Dashboard() {
  const { connectionStatus } = useWebSocketStore();
  const isWebSocketConnected = connectionStatus?.websocket === 'connected';

  return (
    <div className="min-h-screen ">

      {/* Main Dashboard Grid - Responsive layout */}
        <div className="p-1 pt-2 h-[calc(100vh-90px)]">
          <div className="grid grid-cols-1 lg:grid-cols-12 gap-2 md:gap-2 h-full">
            
            {/* Left Column */}
            <div className="lg:col-span-4 h-full flex flex-col min-h-0 order-1">
              <OrderQueue className="flex-1 min-h-0" connectionStatus = {isWebSocketConnected}/>
            </div>

            {/* Middle Column */}
            <div className="bg-white rounded-lg lg:col-span-5 h-full flex flex-col space-y-3 md:space-y-3 min-h-0 order-3 lg:order-2">
              <div className="flex-shrink-0">
                 <OrderStatusPanel />
              </div>
                
              {/* OrderDetails fills remaining space */}
              <div className="flex-1 min-h-0 overflow-hidden px-3 pb-3">
                <OrderDetails />
              </div>

             
            </div>

            {/* Right Column */}
            <div className="lg:col-span-3 h-full flex flex-col space-y-3 md:space-y-3 min-h-0 order-2 lg:order-3">
              <div className="flex-1 min-h-0 overflow-hidden">
                <AlertsPanel />
              </div>
              {/* <div className="flex-1 min-h-0">
                <SystemPanel />
              </div> */}
           

                 {/* IngredientsIndicator takes flexible height with min/max constraints */}
              <div className="flex-shrink-0 min-h-0 max-h-[50vh] overflow-hidden">
                <IngredientsIndicator />
              </div>
            </div>
          </div>
        </div>

    </div>
  );
} 