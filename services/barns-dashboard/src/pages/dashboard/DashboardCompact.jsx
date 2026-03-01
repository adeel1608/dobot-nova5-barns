/**
 * DashboardCompact
 * POS-optimized dashboard layout. Mirrors the default dashboard with all 5 panels,
 * but uses the compact 48px navbar so the content area gains ~42px of vertical space
 * compared to the default 90px navbar: h-[calc(100vh-48px)] vs h-[calc(100vh-90px)].
 *
 * Grid proportions are identical to the default dashboard (4-5-3) so that the layout
 * scales naturally at any resolution — including 1024x768 — without any cropping.
 */

import React from 'react';
import { useWebSocketStore } from '../../store/index';
import useStore from '../../store';
import OrderQueue from './components/OrderQueue';
import OrderDetails from './components/OrderDetails';
import AlertsPanel from './components/AlertsPanel';
import IngredientsIndicator from './components/IngredientsIndicator';
import { useTranslation } from '../../store/translationsStore';

function OrderStatusCompact() {
  const { t } = useTranslation('dashboard');
  const { orderStats } = useStore();

  return (
    <div className="flex-shrink-0 px-3 pt-2 pb-1">
      <div className="flex gap-2">
        <div className="bg-[#E6F1ED] px-4 py-2 rounded-lg flex items-center gap-2 flex-1 min-w-0">
          <span className="text-2xl font-bold text-gray-700 flex-shrink-0">
            {orderStats?.processing || 0}
          </span>
          <span className="text-sm font-semibold text-gray-600 truncate">{t('processing')}</span>
        </div>
        <div className="bg-[#E6F1ED] px-4 py-2 rounded-lg flex items-center gap-2 flex-1 min-w-0">
          <span className="text-2xl font-bold text-gray-700 flex-shrink-0">
            {orderStats?.queued || 0}
          </span>
          <span className="text-sm font-semibold text-gray-600 truncate">{t('inQueue')}</span>
        </div>
      </div>
    </div>
  );
}

export default function DashboardCompact() {
  const { connectionStatus } = useWebSocketStore();
  const isWebSocketConnected = connectionStatus?.websocket === 'connected';

  return (
    <div className="p-1 pt-2 h-[calc(100vh-48px)]">
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-2 h-full">

        {/* Left — Order Queue (same span as default) */}
        <div className="lg:col-span-4 h-full flex flex-col min-h-0 order-1">
          <OrderQueue
            connectionStatus={isWebSocketConnected}
            className="flex-1 min-h-0"
          />
        </div>

        {/* Middle — compact status strip + Order Details */}
        <div className="bg-white rounded-lg lg:col-span-5 h-full flex flex-col space-y-2 min-h-0 order-3 lg:order-2">
          <OrderStatusCompact />
          <div className="flex-1 min-h-0 overflow-hidden px-3 pb-3">
            <OrderDetails />
          </div>
        </div>

        {/* Right — Alerts + Ingredients (same as default) */}
        <div className="lg:col-span-3 h-full flex flex-col space-y-2 min-h-0 order-2 lg:order-3">
          <div className="flex-1 min-h-0 overflow-hidden">
            <AlertsPanel />
          </div>
          <div className="flex-shrink-0 min-h-0 max-h-[45vh] overflow-hidden">
            <IngredientsIndicator />
          </div>
        </div>

      </div>
    </div>
  );
}
