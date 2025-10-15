import React from 'react';
import useStore from '../../../store';

export default function OrderStatusPanel() {
  const { orderStats } = useStore();

  // Use the global order statistics from the store instead of calculating from loaded orders
  // This ensures we show the total count across all orders in the database, not just the loaded batch

  return (
    <div className="rounded-lg p-4 space-y-4">
      {/* <h2 className="text-lg font-semibold text-gray-800 ">Orders Overview</h2> */}

      {/* Order Stats Cards - Horizontal Layout */}
      <div className="flex flex-col sm:flex-row gap-3">
        {/* Total Orders Card */}
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats?.total || 0}</div>
          <div className="text-sm font-semibold text-gray-600">Total Orders</div>
        </div>

        {/* Processing Card */}
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats?.processing || 0}</div>
          <div className="text-sm font-semibold text-gray-600">Processing</div>
        </div>

        {/* In Queue Card */}
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats?.queued || 0}</div>
          <div className="text-sm font-semibold text-gray-600">In Queue</div>
        </div>

        {/* Completed Card */}
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats?.completed || 0}</div>
          <div className="text-sm font-semibold text-gray-600">Completed</div>
        </div>
      </div>
    </div>
  );
}
