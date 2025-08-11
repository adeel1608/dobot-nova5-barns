import React from 'react';
import useStore from '../../../store';

export default function OrderStatusPanel() {
  const { orders } = useStore();

  const orderStats = {
    total: orders?.length || 0,
    processing: orders?.filter(o => o.status?.toUpperCase() === 'PROCESSING').length || 0,
    queued: orders?.filter(o => o.status?.toUpperCase() === 'QUEUED').length || 0,
    completed: orders?.filter(o => o.status?.toUpperCase() === 'COMPLETED').length || 0,
    errors: orders?.filter(o => o.status?.toUpperCase() === 'ERROR').length || 0,
    manualRequired: orders?.filter(o => o.cups?.some(cup =>
      cup.addons && cup.addons.includes('manual_required')
    )).length || 0
  };

  return (
    <div className="rounded-lg p-4 space-y-4">
      {/* <h2 className="text-lg font-semibold text-gray-800 ">Orders Overview</h2> */}

      {/* Order Stats Cards - Horizontal Layout */}
      <div className="flex flex-col sm:flex-row gap-3">
        {/* Total Orders Card */}
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats.total}</div>
          <div className="text-sm font-semibold text-gray-600">Total Orders</div>
        </div>

        {/* Processing Card */}
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats.processing}</div>
          <div className="text-sm font-semibold text-gray-600">Processing</div>
        </div>

        {/* In Queue Card */}
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats.queued}</div>
          <div className="text-sm font-semibold text-gray-600">In Queue</div>
        </div>

        {/* Completed Card */}
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats.completed}</div>
          <div className="text-sm font-semibold text-gray-600">Completed</div>
        </div>
      </div>
    </div>
  );
}
