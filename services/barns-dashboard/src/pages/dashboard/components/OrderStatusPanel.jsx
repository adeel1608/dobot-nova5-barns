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
    <div className="bg-white rounded-lg shadow p-4 space-y-4 ">
      <h2 className="text-lg font-semibold text-gray-800  border-b border-gray-200">Orders Overview</h2>

      {/* Quick Stats Grid */}
      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-2 md:gap-3">
        <div className="bg-blue-50 p-2 md:p-3 rounded-lg text-center border-b-4   border-blue-500">
          <div className="text-xl md:text-2xl font-bold text-blue-600">{orderStats.total}</div>
          <div className="text-xs text-blue-600">Total Orders</div>
        </div>
        <div className="bg-yellow-50 p-2 md:p-3 rounded-lg text-center border-b-4 border-yellow-500">
          <div className="text-xl md:text-2xl font-bold text-yellow-600">{orderStats.processing}</div>
          <div className="text-xs text-yellow-600">Processing</div>
        </div>
        <div className="bg-gray-50 p-2 md:p-3 rounded-lg text-center border-b-4 border-gray-500">
          <div className="text-xl md:text-2xl font-bold text-gray-600">{orderStats.queued}</div>
          <div className="text-xs text-gray-600">In Queue</div>
        </div>
        <div className="bg-green-50 p-2 md:p-3 rounded-lg text-center border-b-4 border-green-500">
          <div className="text-xl md:text-2xl font-bold text-green-600">{orderStats.completed}</div>
          <div className="text-xs text-green-600">Completed</div>
        </div>
      </div>

      {/* Extended Order Info */}
      {/* <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-2 md:gap-3">
        <div className="flex justify-between items-center px-3 py-2 rounded bg-red-50 border border-red-100">
          <span className="text-sm text-red-600">Errors</span>
          <span className="text-lg font-semibold text-red-600">{orderStats.errors}</span>
        </div>
        <div className="flex justify-between items-center px-3 py-2 rounded bg-orange-50 border border-orange-100">
          <span className="text-sm text-orange-600">Manual Steps Required</span>
          <span className="text-lg font-semibold text-orange-600">{orderStats.manualRequired}</span>
        </div>
      </div> */}
    </div>
  );
}
