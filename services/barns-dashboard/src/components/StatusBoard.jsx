import React, { useState } from 'react';
import useStore from '../store';

export default function StatusBoard() {
  const { orders, systemStatus, schedulerStatus, errors } = useStore();
  const [activeTab, setActiveTab] = useState('stats');

  // Format orders for display
  const formatOrderForDisplay = (order) => ({
    ...order,
    itemName: order.cups && order.cups.length > 0 
      ? order.cups.map(cup => `${cup.drink_type || cup.type} (${cup.cup_size || cup.size})`).join(', ')
      : 'Unknown Item',
    status: order.status?.toUpperCase() || 'QUEUED',
    manualRequired: order.cups && order.cups.some(cup => 
      cup.addons && cup.addons.includes('manual_required')
    )
  });

  const displayOrders = orders.map(formatOrderForDisplay);

  // Calculate stats
  const totalOrders = displayOrders.length;
  const queuedOrders = displayOrders.filter(o => o.status === 'QUEUED').length;
  const processingOrders = displayOrders.filter(o => o.status === 'PROCESSING').length;
  const completedOrders = displayOrders.filter(o => o.status === 'COMPLETED').length;
  const errorOrders = displayOrders.filter(o => o.status === 'ERROR').length;
  const manualOrders = displayOrders.filter(o => o.manualRequired).length;

  // Service health stats
  const getServiceStatusColor = (status) => {
    switch (status) {
      case 'online': return 'bg-green-100 text-green-800';
      case 'offline': return 'bg-red-100 text-red-800';
      default: return 'bg-gray-100 text-gray-800';
    }
  };

  const getServiceStatusIcon = (status) => {
    switch (status) {
      case 'online': return '🟢';
      case 'offline': return '🔴';
      default: return '⚪';
    }
  };

  const servicesList = [
    { name: 'OMS', key: 'oms', description: 'Order Management System' },
    { name: 'Scheduler', key: 'scheduler', description: 'Process Scheduler' },
    { name: 'Routine', key: 'routine', description: 'Routine Executor' },
    { name: 'Validation', key: 'validation', description: 'Quality Validation' },
    { name: 'Video Stream', key: 'videoStream', description: 'Camera Feed' }
  ];

  return (
    <div className="bg-white rounded shadow">
      <div className="p-4 border-b border-gray-200">
        <h2 className="text-xl font-bold">System Status</h2>
      </div>
      
      {/* Tab navigation */}
      <div className="border-b border-gray-200">
        <nav className="flex -mb-px">
          <button
            onClick={() => setActiveTab('stats')}
            className={`px-4 py-2 text-sm font-medium border-b-2 ${
              activeTab === 'stats'
                ? 'border-blue-500 text-blue-600'
                : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
            }`}
          >
            Order Stats
          </button>
          <button
            onClick={() => setActiveTab('details')}
            className={`px-4 py-2 text-sm font-medium border-b-2 ${
              activeTab === 'details'
                ? 'border-blue-500 text-blue-600'
                : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
            }`}
          >
            Order Details
          </button>
          <button
            onClick={() => setActiveTab('services')}
            className={`px-4 py-2 text-sm font-medium border-b-2 ${
              activeTab === 'services'
                ? 'border-blue-500 text-blue-600'
                : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
            }`}
          >
            BARNS Services
          </button>
        </nav>
      </div>
      
      {/* Tab content */}
      <div className="p-4">
        {activeTab === 'stats' && (
          <div className="grid grid-cols-2 gap-4">
            <div className="flex flex-col p-4 bg-blue-50 rounded border border-blue-100">
              <span className="text-sm text-gray-500">Total Orders</span>
              <span className="text-2xl font-bold">{totalOrders}</span>
            </div>
            
            <div className="flex flex-col p-4 bg-yellow-50 rounded border border-yellow-100">
              <span className="text-sm text-gray-500">In Queue</span>
              <span className="text-2xl font-bold">{queuedOrders}</span>
            </div>
            
            <div className="flex flex-col p-4 bg-purple-50 rounded border border-purple-100">
              <span className="text-sm text-gray-500">Processing</span>
              <span className="text-2xl font-bold">{processingOrders}</span>
            </div>
            
            <div className="flex flex-col p-4 bg-green-50 rounded border border-green-100">
              <span className="text-sm text-gray-500">Completed</span>
              <span className="text-2xl font-bold">{completedOrders}</span>
            </div>
            
            <div className="flex flex-col p-4 bg-red-50 rounded border border-red-100">
              <span className="text-sm text-gray-500">Errors</span>
              <span className="text-2xl font-bold">{errorOrders}</span>
            </div>
            
            <div className="flex flex-col p-4 bg-orange-50 rounded border border-orange-100">
              <span className="text-sm text-gray-500">Manual Steps</span>
              <span className="text-2xl font-bold">{manualOrders}</span>
            </div>
          </div>
        )}
        
        {activeTab === 'details' && (
          <div className="overflow-x-auto">
            <table className="w-full text-sm border-collapse">
              <thead>
                <tr className="bg-gray-50 text-left">
                  <th className="py-2 px-3 border-b">Order #</th>
                  <th className="py-2 px-3 border-b">Items</th>
                  <th className="py-2 px-3 border-b">Status</th>
                  <th className="py-2 px-3 border-b">Manual?</th>
                </tr>
              </thead>
              <tbody>
                {displayOrders.length === 0 ? (
                  <tr>
                    <td colSpan="4" className="py-4 text-center text-gray-500">
                      {errors.orders ? 'Unable to load orders' : 'No orders available'}
                    </td>
                  </tr>
                ) : (
                  displayOrders.slice(0, 10).map(o => (
                    <tr key={o.id}
                        className={`border-b ${
                          o.status==='COMPLETED' ? 'bg-green-50' :
                          o.status==='PROCESSING' ? 'bg-yellow-50' :
                          o.status==='ERROR' ? 'bg-red-50' :
                          o.manualRequired ? 'bg-orange-50' : ''
                        }`}
                    >
                      <td className="py-2 px-3">#{o.id}</td>
                      <td className="py-2 px-3 text-xs">{o.itemName}</td>
                      <td className="py-2 px-3">
                        <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
                          o.status === 'COMPLETED' ? 'bg-green-100 text-green-800' :
                          o.status === 'PROCESSING' ? 'bg-yellow-100 text-yellow-800' :
                          o.status === 'ERROR' ? 'bg-red-100 text-red-800' :
                          'bg-gray-100 text-gray-800'
                        }`}>
                          {o.status}
                        </span>
                      </td>
                      <td className="py-2 px-3">
                        {o.manualRequired ? (
                          <span className="inline-flex items-center px-2 py-0.5 rounded text-xs font-medium bg-orange-100 text-orange-800">
                            Yes
                          </span>
                        ) : (
                          <span className="inline-flex items-center px-2 py-0.5 rounded text-xs font-medium bg-gray-100 text-gray-800">
                            No
                          </span>
                        )}
                      </td>
                    </tr>
                  ))
                )}
              </tbody>
            </table>
            {displayOrders.length > 10 && (
              <div className="mt-2 text-center text-sm text-gray-500">
                Showing first 10 of {displayOrders.length} orders
              </div>
            )}
          </div>
        )}
        
        {activeTab === 'services' && (
          <div className="space-y-4">
            {/* Service Status Grid */}
            <div className="grid grid-cols-1 gap-3">
              {servicesList.map((service) => {
                const status = systemStatus[service.key];
                const hasError = errors[service.key];
                
                return (
                  <div key={service.key} className={`p-4 rounded border ${
                    hasError ? 'bg-red-50 border-red-200' :
                    status === 'online' ? 'bg-green-50 border-green-200' :
                    status === 'offline' ? 'bg-red-50 border-red-200' :
                    'bg-gray-50 border-gray-200'
                  }`}>
                    <div className="flex justify-between items-start">
                      <div className="flex-1">
                        <div className="flex items-center">
                          <span className="mr-2">{getServiceStatusIcon(status)}</span>
                          <h3 className="font-medium">{service.name}</h3>
                        </div>
                        <p className="text-sm text-gray-600 mt-1">{service.description}</p>
                        {hasError && (
                          <p className="text-sm text-red-600 mt-1">Error: {errors[service.key]}</p>
                        )}
                      </div>
                      <span className={`inline-flex items-center px-2 py-0.5 rounded text-xs font-medium ${getServiceStatusColor(status)}`}>
                        {status === 'online' ? 'Online' :
                         status === 'offline' ? 'Offline' : 'Unknown'}
                      </span>
                    </div>
                  </div>
                );
              })}
            </div>
            
            {/* Scheduler Status */}
            {schedulerStatus && (
              <div className="p-4 bg-blue-50 rounded border border-blue-200">
                <h3 className="font-medium mb-2">Scheduler Status</h3>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-2 text-sm">
                  <div>
                    <span className="text-gray-600">Status:</span>
                    <span className="ml-2 font-medium">{schedulerStatus.status || 'Unknown'}</span>
                  </div>
                  {schedulerStatus.current_order && (
                    <div>
                      <span className="text-gray-600">Current Order:</span>
                      <span className="ml-2 font-medium">#{schedulerStatus.current_order}</span>
                    </div>
                  )}
                  {schedulerStatus.queue_length !== undefined && (
                    <div>
                      <span className="text-gray-600">Queue Length:</span>
                      <span className="ml-2 font-medium">{schedulerStatus.queue_length}</span>
                    </div>
                  )}
                  {schedulerStatus.last_completed && (
                    <div>
                      <span className="text-gray-600">Last Completed:</span>
                      <span className="ml-2 font-medium">#{schedulerStatus.last_completed}</span>
                    </div>
                  )}
                </div>
              </div>
            )}
            
            {/* System Health Summary */}
            <div className="p-4 bg-gray-50 rounded border border-gray-200">
              <h3 className="font-medium mb-2">Overall System Health</h3>
              <div className="flex items-center justify-between">
                <span className="text-sm text-gray-600">Services Online:</span>
                <span className="text-lg font-semibold">
                  {Object.values(systemStatus).filter(s => s === 'online').length} / {servicesList.length}
                </span>
              </div>
              <div className="flex items-center justify-between mt-1">
                <span className="text-sm text-gray-600">Active Errors:</span>
                <span className="text-lg font-semibold text-red-600">
                  {Object.values(errors).filter(e => e !== null).length}
                </span>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
