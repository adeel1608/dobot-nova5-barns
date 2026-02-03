import React, { useState } from 'react';
import useStore from '../../../store';
import { useTranslation } from '../../../store/translationsStore';
import { SERVICE_OFFLINE_MESSAGE } from '../../../utils/errorHandler';

export default function SystemPanel() {
  const { t } = useTranslation('dashboard');
  const {
    systemStatus,
    emergencyStop,
    resumeOperations,
    isLoading,
    errors,
    orders
  } = useStore();
  
  const [activeTab, setActiveTab] = useState('overview');

  const handleEmergencyStop = async () => {
    if (window.confirm('⚠️ Emergency Stop will halt all operations. Continue?')) {
      try {
        await emergencyStop();
      } catch (error) {
        console.error('Emergency stop failed:', error);
      }
    }
  };

  const handleResumeOperations = async () => {
    try {
      await resumeOperations();
    } catch (error) {
      console.error('Resume operations failed:', error);
    }
  };

  const allSystemsOperational = systemStatus?.overallHealth === 100;
  const hasErrors = errors.system || systemStatus?.overallHealth < 100;

  // Calculate order statistics
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

  const tabs = [
    { id: 'overview', label: 'Overview', icon: '📊' },
    { id: 'orders', label: 'Orders', icon: '📋' },
    { id: 'services', label: 'Services', icon: '⚙️' }
  ];

  const services = [
    { name: 'Order Management', status: systemStatus?.orderManagement || 'online', key: 'orderManagement' },
    { name: 'Scheduler', status: systemStatus?.scheduler || 'running', key: 'scheduler' },
    { name: 'Robotic Arms', status: systemStatus?.roboticArms || 'active', key: 'roboticArms' },
    { name: 'Vision System', status: systemStatus?.vision || 'streaming', key: 'vision' },
    { name: 'Network', status: systemStatus?.network || 'connected', key: 'network' },
    { name: 'Database', status: systemStatus?.database || 'online', key: 'database' }
  ];

  const getServiceStatusColor = (status) => {
    const normalStatuses = ['online', 'running', 'active', 'streaming', 'connected'];
    return normalStatuses.includes(status) ? 'text-green-600' : 'text-red-600';
  };

  const getServiceStatusBg = (status) => {
    const normalStatuses = ['online', 'running', 'active', 'streaming', 'connected'];
    return normalStatuses.includes(status) ? 'bg-green-100' : 'bg-red-100';
  };

  return (
    <div className="bg-white rounded-lg shadow-md  flex flex-col h-full">
      {/* Header with System Status - Responsive */}
      <div className="p-3 md:p-4 border-b border-gray-200 flex-shrink-0">
        <div className="flex flex-col sm:flex-row sm:items-center justify-between mb-3 md:mb-4 space-y-2 sm:space-y-0">
          <div className="flex items-center space-x-2 md:space-x-3">
            <h2 className="text-base md:text-lg font-semibold text-gray-900">System Control</h2>
            <div className={`w-2 h-2 rounded-full ${
              allSystemsOperational ? 'bg-green-500 animate-pulse' : 'bg-red-500'
            }`}></div>
          </div>
          
          <div className="text-left sm:text-right">
            <div className="text-sm font-medium text-gray-900">
              {systemStatus?.overallHealth || 100}% Health
            </div>
            <div className="text-xs text-gray-500">
              {allSystemsOperational ? 'All Systems Operational' : 'Issues Detected'}
            </div>
          </div>
        </div>

        {/* Health Bar */}
        <div className="w-full bg-gray-200 rounded-full h-2">
          <div 
            className={`h-2 rounded-full transition-all duration-500 ${
              (systemStatus?.overallHealth || 100) >= 80 ? 'bg-green-500' :
              (systemStatus?.overallHealth || 100) >= 60 ? 'bg-yellow-500' :
              'bg-red-500'
            }`}
            style={{ width: `${systemStatus?.overallHealth || 100}%` }}
          ></div>
        </div>
      </div>

      {/* Tabs - Responsive */}
      <div className="flex border-b border-gray-200 flex-shrink-0">
        {tabs.map((tab) => (
          <button
            key={tab.id}
            onClick={() => setActiveTab(tab.id)}
            className={`flex-1 px-2 md:px-3 py-2 text-xs md:text-sm font-medium transition-colors ${
              activeTab === tab.id
                ? 'text-green-600 border-b-2 border-green-600 bg-green-50'
                : 'text-gray-500 hover:text-gray-700 hover:bg-gray-50'
            }`}
          >
            <span className="mr-1">{tab.icon}</span>
            <span className="hidden sm:inline">{tab.label}</span>
          </button>
        ))}
      </div>

      {/* Content Area - Responsive */}
      <div className="flex-1 overflow-hidden">
        <div className="h-full overflow-y-auto p-2 md:p-4">
          {activeTab === 'overview' && (
            <div className="space-y-3 md:space-y-4">
              {/* Quick Stats - Responsive 4x1 grid */}
              <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-2 md:gap-3">
                <div className="bg-blue-50 p-2 md:p-3 rounded-lg text-center">
                  <div className="text-xl md:text-2xl font-bold text-blue-600">{orderStats.total}</div>
                  <div className="text-xs text-blue-600">Total Orders</div>
                </div>
                <div className="bg-yellow-50 p-2 md:p-3 rounded-lg text-center">
                  <div className="text-xl md:text-2xl font-bold text-yellow-600">{orderStats.processing}</div>
                  <div className="text-xs text-yellow-600">Processing</div>
                </div>
                <div className="bg-gray-50 p-2 md:p-3 rounded-lg text-center">
                  <div className="text-xl md:text-2xl font-bold text-gray-600">{orderStats.queued}</div>
                  <div className="text-xs text-gray-600">In Queue</div>
                </div>
                <div className="bg-green-50 p-2 md:p-3 rounded-lg text-center">
                  <div className="text-xl md:text-2xl font-bold text-green-600">{orderStats.completed}</div>
                  <div className="text-xs text-green-600">Completed</div>
                </div>
              </div>

              {/* System Status Overview */}
              <div>
                <h3 className="text-sm font-medium text-gray-700 mb-2">Core Systems</h3>
                <div className="space-y-1">
                  {services.slice(0, 4).map((service) => (
                    <div key={service.key} className="flex items-center justify-between text-xs">
                      <span className="text-gray-700">{service.name}</span>
                      <span className={`px-1.5 py-0.5 rounded-full text-xs ${getServiceStatusBg(service.status)} ${getServiceStatusColor(service.status)}`}>
                        {service.status}
                      </span>
                    </div>
                  ))}
                </div>
              </div>
            </div>
          )}

          {activeTab === 'orders' && (
            <div className="space-y-3 md:space-y-4">
              <div className="grid grid-cols-1 gap-2 md:gap-3">
                <div className="flex justify-between items-center">
                  <span className="text-sm text-gray-600">Queue</span>
                  <span className="text-base md:text-lg font-semibold">{orderStats.queued}</span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-sm text-gray-600">Processing</span>
                  <span className="text-base md:text-lg font-semibold">{orderStats.processing}</span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-sm text-gray-600">Completed</span>
                  <span className="text-base md:text-lg font-semibold">{orderStats.completed}</span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-sm text-gray-600">Errors</span>
                  <span className="text-base md:text-lg font-semibold text-red-600">{orderStats.errors}</span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-sm text-gray-600">Manual Steps Required</span>
                  <span className="text-base md:text-lg font-semibold text-orange-600">{orderStats.manualRequired}</span>
                </div>
              </div>
            </div>
          )}

          {activeTab === 'services' && (
            <div className="space-y-1 md:space-y-2">
              {services.map((service) => (
                <div key={service.key} className="flex items-center justify-between p-1.5 md:p-2 rounded border border-gray-100">
                  <span className="text-xs md:text-sm text-gray-700">{service.name}</span>
                  <span className={`px-2 py-1 rounded-full text-xs font-medium ${getServiceStatusBg(service.status)} ${getServiceStatusColor(service.status)}`}>
                    {service.status}
                  </span>
                </div>
              ))}
            </div>
          )}
        </div>
      </div>

      {/* Action Buttons - Responsive */}
      <div className="p-3 md:p-4 border-t border-gray-100 flex-shrink-0">
        {/* Primary Actions in responsive grid */}
        <div className="grid grid-cols-1 gap-2">
          {hasErrors ? (
            <button
              onClick={handleResumeOperations}
              disabled={isLoading}
              className="px-3 md:px-4 py-2 md:py-3 bg-green-600 hover:bg-green-700 text-white text-sm font-medium rounded-lg transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
            >
              {isLoading ? 'Resuming...' : 'Resume Operations'}
            </button>
          ) : (
            <button
              onClick={handleEmergencyStop}
              disabled={isLoading}
              className="px-3 md:px-4 py-2 md:py-3 bg-red-600 hover:bg-red-700 text-white text-sm font-medium rounded-lg transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
            >
              Emergency Stop
            </button>
          )}
        </div>
        
        {/* Error Display */}
        {errors.system && (
          <div className="p-2 bg-red-50 border border-red-200 rounded mt-3">
            <div className="text-xs text-red-700">
              <span className="font-medium">{t('error')}:</span> {errors.system === SERVICE_OFFLINE_MESSAGE ? t('serviceOfflineUnreachable') : errors.system}
            </div>
          </div>
        )}
      </div>
    </div>
  );
} 