import React, { useState, useEffect } from 'react';
import useStore from '../store';

export default function LogsPanel() {
  const { systemLogs, clearLogs, systemStatus, checkSystemHealth } = useStore();
  const [filterLevel, setFilterLevel] = useState('ALL');
  const [filterService, setFilterService] = useState('ALL');
  const [searchTerm, setSearchTerm] = useState('');
  const [autoRefresh, setAutoRefresh] = useState(true);

  // Auto-refresh system health every 30 seconds
  useEffect(() => {
    if (autoRefresh) {
      const interval = setInterval(() => {
        checkSystemHealth();
      }, 30000);
      
      return () => clearInterval(interval);
    }
  }, [autoRefresh, checkSystemHealth]);

  // Filter logs based on level, service, and search term
  const filteredLogs = systemLogs.filter(log => {
    // Filter by level
    if (filterLevel !== 'ALL' && log.level.toLowerCase() !== filterLevel.toLowerCase()) {
      return false;
    }
    
    // Filter by service
    if (filterService !== 'ALL' && log.service !== filterService) {
      return false;
    }
    
    // Filter by search term
    if (searchTerm && !log.message.toLowerCase().includes(searchTerm.toLowerCase())) {
      return false;
    }
    
    return true;
  });

  const getLevelIcon = (level) => {
    switch (level.toLowerCase()) {
      case 'error':
        return '❌';
      case 'warning':
        return '⚠️';
      case 'info':
        return 'ℹ️';
      default:
        return '📝';
    }
  };

  const getLevelColor = (level) => {
    switch (level.toLowerCase()) {
      case 'error':
        return 'text-red-600 bg-red-50';
      case 'warning':
        return 'text-yellow-600 bg-yellow-50';
      case 'info':
        return 'text-blue-600 bg-blue-50';
      default:
        return 'text-gray-600 bg-gray-50';
    }
  };

  const getServiceStatus = (service) => {
    const serviceKey = service.toLowerCase().replace(/\s+/g, '');
    const status = systemStatus[serviceKey] || 'unknown';
    
    switch (status) {
      case 'online':
        return <span className="text-green-600">🟢 Online</span>;
      case 'offline':
        return <span className="text-red-600">🔴 Offline</span>;
      default:
        return <span className="text-gray-600">⚪ Unknown</span>;
    }
  };

  const formatTimestamp = (timestamp) => {
    return new Date(timestamp).toLocaleString();
  };

  const services = ['ALL', 'OMS', 'Scheduler', 'Routine', 'Validation', 'VideoStream', 'WebSocket', 'System'];

  return (
    <div className="bg-white rounded shadow h-full flex flex-col">
      {/* Header */}
      <div className="flex flex-col lg:flex-row lg:items-center justify-between p-4 border-b border-gray-200">
        <div className="flex items-center mb-2 lg:mb-0">
          <h2 className="text-xl font-bold">System Logs</h2>
          <span className="ml-2 text-sm text-gray-500">
            ({filteredLogs.length} entries)
          </span>
        </div>
        
        <div className="flex flex-col sm:flex-row space-y-2 sm:space-y-0 sm:space-x-2">
          {/* Auto-refresh toggle */}
          <label className="flex items-center text-sm">
            <input
              type="checkbox"
              checked={autoRefresh}
              onChange={(e) => setAutoRefresh(e.target.checked)}
              className="mr-1"
            />
            Auto-refresh
          </label>
          
          {/* Clear logs button */}
          <button
            onClick={clearLogs}
            className="px-3 py-1 bg-red-100 hover:bg-red-200 text-red-800 rounded text-sm font-medium"
          >
            Clear Logs
          </button>
        </div>
      </div>
      
      {/* System Status Bar */}
      <div className="px-4 py-2 bg-gray-50 border-b border-gray-200">
        <div className="flex flex-wrap gap-4 text-sm">
          <div className="flex items-center">
            <span className="font-medium mr-2">OMS:</span>
            {getServiceStatus('oms')}
          </div>
          <div className="flex items-center">
            <span className="font-medium mr-2">Scheduler:</span>
            {getServiceStatus('scheduler')}
          </div>
          <div className="flex items-center">
            <span className="font-medium mr-2">Routine:</span>
            {getServiceStatus('routine')}
          </div>
          <div className="flex items-center">
            <span className="font-medium mr-2">Validation:</span>
            {getServiceStatus('validation')}
          </div>
          <div className="flex items-center">
            <span className="font-medium mr-2">Video:</span>
            {getServiceStatus('videoStream')}
          </div>
        </div>
      </div>
      
      {/* Filters */}
      <div className="flex flex-col sm:flex-row space-y-2 sm:space-y-0 sm:space-x-2 p-4 border-b border-gray-200">
        {/* Search */}
        <div className="relative flex-1">
          <input
            type="text"
            placeholder="Search logs..."
            value={searchTerm}
            onChange={(e) => setSearchTerm(e.target.value)}
            className="w-full pl-8 pr-3 py-1 border border-gray-300 rounded text-sm"
          />
          <svg className="w-4 h-4 absolute left-2 top-1/2 transform -translate-y-1/2 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
          </svg>
        </div>
        
        {/* Level filter */}
        <select
          value={filterLevel}
          onChange={(e) => setFilterLevel(e.target.value)}
          className="px-3 py-1 border border-gray-300 rounded text-sm"
        >
          <option value="ALL">All Levels</option>
          <option value="ERROR">Errors</option>
          <option value="WARNING">Warnings</option>
          <option value="INFO">Info</option>
        </select>
        
        {/* Service filter */}
        <select
          value={filterService}
          onChange={(e) => setFilterService(e.target.value)}
          className="px-3 py-1 border border-gray-300 rounded text-sm"
        >
          {services.map(service => (
            <option key={service} value={service}>
              {service === 'ALL' ? 'All Services' : service}
            </option>
          ))}
        </select>
      </div>
      
      {/* Logs content */}
      <div className="flex-1 overflow-y-auto">
        {filteredLogs.length === 0 ? (
          <div className="text-center p-8 text-gray-500">
            {systemLogs.length === 0 ? (
              <p>No logs recorded yet.</p>
            ) : (
              <p>No logs match your filters.</p>
            )}
          </div>
        ) : (
          <div className="divide-y divide-gray-200">
            {filteredLogs.map(log => (
              <div key={log.id} className={`p-3 hover:bg-gray-50 ${getLevelColor(log.level)}`}>
                <div className="flex flex-col sm:flex-row sm:items-start justify-between">
                  <div className="flex-1">
                    <div className="flex items-center mb-1">
                      <span className="mr-2">{getLevelIcon(log.level)}</span>
                      <span className="font-medium text-sm">{log.service}</span>
                      <span className={`ml-2 px-2 py-0.5 rounded-full text-xs font-medium ${
                        log.level.toLowerCase() === 'error' ? 'bg-red-100 text-red-800' :
                        log.level.toLowerCase() === 'warning' ? 'bg-yellow-100 text-yellow-800' :
                        'bg-blue-100 text-blue-800'
                      }`}>
                        {log.level.toUpperCase()}
                      </span>
                    </div>
                    
                    <p className="text-sm text-gray-800 mb-1">{log.message}</p>
                    
                    {log.details && (
                      <details className="mt-2">
                        <summary className="cursor-pointer text-xs text-gray-600 hover:text-gray-800">
                          Show details
                        </summary>
                        <pre className="mt-1 text-xs text-gray-600 bg-gray-100 p-2 rounded overflow-x-auto">
                          {typeof log.details === 'string' ? log.details : JSON.stringify(log.details, null, 2)}
                        </pre>
                      </details>
                    )}
                  </div>
                  
                  <div className="text-xs text-gray-500 mt-1 sm:mt-0 sm:ml-4">
                    {formatTimestamp(log.timestamp)}
                  </div>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
} 