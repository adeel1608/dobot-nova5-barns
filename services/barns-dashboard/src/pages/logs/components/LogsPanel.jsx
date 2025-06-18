import React, { useState, useEffect, useRef } from 'react';
import useStore from '../../../store';

export default function LogsPanel() {
  const { systemLogs, clearLogs, systemStatus, checkSystemHealth } = useStore();
  const [filterLevel, setFilterLevel] = useState('ALL');
  const [filterService, setFilterService] = useState('ALL');
  const [searchTerm, setSearchTerm] = useState('');
  const [autoRefresh, setAutoRefresh] = useState(true);
  const [autoScroll, setAutoScroll] = useState(true);
  const [viewMode, setViewMode] = useState('detailed'); // 'detailed' or 'compact'
  const [showStats, setShowStats] = useState(true);
  const logsEndRef = useRef(null);
  const logsContainerRef = useRef(null);

  // Auto-refresh system health every 30 seconds
  useEffect(() => {
    if (autoRefresh) {
      const interval = setInterval(() => {
        checkSystemHealth();
      }, 30000);
      
      return () => clearInterval(interval);
    }
  }, [autoRefresh, checkSystemHealth]);

  // Auto-scroll to bottom when new logs arrive
  useEffect(() => {
    if (autoScroll && logsEndRef.current) {
      logsEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [systemLogs, autoScroll]);

  // Filter logs based on level, service, and search term
  const filteredLogs = systemLogs.filter(log => {
    if (filterLevel !== 'ALL' && log.level.toLowerCase() !== filterLevel.toLowerCase()) {
      return false;
    }
    
    if (filterService !== 'ALL' && log.service !== filterService) {
      return false;
    }
    
    if (searchTerm && !log.message.toLowerCase().includes(searchTerm.toLowerCase())) {
      return false;
    }
    
    return true;
  });

  // Get log statistics
  const getLogStats = () => {
    const stats = {
      total: systemLogs.length,
      error: systemLogs.filter(log => log.level.toLowerCase() === 'error').length,
      warning: systemLogs.filter(log => log.level.toLowerCase() === 'warning').length,
      info: systemLogs.filter(log => log.level.toLowerCase() === 'info').length,
      success: systemLogs.filter(log => log.level.toLowerCase() === 'success').length,
    };
    return stats;
  };

  const stats = getLogStats();

  const getLevelIcon = (level) => {
    switch (level.toLowerCase()) {
      case 'error':
        return (
          <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
            <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7 4a1 1 0 11-2 0 1 1 0 012 0zm-1-9a1 1 0 00-1 1v4a1 1 0 102 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
          </svg>
        );
      case 'warning':
        return (
          <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
            <path fillRule="evenodd" d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
          </svg>
        );
      case 'info':
        return (
          <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
            <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7-4a1 1 0 11-2 0 1 1 0 012 0zM9 9a1 1 0 000 2v3a1 1 0 001 1h1a1 1 0 100-2v-3a1 1 0 00-1-1H9z" clipRule="evenodd" />
          </svg>
        );
      case 'success':
        return (
          <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
            <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
          </svg>
        );
      default:
        return (
          <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
            <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7-4a1 1 0 11-2 0 1 1 0 012 0zM9 9a1 1 0 000 2v3a1 1 0 001 1h1a1 1 0 100-2v-3a1 1 0 00-1-1H9z" clipRule="evenodd" />
          </svg>
        );
    }
  };

  const getLevelColor = (level) => {
    switch (level.toLowerCase()) {
      case 'error':
        return 'text-red-600 bg-red-50 border-red-200';
      case 'warning':
        return 'text-yellow-600 bg-yellow-50 border-yellow-200';
      case 'info':
        return 'text-blue-600 bg-blue-50 border-blue-200';
      case 'success':
        return 'text-green-600 bg-green-50 border-green-200';
      default:
        return 'text-gray-600 bg-gray-50 border-gray-200';
    }
  };

  const getServiceStatus = (service) => {
    const serviceKey = service.toLowerCase().replace(/\s+/g, '');
    const status = systemStatus[serviceKey] || 'unknown';
    
    switch (status) {
      case 'online':
        return (
          <div className="flex items-center text-green-600">
            <div className="w-2 h-2 bg-green-500 rounded-full mr-2 animate-pulse"></div>
            Online
          </div>
        );
      case 'offline':
        return (
          <div className="flex items-center text-red-600">
            <div className="w-2 h-2 bg-red-500 rounded-full mr-2"></div>
            Offline
          </div>
        );
      default:
        return (
          <div className="flex items-center text-gray-500">
            <div className="w-2 h-2 bg-gray-400 rounded-full mr-2"></div>
            Unknown
          </div>
        );
    }
  };

  const formatTimestamp = (timestamp) => {
    const date = new Date(timestamp);
    return date.toLocaleTimeString('en-US', { 
      hour12: false, 
      hour: '2-digit', 
      minute: '2-digit', 
      second: '2-digit' 
    });
  };

  const exportLogs = () => {
    const logData = filteredLogs.map(log => ({
      timestamp: log.timestamp,
      service: log.service,
      level: log.level,
      message: log.message,
      details: log.details
    }));
    
    const dataStr = JSON.stringify(logData, null, 2);
    const dataBlob = new Blob([dataStr], { type: 'application/json' });
    const url = URL.createObjectURL(dataBlob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `barns-logs-${new Date().toISOString().split('T')[0]}.json`;
    link.click();
    URL.revokeObjectURL(url);
  };

  const services = ['ALL', ...new Set(systemLogs.map(log => log.service))];

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-200 flex flex-col" style={{ height: 'calc(100vh - 200px)' }}>
      {/* Header */}
      <div className="bg-white border-b border-gray-200 p-4">
        <div className="flex flex-col lg:flex-row lg:items-center justify-between">
          <div className="flex items-center mb-3 lg:mb-0">
            <div className="flex items-center">
              <svg className="w-6 h-6 text-blue-600 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
              </svg>
              <h2 className="text-lg font-semibold text-gray-900">System Logs</h2>
              <span className="ml-3 px-2 py-1 bg-gray-100 rounded-full text-xs font-medium text-gray-700">
                {filteredLogs.length} entries
              </span>
            </div>
          </div>
          
          <div className="flex items-center space-x-3">
            {/* View Mode Toggle */}
            <div className="flex bg-gray-100 rounded-lg p-1">
              <button
                onClick={() => setViewMode('detailed')}
                className={`px-3 py-1 text-xs font-medium rounded-md transition-colors ${
                  viewMode === 'detailed' 
                    ? 'bg-blue-600 text-white' 
                    : 'text-gray-600 hover:text-gray-900'
                }`}
              >
                Detailed
              </button>
              <button
                onClick={() => setViewMode('compact')}
                className={`px-3 py-1 text-xs font-medium rounded-md transition-colors ${
                  viewMode === 'compact' 
                    ? 'bg-blue-600 text-white' 
                    : 'text-gray-600 hover:text-gray-900'
                }`}
              >
                Compact
              </button>
            </div>
            
            {/* Controls */}
            <div className="flex items-center space-x-2">
              <label className="flex items-center text-sm text-gray-600">
                <input
                  type="checkbox"
                  checked={autoScroll}
                  onChange={(e) => setAutoScroll(e.target.checked)}
                  className="mr-2 rounded bg-white border-gray-300 text-blue-600 focus:ring-blue-500"
                />
                Auto-scroll
              </label>
              
              <label className="flex items-center text-sm text-gray-600">
                <input
                  type="checkbox"
                  checked={autoRefresh}
                  onChange={(e) => setAutoRefresh(e.target.checked)}
                  className="mr-2 rounded bg-white border-gray-300 text-blue-600 focus:ring-blue-500"
                />
                Auto-refresh
              </label>
              
              <button
                onClick={exportLogs}
                className="px-3 py-1 bg-blue-600 hover:bg-blue-700 text-white rounded-md text-sm font-medium transition-colors flex items-center"
              >
                <svg className="w-4 h-4 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 10v6m0 0l-3-3m3 3l3-3m2 8H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
                </svg>
                Export
              </button>
              
              <button
                onClick={clearLogs}
                className="px-3 py-1 bg-red-600 hover:bg-red-700 text-white rounded-md text-sm font-medium transition-colors flex items-center"
              >
                <svg className="w-4 h-4 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1-1H8a1 1 0 00-1 1v3M4 7h16" />
                </svg>
                Clear
              </button>
            </div>
          </div>
        </div>
      </div>

      {/* Stats Bar */}
      {showStats && (
        <div className="bg-gray-50 border-b border-gray-200 px-4 py-3">
          <div className="flex flex-wrap items-center justify-between gap-4">
            <div className="flex items-center space-x-6">
              <div className="flex items-center text-sm">
                <span className="text-gray-500 mr-2">Total:</span>
                <span className="font-bold text-gray-900">{stats.total}</span>
              </div>
              <div className="flex items-center text-sm">
                <div className="w-2 h-2 bg-red-500 rounded-full mr-2"></div>
                <span className="text-gray-500 mr-1">Errors:</span>
                <span className="font-bold text-red-600">{stats.error}</span>
              </div>
              <div className="flex items-center text-sm">
                <div className="w-2 h-2 bg-yellow-500 rounded-full mr-2"></div>
                <span className="text-gray-500 mr-1">Warnings:</span>
                <span className="font-bold text-yellow-600">{stats.warning}</span>
              </div>
              <div className="flex items-center text-sm">
                <div className="w-2 h-2 bg-blue-500 rounded-full mr-2"></div>
                <span className="text-gray-500 mr-1">Info:</span>
                <span className="font-bold text-blue-600">{stats.info}</span>
              </div>
              <div className="flex items-center text-sm">
                <div className="w-2 h-2 bg-green-500 rounded-full mr-2"></div>
                <span className="text-gray-500 mr-1">Success:</span>
                <span className="font-bold text-green-600">{stats.success}</span>
              </div>
            </div>
            
            <button
              onClick={() => setShowStats(false)}
              className="text-gray-400 hover:text-gray-600"
            >
              <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>
        </div>
      )}

      {/* System Status Bar */}
      <div className="bg-gray-50 border-b border-gray-200 px-4 py-3">
        <div className="flex flex-wrap gap-6 text-sm">
          <div className="flex items-center">
            <span className="text-gray-600 mr-2 font-medium">OMS:</span>
            {getServiceStatus('oms')}
          </div>
          <div className="flex items-center">
            <span className="text-gray-600 mr-2 font-medium">Scheduler:</span>
            {getServiceStatus('scheduler')}
          </div>
          <div className="flex items-center">
            <span className="text-gray-600 mr-2 font-medium">Routine:</span>
            {getServiceStatus('routine')}
          </div>
          <div className="flex items-center">
            <span className="text-gray-600 mr-2 font-medium">Validation:</span>
            {getServiceStatus('validation')}
          </div>
          <div className="flex items-center">
            <span className="text-gray-600 mr-2 font-medium">Video:</span>
            {getServiceStatus('videoStream')}
          </div>
        </div>
      </div>
      
      {/* Filters */}
      <div className="bg-white border-b border-gray-200 p-4">
        <div className="flex flex-col lg:flex-row gap-3">
          {/* Search */}
          <div className="relative flex-1">
            <input
              type="text"
              placeholder="Search logs..."
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="w-full pl-10 pr-4 py-2 bg-white border border-gray-300 rounded-lg text-gray-900 placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
            />
            <svg className="w-5 h-5 absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
            </svg>
          </div>
          
          {/* Level filter */}
          <select
            value={filterLevel}
            onChange={(e) => setFilterLevel(e.target.value)}
            className="px-4 py-2 bg-white border border-gray-300 rounded-lg text-gray-900 focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            <option value="ALL">All Levels</option>
            <option value="ERROR">🔴 Errors</option>
            <option value="WARNING">🟡 Warnings</option>
            <option value="INFO">🔵 Info</option>
            <option value="SUCCESS">🟢 Success</option>
          </select>
          
          {/* Service filter */}
          <select
            value={filterService}
            onChange={(e) => setFilterService(e.target.value)}
            className="px-4 py-2 bg-white border border-gray-300 rounded-lg text-gray-900 focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            {services.map(service => (
              <option key={service} value={service}>
                {service === 'ALL' ? 'All Services' : service}
              </option>
            ))}
          </select>
        </div>
      </div>
      
      {/* Logs content */}
      <div className="flex-1 overflow-hidden bg-gray-50">
        <div 
          ref={logsContainerRef}
          className="h-full overflow-y-auto"
          style={{ scrollbarWidth: 'thin', scrollbarColor: '#cbd5e1 #f1f5f9' }}
        >
          {filteredLogs.length === 0 ? (
            <div className="text-center p-12 text-gray-500">
              <svg className="w-16 h-16 mx-auto mb-4 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
              </svg>
              <p className="text-lg font-medium mb-2 text-gray-900">
                {systemLogs.length === 0 ? 'No logs recorded yet' : 'No logs match your filters'}
              </p>
              <p className="text-sm">
                {systemLogs.length === 0 
                  ? 'System logs will appear here as they are generated' 
                  : 'Try adjusting your search terms or filters'
                }
              </p>
            </div>
          ) : (
            <div className="p-4">
              {filteredLogs.map(log => (
                <div 
                  key={log.id} 
                  className={`mb-2 p-3 rounded-lg border-l-4 hover:bg-white transition-colors ${getLevelColor(log.level)} ${
                    viewMode === 'compact' ? 'py-2' : ''
                  }`}
                >
                  {viewMode === 'detailed' ? (
                    <div>
                      <div className="flex items-center justify-between mb-2">
                        <div className="flex items-center">
                          <div className="flex items-center mr-3">
                            {getLevelIcon(log.level)}
                          </div>
                          <span className="font-mono font-bold text-sm text-gray-900">{log.service}</span>
                          <span className={`ml-2 px-2 py-1 rounded-full text-xs font-bold ${
                            log.level.toLowerCase() === 'error' ? 'bg-red-100 text-red-800' :
                            log.level.toLowerCase() === 'warning' ? 'bg-yellow-100 text-yellow-800' :
                            log.level.toLowerCase() === 'success' ? 'bg-green-100 text-green-800' :
                            'bg-blue-100 text-blue-800'
                          }`}>
                            {log.level.toUpperCase()}
                          </span>
                        </div>
                        
                        <div className="font-mono text-xs text-gray-500">
                          {formatTimestamp(log.timestamp)}
                        </div>
                      </div>
                      
                      <p className="text-gray-900 mb-2 leading-relaxed">{log.message}</p>
                      
                      {log.details && (
                        <details className="mt-3">
                          <summary className="cursor-pointer text-sm text-gray-600 hover:text-gray-800 flex items-center">
                            <svg className="w-4 h-4 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
                            </svg>
                            Show details
                          </summary>
                          <pre className="mt-2 text-xs text-gray-700 bg-gray-100 p-3 rounded border border-gray-200 overflow-x-auto font-mono">
                            {typeof log.details === 'string' ? log.details : JSON.stringify(log.details, null, 2)}
                          </pre>
                        </details>
                      )}
                    </div>
                  ) : (
                    <div className="flex items-center justify-between">
                      <div className="flex items-center flex-1 min-w-0">
                        <div className="flex items-center mr-3 flex-shrink-0">
                          {getLevelIcon(log.level)}
                        </div>
                        <span className="font-mono text-xs text-gray-500 mr-3 flex-shrink-0 w-20">
                          {formatTimestamp(log.timestamp)}
                        </span>
                        <span className="font-mono font-bold text-xs mr-3 flex-shrink-0 text-gray-900">
                          {log.service}
                        </span>
                        <span className="text-gray-900 text-sm truncate">
                          {log.message}
                        </span>
                      </div>
                    </div>
                  )}
                </div>
              ))}
              <div ref={logsEndRef} />
            </div>
          )}
        </div>
      </div>
    </div>
  );
} 