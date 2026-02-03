/**
 * Monitoring Panel
 * Real-time system monitoring dashboard with InfluxDB integration
 * Provides enhanced log visualization and analytics
 */

import React, { useState, useEffect, useCallback } from 'react';
import {
  LineChart, Line, AreaChart, Area, PieChart, Pie, Cell,
  BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, Legend,
  ResponsiveContainer
} from 'recharts';
import {
  getRecentLogs,
  getLogVolumeByLevel,
  getErrorDistributionByService,
  getTopErrors,
  getErrorRateByService,
  getAvailableServices,
  getLogStatistics
} from '../../../utils/influxClient';
import { useTranslation } from '../../../store/translationsStore';
import '../../../utils/debugInflux'; // Loads debug utility into window

// Service colors matching Grafana dashboard
const SERVICE_COLORS = {
  api_bridge: '#73BF69',
  scheduler: '#96D98D',
  routine: '#5794F2',
  oms: '#FF9830',
  video_stream: '#FA6400',
  robot_arm: '#8F3BB8',
  validation: '#F2495C'
};

// Log level colors
const LEVEL_COLORS = {
  ERROR: '#ef4444',
  WARNING: '#f59e0b',
  INFO: '#3b82f6',
  DEBUG: '#6366f1',
  SUCCESS: '#10b981'
};

export default function MonitoringPanel() {
  const { t } = useTranslation('settings');
  // State
  const [timeRange, setTimeRange] = useState('-30m');
  const [selectedServices, setSelectedServices] = useState(['.*']);
  const [selectedLevels, setSelectedLevels] = useState(['.*']);
  const [autoRefresh, setAutoRefresh] = useState(true);
  const [refreshInterval, setRefreshInterval] = useState(2000); // Faster refresh for fast logs
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [connectionStatus, setConnectionStatus] = useState('checking'); // 'checking', 'connected', 'error'
  const [lastUpdate, setLastUpdate] = useState(null);
  const [logLimit, setLogLimit] = useState(100); // Increased default
  const [showServiceDropdown, setShowServiceDropdown] = useState(false);
  const [newLogsCount, setNewLogsCount] = useState(0); // Track new logs since last view
  const [autoScroll, setAutoScroll] = useState(true); // Auto-scroll to newest logs
  const [searchQuery, setSearchQuery] = useState(''); // Search filter for logs
  const logsContainerRef = React.useRef(null);

  // Data state
  const [recentLogs, setRecentLogs] = useState([]);
  const [logVolume, setLogVolume] = useState([]);
  const [errorDistribution, setErrorDistribution] = useState([]);
  const [topErrors, setTopErrors] = useState([]);
  const [errorRate, setErrorRate] = useState([]);
  const [availableServices, setAvailableServices] = useState([]);
  const [logStats, setLogStats] = useState({});

  /**
   * Fetch all monitoring data
   */
  const fetchMonitoringData = useCallback(async () => {
    try {
      setLoading(true);
      setError(null);
      setConnectionStatus('checking');

      const serviceFilter = selectedServices.includes('.*') ? '.*' : selectedServices.join('|');
      const levelFilter = selectedLevels.includes('.*') ? '.*' : selectedLevels.join('|');

      // Fetch all data in parallel - Increased limit for fast logs
      const [
        logs,
        volume,
        errorDist,
        topErrs,
        errRate,
        services,
        stats
      ] = await Promise.all([
        getRecentLogs(timeRange, serviceFilter, levelFilter, 1000), // Increased from 500 to 1000
        getLogVolumeByLevel(timeRange, serviceFilter, levelFilter, '30s'),
        getErrorDistributionByService(timeRange, serviceFilter),
        getTopErrors(timeRange, serviceFilter, 20), // Increased from 10 to 20
        getErrorRateByService(timeRange, serviceFilter, '30s'),
        getAvailableServices('-1h'),
        getLogStatistics(timeRange, serviceFilter)
      ]);

      // Process recent logs - Handle JSON messages properly
      const processedLogs = logs
        .map(log => {
          let message = log.msg || log._value || '';
          
          // If message is an object/JSON, stringify it
          if (typeof message === 'object' && message !== null) {
            try {
              message = JSON.stringify(message);
            } catch (e) {
              message = String(message);
            }
          }
          
          return {
            time: new Date(log._time).toLocaleString(),
            service: log.service,
            level: log.level,
            message: String(message),
            timestamp: log._time,
            rawData: log // Keep raw data for debugging
          };
        })
        // Sort by timestamp descending (newest first)
        .sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp));

      // Process log volume (group by time and level)
      const volumeMap = {};
      volume.forEach(record => {
        const time = new Date(record._time).toLocaleTimeString();
        if (!volumeMap[time]) {
          volumeMap[time] = { time };
        }
        volumeMap[time][record.level] = record._value || 0;
      });
      const processedVolume = Object.values(volumeMap);

      // Process error distribution
      const processedErrorDist = errorDist
        .filter(record => record.service && record._value)
        .map(record => ({
          service: record.service,
          count: parseInt(record._value) || 0,
          color: SERVICE_COLORS[record.service] || '#94a3b8'
        }))
        .filter(item => item.count > 0);

      // Process top errors
      const processedTopErrors = topErrs.map(record => ({
        service: record.service,
        message: record.error_msg || record._value || 'Unknown error',
        count: parseInt(record._value) || 0
      })).filter(e => e.count > 0);

      // Process error rate over time
      const errRateMap = {};
      errRate.forEach(record => {
        const time = new Date(record._time).toLocaleTimeString();
        if (!errRateMap[time]) {
          errRateMap[time] = { time };
        }
        errRateMap[time][record.service] = record._value || 0;
      });
      const processedErrorRate = Object.values(errRateMap);

      // Process services
      const processedServices = services.map(s => s._value).filter(Boolean);

      // Process statistics
      const processedStats = {};
      stats.forEach(record => {
        processedStats[record.level] = record._value || 0;
      });

      // Track new logs
      const prevLogCount = recentLogs.length;
      const newLogCount = processedLogs.length;
      if (prevLogCount > 0 && newLogCount > prevLogCount) {
        setNewLogsCount(newLogCount - prevLogCount);
        // Reset counter after 3 seconds
        setTimeout(() => setNewLogsCount(0), 3000);
      }

      // Update state
      setRecentLogs(processedLogs);
      setLogVolume(processedVolume);
      setErrorDistribution(processedErrorDist);
      setTopErrors(processedTopErrors);
      setErrorRate(processedErrorRate);
      setAvailableServices(processedServices);
      setLogStats(processedStats);
      setConnectionStatus('connected');
      setLastUpdate(new Date());

    } catch (err) {
      console.error('Failed to fetch monitoring data:', err);
      setError(err.message);
      setConnectionStatus('error');
    } finally {
      setLoading(false);
    }
  }, [timeRange, selectedServices, selectedLevels]);

  // Initial fetch and auto-refresh
  useEffect(() => {
    fetchMonitoringData();
  }, [fetchMonitoringData]);

  useEffect(() => {
    if (autoRefresh) {
      const interval = setInterval(fetchMonitoringData, refreshInterval);
      return () => clearInterval(interval);
    }
  }, [autoRefresh, refreshInterval, fetchMonitoringData]);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (showServiceDropdown && !event.target.closest('.service-dropdown-container')) {
        setShowServiceDropdown(false);
      }
    };
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [showServiceDropdown]);

  // Auto-scroll to top when new logs arrive (logs are newest first)
  useEffect(() => {
    if (autoScroll && logsContainerRef.current && recentLogs.length > 0) {
      logsContainerRef.current.scrollTop = 0;
    }
  }, [recentLogs, autoScroll]);

  /**
   * Toggle service selection (multi-select)
   */
  const toggleService = (service) => {
    if (service === 'all') {
      setSelectedServices(['.*']);
    } else {
      setSelectedServices(prev => {
        const filtered = prev.filter(s => s !== '.*');
        if (filtered.includes(service)) {
          const updated = filtered.filter(s => s !== service);
          return updated.length === 0 ? ['.*'] : updated;
        } else {
          return [...filtered, service];
        }
      });
    }
  };

  const isServiceSelected = (service) => {
    if (service === 'all') return selectedServices.includes('.*');
    return selectedServices.includes(service);
  };

  /**
   * Toggle level selection
   */
  const toggleLevel = (level) => {
    if (level === 'all') {
      setSelectedLevels(['.*']);
    } else {
      setSelectedLevels(prev => {
        const filtered = prev.filter(l => l !== '.*');
        if (filtered.includes(level)) {
          const updated = filtered.filter(l => l !== level);
          return updated.length === 0 ? ['.*'] : updated;
        } else {
          return [...filtered, level];
        }
      });
    }
  };

  /**
   * Export logs to JSON
   */
  const exportLogs = () => {
    const dataStr = JSON.stringify(recentLogs, null, 2);
    const dataBlob = new Blob([dataStr], { type: 'application/json' });
    const url = URL.createObjectURL(dataBlob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `barns-monitoring-${new Date().toISOString().split('T')[0]}.json`;
    link.click();
    URL.revokeObjectURL(url);
  };

  const totalLogs = Object.values(logStats).reduce((sum, val) => sum + val, 0);

  // Filter logs based on search query
  const filteredLogs = React.useMemo(() => {
    if (!searchQuery.trim()) return recentLogs;
    const query = searchQuery.toLowerCase();
    return recentLogs.filter(log => 
      log.message?.toLowerCase().includes(query) ||
      log.service?.toLowerCase().includes(query) ||
      log.level?.toLowerCase().includes(query)
    );
  }, [recentLogs, searchQuery]);

  return (
    <div className="flex flex-col gap-3 h-full" style={{ height: 'calc(100vh - 180px)' }}>
      {/* Compact Header with Consistent Design */}
      <div className="bg-white rounded-xl shadow-sm border border-gray-200 p-4 flex-shrink-0">
        <div className="flex items-center justify-between gap-6">
          
          {/* Left Section: Title, Status & Stats */}
          <div className="flex items-center gap-6">
            {/* Title with Status Indicator */}
            <div className="flex items-center gap-2">
              <svg className="w-5 h-5 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z" />
              </svg>
              <h2 className="text-lg font-bold text-gray-900">{t('monitoringTitle')}</h2>
              <div className="flex items-center gap-1.5 ml-2">
                <div className={`w-2 h-2 rounded-full ${
                  connectionStatus === 'connected' ? 'bg-green-500 animate-pulse' :
                  connectionStatus === 'checking' ? 'bg-yellow-500 animate-pulse' :
                  'bg-red-500'
                }`}></div>
                <span className="text-xs text-gray-500">
                  {lastUpdate ? lastUpdate.toLocaleTimeString() : t('connecting')}
                </span>
              </div>
            </div>
            
            {/* Stats with separators */}
            <div className="flex items-center gap-3 text-xs border-l border-gray-200 pl-6">
              <div className="flex items-center gap-1">
                <span className="text-gray-500">{t('total')}:</span>
                <span className="font-bold text-gray-900">{totalLogs.toLocaleString()}</span>
              </div>
              <div className="flex items-center gap-1">
                <span className="text-gray-500">{t('errors')}:</span>
                <span className="font-bold text-red-600">{(logStats.ERROR || 0).toLocaleString()}</span>
              </div>
              <div className="flex items-center gap-1">
                <span className="text-gray-500">{t('warn')}:</span>
                <span className="font-bold text-yellow-600">{(logStats.WARNING || 0).toLocaleString()}</span>
              </div>
              <div className="flex items-center gap-1">
                <span className="text-gray-500">{t('info')}:</span>
                <span className="font-bold text-blue-600">{(logStats.INFO || 0).toLocaleString()}</span>
              </div>
              <div className="flex items-center gap-1">
                <span className="text-gray-500">{t('debug')}:</span>
                <span className="font-bold text-purple-600">{(logStats.DEBUG || 0).toLocaleString()}</span>
              </div>
            </div>
          </div>

          {/* Right Section: Filters & Controls */}
          <div className="flex items-center gap-3">
            {/* Service Multi-Select - Styled as Select */}
            <div className="relative service-dropdown-container">
              <button
                type="button"
                onClick={() => setShowServiceDropdown(!showServiceDropdown)}
                className="no-outline-shadow"
                style={{ 
                  padding: '0.5rem 0.75rem',
                  backgroundColor: '#ffffff',
                  border: '1px solid #d1d5db',
                  borderRadius: '0.5rem',
                  fontSize: '0.875rem',
                  color: '#111827',
                  fontWeight: '500',
                  cursor: 'pointer',
                  transition: 'all 0.2s',
                  boxShadow: 'none',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'space-between',
                  minWidth: '140px',
                  backgroundImage: `url("data:image/svg+xml,%3csvg xmlns='http://www.w3.org/2000/svg' fill='none' viewBox='0 0 20 20'%3e%3cpath stroke='%236b7280' stroke-linecap='round' stroke-linejoin='round' stroke-width='1.5' d='M6 8l4 4 4-4'/%3e%3c/svg%3e")`,
                  backgroundPosition: 'right 0.5rem center',
                  backgroundRepeat: 'no-repeat',
                  backgroundSize: '1.5em 1.5em',
                  paddingRight: '2.5rem'
                }}
                onMouseEnter={(e) => e.currentTarget.style.backgroundColor = '#f9fafb'}
                onMouseLeave={(e) => e.currentTarget.style.backgroundColor = '#ffffff'}
              >
                <span style={{ overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                  {selectedServices.includes('.*') 
                    ? t('allServices') 
                    : selectedServices.length === 1 
                      ? selectedServices[0]
                      : `${selectedServices.length} selected`
                  }
                </span>
              </button>
              {showServiceDropdown && (
                <div className="absolute top-full left-0 mt-2 bg-white border border-gray-200 rounded-xl shadow-lg z-50 min-w-[220px]">
                  <div className="p-2 max-h-80 overflow-y-auto">
                    <label className="flex items-center px-3 py-2 hover:bg-blue-50 cursor-pointer rounded-lg transition-colors">
                      <input
                        type="checkbox"
                        checked={isServiceSelected('all')}
                        onChange={(e) => {
                          e.stopPropagation();
                          toggleService('all');
                        }}
                        className="w-4 h-4 mr-3 rounded text-blue-600 cursor-pointer focus:ring-2 focus:ring-blue-500"
                      />
                      <span className="text-sm font-semibold text-gray-900">{t('allServices')}</span>
                      <span className="ml-auto text-xs text-gray-500">({availableServices.length})</span>
                    </label>
                    
                    <div className="border-t border-gray-200 my-2"></div>
                    
                    {availableServices.map(service => (
                      <label 
                        key={service} 
                        className="flex items-center px-3 py-2 hover:bg-blue-50 cursor-pointer rounded-lg transition-colors group"
                      >
                        <input
                          type="checkbox"
                          checked={isServiceSelected(service)}
                          onChange={(e) => {
                            e.stopPropagation();
                            toggleService(service);
                          }}
                          className="w-4 h-4 mr-3 rounded text-blue-600 cursor-pointer focus:ring-2 focus:ring-blue-500"
                        />
                        <span className="text-sm text-gray-700 group-hover:text-gray-900">{service}</span>
                        <div 
                          className="ml-auto w-3 h-3 rounded-full flex-shrink-0"
                          style={{ backgroundColor: SERVICE_COLORS[service] || '#94a3b8' }}
                        ></div>
                      </label>
                    ))}
                  </div>
                  
                  <div className="px-4 py-2 bg-gray-50 border-t border-gray-200 rounded-b-xl flex items-center justify-between">
                    <span className="text-xs text-gray-600">
                      {selectedServices.includes('.*') 
                        ? t('allServicesSelected')
                        : t('selectedOfSelected').replace('{count}', selectedServices.length).replace('{total}', availableServices.length)
                      }
                    </span>
                    <button
                      onClick={(e) => {
                        e.stopPropagation();
                        setShowServiceDropdown(false);
                      }}
                      className="text-sm text-blue-600 hover:text-blue-700 font-semibold transition-colors"
                    >
                      {t('done')}
                    </button>
                  </div>
                </div>
              )}
            </div>

            {/* Level Filter */}
            <select
              value={selectedLevels.includes('.*') ? 'all' : selectedLevels[0]}
              onChange={(e) => {
                const value = e.target.value;
                if (value === 'all') {
                  setSelectedLevels(['.*']);
                } else {
                  setSelectedLevels([value]);
                }
              }}
              className="px-3 py-2 bg-white border border-gray-300 rounded-lg text-sm text-gray-900 font-medium hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-colors"
            >
              <option value="all">{t('allLevels')}</option>
              {Object.keys(LEVEL_COLORS).map(level => (
                <option key={level} value={level}>
                  {level === 'ERROR' ? t('error') : level === 'WARNING' ? t('warning') : level === 'INFO' ? t('info') : level === 'DEBUG' ? t('debug') : level === 'SUCCESS' ? t('success') : level}
                </option>
              ))}
            </select>

            {/* Time Range */}
            <select
              value={timeRange}
              onChange={(e) => setTimeRange(e.target.value)}
              className="px-3 py-2 bg-white border border-gray-300 rounded-lg text-sm text-gray-900 font-medium hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-colors"
            >
              <option value="-5m">{t('last5Min')}</option>
              <option value="-15m">{t('last15Min')}</option>
              <option value="-30m">{t('last30Min')}</option>
              <option value="-1h">{t('last1Hour')}</option>
              <option value="-3h">{t('last3Hours')}</option>
              <option value="-6h">{t('last6Hours')}</option>
              <option value="-12h">{t('last12Hours')}</option>
              <option value="-24h">{t('last24Hours')}</option>
            </select>

            {/* Auto Refresh Toggle with Interval Selector */}
            <div className="flex items-center gap-2">
              <label className="flex items-center gap-2 px-3 py-2 bg-gray-50 hover:bg-gray-100 border border-gray-200 rounded-lg cursor-pointer transition-colors">
                <input
                  type="checkbox"
                  checked={autoRefresh}
                  onChange={(e) => setAutoRefresh(e.target.checked)}
                  className="w-4 h-4 rounded text-blue-600 cursor-pointer focus:ring-2 focus:ring-blue-500"
                />
                <span className="text-sm font-medium text-gray-700">{t('auto')}</span>
              </label>
              <select
                value={refreshInterval}
                onChange={(e) => setRefreshInterval(parseInt(e.target.value))}
                disabled={!autoRefresh}
                className="px-2 py-2 bg-white border border-gray-300 rounded-lg text-sm text-gray-900 font-medium hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
              >
                <option value="1000">1s</option>
                <option value="2000">2s</option>
                <option value="3000">3s</option>
                <option value="5000">5s</option>
                <option value="10000">10s</option>
              </select>
            </div>

            {/* Refresh Button */}
            <button
              onClick={fetchMonitoringData}
              disabled={loading}
              className="px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 disabled:cursor-not-allowed text-white rounded-lg text-sm font-medium transition-all shadow-sm hover:shadow flex items-center gap-2"
            >
              <svg className={`w-4 h-4 ${loading ? 'animate-spin' : ''}`} fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
              </svg>
              {t('refresh')}
            </button>

            {/* Export Button */}
            <button
              onClick={exportLogs}
              className="px-4 py-2 barns-dark-bg hover:opacity-90 text-white rounded-lg text-sm font-medium transition-all shadow-sm hover:shadow flex items-center gap-2"
            >
              <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 10v6m0 0l-3-3m3 3l3-3m2 8H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
              </svg>
              Export
            </button>
          </div>
        </div>

        {/* Error Banner - single translated message to avoid mixed language */}
        {error && (
          <div className="mt-3 p-3 bg-red-50 border border-red-200 rounded-lg flex items-start gap-2">
            <svg className="w-4 h-4 text-red-600 mt-0.5 flex-shrink-0" fill="currentColor" viewBox="0 0 20 20">
              <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clipRule="evenodd" />
            </svg>
            <div className="flex-1">
              <p className="text-sm font-medium text-red-800">
                {/InfluxDB|500|Internal Server Error/i.test(error)
                  ? t('monitoringServiceUnavailable')
                  : t('connectionError')}
              </p>
            </div>
          </div>
        )}
      </div>

      {/* Main Content Grid - 3 Column Layout - Takes remaining space */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-3 flex-1 min-h-0">
        
        {/* Left Column: Charts */}
        <div className="flex flex-col gap-3 h-full min-h-0">
          {/* Combined Log Volume & Error Rate Chart */}
          <div className="bg-white rounded-xl shadow-sm border border-gray-200 p-3 flex-1 min-h-0 flex flex-col">
            <h3 className="text-sm font-semibold text-gray-900 mb-2 flex-shrink-0">{t('logVolume')}</h3>
            <div className="flex-1 min-h-0">
              <ResponsiveContainer width="100%" height="100%">
              <AreaChart data={logVolume}>
                <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                <XAxis dataKey="time" stroke="#6b7280" style={{ fontSize: '10px' }} />
                <YAxis stroke="#6b7280" style={{ fontSize: '10px' }} />
                <Tooltip 
                  contentStyle={{ backgroundColor: '#fff', border: '1px solid #e5e7eb', borderRadius: '6px', fontSize: '11px' }} 
                />
                <Legend wrapperStyle={{ fontSize: '10px' }} />
                {Object.keys(LEVEL_COLORS).map(level => (
                  <Area
                    key={level}
                    name={level === 'ERROR' ? t('error') : level === 'WARNING' ? t('warning') : level === 'INFO' ? t('info') : level === 'DEBUG' ? t('debug') : level === 'SUCCESS' ? t('success') : level}
                    type="monotone"
                    dataKey={level}
                    stackId="1"
                    stroke={LEVEL_COLORS[level]}
                    fill={LEVEL_COLORS[level]}
                    fillOpacity={0.6}
                  />
                ))}
              </AreaChart>
              </ResponsiveContainer>
            </div>
          </div>

          {/* Error Distribution by Service */}
          <div className="bg-white rounded-xl shadow-sm border border-gray-200 p-3 flex-1 min-h-0 flex flex-col">
            <h3 className="text-sm font-semibold text-gray-900 mb-2 flex-shrink-0">{t('errorDistribution')}</h3>
            {errorDistribution.length === 0 ? (
              <div className="flex items-center justify-center flex-1">
                <div className="text-center text-gray-500">
                  <svg className="w-6 h-6 mx-auto mb-1 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                  </svg>
                  <p className="text-xs font-medium">{t('noErrors')}</p>
                </div>
              </div>
            ) : (
              <div className="flex-1 min-h-0">
                <ResponsiveContainer width="100%" height="100%">
                <PieChart>
                  <Pie
                    data={errorDistribution}
                    dataKey="count"
                    nameKey="service"
                    cx="50%"
                    cy="50%"
                    outerRadius="70%"
                    innerRadius="40%"
                    paddingAngle={2}
                    label={({service, count}) => `${service} (${count})`}
                    labelStyle={{ fontSize: '10px', fill: '#374151' }}
                  >
                    {errorDistribution.map((entry, index) => (
                      <Cell key={`cell-${index}`} fill={entry.color} />
                    ))}
                  </Pie>
                  <Tooltip 
                    contentStyle={{ fontSize: '11px', backgroundColor: '#fff', border: '1px solid #e5e7eb', borderRadius: '4px' }}
                  />
                </PieChart>
                </ResponsiveContainer>
              </div>
            )}
          </div>

          {/* Error Rate by Service Over Time */}
          <div className="bg-white rounded-xl shadow-sm border border-gray-200 p-3 flex-1 min-h-0 flex flex-col">
            <h3 className="text-sm font-semibold text-gray-900 mb-2 flex-shrink-0">{t('errorRateByService')}</h3>
            {errorRate.length === 0 ? (
              <div className="flex items-center justify-center flex-1">
                <div className="text-center text-gray-500">
                  <svg className="w-6 h-6 mx-auto mb-1 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 7h8m0 0v8m0-8l-8 8-4-4-6 6" />
                  </svg>
                  <p className="text-xs font-medium">{t('noErrors')}</p>
                </div>
              </div>
            ) : (
              <div className="flex-1 min-h-0">
                <ResponsiveContainer width="100%" height="100%">
                <LineChart data={errorRate}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                    <XAxis dataKey="time" stroke="#6b7280" style={{ fontSize: '9px' }} />
                    <YAxis stroke="#6b7280" style={{ fontSize: '10px' }} />
                    <Tooltip 
                      contentStyle={{ 
                        backgroundColor: '#fff', 
                        border: '1px solid #e5e7eb', 
                        borderRadius: '6px', 
                        fontSize: '11px' 
                      }} 
                    />
                    <Legend wrapperStyle={{ fontSize: '10px' }} />
                    {availableServices.map(service => (
                      <Line
                        key={service}
                        type="monotone"
                        dataKey={service}
                        stroke={SERVICE_COLORS[service] || '#94a3b8'}
                        strokeWidth={2}
                        dot={{ r: 2 }}
                        name={service}
                      />
                    ))}
                  </LineChart>
                </ResponsiveContainer>
              </div>
            )}
          </div>
        </div>

        {/* Middle Column: Recent Logs (Full Height) */}
        <div className="bg-white rounded-xl shadow-sm border border-gray-200 p-3 flex flex-col h-full min-h-0">
          <div className="flex items-center justify-between mb-2 flex-shrink-0">
            <div className="flex items-center gap-2">
              <h3 className="text-sm font-semibold text-gray-900">{t('recentLogs')}</h3>
              {newLogsCount > 0 && (
                <span className="px-2 py-0.5 bg-green-500 text-white text-xs font-bold rounded-full animate-pulse">
                  {t('newLogsBadge').replace('{count}', newLogsCount)}
                </span>
              )}
              {autoRefresh && connectionStatus === 'connected' && (
                <span className="px-2 py-0.5 bg-blue-500 text-white text-xs font-medium rounded flex items-center gap-1">
                  <div className="w-1.5 h-1.5 bg-white rounded-full animate-pulse"></div>
                  {t('liveBadge')}
                </span>
              )}
              <span className="text-xs text-gray-500">
                ({searchQuery ? t('filteredComma').replace('{count}', filteredLogs.length) : ''}{t('showingOfTotal').replace('{shown}', Math.min(logLimit, filteredLogs.length)).replace('{total}', recentLogs.length)})
              </span>
            </div>
            <div className="flex items-center gap-2">
              <label className="flex items-center gap-1.5 text-xs text-gray-700 cursor-pointer hover:text-gray-900">
                <input
                  type="checkbox"
                  checked={autoScroll}
                  onChange={(e) => setAutoScroll(e.target.checked)}
                  className="w-3.5 h-3.5 rounded text-blue-600 cursor-pointer"
                />
                {t('autoScroll')}
              </label>
              <select
                value={logLimit}
                onChange={(e) => setLogLimit(parseInt(e.target.value))}
                className="px-2 py-1 bg-white border border-gray-300 rounded-lg text-xs focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-colors"
              >
                <option value="25">{t('showN').replace('{n}', 25)}</option>
                <option value="50">{t('showN').replace('{n}', 50)}</option>
                <option value="100">{t('showN').replace('{n}', 100)}</option>
                <option value="200">{t('showN').replace('{n}', 200)}</option>
                <option value="500">{t('showN').replace('{n}', 500)}</option>
                <option value="1000">{t('showN').replace('{n}', 1000)}</option>
              </select>
            </div>
          </div>
          
          {/* Search Box */}
          <div className="mb-2 flex-shrink-0">
            <div className="relative">
              <input
                type="text"
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                placeholder={t('searchLogsPlaceholder')}
                className="w-full px-3 py-1.5 pl-8 bg-gray-50 border border-gray-300 rounded-lg text-xs focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-colors"
              />
              <svg className="w-4 h-4 text-gray-400 absolute left-2.5 top-1/2 -translate-y-1/2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
              </svg>
              {searchQuery && (
                <button
                  onClick={() => setSearchQuery('')}
                  className="absolute right-2 top-1/2 -translate-y-1/2 text-gray-400 hover:text-gray-600 transition-colors"
                >
                  <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                  </svg>
                </button>
              )}
            </div>
          </div>
          <div ref={logsContainerRef} className="flex-1 overflow-y-auto min-h-0">
            <div className="space-y-1">
              {recentLogs.length === 0 ? (
                <div className="text-center py-8 text-gray-500">
                  <svg className="w-8 h-8 mx-auto mb-2 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
                  </svg>
                  <p className="text-sm font-medium">{t('noLogsFound')}</p>
                  <p className="text-xs text-gray-400 mt-0.5">{t('adjustFilters')}</p>
                </div>
              ) : filteredLogs.length === 0 ? (
                <div className="text-center py-8 text-gray-500">
                  <svg className="w-8 h-8 mx-auto mb-2 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
                  </svg>
                  <p className="text-sm font-medium">{t('noMatchingLogs')}</p>
                  <p className="text-xs text-gray-400 mt-0.5">{t('adjustFilters')}</p>
                </div>
              ) : (
                filteredLogs.slice(0, logLimit).map((log, idx) => (
                  <LogEntry key={`${log.timestamp}-${idx}`} log={log} searchQuery={searchQuery} />
                ))
              )}
            </div>
          </div>
        </div>

        {/* Right Column: Top Errors (Full Height) */}
        <div className="bg-white rounded-xl shadow-sm border border-gray-200 p-3 flex flex-col h-full min-h-0">
          <h3 className="text-sm font-semibold text-gray-900 mb-2 flex-shrink-0">{t('topErrors')}</h3>
          <div className="flex-1 overflow-y-auto min-h-0">
            {topErrors.length === 0 ? (
              <div className="text-center py-8 text-gray-500">
                <svg className="w-8 h-8 mx-auto mb-2 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                </svg>
                <p className="text-xs font-medium">{t('noErrors')}</p>
                <p className="text-xs text-gray-400 mt-0.5">{t('systemOk')}</p>
              </div>
            ) : (
              <div className="space-y-1">
                {topErrors.map((error, idx) => (
                  <div key={idx} className="p-2 bg-red-50 border border-red-200 rounded hover:bg-red-100 transition-colors">
                    <div className="flex items-start justify-between mb-1">
                      <span
                        className="px-2 py-0.5 rounded text-xs font-medium text-white"
                        style={{ backgroundColor: SERVICE_COLORS[error.service] || '#94a3b8' }}
                      >
                        {error.service}
                      </span>
                      <span className="text-xs font-bold text-red-600">×{error.count}</span>
                    </div>
                    <p className="text-xs text-gray-900 leading-tight" title={error.message}>
                      {error.message}
                    </p>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}

/**
 * Log Entry Component - Handles individual log display with expand/collapse
 */
function LogEntry({ log, searchQuery }) {
  const [expanded, setExpanded] = useState(false);
  const maxLength = 150;
  const isLongMessage = log.message && log.message.length > maxLength;
  
  // Try to detect if message is JSON
  const isJSON = log.message && (
    log.message.startsWith('{') || 
    log.message.startsWith('[')
  );

  // Highlight search query in text
  const highlightText = (text, query) => {
    if (!query || !text) return text;
    
    const parts = text.split(new RegExp(`(${query})`, 'gi'));
    return parts.map((part, idx) => 
      part.toLowerCase() === query.toLowerCase() ? (
        <mark key={idx} className="bg-yellow-300 text-gray-900 px-0.5 rounded">{part}</mark>
      ) : (
        part
      )
    );
  };
  
  return (
    <div
      className={`p-1.5 rounded border-l-3 transition-all ${
        log.level === 'ERROR' ? 'bg-red-50 border-red-500' :
        log.level === 'WARNING' ? 'bg-yellow-50 border-yellow-500' :
        log.level === 'INFO' ? 'bg-blue-50 border-blue-500' :
        log.level === 'DEBUG' ? 'bg-purple-50 border-purple-500' :
        'bg-gray-50 border-gray-500'
      }`}
    >
      <div className="flex items-center justify-between mb-0.5">
        <div className="flex items-center gap-2">
          <span
            className="text-xs font-medium px-1.5 py-0.5 rounded text-white"
            style={{ backgroundColor: SERVICE_COLORS[log.service] || '#94a3b8' }}
          >
            {log.service}
          </span>
          <span
            className={`text-xs font-bold px-1.5 py-0.5 rounded ${
              log.level === 'ERROR' ? 'bg-red-600 text-white' :
              log.level === 'WARNING' ? 'bg-yellow-600 text-white' :
              log.level === 'INFO' ? 'bg-blue-600 text-white' :
              log.level === 'DEBUG' ? 'bg-purple-600 text-white' :
              'bg-gray-600 text-white'
            }`}
          >
            {log.level}
          </span>
          {isJSON && (
            <span className="text-xs px-1.5 py-0.5 bg-indigo-100 text-indigo-700 rounded font-mono">
              JSON
            </span>
          )}
        </div>
        <span className="text-xs text-gray-500 font-mono">{log.time}</span>
      </div>
      
      <div className="text-xs text-gray-900">
        {isLongMessage && !expanded ? (
          <div>
            <p className="whitespace-pre-wrap break-words">
              {searchQuery ? highlightText(log.message.substring(0, maxLength), searchQuery) : log.message.substring(0, maxLength)}...
            </p>
            <button
              onClick={() => setExpanded(true)}
              className="text-blue-600 hover:text-blue-700 font-medium mt-1 flex items-center gap-1"
            >
              <svg className="w-3 h-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
              </svg>
              Show more
            </button>
          </div>
        ) : (
          <div>
            <pre className="whitespace-pre-wrap break-words font-sans">
              {searchQuery ? highlightText(log.message, searchQuery) : log.message}
            </pre>
            {isLongMessage && (
              <button
                onClick={() => setExpanded(false)}
                className="text-blue-600 hover:text-blue-700 font-medium mt-1 flex items-center gap-1"
              >
                <svg className="w-3 h-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 15l7-7 7 7" />
                </svg>
                Show less
              </button>
            )}
          </div>
        )}
      </div>
    </div>
  );
}

/**
 * Inline Stat Component - Very compact
 */
function InlineStat({ label, value, color }) {
  return (
    <div className="flex items-baseline gap-1">
      <span className="text-xs text-gray-500">{label}:</span>
      <span className={`text-sm font-bold ${color}`}>{value}</span>
    </div>
  );
}

