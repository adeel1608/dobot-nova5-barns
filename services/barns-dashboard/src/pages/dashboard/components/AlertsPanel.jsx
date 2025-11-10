import React, { useState, useEffect, useCallback } from 'react';
import useStore from '../../../store';
import viewAll from '../../../assets/viewall.png';

export default function AlertsPanel() {
  const { 
    alerts, 
    acknowledgeAlert, 
    fetchAlerts, 
    isLoading, 
    errors,
    clearError 
  } = useStore();
  
  const [acknowledging, setAcknowledging] = useState(new Set());
  const [playedAlerts, setPlayedAlerts] = useState(new Set());

  // Function to play buzz sound using Web Audio API
  const playBuzzSound = useCallback(() => {
    try {
      const audioContext = new (window.AudioContext || window.webkitAudioContext)();
      const oscillator = audioContext.createOscillator();
      const gainNode = audioContext.createGain();
      
      // Configure buzz sound (low frequency, short duration)
      oscillator.type = 'sawtooth';
      oscillator.frequency.setValueAtTime(150, audioContext.currentTime);
      oscillator.frequency.setValueAtTime(100, audioContext.currentTime + 0.1);
      
      gainNode.gain.setValueAtTime(0.3, audioContext.currentTime);
      gainNode.gain.exponentialRampToValueAtTime(0.01, audioContext.currentTime + 0.3);
      
      oscillator.connect(gainNode);
      gainNode.connect(audioContext.destination);
      
      oscillator.start(audioContext.currentTime);
      oscillator.stop(audioContext.currentTime + 0.3);
    } catch (error) {
      console.error('Failed to play buzz sound:', error);
    }
  }, []);

  // Fetch alerts on component mount and set up refresh interval
  useEffect(() => {
    fetchAlerts();
    
    // Set up periodic refresh every 30 seconds
    const interval = setInterval(() => {
      fetchAlerts();
    }, 30000);

    return () => clearInterval(interval);
  }, [fetchAlerts]);

    const { navigateToTab } = useStore();
    const handleNavigate = () => {
    navigateToTab('alerts');       // Update state
    window.location.hash = '#/alerts'; // Update URL
  };

  // Map real alerts data to component format
  const mappedAlerts = alerts.map(alert => {
    const ingredient = getIngredientFromAlert(alert);
    const isCupStations = ingredient === 'cup_stations';
    
    return {
      id: alert.id,
      type: mapAlertTypeToDisplayType(alert.alert_type, alert.severity),
      title: getAlertTitle(alert),
      // For cup_stations, always use the message from the event payload
      message: isCupStations && alert.message ? alert.message : (alert.message || getDefaultMessage(alert.alert_type)),
      timestamp: alert.created_at ? new Date(alert.created_at) : new Date(),
      acknowledged: false, // Active alerts are not acknowledged
      source: isCupStations ? 'validation' : mapAlertTypeToSource(alert.alert_type),
      severity: alert.severity || 'medium',
      rawAlert: alert // Keep reference to original alert data
    };
  });

  // Helper function to map alert_type to display type
  function mapAlertTypeToDisplayType(alertType, severity) {
    if (severity === 'critical') return 'error';
    if (alertType === 'ingredient_threshold') return 'warning';
    if (alertType === 'emergency_stop') return 'error';
    if (alertType === 'order_halted') return 'error';
    if (alertType === 'hardware') return 'error';
    return 'info';
  }

  // Helper function to get alert title
  function getAlertTitle(alert) {
    switch (alert.alert_type) {
      case 'ingredient_threshold':
        const ingredient = getIngredientFromAlert(alert);
        // Special handling for cup_stations
        if (ingredient === 'cup_stations') {
          return 'Cup Stations Status';
        }
        return `Low ${ingredient ? ingredient.charAt(0).toUpperCase() + ingredient.slice(1) : 'Ingredient'} Level`;
      case 'order_halted':
        return 'Order Processing Halted';
      case 'emergency_stop':
        return 'Emergency Stop Activated';
      case 'hardware':
        return 'Hardware Issue Detected';
      default:
        return alert.message || 'System Alert';
    }
  }

  // Helper function to get default message
  function getDefaultMessage(alertType) {
    switch (alertType) {
      case 'ingredient_threshold':
        return 'Ingredient level is below threshold and needs attention';
      case 'order_halted':
        return 'Order processing has been halted due to system issue';
      case 'emergency_stop':
        return 'System emergency stop has been activated';
      case 'hardware':
        return 'Hardware component requires attention';
      default:
        return 'System requires attention';
    }
  }

  // Helper function to map alert type to source
  function mapAlertTypeToSource(alertType) {
    switch (alertType) {
      case 'ingredient_threshold':
        return 'inventory';
      case 'order_halted':
        return 'operations';
      case 'emergency_stop':
        return 'system';
      case 'hardware':
        return 'equipment';
      default:
        return 'system';
    }
  }

  // Helper function to extract ingredient from alert
  function getIngredientFromAlert(alert) {
    // Try to get ingredient from payload (can be object or JSON string)
    if (alert.payload) {
      let payload = alert.payload;
      
      // If payload is a string, parse it
      if (typeof payload === 'string') {
        try {
          payload = JSON.parse(payload);
        } catch (e) {
          // If parsing fails, continue to message extraction
          payload = null;
        }
      }
      
      // If we have a payload object with ingredient, return it
      if (payload && payload.ingredient) {
        return payload.ingredient;
      }
    }
    
    // Fallback: try to extract from message
    const message = alert.message || '';
    const ingredients = ['cup_stations', 'milk', 'cup', 'beans', 'syrup', 'coffee'];
    return ingredients.find(ing => message.toLowerCase().includes(ing));
  }

  const unacknowledgedAlerts = mappedAlerts; // All alerts from the store are unacknowledged
  
  // Play buzz sound when alerts are present
  useEffect(() => {
    if (!isLoading && unacknowledgedAlerts.length > 0) {
      // Get current alert IDs
      const currentAlertIds = new Set(unacknowledgedAlerts.map(alert => alert.id));
      
      // Check if there are any new alerts that haven't triggered the sound yet
      const hasNewAlerts = Array.from(currentAlertIds).some(id => !playedAlerts.has(id));
      
      if (hasNewAlerts) {
        // Play buzz sound for new alerts
        playBuzzSound();
        
        // Update played alerts set
        setPlayedAlerts(prev => {
          const newSet = new Set(prev);
          currentAlertIds.forEach(id => newSet.add(id));
          return newSet;
        });
      }
    } else if (unacknowledgedAlerts.length === 0) {
      // Clear played alerts when there are no alerts
      setPlayedAlerts(new Set());
    }
  }, [unacknowledgedAlerts, isLoading, playedAlerts, playBuzzSound]);
  
  const getAlertIcon = (type) => {
    switch (type) {
      case 'error':
        return (
          <div className="flex-shrink-0 w-2 h-2 rounded-full bg-red-500"></div>
        );
      case 'warning':
        return (
          <div className="flex-shrink-0 w-2 h-2 rounded-full bg-yellow-500"></div>
        );
      case 'info':
        return (
          <div className="flex-shrink-0 w-2 h-2 rounded-full bg-blue-500"></div>
        );
      default:
        return (
          <div className="flex-shrink-0 w-2 h-2 rounded-full bg-gray-500"></div>
        );
    }
  };

  const getTimeAgo = (timestamp) => {
    const diff = Date.now() - timestamp.getTime();
    const minutes = Math.floor(diff / (1000 * 60));
    const hours = Math.floor(diff / (1000 * 60 * 60));
    
    if (hours > 0) {
      return `${hours}h ago`;
    } else if (minutes > 0) {
      return `${minutes}m ago`;
    } else {
      return 'now';
    }
  };

  const handleAcknowledge = async (alertId) => {
    if (acknowledging.has(alertId)) return;
    
    setAcknowledging(prev => new Set(prev).add(alertId));
    
    try {
      const success = await acknowledgeAlert(alertId);
      if (success) {
        // Alert will be automatically removed from the list by the store
        console.log(`Alert ${alertId} acknowledged successfully`);
      } else {
        console.error(`Failed to acknowledge alert ${alertId}`);
      }
    } catch (error) {
      console.error('Failed to acknowledge alert:', error);
    } finally {
      setAcknowledging(prev => {
        const newSet = new Set(prev);
        newSet.delete(alertId);
        return newSet;
      });
    }
  };

  const handleAcknowledgeAll = async () => {
    if (unacknowledgedAlerts.length === 0) return;
    
    // Acknowledge all alerts in parallel
    const acknowledgePromises = unacknowledgedAlerts.map(alert => 
      handleAcknowledge(alert.id)
    );
    
    try {
      await Promise.all(acknowledgePromises);
    } catch (error) {
      console.error('Failed to acknowledge all alerts:', error);
    }
  };

  const retryFetchAlerts = () => {
    clearError('alerts');
    fetchAlerts();
  };

  return (
    <div className="bg-white rounded-lg shadow-md flex flex-col h-full">
      {/* CSS Animation for blinking alerts */}
      <style>{`
        @keyframes alertBlink {
          0%, 100% {
            background-color: transparent;
          }
          50% {
            background-color: rgba(226, 92, 83, 0.4)
          }
        }
        
        .alert-blink {
          animation: alertBlink 2s ease-in-out infinite;
        }
        
        .alert-blink-critical {
          animation: alertBlink 1.5s ease-in-out infinite;
        }
      `}</style>
      
      {/* Header - Responsive */}
      <div className="flex flex-col sm:flex-row sm:items-center justify-between p-3 md:p-4 border-b border-gray-200 flex-shrink-0 space-y-2 sm:space-y-0  ">
        <div className="flex items-center justify-between w-full ">
          <h2 className="text-base md:text-lg font-semibold text-gray-900">Active Alerts</h2>
          {/* {unacknowledgedAlerts.length > 0 && (
            <span className="ml-2 inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-red-100 text-red-800">
              {unacknowledgedAlerts.length}
            </span>
          )}
          {errors.alerts && (
            <span className="ml-2 inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-yellow-100 text-yellow-800">
              API Error
            </span>
          )} */}
           <div className="flex gap-x-2">

            <div className="relative group inline-block ">
              <button
                onClick={handleNavigate}
                className="barns-dark-bg"
                style={{ padding: '0.3rem', outline: 'none' }}
              >
                <img src={viewAll} alt="Refresh" className="w-5 h-5 cursor-pointer" />
              </button>
            <div className="absolute bottom-full left-[-50%] transform -translate-x-1/2 mb-2  
                            bg-gray-800 text-white text-xs rounded px-2 py-1 
                            opacity-0 group-hover:opacity-100 transition-opacity z-10 whitespace-nowrap">
              Click to More Details
            </div>
          </div>

        </div>
        </div>
        
        <div className="flex items-center space-x-2">
          {errors.alerts && (
            <h2
              onClick={retryFetchAlerts}
              disabled={isLoading}
              className="text-xs px-2 py-2 mx-1 border border-red-200 bg-red-100 hover:bg-red-200 text-red-800 rounded-lg transition-colors disabled:opacity-10"
            >
              Retry
            </h2>
          )}
          {/* {unacknowledgedAlerts.length > 0 && (
            <h2
              onClick={handleAcknowledgeAll}
              disabled={isLoading || acknowledging.size > 0}
              className="text-lg md:text-sm font-bold barns-dark-bg text-white px-3 py-1  rounded cursor-pointer "
              style={{ color:'white'}}
            >
              Ack All
            </h2>
          )} */}
        </div>
      </div>

      {/* Error Message */}
      {errors.alerts && (
        <div className="px-3 md:px-4 py-2 bg-yellow-50 border-b border-yellow-200">
          <p className="text-xs text-yellow-800">
            <span className="font-medium">Unable to fetch alerts:</span> {errors.alerts}
          </p>
        </div>
      )}

      {/* Alerts List - Responsive */}
      <div className="flex-1 overflow-hidden p-3">
        <div className="h-full overflow-y-auto">
          {isLoading && unacknowledgedAlerts.length === 0 ? (
            <div className="flex flex-col items-center justify-center h-full text-gray-500 p-4 md:p-6">
              <svg className="animate-spin w-8 h-8 text-blue-500 mb-2" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
              </svg>
              <p className="text-sm font-medium">Loading alerts...</p>
            </div>
          ) : unacknowledgedAlerts.length === 0 ? (
            <div className="flex flex-col items-center justify-center h-full text-gray-500 p-4 md:p-6">
              <svg className="w-8 md:w-12 h-8 md:h-12 text-green-400 mb-2 md:mb-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
              <p className="text-sm font-medium">All clear!</p>
              <p className="text-xs text-gray-400">No active alerts</p>
            </div>
          ) : (
            <div className="space-y-3 p-3">
              {unacknowledgedAlerts.map((alert) => (
                <div 
                  key={alert.id} 
                  className={`p-3 md:p-4 hover:bg-gray-50 transition-colors rounded-md border border-gray-200 ${
                    alert.severity === 'critical' ? 'alert-blink-critical' : 'alert-blink'
                  }`}
                >
                  <div className="flex items-start space-x-2 md:space-x-3">
                    {getAlertIcon(alert.type)}
                    
                    <div className="flex-1 min-w-0">
                      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between space-y-1 sm:space-y-0">
                        <p className="text-xs md:text-sm font-medium text-gray-900 truncate">
                          {alert.title}
                        </p>
                        <span className="text-xs text-gray-500 flex-shrink-0">
                          {getTimeAgo(alert.timestamp)}
                        </span>
                      </div>
                      
                      <p className="text-xs text-gray-600 mt-1 line-clamp-2">
                        {alert.message}
                      </p>
                      
                      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between mt-2 space-y-2 sm:space-y-0">
                        <div className="flex items-center space-x-2">
                          <span className={`inline-flex items-center px-2 py-1 rounded-full text-xs font-medium ${
                            alert.source === 'inventory' ? 'bg-purple-100 text-purple-800' :
                            alert.source === 'equipment' ? 'bg-orange-100 text-orange-800' :
                            alert.source === 'maintenance' ? 'bg-blue-100 text-blue-800' :
                            alert.source === 'operations' ? 'bg-green-100 text-green-800' :
                            'bg-gray-100 text-gray-800'
                          }`}>
                            {alert.source}
                          </span>
                          {alert.severity && (
                            <span className={`inline-flex items-center px-2 py-1 rounded-full text-xs font-medium ${
                              alert.severity === 'critical' ? 'bg-red-100 text-red-800' :
                              alert.severity === 'high' ? 'bg-orange-100 text-orange-800' :
                              alert.severity === 'medium' ? 'bg-yellow-100 text-yellow-800' :
                              alert.severity === 'low' ? 'bg-blue-100 text-blue-800' :
                              'bg-gray-100 text-gray-800'
                            }`}>
                              {alert.severity}
                            </span>
                          )}
                        </div>
                        
                        <button
                          onClick={() => handleAcknowledge(alert.id)}
                          disabled={isLoading || acknowledging.has(alert.id) || errors.alerts}
                          className="text-xs px-2 py-1 bg-green-100 hover:bg-green-200 text-green-800 rounded transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                        >
                          {acknowledging.has(alert.id) ? (
                            <span className="flex items-center space-x-1">
                              <svg className="animate-spin w-3 h-3" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                                <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                                <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                              </svg>
                              <span>Ack...</span>
                            </span>
                          ) : 'Acknowledge'}
                        </button>
                      </div>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>
      </div>
    </div>
  );
} 