import React, { useState, useEffect, useRef } from 'react';
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
  const audioContextRef = useRef(null);
  const oscillator1Ref = useRef(null);
  const oscillator2Ref = useRef(null);
  const gainRef = useRef(null);
  const isBuzzingRef = useRef(false);
  const alarmIntervalRef = useRef(null);
  const spokenAlertsRef = useRef(new Set());
  const ttsQueueRef = useRef([]);
  const isSpeakingRef = useRef(false);

  // Start friendly notification beep pattern for alerts
  function startBuzz() {
    if (isBuzzingRef.current) return;
    try {
      const AudioCtx = window.AudioContext || window.webkitAudioContext;
      if (!AudioCtx) return;
      
      const ctx = new AudioCtx();
      const gain = ctx.createGain();
      gain.connect(ctx.destination);
      
      // Create oscillator for notification beeps
      const osc = ctx.createOscillator();
      osc.type = 'sine'; // Pleasant, smooth tone
      osc.frequency.setValueAtTime(880, ctx.currentTime); // A5 - pleasant notification tone
      
      osc.connect(gain);
      osc.start();
      
      // Start silent
      gain.gain.setValueAtTime(0, ctx.currentTime);
      
      audioContextRef.current = ctx;
      oscillator1Ref.current = osc;
      gainRef.current = gain;
      isBuzzingRef.current = true;
      
      // Create a pleasant triple-beep pattern that repeats every 3 seconds
      // Pattern: beep-beep-beep ... pause ... beep-beep-beep
      const playBeepPattern = () => {
        if (!audioContextRef.current) return;
        
        const ctx = audioContextRef.current;
        const now = ctx.currentTime;
        const g = gainRef.current;
        
        // Triple beep pattern with pleasant frequency
        const beepDuration = 0.15; // Short beep
        const beepGap = 0.15; // Gap between beeps
        const volume = 0.3; // Gentle volume
        
        // First beep
        g.gain.cancelScheduledValues(now);
        g.gain.setValueAtTime(0, now);
        g.gain.linearRampToValueAtTime(volume, now + 0.02);
        g.gain.linearRampToValueAtTime(0, now + beepDuration);
        
        // Second beep
        const secondBeepStart = now + beepDuration + beepGap;
        g.gain.setValueAtTime(0, secondBeepStart);
        g.gain.linearRampToValueAtTime(volume, secondBeepStart + 0.02);
        g.gain.linearRampToValueAtTime(0, secondBeepStart + beepDuration);
        
        // Third beep
        const thirdBeepStart = secondBeepStart + beepDuration + beepGap;
        g.gain.setValueAtTime(0, thirdBeepStart);
        g.gain.linearRampToValueAtTime(volume, thirdBeepStart + 0.02);
        g.gain.linearRampToValueAtTime(0, thirdBeepStart + beepDuration);
      };
      
      // Play pattern immediately
      playBeepPattern();
      
      // Repeat pattern every 3 seconds
      alarmIntervalRef.current = setInterval(() => {
        playBeepPattern();
      }, 3000);
      
    } catch (error) {
      console.error('Failed to start notification sound:', error);
    }
  }

  // Stop the notification sound
  function stopBuzz() {
    try {
      // Clear the interval
      if (alarmIntervalRef.current) {
        clearInterval(alarmIntervalRef.current);
        alarmIntervalRef.current = null;
      }
      
      if (isBuzzingRef.current) {
        const ctx = audioContextRef.current;
        const osc = oscillator1Ref.current;
        const gain = gainRef.current;
        
        if (gain && ctx) {
          gain.gain.cancelScheduledValues(ctx.currentTime);
          gain.gain.setValueAtTime(0, ctx.currentTime);
        }
        
        if (osc) {
          osc.stop(ctx ? ctx.currentTime + 0.05 : undefined);
        }
        
        if (ctx && typeof ctx.close === 'function') {
          setTimeout(() => {
            ctx.close().catch(() => {});
          }, 100);
        }
      }
    } catch (error) {
      console.error('Failed to stop notification sound:', error);
    } finally {
      audioContextRef.current = null;
      oscillator1Ref.current = null;
      oscillator2Ref.current = null;
      gainRef.current = null;
      isBuzzingRef.current = false;
    }
  }

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
  
  // Keep buzzer ON while there are active alerts, OFF otherwise
  useEffect(() => {
    if (!isLoading && unacknowledgedAlerts.length > 0) {
      startBuzz();
    } else {
      stopBuzz();
    }
    // Cleanup on unmount
    return () => {
      stopBuzz();
    };
  }, [unacknowledgedAlerts.length, isLoading]);
  
  // Helper: build speech text for an alert
  function getAlertSpeechText(alert) {
    const title = alert.title || 'Alert';
    const severity = alert.severity ? `Severity ${alert.severity}.` : '';
    const message = alert.message ? alert.message : '';
    return `${title}. ${severity} ${message}`.replace(/\s+/g, ' ').trim();
  }

  // Speak next item in queue if not already speaking
  function processTtsQueue() {
    if (isSpeakingRef.current) return;
    if (!window || !window.speechSynthesis) return;
    const q = ttsQueueRef.current;
    if (q.length === 0) return;
    const text = q.shift();
    try {
      const utter = new SpeechSynthesisUtterance(text);
      utter.rate = 1;   // normal speed
      utter.pitch = 1;  // normal pitch
      utter.volume = 1; // max volume
      // Prefer an English voice if available
      const synth = window.speechSynthesis;
      const voices = synth.getVoices ? synth.getVoices() : [];
      const enVoice = voices.find(v => v.lang && v.lang.toLowerCase().startsWith('en'));
      if (enVoice) utter.voice = enVoice;
      isSpeakingRef.current = true;
      utter.onend = () => {
        isSpeakingRef.current = false;
        // Continue with next queued item
        processTtsQueue();
      };
      utter.onerror = () => {
        isSpeakingRef.current = false;
        processTtsQueue();
      };
      synth.speak(utter);
    } catch {
      // Ignore TTS failures; continue
      isSpeakingRef.current = false;
    }
  }

  // Queue speaking of newly arrived alerts (once per alert id)
  useEffect(() => {
    if (!isLoading && unacknowledgedAlerts.length > 0) {
      const newAlerts = unacknowledgedAlerts.filter(a => !spokenAlertsRef.current.has(a.id));
      newAlerts.forEach(a => {
        spokenAlertsRef.current.add(a.id);
        const text = getAlertSpeechText(a);
        if (text) {
          ttsQueueRef.current.push(text);
        }
      });
      // Start/continue processing queue
      processTtsQueue();
    } else if (unacknowledgedAlerts.length === 0) {
      // Reset spoken ids and cancel any ongoing speech
      spokenAlertsRef.current = new Set();
      try {
        if (window && window.speechSynthesis) {
          window.speechSynthesis.cancel();
        }
      } catch { /* no-op */ }
    }
    // Cleanup on unmount: stop speaking
    return () => {
      try {
        if (window && window.speechSynthesis) {
          window.speechSynthesis.cancel();
        }
      } catch { /* no-op */ }
    };
  }, [unacknowledgedAlerts.length, isLoading]);

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