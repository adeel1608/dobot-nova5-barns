import React, { useState, useEffect, useRef } from 'react';
import useStore from '../../../store';
import { useTranslation } from '../../../store/translationsStore';

function CameraStreamComponent({
  cameraId,
  camera,
  streamsEnabled,
  isInCooldown,
  cameraCooldown,
  isError,
  onStreamError,
  onStreamReady,
  onFullscreen,
  addLog,
  t,
}) {
  const [imageError, setImageError] = useState(false);
  const [imgKey, setImgKey] = useState(null); // Do not start connection until we set this once
  const hasFiredLoadRef = useRef(false);
  const lastFrameTimeRef = useRef(Date.now());
  const reconnectTimeoutRef = useRef(null);
  const isMountedRef = useRef(true);
  const lastReconnectTimeRef = useRef(0); // Track last reconnection time
  const hasInitializedRef = useRef(false); // Track if initial connection was made
  const reconnectCooldownMs = 10000; // Minimum 10 seconds between reconnections
  const isOffline = camera.status === 'offline' || isError || imageError;
  const streamUrl = `http://localhost:8001/stream/${cameraId}?k=${imgKey}`;
  const displayName = (cameraId === 'ceiling' && t) ? t('ceilingCameraName') : camera.name;
  const displayType = (cameraId === 'ceiling' && t) ? t('rtspStream') : camera.type;

  // Cleanup on unmount
  useEffect(() => {
    isMountedRef.current = true;
    return () => {
      isMountedRef.current = false;
      hasInitializedRef.current = false; // Reset on unmount
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
    };
  }, []);

  // Initial connection when streams are enabled (runs only once)
  useEffect(() => {
    if (!streamsEnabled || isInCooldown || hasInitializedRef.current) {
      return;
    }

    // Check reconnection cooldown
    const timeSinceLastReconnect = Date.now() - lastReconnectTimeRef.current;
    if (timeSinceLastReconnect < reconnectCooldownMs) {
      addLog('VideoStream', 'info', `Reconnection on cooldown for ${cameraId} (${Math.ceil((reconnectCooldownMs - timeSinceLastReconnect) / 1000)}s remaining)`);
      return;
    }

    // Clear any pending reconnect
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }

    // Mark as initialized to prevent duplicate attempts
    hasInitializedRef.current = true;

    // Debounce initial connection - wait 1 second before connecting
    reconnectTimeoutRef.current = setTimeout(() => {
      if (!isMountedRef.current) return;
      // Record start BEFORE setting key to ensure single connect
      if (!lastReconnectTimeRef.current) {
        lastReconnectTimeRef.current = Date.now();
        setImageError(false); // Reset error state
        setImgKey(Date.now()); // Single connection start
        addLog('VideoStream', 'info', `Initializing stream for ${cameraId}`);
      }
    }, 1000);

    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
    };
  }, [streamsEnabled, isInCooldown, cameraId, addLog]);

  // Reset onLoad guard when imgKey changes (new image element)
  useEffect(() => {
    hasFiredLoadRef.current = false;
  }, [imgKey]);

  // Periodic stream health check - refresh if stream might be stalled (REDUCED FREQUENCY)
  useEffect(() => {
    if (!streamsEnabled || isInCooldown || isOffline) return;

    const healthCheckInterval = setInterval(() => {
      if (!isMountedRef.current) return; // Don't reconnect if unmounted

      const timeSinceLastFrame = Date.now() - lastFrameTimeRef.current;
      const timeSinceLastReconnect = Date.now() - lastReconnectTimeRef.current;

      // Only reconnect if:
      // 1. Stream is stalled for 90+ seconds (increased from 60)
      // 2. We haven't reconnected in the last 10 seconds (cooldown)
      if (timeSinceLastFrame > 90000 && timeSinceLastReconnect > reconnectCooldownMs) {
        addLog('VideoStream', 'warning', `Stream ${cameraId} stalled for ${Math.floor(timeSinceLastFrame / 1000)}s - forcing reconnection`);
        lastReconnectTimeRef.current = Date.now();
        setImgKey(Date.now());
        lastFrameTimeRef.current = Date.now();
      }
    }, 60000); // Check every 60 seconds (reduced frequency)

    return () => clearInterval(healthCheckInterval);
  }, [streamsEnabled, isInCooldown, isOffline, cameraId, addLog]);

  // Show cooldown loading screen
  if (isInCooldown || cameraCooldown.in_cooldown) {
    return (
      <div className="bg-white rounded-xl shadow-md border border-gray-200 overflow-hidden relative group hover:shadow-lg transition-all duration-300 w-full h-full">
        <div className="w-full h-full relative bg-gray-900">
          <div className="absolute inset-0 flex flex-col items-center justify-center text-white">
            <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-white mb-3"></div>
            <p className="text-sm font-semibold">{displayName}</p>
            <p className="text-xs text-gray-400">
              {cameraCooldown.remaining_seconds > 0
                ? (t ? t('reconnectingSeconds').replace('{seconds}', cameraCooldown.remaining_seconds) : `Reconnecting... (${cameraCooldown.remaining_seconds}s)`)
                : (t ? t('reconnecting') : 'Reconnecting...')}
            </p>
          </div>
        </div>
      </div>
    );
  }

  // Don't render stream if streams are disabled (tab hidden or component unmounting)
  if (!streamsEnabled) {
    return (
      <div className="bg-white rounded-xl shadow-md border border-gray-200 overflow-hidden relative group hover:shadow-lg transition-all duration-300 w-full h-full">
        <div className="w-full h-full relative bg-gray-900">
          <div className="absolute inset-0 flex flex-col items-center justify-center text-white">
            <svg className="w-14 h-14 text-gray-400 mb-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
            </svg>
            <p className="text-sm font-semibold">{displayName}</p>
            <p className="text-xs text-gray-400">{t ? t('streamPaused') : 'Stream paused (saving resources)'}</p>
          </div>
        </div>
      </div>
    );
  }

  // If imgKey not set yet (initial connect debounce), render lightweight placeholder
  if (imgKey === null) {
    return (
      <div className="bg-white rounded-xl shadow-md border border-gray-200 overflow-hidden relative group hover:shadow-lg transition-all duration-300 w-full h-full">
        <div className="w-full h-full relative bg-gray-900 flex items-center justify-center">
          <div className="animate-pulse text-gray-300 text-xs">{t ? t('startingStream') : 'Starting stream...'}</div>
        </div>
      </div>
    );
  }

  return (
    <div className="bg-white rounded-xl shadow-md border border-gray-200 overflow-hidden relative group hover:shadow-lg transition-all duration-300 w-full h-full">
      <div className="w-full h-full relative bg-gray-100">
        {isOffline ? (
          <div className="absolute inset-0 flex flex-col items-center justify-center bg-gray-900 text-white">
            <svg className="w-14 h-14 text-red-400 mb-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
            </svg>
            <p className="text-sm font-semibold">{displayName}</p>
            <p className="text-xs text-gray-300">{t ? t('streamUnavailable') : 'Stream Unavailable'}</p>
          </div>
        ) : (
          <img
            key={imgKey}
            src={streamUrl}
            alt={`${displayName} feed`}
            className="w-full h-full object-cover"
            onError={(e) => {
              // Prevent error loops - only handle first error
              if (!imageError) {
                addLog('VideoStream', 'warning', `Stream error for ${cameraId}: ${e.type}`);
                setImageError(true);
                onStreamError(cameraId);
              }
            }}
            onLoad={() => {
              if (hasFiredLoadRef.current) return;
              hasFiredLoadRef.current = true;
              lastFrameTimeRef.current = Date.now(); // Track frame receipt
              setImageError(false);
              onStreamReady(cameraId);
            }}
          />
        )}

        {/* Overlay */}
        <div className="absolute inset-0 bg-gradient-to-t from-black/70 via-transparent to-black/70 opacity-90">
          <div className="absolute top-3 left-3 right-3 flex justify-between items-start">
            <div className="bg-black/70 px-3 py-2 rounded-lg">
              <p className="text-white text-sm">{displayName}</p>
              <p className="text-gray-300 text-xs">{displayType}</p>
            </div>
            <div className="flex items-center space-x-2">
              <div className={`w-2.5 h-2.5 rounded-full ${isOffline ? 'bg-red-500' : 'bg-green-500'} animate-pulse`} />
              <span className={`text-xs px-2 py-0.5 rounded-md ${isOffline ? 'bg-red-500' : 'bg-green-500'} text-white`}>
                {isOffline ? (t ? t('offline') : 'Offline') : (t ? t('live') : 'Live')}
              </span>
            </div>
          </div>
          <div className="absolute bottom-3 right-3">
            <button
              onClick={() => !isOffline && onFullscreen(cameraId, camera)}
              disabled={isOffline}
              className="bg-black/70 text-white p-2 rounded-md hover:bg-black/90 disabled:opacity-50 transition-all group-hover:opacity-100 opacity-0 duration-300"
              title={t ? t('fullscreen') : 'Fullscreen'}
            >
              <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5v-4m0 4h-4m4 0l-5-5" />
              </svg>
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}

const CameraStream = React.memo(CameraStreamComponent);

export default function UnifiedCameraPanel() {
  const { t } = useTranslation('cameras');
  const { addLog } = useStore();
  const [cameras, setCameras] = useState({});
  const [selectedView, setSelectedView] = useState('all');
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [fullscreenCamera, setFullscreenCamera] = useState(null);
  const [streamErrors, setStreamErrors] = useState({});
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [isTabVisible, setIsTabVisible] = useState(true);
  const [streamsEnabled, setStreamsEnabled] = useState(false); // Start disabled, enable after mount
  const [cooldownStatus, setCooldownStatus] = useState({});
  const [isInCooldown, setIsInCooldown] = useState(false);
  const hasStoppedStreams = useRef(false);
  const unmountTimeoutRef = useRef(null);
  const cooldownCheckIntervalRef = useRef(null);
  const visibilityTimeoutRef = useRef(null);
  const cleanupTimerRef = useRef(null);
  const isMountedRef = useRef(false);
  const fetchCamerasTimeoutRef = useRef(null);
  const lastFetchTimeRef = useRef(0);
  const enableStreamsTimeoutRef = useRef(null);
  const fullscreenTransitionRef = useRef(false); // Tracks fullscreen enter/exit transitions
  const wasInFullscreenRef = useRef(false); // Tracks previous fullscreen state

  // Check cooldown status for all cameras (memoized to prevent infinite loops)
  const checkCooldownStatus = React.useCallback(async () => {
    try {
      const response = await fetch('http://localhost:8001/stream/cooldown/all');
      if (response.ok) {
        const data = await response.json();
        const cooldowns = data.cooldowns || {};
        
        // Check if any camera is in cooldown
        const anyInCooldown = Object.values(cooldowns).some(
          (cooldown) => cooldown.in_cooldown
        );
        
        // Only update state if values actually changed to prevent infinite loops
        setCooldownStatus(prev => {
          const changed = JSON.stringify(prev) !== JSON.stringify(cooldowns);
          return changed ? cooldowns : prev;
        });
        
        setIsInCooldown(prev => prev !== anyInCooldown ? anyInCooldown : prev);
        
        return anyInCooldown;
      }
    } catch (err) {
      console.warn('Failed to check cooldown status:', err);
    }
    return false;
  }, []); // No dependencies - stable function

  // Wait for cooldown to expire (memoized)
  const waitForCooldown = React.useCallback(() => {
    if (cooldownCheckIntervalRef.current) {
      clearInterval(cooldownCheckIntervalRef.current);
    }
    
    const checkInterval = setInterval(async () => {
      const stillInCooldown = await checkCooldownStatus();
      if (!stillInCooldown) {
        clearInterval(checkInterval);
        cooldownCheckIntervalRef.current = null;
        setIsInCooldown(false);
        hasStoppedStreams.current = false;
        setStreamsEnabled(true); // Re-enable streams after cooldown
        addLog('VideoStream', 'info', 'Cooldown expired - streams resuming');
      }
    }, 500); // Check every 500ms (reduced frequency to prevent loops)
    
    cooldownCheckIntervalRef.current = checkInterval;
  }, [checkCooldownStatus, addLog]);

  useEffect(() => {
    const fetchCameras = async () => {
      // Debounce: Don't fetch if we fetched less than 2 seconds ago
      const now = Date.now();
      if (now - lastFetchTimeRef.current < 2000) {
        return;
      }
      lastFetchTimeRef.current = now;
      
      try {
        setLoading(true);
        const response = await fetch('http://localhost:8001/cameras');
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const data = await response.json();
        setCameras(data.cameras);
        
        // Extract and set cooldown status
        const cooldowns = {};
        let anyInCooldown = false;
        Object.entries(data.cameras || {}).forEach(([id, camera]) => {
          if (camera.cooldown) {
            cooldowns[id] = camera.cooldown;
            if (camera.cooldown.in_cooldown) {
              anyInCooldown = true;
            }
          }
        });
        setCooldownStatus(cooldowns);
        setIsInCooldown(anyInCooldown);
        
        addLog('VideoStream', 'info', `Loaded ${Object.keys(data.cameras).length} camera feeds`);
        setError(null);
      } catch (err) {
        const fallback = {
          ceiling: { name: 'Ceiling Camera', type: 'RTSP Stream', status: 'active' },
        };
        setCameras(fallback);
        const msg = `Failed to load cameras: ${err.message}`;
        setError(msg);
        addLog('VideoStream', 'error', msg, err);
      } finally {
        setLoading(false);
      }
    };

    // Clear any pending fetch timeout from previous mount
    if (fetchCamerasTimeoutRef.current) {
      clearTimeout(fetchCamerasTimeoutRef.current);
    }

    // Initial fetch with small delay to allow hot reload detection
    fetchCamerasTimeoutRef.current = setTimeout(fetchCameras, 100);
    const interval = setInterval(fetchCameras, 30000);
    
    return () => {
      clearInterval(interval);
      if (fetchCamerasTimeoutRef.current) {
        clearTimeout(fetchCamerasTimeoutRef.current);
      }
      if (cooldownCheckIntervalRef.current) {
        clearInterval(cooldownCheckIntervalRef.current);
      }
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []); // Only run once on mount

  // On mount: immediately check cooldown status so we don't try to render streams during cooldown
  useEffect(() => {
    (async () => {
      await checkCooldownStatus();
    })();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Function to stop all streams (memoized)
  const stopAllStreams = React.useCallback(async () => {
    // Prevent multiple concurrent stop calls
    if (hasStoppedStreams.current) {
      return;
    }
    
    try {
      hasStoppedStreams.current = true;
      const response = await fetch('http://localhost:8001/stream/stop-all', {
        method: 'POST',
      });
      if (response.ok) {
        const data = await response.json();
        addLog('VideoStream', 'info', `Stopped ${data.stopped_cameras?.length || 0} streams - cooldown ${data.cooldown_seconds}s`);
        setIsInCooldown(true);
        // Start checking cooldown status
        await checkCooldownStatus();
        waitForCooldown();
      }
    } catch (err) {
      addLog('VideoStream', 'warning', `Failed to stop streams: ${err.message}`);
      // Reset flag on error to allow retry
      hasStoppedStreams.current = false;
    }
  }, [checkCooldownStatus, waitForCooldown, addLog]);

  // Handle Page Visibility API - stop streams when tab is hidden
  // Only stop after tab has been hidden for 5+ seconds to avoid brief focus changes
  useEffect(() => {
    const handleVisibilityChange = async () => {
      const isVisible = !document.hidden;
      const inFullscreenMode = document.fullscreenElement || document.webkitFullscreenElement || document.mozFullScreenElement;
      
      setIsTabVisible(isVisible);
      
      if (!isVisible) {
        // Always ignore visibility changes when actively in fullscreen mode
        if (inFullscreenMode) {
          addLog('VideoStream', 'info', 'Visibility change while in fullscreen mode - ignoring');
          return;
        }
        
        // Ignore if we're transitioning to/from fullscreen (covers both enter and exit)
        if (fullscreenTransitionRef.current) {
          addLog('VideoStream', 'info', 'Fullscreen transition in progress - ignoring visibility change');
          return;
        }
        
        // Ignore if our internal state shows we're in fullscreen (even if browser disagrees)
        if (isFullscreen || wasInFullscreenRef.current) {
          addLog('VideoStream', 'info', 'Fullscreen mode active - ignoring visibility change');
          return;
        }
        
        // Tab is hidden - wait 5 seconds before stopping streams
        // This prevents stopping on brief focus changes (clicking console, etc.)
        addLog('VideoStream', 'info', 'Tab hidden - will stop streams if hidden for 5s');
        
        // Clear any existing timeout first
        if (visibilityTimeoutRef.current) {
          clearTimeout(visibilityTimeoutRef.current);
        }
        
        visibilityTimeoutRef.current = setTimeout(() => {
          if (!isMountedRef.current) return; // Component unmounted
          // Double-check we're not in fullscreen before actually stopping
          if (!fullscreenTransitionRef.current && !isFullscreen && !wasInFullscreenRef.current) {
            addLog('VideoStream', 'info', 'Tab still hidden after 5s - disabling streams (will disconnect naturally)');
            setStreamsEnabled(false); // Unmount img elements - they'll disconnect naturally
            // No need to call stop-all - backend handles natural disconnections without cooldown
          } else {
            addLog('VideoStream', 'info', 'Tab hidden timeout cancelled - fullscreen mode detected');
          }
        }, 5000); // Wait 5 seconds before actually stopping
        
      } else {
        // Tab is visible - cancel any pending stop and re-enable if needed
        if (visibilityTimeoutRef.current) {
          clearTimeout(visibilityTimeoutRef.current);
          visibilityTimeoutRef.current = null;
          addLog('VideoStream', 'info', 'Tab visible again - cancelled stream stop');
        }
        
        if (unmountTimeoutRef.current) {
          clearTimeout(unmountTimeoutRef.current);
        }
        
        // Only re-enable if streams were actually stopped AND component is still mounted
        if (!streamsEnabled && isMountedRef.current) {
          addLog('VideoStream', 'info', 'Tab visible - checking cooldown status');
          
          // Check if we're in cooldown
          const inCooldown = await checkCooldownStatus();
          if (inCooldown) {
            addLog('VideoStream', 'info', 'In cooldown - waiting before resuming streams');
            setIsInCooldown(true);
            waitForCooldown();
          } else {
            hasStoppedStreams.current = false;
            setIsInCooldown(false);
            setStreamsEnabled(true); // Remount img elements
            // DON'T force remount with reload token - let existing streams continue
          }
        }
      }
    };

    // Handle fullscreen changes (ESC key exits fullscreen without clicking close button)
    const handleFullscreenChange = () => {
      const inFullscreenMode = document.fullscreenElement || document.webkitFullscreenElement || document.mozFullScreenElement;
      
      if (inFullscreenMode) {
        // Just entered fullscreen - ensure transition flag is set
        fullscreenTransitionRef.current = true;
        wasInFullscreenRef.current = true;
        setTimeout(() => {
          fullscreenTransitionRef.current = false;
        }, 2000);
        addLog('VideoStream', 'info', 'Fullscreen mode activated via browser');
      } else if (!inFullscreenMode && isFullscreen) {
        // Exited fullscreen but our state still shows fullscreen - update it
        addLog('VideoStream', 'info', 'Fullscreen exited via ESC or browser action');
        setIsFullscreen(false);
        setFullscreenCamera(null);
        
        // Mark transition to prevent visibility handler from stopping streams
        fullscreenTransitionRef.current = true;
        wasInFullscreenRef.current = false;
        setTimeout(() => {
          fullscreenTransitionRef.current = false;
        }, 2000); // 2-second grace period
      }
    };

    document.addEventListener('visibilitychange', handleVisibilityChange);
    document.addEventListener('fullscreenchange', handleFullscreenChange);
    document.addEventListener('webkitfullscreenchange', handleFullscreenChange);
    document.addEventListener('mozfullscreenchange', handleFullscreenChange);
    
    return () => {
      document.removeEventListener('visibilitychange', handleVisibilityChange);
      document.removeEventListener('fullscreenchange', handleFullscreenChange);
      document.removeEventListener('webkitfullscreenchange', handleFullscreenChange);
      document.removeEventListener('mozfullscreenchange', handleFullscreenChange);
      
      if (unmountTimeoutRef.current) {
        clearTimeout(unmountTimeoutRef.current);
      }
      if (visibilityTimeoutRef.current) {
        clearTimeout(visibilityTimeoutRef.current);
      }
    };
  }, [stopAllStreams, checkCooldownStatus, waitForCooldown, addLog, streamsEnabled, isFullscreen]);

  // Fullscreen stream health check (DISABLED - use main stream)
  // Fullscreen now reuses the main stream instead of creating a new one
  useEffect(() => {
    // Health check disabled - fullscreen uses same stream as grid view
    return () => {};
  }, [isFullscreen, streamsEnabled, isInCooldown, fullscreenCamera, addLog]);

  // Cleanup: Streams will disconnect naturally when component unmounts
  // No need to call stop-all - backend handles disconnections gracefully
  useEffect(() => {
    // Mark as mounted
    isMountedRef.current = true;
    
    // Clear any pending cleanup from previous unmount (hot reload case)
    if (cleanupTimerRef.current) {
      clearTimeout(cleanupTimerRef.current);
      cleanupTimerRef.current = null;
      addLog('VideoStream', 'info', 'Hot reload detected');
    }
    
    addLog('VideoStream', 'info', 'Camera panel mounted');
    
    // Enable streams after a delay to allow component to stabilize
    // This prevents race conditions during hot reload or rapid navigation
    if (enableStreamsTimeoutRef.current) {
      clearTimeout(enableStreamsTimeoutRef.current);
    }
    
    enableStreamsTimeoutRef.current = setTimeout(() => {
      if (isMountedRef.current) {
        setStreamsEnabled(true);
        addLog('VideoStream', 'info', 'Streams enabled after mount stabilization');
      }
    }, 1000); // Wait 1 second for component to fully mount
    
    return () => {
      // Mark as unmounted
      isMountedRef.current = false;
      
      // Clear enable timeout
      if (enableStreamsTimeoutRef.current) {
        clearTimeout(enableStreamsTimeoutRef.current);
        enableStreamsTimeoutRef.current = null;
      }
      
      // Disable streams first to prevent new connections
      setStreamsEnabled(false);
      
      // Streams will disconnect naturally when img elements unmount
      // Backend detects disconnection and cleans up WITHOUT cooldown
      addLog('VideoStream', 'info', 'Camera panel unmounting - streams will disconnect naturally');
      
      // Reset the stop flag so fresh mounts can stop if needed
      hasStoppedStreams.current = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []); // Only run on mount/unmount

  const handleFullscreen = async (id, camera) => {
    setIsFullscreen(true);
    setFullscreenCamera({ id, ...camera });
    // DON'T reset fullscreen image key - reuse existing stream
    
    // Mark that we're transitioning to fullscreen
    // This prevents visibility handler from stopping streams during transition
    fullscreenTransitionRef.current = true;
    wasInFullscreenRef.current = true;
    
    addLog('VideoStream', 'info', `Entering fullscreen mode for ${camera.name}`);
    
    // Request browser native fullscreen for better experience
    try {
      const elem = document.documentElement;
      if (elem.requestFullscreen) {
        await elem.requestFullscreen();
      } else if (elem.webkitRequestFullscreen) {
        await elem.webkitRequestFullscreen();
      } else if (elem.mozRequestFullScreen) {
        await elem.mozRequestFullScreen();
      }
    } catch (err) {
      // Fallback to custom fullscreen if native fails
      addLog('VideoStream', 'info', 'Native fullscreen not available, using custom fullscreen');
    }
    
    // Clear transition flag after a delay to allow browser events to settle
    setTimeout(() => {
      fullscreenTransitionRef.current = false;
    }, 2000); // 2-second grace period
  };
  
  const exitFullscreen = async () => {
    setIsFullscreen(false);
    setFullscreenCamera(null);
    
    // Mark that we're transitioning out of fullscreen
    // This prevents visibility handler from stopping streams during transition
    fullscreenTransitionRef.current = true;
    wasInFullscreenRef.current = false;
    
    addLog('VideoStream', 'info', 'Exiting fullscreen mode');
    
    // Exit browser native fullscreen if active
    try {
      if (document.fullscreenElement || document.webkitFullscreenElement || document.mozFullScreenElement) {
        if (document.exitFullscreen) {
          await document.exitFullscreen();
        } else if (document.webkitExitFullscreen) {
          await document.webkitExitFullscreen();
        } else if (document.mozCancelFullScreen) {
          await document.mozCancelFullScreen();
        }
      }
    } catch (err) {
      // Ignore errors when exiting fullscreen
    }
    
    // Clear transition flag after a delay to allow browser events to settle
    setTimeout(() => {
      fullscreenTransitionRef.current = false;
    }, 2000); // 2-second grace period
  };

  const handleStreamError = (id) => {
    setStreamErrors(prev => {
      if (prev[id] === true) return prev;
      return { ...prev, [id]: true };
    });
    addLog('VideoStream', 'warning', `Camera ${id} stream error`);
  };
  const handleStreamReady = (id) => {
    setStreamErrors(prev => {
      if (prev[id] === false) return prev;
      return { ...prev, [id]: false };
    });
  };

  const cameraEntries = Object.entries(cameras);
  const visibleCameras = selectedView === 'all' ? cameraEntries : cameraEntries.filter(([id]) => id === selectedView);

  const errorCount = Object.values(streamErrors).filter(Boolean).length;
  const offlineCount = cameraEntries.filter(([, cam]) => cam.status === 'offline').length;
  const totalIssues = errorCount + offlineCount;

  // (Component removed - hoisted to top-level as CameraStreamComponent)

  if (loading) {
    return (
      <div className="p-12 text-center text-gray-500">{t('loadingCameraFeeds')}</div>
    );
  }

  if (isFullscreen && fullscreenCamera) {
    // Reuse the same stream from the CameraStream component - don't create a new connection
    // Find the corresponding camera stream component
    const matchingCamera = visibleCameras.find(([id]) => id === fullscreenCamera.id);
    
    return (
      <div className="fixed inset-0 bg-black z-50 flex flex-col">
        <div className="p-4 bg-gray-900 border-b border-gray-800 flex justify-between items-center">
          <div>
            <h2 className="text-lg font-bold text-white">{fullscreenCamera.id === 'ceiling' ? t('ceilingCameraName') : fullscreenCamera.name}</h2>
            <p className="text-sm text-gray-400">{fullscreenCamera.id === 'ceiling' ? t('rtspStream') : fullscreenCamera.type}</p>
          </div>
          <button onClick={exitFullscreen} className="text-white hover:text-gray-300 p-2 rounded-lg hover:bg-gray-800" title={t('close')}>
            <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
            </svg>
          </button>
        </div>
        <div className="flex-1 relative bg-black">
          {/* Render the same CameraStream component in fullscreen - reuses the connection */}
          {matchingCamera && (
            <div className="w-full h-full">
              <CameraStream
                key={`fullscreen-${matchingCamera[0]}`}
                cameraId={matchingCamera[0]}
                camera={matchingCamera[1]}
                streamsEnabled={streamsEnabled}
                isInCooldown={isInCooldown}
                cameraCooldown={cooldownStatus[matchingCamera[0]] || { in_cooldown: false, remaining_seconds: 0 }}
                isError={!!streamErrors[matchingCamera[0]]}
                onStreamError={handleStreamError}
                onStreamReady={handleStreamReady}
                onFullscreen={handleFullscreen}
                addLog={addLog}
                t={t}
              />
            </div>
          )}
        </div>
      </div>
    );
  }

  return (
    <div className="bg-white rounded-xl shadow border border-gray-200">
      <div className="p-4 flex justify-between items-center border-b border-gray-200">
        <div className="flex items-center space-x-3">
          <h2 className="text-lg font-semibold text-gray-800">{t('liveCameraFeeds')}</h2>
          {totalIssues > 0 && (
            <span className="px-2 py-1 rounded-full bg-red-100 text-red-800 text-xs font-medium">
              {t('countOffline').replace('{count}', totalIssues)}
            </span>
          )}
        </div>
        <select
          value={selectedView}
          onChange={(e) => setSelectedView(e.target.value)}
          className="px-3 py-1.5 border border-gray-300 rounded-md text-sm"
        >
          <option value="all">{t('allCameras')}</option>
          {cameraEntries.map(([id, camera]) => (
            <option key={id} value={id}>{id === 'ceiling' ? t('ceilingCameraName') : camera.name}</option>
          ))}
        </select>
      </div>

      {/* Grid with 2 columns and 2 rows filling the height */}
      <div className="p-4 h-[calc(100vh-160px)] ">
        {visibleCameras.length === 0 ? (
          <p className="text-center text-gray-500 py-8">{t('noCameraFeeds')}</p>
        ) : (
          <div className="grid grid-cols-2 grid-rows-2 gap-4 h-full w-full">
            {visibleCameras.slice(0, 4).map(([id, camera]) => {
              // If we're in fullscreen for this camera, don't render its grid tile
              if (isFullscreen && fullscreenCamera && fullscreenCamera.id === id) {
                return (
                  <div key={id} className="flex">
                    {/* Reserved space while fullscreen is active */}
                  </div>
                );
              }
              return (
                <div key={id} className="flex">
                  <CameraStream
                    cameraId={id}
                    camera={camera}
                    streamsEnabled={streamsEnabled}
                    isInCooldown={isInCooldown}
                    cameraCooldown={cooldownStatus[id] || { in_cooldown: false, remaining_seconds: 0 }}
                    isError={!!streamErrors[id]}
                    onStreamError={handleStreamError}
                    onStreamReady={handleStreamReady}
                    onFullscreen={handleFullscreen}
                    addLog={addLog}
                    t={t}
                  />
                </div>
              );
            })}
          </div>
        )}
      </div>
    </div>
  );
}
