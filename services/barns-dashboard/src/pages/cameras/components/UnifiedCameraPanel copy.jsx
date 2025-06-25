import React, { useState, useEffect } from 'react';
import useStore from '../../../store';

export default function UnifiedCameraPanel() {
  const { addLog } = useStore();
  const [cameras, setCameras] = useState({});
  const [selectedView, setSelectedView] = useState('all');
  const [showControls, setShowControls] = useState(true);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [fullscreenCamera, setFullscreenCamera] = useState(null);
  const [streamErrors, setStreamErrors] = useState({});
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  // Fetch camera list from video-stream service
  useEffect(() => {
    const fetchCameras = async () => {
      try {
        setLoading(true);
        const response = await fetch('http://localhost:8001/cameras');
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        const data = await response.json();
        setCameras(data.cameras);
        addLog('VideoStream', 'info', `Loaded ${Object.keys(data.cameras).length} camera feeds`);
        setError(null);
      } catch (err) {
        const errorMsg = `Failed to load cameras: ${err.message}`;
        setError(errorMsg);
        addLog('VideoStream', 'error', errorMsg, err);
        
        // Fallback to mock data for demo purposes
        const mockCameras = {
          'webcam': { name: 'Live Webcam', type: 'Test Video Stream', status: 'active' },
          'pattern': { name: 'Pattern Demo', type: 'Test Video Stream', status: 'active' },
          'camera1': { name: 'Camera 1', type: 'Main Area', status: 'offline' },
          'camera2': { name: 'Camera 2', type: 'Secondary View', status: 'offline' }
        };
        setCameras(mockCameras);
      } finally {
        setLoading(false);
      }
    };

    fetchCameras();
    
    // Refresh camera list every 30 seconds
    const interval = setInterval(fetchCameras, 30000);
    return () => clearInterval(interval);
  }, [addLog]);

  const handleFullscreen = (cameraId, camera) => {
    setIsFullscreen(true);
    setFullscreenCamera({ id: cameraId, ...camera });
  };

  const exitFullscreen = () => {
    setIsFullscreen(false);
    setFullscreenCamera(null);
  };

  const toggleControls = () => {
    setShowControls(!showControls);
  };

  const handleStreamError = (cameraId) => {
    setStreamErrors(prev => ({ ...prev, [cameraId]: true }));
    addLog('VideoStream', 'warning', `Camera ${cameraId} stream error`);
  };

  const handleStreamReady = (cameraId) => {
    setStreamErrors(prev => ({ ...prev, [cameraId]: false }));
  };

  // Convert camera object to array for easier handling
  const cameraEntries = Object.entries(cameras);
  
  // Filter cameras based on selected view
  const visibleCameras = selectedView === 'all' 
    ? cameraEntries 
    : cameraEntries.filter(([cameraId]) => cameraId === selectedView);

  // Optimized grid layout for 2x2 with responsive fallbacks
  const getGridColumns = (count) => {
    if (count === 1) return 'grid-cols-1';
    if (count <= 4) return 'grid-cols-1 md:grid-cols-2';
    return 'grid-cols-1 md:grid-cols-2 lg:grid-cols-3';
  };

  // Count error streams and offline cameras
  const errorCount = Object.values(streamErrors).filter(Boolean).length;
  const offlineCount = cameraEntries.filter(([, camera]) => camera.status === 'offline').length;
  const totalIssues = errorCount + offlineCount;

  const CameraStream = ({ cameraId, camera }) => {
    const [imageError, setImageError] = useState(false);
    const streamUrl = `http://localhost:8001/stream/${cameraId}`;
    const isOffline = camera.status === 'offline' || streamErrors[cameraId] || imageError;

    return (
      <div className="bg-white rounded-xl shadow-lg overflow-hidden relative group hover:shadow-xl transition-all duration-300 border border-gray-200 pt-4">
        <div className="aspect-video bg-gray-100 relative">
          {isOffline ? (
            <div className="absolute inset-0 flex flex-col items-center justify-center bg-gray-900 text-white">
              <svg className="w-16 h-16 text-red-400 mb-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} 
                      d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
              <p className="text-lg font-medium mb-2">Stream Unavailable</p>
              <p className="text-sm text-gray-400">{camera.name}</p>
            </div>
          ) : (
            <img
              src={streamUrl}
              alt={`${camera.name} feed`}
              className="w-full h-full object-cover"
              onError={() => {
                setImageError(true);
                handleStreamError(cameraId);
              }}
              onLoad={() => {
                setImageError(false);
                handleStreamReady(cameraId);
              }}
            />
          )}
          
          {/* Overlay with camera info */}
          <div className="absolute inset-0 bg-gradient-to-t from-black/70 via-transparent to-black/70 opacity-90">
            <div className="absolute top-4 left-4 right-4 flex justify-between items-start">
              <div className="bg-black/80 backdrop-blur-sm rounded-xl px-4 py-3">
                <h3 className="text-white font-semibold text-base">{camera.name}</h3>
                <p className="text-gray-300 text-sm">{camera.type}</p>
              </div>
              <div className="flex items-center space-x-3">
                <div className={`w-3 h-3 rounded-full ${
                  isOffline ? 'bg-red-500' : 'bg-green-500'
                } animate-pulse`}></div>
                <span className={`text-sm font-semibold px-3 py-1.5 rounded-lg ${
                  isOffline 
                    ? 'bg-red-500 text-white' 
                    : 'bg-green-500 text-white'
                }`}>
                  {isOffline ? 'Offline' : 'Live'}
                </span>
              </div>
            </div>
            
            <div className="absolute bottom-4 right-4">
              <button
                onClick={() => !isOffline && handleFullscreen(cameraId, camera)}
                disabled={isOffline}
                className="bg-black/80 backdrop-blur-sm text-white p-3 rounded-xl hover:bg-black/90 disabled:opacity-50 disabled:cursor-not-allowed transition-all group-hover:opacity-100 opacity-0 duration-300"
                title="Fullscreen"
              >
                <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5v-4m0 4h-4m4 0l-5-5" />
                </svg>
              </button>
            </div>
          </div>
        </div>
      </div>
    );
  };

  if (loading) {
    return (
      <div className="bg-white rounded-xl shadow-sm border border-gray-200">
        <div className="p-6 border-b border-gray-200">
          <h2 className="text-xl font-semibold text-gray-900">Live Camera Feeds</h2>
        </div>
        <div className="p-12 text-center">
          <div className="animate-spin h-10 w-10 border-4 border-blue-500 border-t-transparent rounded-full mx-auto mb-4"></div>
          <p className="text-gray-600">Loading camera feeds...</p>
        </div>
      </div>
    );
  }

  // Fullscreen view
  if (isFullscreen && fullscreenCamera) {
    const streamUrl = `http://localhost:8001/stream/${fullscreenCamera.id}`;
    const isOffline = fullscreenCamera.status === 'offline' || streamErrors[fullscreenCamera.id];

    return (
      <div className="fixed inset-0 bg-black z-50 flex flex-col">
        <div className="p-4 flex justify-between items-center bg-gray-900 border-b border-gray-700">
          <div>
            <h2 className="text-xl font-bold text-white">{fullscreenCamera.name}</h2>
            <p className="text-gray-300 text-sm">{fullscreenCamera.type}</p>
          </div>
          <div className="flex items-center space-x-4">
            <div className="flex items-center space-x-2">
              <div className={`w-3 h-3 rounded-full ${
                isOffline ? 'bg-red-500' : 'bg-green-500'
              }`}></div>
              <span className="text-white text-sm">
                {isOffline ? 'Offline' : 'Live'}
              </span>
            </div>
            <button 
              onClick={exitFullscreen}
              className="text-white hover:text-gray-300 p-2 rounded-lg hover:bg-gray-800 transition-colors"
            >
              <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>
        </div>
        <div className="flex-1 relative">
          {isOffline ? (
            <div className="absolute inset-0 flex flex-col items-center justify-center bg-gray-900 text-white">
              <svg className="w-20 h-20 text-red-400 mb-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} 
                      d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
              <p className="text-xl font-semibold mb-2">Stream Unavailable</p>
              <p className="text-sm text-gray-400 max-w-md text-center">
                The camera stream could not be loaded. Please verify that the camera server is running and accessible.
              </p>
            </div>
          ) : (
            <img
              src={streamUrl}
              alt={`${fullscreenCamera.name} fullscreen`}
              className="w-full h-full object-contain"
              onError={() => handleStreamError(fullscreenCamera.id)}
            />
          )}
        </div>
      </div>
    );
  }

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-200">
      {/* Compact Header */}
      <div className="p-4 border-b border-gray-200">
        <div className="flex flex-col sm:flex-row justify-between items-start sm:items-center space-y-3 sm:space-y-0">
          <div className="flex items-center space-x-3">
            <h2 className="text-lg font-semibold text-gray-900">Live Camera Feeds</h2>
            {totalIssues > 0 && (
              <span className="inline-flex items-center px-2.5 py-1 rounded-full text-xs font-medium bg-red-100 text-red-800">
                {totalIssues} Stream{totalIssues !== 1 ? 's' : ''} Offline
              </span>
            )}
            {error && (
              <span className="inline-flex items-center px-2.5 py-1 rounded-full text-xs font-medium bg-yellow-100 text-yellow-800">
                API Error
              </span>
            )}
          </div>
          
          <div className="flex items-center space-x-3">
            <div className="flex items-center space-x-2 text-sm text-gray-600">
              <span>{cameraEntries.length} camera{cameraEntries.length !== 1 ? 's' : ''}</span>
              <div className="flex items-center space-x-1">
                <div className="w-2 h-2 bg-green-500 rounded-full"></div>
                <span className="text-xs">Live</span>
              </div>
            </div>
            
            <select
              value={selectedView}
              onChange={(e) => setSelectedView(e.target.value)}
              className="px-3 py-1.5 border border-gray-300 rounded-lg text-sm bg-white focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            >
              <option value="all">All Cameras</option>
              {cameraEntries.map(([cameraId, camera]) => (
                <option key={cameraId} value={cameraId}>{camera.name}</option>
              ))}
            </select>
          </div>
        </div>
      </div>

      {/* Camera Grid - Optimized for larger feeds */}
      <div className="p-6">
        {error && (
          <div className="mb-6 p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
            <div className="flex items-center">
              <svg className="w-5 h-5 text-yellow-600 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
              </svg>
              <div>
                <p className="text-sm font-medium text-yellow-800">Video Service Issue</p>
                <p className="text-xs text-yellow-700 mt-1">{error} - Using fallback data for display.</p>
              </div>
            </div>
          </div>
        )}
        
        {cameraEntries.length === 0 ? (
          <div className="text-center py-16 text-gray-500">
            <svg className="mx-auto h-20 w-20 mb-4 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1} 
                    d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
            </svg>
            <h3 className="text-lg font-medium text-gray-900 mb-2">No Camera Feeds Available</h3>
            <p className="text-gray-600">Please check your camera configuration and server connection.</p>
          </div>
        ) : (
          <div className={`grid ${getGridColumns(visibleCameras.length)} gap-6`}>
            {visibleCameras.map(([cameraId, camera]) => (
              <CameraStream
                key={cameraId}
                cameraId={cameraId}
                camera={camera}
              />
            ))}
          </div>
        )}
      </div>

      {/* Footer info */}
      {cameraEntries.length > 0 && (
        <div className="px-6 pb-4">
          <p className="text-xs text-gray-500">
            {totalIssues > 0 
              ? '* Some camera streams are unavailable. Check camera server connections.'
              : '* All camera feeds are operational. Click any camera for fullscreen view.'}
          </p>
        </div>
      )}
    </div>
  );
} 