import React, { useState, useEffect } from 'react';
import useStore from '../../../store';

export default function UnifiedCameraPanel() {
  const { addLog } = useStore();
  const [cameras, setCameras] = useState({});
  const [selectedView, setSelectedView] = useState('all');
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [fullscreenCamera, setFullscreenCamera] = useState(null);
  const [streamErrors, setStreamErrors] = useState({});
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    const fetchCameras = async () => {
      try {
        setLoading(true);
        const response = await fetch('http://localhost:8001/cameras');
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const data = await response.json();
        setCameras(data.cameras);
        addLog('VideoStream', 'info', `Loaded ${Object.keys(data.cameras).length} camera feeds`);
        setError(null);
      } catch (err) {
        const fallback = {
          ceiling: { name: 'Ceiling Camera', type: 'RTSP Stream', status: 'active' },
          pattern: { name: 'Pattern Demo', type: 'Test Video Stream', status: 'active' },
          camera1: { name: 'Camera 1', type: 'Main Area', status: 'offline' },
          camera2: { name: 'Camera 2', type: 'Secondary View', status: 'offline' }
        };
        setCameras(fallback);
        const msg = `Failed to load cameras: ${err.message}`;
        setError(msg);
        addLog('VideoStream', 'error', msg, err);
      } finally {
        setLoading(false);
      }
    };

    fetchCameras();
    const interval = setInterval(fetchCameras, 30000);
    return () => clearInterval(interval);
  }, [addLog]);

  const handleFullscreen = (id, camera) => {
    setIsFullscreen(true);
    setFullscreenCamera({ id, ...camera });
  };
  const exitFullscreen = () => {
    setIsFullscreen(false);
    setFullscreenCamera(null);
  };

  const handleStreamError = (id) => {
    setStreamErrors(prev => ({ ...prev, [id]: true }));
    addLog('VideoStream', 'warning', `Camera ${id} stream error`);
  };
  const handleStreamReady = (id) => {
    setStreamErrors(prev => ({ ...prev, [id]: false }));
  };

  const cameraEntries = Object.entries(cameras);
  const visibleCameras = selectedView === 'all' ? cameraEntries : cameraEntries.filter(([id]) => id === selectedView);

  const errorCount = Object.values(streamErrors).filter(Boolean).length;
  const offlineCount = cameraEntries.filter(([, cam]) => cam.status === 'offline').length;
  const totalIssues = errorCount + offlineCount;

  const CameraStream = ({ cameraId, camera }) => {
    const [imageError, setImageError] = useState(false);
    const isOffline = camera.status === 'offline' || streamErrors[cameraId] || imageError;
    const streamUrl = `http://localhost:8001/stream/${cameraId}`;

    return (
      <div className="bg-white rounded-xl shadow-md border border-gray-200 overflow-hidden relative group hover:shadow-lg transition-all duration-300 w-full h-full">
        <div className="w-full h-full relative bg-gray-100">
          {isOffline ? (
            <div className="absolute inset-0 flex flex-col items-center justify-center bg-gray-900 text-white">
              <svg className="w-14 h-14 text-red-400 mb-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
              <p className="text-sm font-semibold">{camera.name}</p>
              <p className="text-xs text-gray-300">Stream Unavailable</p>
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

          {/* Overlay */}
          <div className="absolute inset-0 bg-gradient-to-t from-black/70 via-transparent to-black/70 opacity-90">
            <div className="absolute top-3 left-3 right-3 flex justify-between items-start">
              <div className="bg-black/70 px-3 py-2 rounded-lg">
                <h3 className="text-white font-semibold text-sm">{camera.name}</h3>
                <p className="text-gray-300 text-xs">{camera.type}</p>
              </div>
              <div className="flex items-center space-x-2">
                <div className={`w-2.5 h-2.5 rounded-full ${isOffline ? 'bg-red-500' : 'bg-green-500'} animate-pulse`} />
                <span className={`text-xs px-2 py-0.5 rounded-md ${isOffline ? 'bg-red-500' : 'bg-green-500'} text-white`}>
                  {isOffline ? 'Offline' : 'Live'}
                </span>
              </div>
            </div>
            <div className="absolute bottom-3 right-3">
              <button
                onClick={() => !isOffline && handleFullscreen(cameraId, camera)}
                disabled={isOffline}
                className="bg-black/70 text-white p-2 rounded-md hover:bg-black/90 disabled:opacity-50 transition-all group-hover:opacity-100 opacity-0 duration-300"
                title="Fullscreen"
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
  };

  if (loading) {
    return (
      <div className="p-12 text-center text-gray-500">Loading camera feeds...</div>
    );
  }

  if (isFullscreen && fullscreenCamera) {
    const streamUrl = `http://localhost:8001/stream/${fullscreenCamera.id}`;
    const isOffline = fullscreenCamera.status === 'offline' || streamErrors[fullscreenCamera.id];
    return (
      <div className="fixed inset-0 bg-black z-50 flex flex-col">
        <div className="p-4 bg-gray-900 border-b border-gray-800 flex justify-between items-center">
          <div>
            <h2 className="text-lg font-bold text-white">{fullscreenCamera.name}</h2>
            <p className="text-sm text-gray-400">{fullscreenCamera.type}</p>
          </div>
          <button onClick={exitFullscreen} className="text-white hover:text-gray-300 p-2 rounded-lg hover:bg-gray-800">
            <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
            </svg>
          </button>
        </div>
        <div className="flex-1 relative bg-black">
          {isOffline ? (
            <div className="absolute inset-0 flex flex-col items-center justify-center text-white">
              <p className="text-lg font-semibold mb-2">Stream Unavailable</p>
              <p className="text-sm text-gray-400">Please check the camera feed or connection.</p>
            </div>
          ) : (
            <img
              src={streamUrl}
              alt="fullscreen camera"
              className="w-full h-full object-contain"
              onError={() => handleStreamError(fullscreenCamera.id)}
            />
          )}
        </div>
      </div>
    );
  }

  return (
    <div className="bg-white rounded-xl shadow border border-gray-200">
      <div className="p-4 flex justify-between items-center border-b border-gray-200">
        <div className="flex items-center space-x-3">
          <h2 className="text-lg font-semibold text-gray-800">Live Camera Feeds</h2>
          {totalIssues > 0 && (
            <span className="px-2 py-1 rounded-full bg-red-100 text-red-800 text-xs font-medium">
              {totalIssues} Offline
            </span>
          )}
        </div>
        <select
          value={selectedView}
          onChange={(e) => setSelectedView(e.target.value)}
          className="px-3 py-1.5 border border-gray-300 rounded-md text-sm"
        >
          <option value="all">All Cameras</option>
          {cameraEntries.map(([id, camera]) => (
            <option key={id} value={id}>{camera.name}</option>
          ))}
        </select>
      </div>

      {/* Grid with 2 columns and 2 rows filling the height */}
      <div className="p-4 h-[calc(100vh-160px)] ">
        {visibleCameras.length === 0 ? (
          <p className="text-center text-gray-500 py-8">No camera feeds available.</p>
        ) : (
          <div className="grid grid-cols-2 grid-rows-2 gap-4 h-full w-full">
            {visibleCameras.slice(0, 4).map(([id, camera]) => (
              <div key={id} className="flex">
                <CameraStream cameraId={id} camera={camera} />
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
