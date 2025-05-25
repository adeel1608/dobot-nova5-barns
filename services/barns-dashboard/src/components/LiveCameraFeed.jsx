import React, { useState, useEffect } from 'react';
import useStore from '../store';

export default function LiveCameraFeed() {
  const { addLog } = useStore();
  const [cameras, setCameras] = useState({});
  const [selectedCamera, setSelectedCamera] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  // Fetch camera list from video-stream service
  useEffect(() => {
    const fetchCameras = async () => {
      try {
        setLoading(true);
        const response = await fetch('http://localhost:8004/cameras');
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
      } finally {
        setLoading(false);
      }
    };

    fetchCameras();
    
    // Refresh camera list every 30 seconds
    const interval = setInterval(fetchCameras, 30000);
    return () => clearInterval(interval);
  }, [addLog]);

  const handleCameraError = (cameraId, error) => {
    addLog('VideoStream', 'warning', `Camera ${cameraId} stream error: ${error}`);
  };

  const CameraStream = ({ cameraId, camera, className = "" }) => {
    const [streamError, setStreamError] = useState(false);
    const streamUrl = `http://localhost:8004/stream/${cameraId}`;

    return (
      <div className={`relative bg-gray-900 rounded-lg overflow-hidden ${className}`}>
        {/* Camera Header */}
        <div className="absolute top-0 left-0 right-0 z-10 bg-gradient-to-b from-black/70 to-transparent p-3">
          <div className="flex justify-between items-start">
            <div>
              <h3 className="text-white font-medium text-sm">{camera.name}</h3>
              <p className="text-gray-300 text-xs">{camera.type}</p>
            </div>
            <div className="flex items-center space-x-2">
              <div className={`w-2 h-2 rounded-full ${
                streamError ? 'bg-red-500' : 
                camera.status === 'active' ? 'bg-green-500' : 'bg-yellow-500'
              }`}></div>
              <span className="text-white text-xs">
                {streamError ? 'Error' : camera.status === 'active' ? 'Live' : 'Offline'}
              </span>
            </div>
          </div>
        </div>

        {/* Video Stream */}
        {!streamError ? (
          <img
            src={streamUrl}
            alt={`${camera.name} feed`}
            className="w-full h-full object-cover"
            onError={() => {
              setStreamError(true);
              handleCameraError(cameraId, 'Stream connection failed');
            }}
            onLoad={() => setStreamError(false)}
          />
        ) : (
          <div className="w-full h-full flex items-center justify-center bg-gray-800">
            <div className="text-center text-gray-400">
              <svg className="mx-auto h-12 w-12 mb-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1} 
                      d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
              <p className="text-sm">Stream Unavailable</p>
              <p className="text-xs mt-1">{camera.name}</p>
            </div>
          </div>
        )}

        {/* Click overlay for fullscreen */}
        <div 
          className="absolute inset-0 cursor-pointer"
          onClick={() => setSelectedCamera({ cameraId, camera, streamUrl })}
          title="Click to view fullscreen"
        />
      </div>
    );
  };

  if (loading) {
    return (
      <div className="bg-white rounded shadow">
        <div className="p-4 border-b border-gray-200">
          <h2 className="text-xl font-bold">Live Camera Feed</h2>
        </div>
        <div className="p-8 text-center">
          <div className="animate-spin h-8 w-8 border-4 border-blue-500 border-t-transparent rounded-full mx-auto mb-4"></div>
          <p className="text-gray-600">Loading camera feeds...</p>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="bg-white rounded shadow">
        <div className="p-4 border-b border-gray-200">
          <h2 className="text-xl font-bold">Live Camera Feed</h2>
        </div>
        <div className="p-8 text-center">
          <div className="text-red-500 mb-4">
            <svg className="mx-auto h-12 w-12 mb-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} 
                    d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
            <p className="font-medium">Video Service Unavailable</p>
            <p className="text-sm mt-1 text-gray-600">{error}</p>
          </div>
          <button 
            onClick={() => window.location.reload()}
            className="px-4 py-2 bg-blue-600 text-white rounded hover:bg-blue-700"
          >
            Retry Connection
          </button>
        </div>
      </div>
    );
  }

  const cameraEntries = Object.entries(cameras);

  return (
    <div className="bg-white rounded shadow">
      <div className="p-4 border-b border-gray-200">
        <div className="flex justify-between items-center">
          <h2 className="text-xl font-bold">Live Camera Feed</h2>
          <div className="flex items-center space-x-4">
            <span className="text-sm text-gray-600">
              {cameraEntries.length} cameras
            </span>
            <div className="flex items-center space-x-1">
              <div className="w-2 h-2 bg-green-500 rounded-full"></div>
              <span className="text-xs text-gray-600">Live</span>
            </div>
          </div>
        </div>
      </div>

      <div className="p-4">
        {cameraEntries.length === 0 ? (
          <div className="text-center py-8 text-gray-500">
            <svg className="mx-auto h-12 w-12 mb-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1} 
                    d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
            </svg>
            <p>No camera feeds available</p>
          </div>
        ) : (
          <div className="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-3 gap-4">
            {cameraEntries.map(([cameraId, camera]) => (
              <CameraStream
                key={cameraId}
                cameraId={cameraId}
                camera={camera}
                className="aspect-video min-h-[200px]"
              />
            ))}
          </div>
        )}
      </div>

      {/* Fullscreen Modal */}
      {selectedCamera && (
        <div className="fixed inset-0 bg-black bg-opacity-90 flex items-center justify-center z-50">
          <div className="relative w-full h-full max-w-6xl max-h-[90vh] p-4">
            {/* Close Button */}
            <button
              onClick={() => setSelectedCamera(null)}
              className="absolute top-4 right-4 z-20 text-white hover:text-gray-300 bg-black bg-opacity-50 rounded-full p-2"
            >
              <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>

            {/* Fullscreen Video */}
            <div className="relative w-full h-full bg-gray-900 rounded-lg overflow-hidden">
              <div className="absolute top-4 left-4 z-10 bg-black bg-opacity-70 rounded px-3 py-2">
                <h3 className="text-white font-medium">{selectedCamera.camera.name}</h3>
                <p className="text-gray-300 text-sm">{selectedCamera.camera.type}</p>
              </div>
              
              <img
                src={selectedCamera.streamUrl}
                alt={`${selectedCamera.camera.name} fullscreen`}
                className="w-full h-full object-contain"
                onError={() => setSelectedCamera(null)}
              />
            </div>
          </div>
        </div>
      )}
    </div>
  );
} 