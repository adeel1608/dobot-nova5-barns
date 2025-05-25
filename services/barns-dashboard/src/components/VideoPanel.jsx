import React, { useState } from 'react';
import ReactPlayer from 'react-player';

export default function VideoPanel() {
  const [selectedView, setSelectedView] = useState('all');
  const [showControls, setShowControls] = useState(true);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [fullscreenCamera, setFullscreenCamera] = useState(null);
  const [streamErrors, setStreamErrors] = useState({});

  const cameras = [
    { id: 'arm1', name: 'Robotic Arm 1', url: 'rtsp://localhost:8554/arm1' },
    { id: 'arm2', name: 'Robotic Arm 2', url: 'rtsp://localhost:8554/arm2' },
    { id: 'conveyor', name: 'Conveyor Belt', url: 'rtsp://localhost:8554/conveyor' },
    { id: 'packaging', name: 'Packaging Area', url: 'rtsp://localhost:8554/packaging' },
  ];

  const handleFullscreen = (camera) => {
    setIsFullscreen(true);
    setFullscreenCamera(camera);
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
  };

  const handleStreamReady = (cameraId) => {
    setStreamErrors(prev => ({ ...prev, [cameraId]: false }));
  };

  // Filter cameras based on selected view
  const visibleCameras = selectedView === 'all' 
    ? cameras 
    : cameras.filter(camera => camera.id === selectedView);

  // Determine grid columns based on number of visible cameras
  const gridColumns = visibleCameras.length === 1 
    ? 'grid-cols-1' 
    : visibleCameras.length === 2 
      ? 'grid-cols-1 md:grid-cols-2' 
      : 'grid-cols-1 md:grid-cols-2';

  if (isFullscreen && fullscreenCamera) {
    return (
      <div className="fixed inset-0 bg-black z-50 flex flex-col">
        <div className="p-4 flex justify-between items-center bg-gray-900">
          <h2 className="text-xl font-bold text-white">{fullscreenCamera.name} - Live Feed</h2>
          <button 
            onClick={exitFullscreen}
            className="text-white hover:text-gray-300"
          >
            <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
            </svg>
          </button>
        </div>
        <div className="flex-1 relative">
          {streamErrors[fullscreenCamera.id] ? (
            <div className="absolute inset-0 flex flex-col items-center justify-center bg-gray-900 text-white">
              <svg className="w-16 h-16 text-red-500 mb-4" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
              </svg>
              <p className="text-xl font-semibold mb-2">Stream Unavailable</p>
              <p className="text-sm text-gray-400 max-w-md text-center">
                The camera stream could not be loaded. Please verify that the camera server is running and accessible.
              </p>
            </div>
          ) : (
            <ReactPlayer 
              url={fullscreenCamera.url} 
              playing 
              controls={showControls} 
              muted 
              width="100%" 
              height="100%"
              onError={() => handleStreamError(fullscreenCamera.id)}
              onReady={() => handleStreamReady(fullscreenCamera.id)}
              style={{ position: 'absolute', top: 0, left: 0 }}
            />
          )}
          <div className="absolute bottom-4 right-4 flex space-x-2">
            <button
              onClick={toggleControls}
              className="p-2 bg-gray-800 bg-opacity-75 rounded-full text-white hover:bg-opacity-100"
            >
              {showControls ? (
                <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" />
                </svg>
              ) : (
                <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13.875 18.825A10.05 10.05 0 0112 19c-4.478 0-8.268-2.943-9.543-7a9.97 9.97 0 011.563-3.029m5.858.908a3 3 0 114.243 4.243M9.878 9.878l4.242 4.242M9.88 9.88l-3.29-3.29m7.532 7.532l3.29 3.29M3 3l3.59 3.59m0 0A9.953 9.953 0 0112 5c4.478 0 8.268 2.943 9.543 7a10.025 10.025 0 01-4.132 5.411m0 0L21 21" />
                </svg>
              )}
            </button>
          </div>
        </div>
      </div>
    );
  }

  // Count error streams
  const errorCount = Object.values(streamErrors).filter(Boolean).length;

  return (
    <div className="bg-white rounded shadow">
      <div className="p-4 border-b border-gray-200 flex flex-col sm:flex-row justify-between items-start sm:items-center space-y-2 sm:space-y-0">
        <div className="flex items-center">
          <h2 className="text-xl font-bold">Live Camera Feeds</h2>
          {errorCount > 0 && (
            <span className="ml-2 inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-red-100 text-red-800">
              {errorCount} Stream{errorCount !== 1 ? 's' : ''} Offline
            </span>
          )}
        </div>
        
        <div className="flex space-x-2">
          <select
            value={selectedView}
            onChange={(e) => setSelectedView(e.target.value)}
            className="px-3 py-1 border border-gray-300 rounded text-sm"
          >
            <option value="all">All Cameras</option>
            {cameras.map(camera => (
              <option key={camera.id} value={camera.id}>{camera.name}</option>
            ))}
          </select>
          
          <button
            onClick={toggleControls}
            className="px-3 py-1 border border-gray-300 rounded text-sm hover:bg-gray-50"
          >
            {showControls ? 'Hide Controls' : 'Show Controls'}
          </button>
        </div>
      </div>
      
      <div className={`grid ${gridColumns} gap-4 p-4`}>
        {visibleCameras.map(camera => (
          <div key={camera.id} className="bg-gray-900 rounded overflow-hidden relative">
            <div className="aspect-video">
              {streamErrors[camera.id] ? (
                <div className="absolute inset-0 flex flex-col items-center justify-center bg-gray-900 text-white">
                  <svg className="w-10 h-10 text-red-500 mb-2" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
                  </svg>
                  <p className="text-sm font-semibold">Stream Unavailable</p>
                </div>
              ) : (
                <ReactPlayer 
                  url={camera.url} 
                  playing 
                  controls={showControls} 
                  muted 
                  width="100%" 
                  height="100%" 
                  onError={() => handleStreamError(camera.id)}
                  onReady={() => handleStreamReady(camera.id)}
                />
              )}
            </div>
            <div className="absolute top-2 left-2 right-2 flex justify-between items-center">
              <span className={`${streamErrors[camera.id] ? 'bg-red-500' : 'bg-black bg-opacity-70'} text-white text-xs px-2 py-1 rounded`}>
                {camera.name}
                {streamErrors[camera.id] && ' (Offline)'}
              </span>
              <button
                onClick={() => handleFullscreen(camera)}
                className="bg-black bg-opacity-70 text-white p-1 rounded hover:bg-opacity-100"
                title="Fullscreen"
                disabled={streamErrors[camera.id]}
              >
                <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5v-4m0 4h-4m4 0l-5-5" />
                </svg>
              </button>
            </div>
          </div>
        ))}
      </div>
      
      <div className="px-4 pb-4">
        <p className="text-sm text-gray-500">
          {Object.values(streamErrors).some(Boolean) 
            ? '* Some camera streams are unavailable. Check camera server connections.'
            : '* If RTSP streams are not loading, verify that the camera server is running and accessible.'}
        </p>
      </div>
    </div>
  );
}
