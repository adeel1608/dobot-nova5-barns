import React, { useState } from 'react';
import useStore from '../store';

export default function SystemControls() {
  const { stopSystem, resumeOperation, isLoading, errors, clearError } = useStore();
  const [showConfirm, setShowConfirm] = useState(false);
  const [stopReason, setStopReason] = useState('');

  const handleStopClick = () => {
    setShowConfirm(true);
  };

  const confirmStop = async () => {
    try {
      const success = await stopSystem(stopReason);
      if (success) {
        setShowConfirm(false);
        setStopReason('');
      }
    } catch (err) {
      console.error("Failed to stop system:", err);
    }
  };

  const cancelStop = () => {
    setShowConfirm(false);
    setStopReason('');
  };

  const retrySystemOperation = () => {
    clearError('system');
  };

  return (
    <div className="card card-elevated">
      <div className="p-6 border-b border-gray-200 flex justify-between items-center">
        <div className="flex items-center space-x-3">
          <div className="p-2 bg-gradient-to-br from-green-500 to-green-600 rounded-lg">
            <svg className="w-6 h-6 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 100 4m0-4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 100 4m0-4v2m0-6V4" />
            </svg>
          </div>
          <div>
            <h2 className="text-xl font-bold text-gray-900">System Controls</h2>
            <p className="text-sm text-gray-600">Manage BARNS system operations</p>
          </div>
          {errors.system && (
            <span className="badge badge-error">
              API Error
            </span>
          )}
        </div>
        
        {errors.system && (
          <button 
            onClick={retrySystemOperation}
            className="btn-secondary text-sm"
          >
            <svg className="w-4 h-4 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0V9a8 8 0 1115.356 2m-15.356-2H9" />
            </svg>
            Retry Connection
          </button>
        )}
      </div>
      
      {/* API Error display */}
      {errors.system && (
        <div className="border-b border-red-200 bg-gradient-to-r from-red-50 to-pink-50 px-6 py-4">
          <div className="flex items-start space-x-3">
            <svg className="w-6 h-6 text-red-500 mt-0.5 flex-shrink-0" fill="currentColor" viewBox="0 0 20 20">
              <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7 4a1 1 0 11-2 0 1 1 0 012 0zm-1-9a1 1 0 00-1 1v4a1 1 0 102 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
            </svg>
            <div className="flex-1">
              <div className="flex items-center justify-between">
                <h4 className="text-base font-semibold text-red-800">System Control Error</h4>
                <button
                  onClick={retrySystemOperation}
                  className="inline-flex items-center px-3 py-1 bg-red-100 hover:bg-red-200 text-red-800 text-sm font-medium rounded-md transition-colors duration-200"
                >
                  <svg className="w-4 h-4 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0V9a8 8 0 1115.356 2m-15.356-2H9" />
                  </svg>
                  Retry
                </button>
              </div>
              <p className="text-sm text-red-700 mt-1 font-medium">{errors.system}</p>
              
              {/* Helpful suggestions based on error type */}
              <div className="mt-3 p-3 bg-red-100 rounded-lg border border-red-200">
                <p className="text-xs font-medium text-red-800 mb-2">💡 Troubleshooting Steps:</p>
                <ul className="text-xs text-red-700 space-y-1">
                  {errors.system.includes('offline') || errors.system.includes('unreachable') ? (
                    <>
                      <li>• Check if the BARNS services are running</li>
                      <li>• Verify network connectivity to the system</li>
                      <li>• Try refreshing the page or reconnecting</li>
                    </>
                  ) : errors.system.includes('not found') || errors.system.includes('404') ? (
                    <>
                      <li>• The system control feature may need to be updated</li>
                      <li>• Contact system administrator for assistance</li>
                      <li>• Check system logs for more details</li>
                    </>
                  ) : errors.system.includes('timeout') ? (
                    <>
                      <li>• The system may be busy processing other requests</li>
                      <li>• Wait a moment and try again</li>
                      <li>• Check system performance and resources</li>
                    </>
                  ) : (
                    <>
                      <li>• Try the operation again in a few moments</li>
                      <li>• Check the System Logs tab for detailed information</li>
                      <li>• Contact support if the problem persists</li>
                    </>
                  )}
                </ul>
              </div>
              
              <div className="flex items-center space-x-3 mt-3">
                <span className="inline-flex items-center text-xs text-red-600 font-medium">
                  <svg className="w-3 h-3 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
                  </svg>
                  Check the System Logs tab for more details
                </span>
                <button
                  onClick={() => clearError('system')}
                  className="inline-flex items-center text-xs text-gray-500 hover:text-gray-700 font-medium"
                >
                  <svg className="w-3 h-3 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                  </svg>
                  Dismiss
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
      
      <div className="p-6">
        <div className="grid gap-6 lg:grid-cols-2">
          {/* Emergency Controls */}
          <div className="bg-gradient-to-br from-red-50 via-red-100 to-red-150 p-8 rounded-2xl border-2 border-red-200 shadow-lg">
            <div className="flex items-center space-x-4 mb-6">
              <div className="p-3 bg-gradient-to-br from-red-500 to-red-600 rounded-xl shadow-lg">
                <svg className="w-7 h-7 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.996-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z" />
                </svg>
              </div>
              <div>
                <h3 className="text-lg font-bold text-gray-900">Emergency Controls</h3>
                <p className="text-sm text-red-600 font-medium">Critical system operations</p>
              </div>
            </div>
            
            <div className="space-y-4">
              {/* Emergency Stop Button - More prominent */}
              <div className="relative">
                <button
                  onClick={handleStopClick}
                  disabled={isLoading || showConfirm || errors.system}
                  className="w-full bg-gradient-to-r from-red-600 via-red-700 to-red-800 hover:from-red-700 hover:via-red-800 hover:to-red-900 disabled:from-red-300 disabled:to-red-400 text-white font-bold py-5 px-8 rounded-xl uppercase tracking-wider flex items-center justify-center transition-all duration-300 shadow-xl hover:shadow-2xl disabled:shadow-none transform hover:scale-[1.02] disabled:scale-100 border-2 border-red-800 hover:border-red-900"
                >
                  {isLoading ? (
                    <>
                      <svg className="animate-spin h-6 w-6 mr-3" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                        <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                        <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                      </svg>
                      <span className="text-lg">Stopping System...</span>
                    </>
                  ) : (
                    <>
                      <svg className="w-7 h-7 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={3} d="M6 18L18 6M6 6l12 12" />
                      </svg>
                      <span className="text-lg">🛑 Emergency Stop</span>
                    </>
                  )}
                </button>
                {!errors.system && (
                  <div className="absolute -top-1 -right-1 w-4 h-4 bg-red-500 rounded-full animate-pulse shadow-lg"></div>
                )}
              </div>

              {/* Divider */}
              <div className="flex items-center my-6">
                <div className="flex-grow border-t border-red-300"></div>
                <span className="flex-shrink mx-4 text-red-500 text-sm font-medium">OR</span>
                <div className="flex-grow border-t border-red-300"></div>
              </div>
              
              {/* Resume Operation Button - Enhanced */}
              <button
                onClick={resumeOperation}
                disabled={isLoading || errors.system}
                className="w-full bg-gradient-to-r from-green-600 via-green-700 to-green-800 hover:from-green-700 hover:via-green-800 hover:to-green-900 disabled:from-gray-300 disabled:to-gray-400 text-white font-semibold py-4 px-6 rounded-xl flex items-center justify-center transition-all duration-300 shadow-lg hover:shadow-xl disabled:shadow-none transform hover:scale-[1.01] disabled:scale-100 border-2 border-green-800 hover:border-green-900"
              >
                {isLoading ? (
                  <>
                    <svg className="animate-spin h-5 w-5 mr-3" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                      <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                      <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                    </svg>
                    <span>Resuming Operations...</span>
                  </>
                ) : (
                  <>
                    <svg className="w-6 h-6 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14.828 14.828a4 4 0 01-5.656 0M9 10h1m4 0h1m-6 4h.01M19 10a9 9 0 11-18 0 9 9 0 0118 0z" />
                    </svg>
                    <span className="text-base">▶️ Resume Operation</span>
                  </>
                )}
              </button>

              {/* Safety Notice */}
              <div className="mt-6 p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
                <div className="flex items-start space-x-3">
                  <svg className="w-5 h-5 text-yellow-600 mt-0.5 flex-shrink-0" fill="currentColor" viewBox="0 0 20 20">
                    <path fillRule="evenodd" d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
                  </svg>
                  <div>
                    <p className="text-sm font-medium text-yellow-800">Safety Notice</p>
                    <p className="text-xs text-yellow-700 mt-1">Emergency stop will halt all robotic operations immediately. Use only when necessary.</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
          
          {/* System Status */}
          <div className="bg-gradient-to-br from-green-50 to-emerald-100 p-6 rounded-xl border border-green-200">
            <div className="flex items-center space-x-3 mb-4">
              <div className="p-2 bg-green-500 rounded-lg">
                <svg className="w-5 h-5 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                </svg>
              </div>
              <h3 className="font-semibold text-gray-900">System Status</h3>
            </div>
            <div className="space-y-3">
              <div className="flex items-center justify-between p-3 bg-white bg-opacity-70 rounded-lg">
                <div className="flex items-center space-x-3">
                  <div className={`w-3 h-3 rounded-full ${errors.system ? 'bg-red-500' : 'bg-green-500'} shadow-sm`}></div>
                  <span className="font-medium text-gray-700">Robotic Arms</span>
                </div>
                <span className={`badge ${errors.system ? 'badge-error' : 'badge-success'}`}>
                  {errors.system ? 'Offline' : 'Online'}
                </span>
              </div>
              
              <div className="flex items-center justify-between p-3 bg-white bg-opacity-70 rounded-lg">
                <div className="flex items-center space-x-3">
                  <div className={`w-3 h-3 rounded-full ${errors.system ? 'bg-red-500' : 'bg-green-500'} shadow-sm`}></div>
                  <span className="font-medium text-gray-700">Vision System</span>
                </div>
                <span className={`badge ${errors.system ? 'badge-error' : 'badge-success'}`}>
                  {errors.system ? 'Offline' : 'Active'}
                </span>
              </div>
              
              <div className="flex items-center justify-between p-3 bg-white bg-opacity-70 rounded-lg">
                <div className="flex items-center space-x-3">
                  <div className={`w-3 h-3 rounded-full ${errors.system ? 'bg-red-500' : 'bg-green-500'} shadow-sm`}></div>
                  <span className="font-medium text-gray-700">Scheduler</span>
                </div>
                <span className={`badge ${errors.system ? 'badge-error' : 'badge-success'}`}>
                  {errors.system ? 'Stopped' : 'Running'}
                </span>
              </div>
              
              <div className="flex items-center justify-between p-3 bg-white bg-opacity-70 rounded-lg">
                <div className="flex items-center space-x-3">
                  <div className={`w-3 h-3 rounded-full ${errors.websocket ? 'bg-red-500' : 'bg-green-500'} shadow-sm`}></div>
                  <span className="font-medium text-gray-700">WebSocket</span>
                </div>
                <span className={`badge ${errors.websocket ? 'badge-error' : 'badge-success'}`}>
                  {errors.websocket ? 'Disconnected' : 'Connected'}
                </span>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Stop Confirmation Modal */}
      {showConfirm && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50 p-4">
          <div className="card max-w-md w-full">
            <div className="p-6">
              <div className="flex items-center space-x-3 mb-4">
                <div className="p-2 bg-red-500 rounded-lg">
                  <svg className="w-6 h-6 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.996-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z" />
                  </svg>
                </div>
                <h3 className="text-xl font-bold text-red-600">Confirm Emergency Stop</h3>
              </div>
              
              <p className="text-gray-700 mb-4">This will immediately halt all system operations. Are you sure you want to proceed?</p>
              
              <div className="mb-6">
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Reason for stopping (optional):
                </label>
                <textarea
                  value={stopReason}
                  onChange={(e) => setStopReason(e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-green-500 focus:border-green-500"
                  rows="3"
                  placeholder="Enter reason for emergency stop..."
                ></textarea>
              </div>
              
              <div className="flex justify-end space-x-3">
                <button
                  onClick={cancelStop}
                  className="btn-secondary"
                >
                  Cancel
                </button>
                <button
                  onClick={confirmStop}
                  className="bg-gradient-to-r from-red-600 to-red-700 hover:from-red-700 hover:to-red-800 text-white font-medium py-2 px-4 rounded-lg transition-all duration-200"
                >
                  Confirm Stop
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
