import React, { useState } from 'react';
import useStore from '../store';

export default function AlertPanel() {
  const { alerts, mockAlerts, resumeOperation, isLoading, errors, clearError } = useStore();
  const [selectedAlert, setSelectedAlert] = useState(null);

  // Use mock data if there's an API error
  const displayAlerts = errors.alerts ? mockAlerts : alerts;

  const acknowledgeAlert = (alertId) => {
    // Implement alert acknowledgement
    console.log(`Acknowledging alert ${alertId}`);
    // In a real implementation, you would call an API endpoint to acknowledge the alert
  };

  const handleAlertClick = (alert) => {
    setSelectedAlert(alert);
  };

  const closeAlertDetails = () => {
    setSelectedAlert(null);
  };

  const retryFetchAlerts = () => {
    clearError('alerts');
    useStore.getState().fetchAlerts();
  };

  // Helper function to get the severity badge
  const getSeverityBadge = (severity) => {
    switch (severity?.toLowerCase()) {
      case 'critical':
        return <span className="bg-red-100 text-red-800 text-xs font-medium px-2.5 py-0.5 rounded">Critical</span>;
      case 'high':
        return <span className="bg-orange-100 text-orange-800 text-xs font-medium px-2.5 py-0.5 rounded">High</span>;
      case 'medium':
        return <span className="bg-yellow-100 text-yellow-800 text-xs font-medium px-2.5 py-0.5 rounded">Medium</span>;
      case 'low':
        return <span className="bg-blue-100 text-blue-800 text-xs font-medium px-2.5 py-0.5 rounded">Low</span>;
      default:
        return <span className="bg-gray-100 text-gray-800 text-xs font-medium px-2.5 py-0.5 rounded">Info</span>;
    }
  };

  return (
    <div className="bg-white rounded shadow">
      <div className="p-4 border-b border-gray-200 flex justify-between items-center">
        <div className="flex items-center">
          <h2 className="text-xl font-bold">Active Alerts</h2>
          {errors.alerts && (
            <span className="ml-2 inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-red-100 text-red-800">
              API Error
            </span>
          )}
        </div>
        <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
          displayAlerts.length > 0 ? 'bg-red-100 text-red-800' : 'bg-green-100 text-green-800'
        }`}>
          {displayAlerts.length > 0 ? `${displayAlerts.length} Active` : 'All Clear'}
        </span>
      </div>
      
      {/* API Error display */}
      {errors.alerts && (
        <div className="border-b border-red-200 bg-red-50 px-4 py-2 text-sm text-red-700 flex justify-between items-center">
          <div>
            <span className="font-medium">API Error:</span> {errors.alerts}
            <p className="text-xs mt-1">Using mock data for display purposes.</p>
          </div>
          <button 
            onClick={retryFetchAlerts}
            className="px-2 py-1 bg-red-100 hover:bg-red-200 text-red-800 rounded text-xs font-medium"
          >
            Retry
          </button>
        </div>
      )}
      
      <div className="p-4 max-h-80 overflow-y-auto">
        {isLoading ? (
          <div className="flex justify-center items-center p-8">
            <svg className="animate-spin h-8 w-8 text-blue-500" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
              <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
              <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
            </svg>
          </div>
        ) : displayAlerts.length === 0 ? (
          <div className="flex flex-col items-center justify-center py-8 text-gray-500">
            <svg xmlns="http://www.w3.org/2000/svg" className="h-12 w-12 text-green-500 mb-2" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
            <p>No active alerts at this time.</p>
          </div>
        ) : (
          <div className="space-y-3">
            {displayAlerts.map(alert => (
              <div
                key={alert.id}
                onClick={() => handleAlertClick(alert)}
                className={`p-3 rounded border-l-4 cursor-pointer hover:bg-gray-50 ${
                  alert.type === 'ingredient'
                    ? 'border-yellow-500 bg-yellow-50'
                    : alert.type === 'hardware'
                    ? 'border-red-500 bg-red-50'
                    : 'border-blue-500 bg-blue-50'
                }`}
              >
                <div className="flex justify-between items-start">
                  <div>
                    <div className="font-medium">{alert.message || alert.type}</div>
                    <div className="text-xs text-gray-500 mt-1">
                      {alert.timestamp ? new Date(alert.timestamp).toLocaleString() : 'Just now'}
                    </div>
                  </div>
                  <div className="flex items-center space-x-2">
                    {alert.severity && getSeverityBadge(alert.severity)}
                    <button
                      onClick={(e) => {
                        e.stopPropagation();
                        acknowledgeAlert(alert.id);
                      }}
                      className="text-xs bg-gray-100 hover:bg-gray-200 text-gray-700 px-2 py-1 rounded"
                      disabled={errors.alerts}
                    >
                      Ack
                    </button>
                  </div>
                </div>
                {alert.type === 'ingredient' && (
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      if (!errors.system) {
                        resumeOperation();
                      }
                    }}
                    disabled={isLoading || errors.system}
                    className="mt-2 bg-yellow-600 hover:bg-yellow-700 disabled:bg-yellow-400 text-white text-xs font-semibold px-3 py-1 rounded flex items-center space-x-1"
                  >
                    {isLoading ? (
                      <>
                        <svg className="animate-spin h-3 w-3" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                          <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                          <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                        </svg>
                        <span>Resuming...</span>
                      </>
                    ) : errors.system ? (
                      <>
                        <svg className="w-3 h-3" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
                        </svg>
                        <span>System Error</span>
                      </>
                    ) : (
                      <>
                        <svg className="w-3 h-3" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z" />
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
                        </svg>
                        <span>Resume Operation</span>
                      </>
                    )}
                  </button>
                )}
              </div>
            ))}
          </div>
        )}
      </div>
      
      {/* Alert Details Modal */}
      {selectedAlert && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-white rounded-lg max-w-md w-full p-6">
            <div className="flex justify-between items-start mb-4">
              <h3 className="text-xl font-bold">Alert Details</h3>
              <button 
                onClick={closeAlertDetails}
                className="text-gray-500 hover:text-gray-700"
              >
                <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                </svg>
              </button>
            </div>
            
            <div className="bg-gray-50 p-4 rounded mb-4">
              <div className="flex justify-between items-center mb-2">
                <h4 className="font-medium">{selectedAlert.message || selectedAlert.type}</h4>
                {selectedAlert.severity && getSeverityBadge(selectedAlert.severity)}
              </div>
              <p className="text-sm text-gray-600 mb-2">
                <span className="font-medium">Type:</span> {selectedAlert.type}
              </p>
              {selectedAlert.timestamp && (
                <p className="text-sm text-gray-600 mb-2">
                  <span className="font-medium">Time:</span> {new Date(selectedAlert.timestamp).toLocaleString()}
                </p>
              )}
              {selectedAlert.location && (
                <p className="text-sm text-gray-600 mb-2">
                  <span className="font-medium">Location:</span> {selectedAlert.location}
                </p>
              )}
              {selectedAlert.details && (
                <div className="mt-2">
                  <p className="text-sm font-medium text-gray-600">Details:</p>
                  <p className="text-sm text-gray-600 mt-1">{selectedAlert.details}</p>
                </div>
              )}
            </div>
            
            <div className="flex justify-end space-x-3">
              <button
                onClick={closeAlertDetails}
                className="px-4 py-2 border border-gray-300 rounded-md text-gray-700 hover:bg-gray-50"
              >
                Close
              </button>
              <button
                onClick={() => {
                  acknowledgeAlert(selectedAlert.id);
                  closeAlertDetails();
                }}
                className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700"
                disabled={errors.alerts}
              >
                Acknowledge
              </button>
              {selectedAlert.type === 'ingredient' && (
                <button
                  onClick={() => {
                    if (!errors.system) {
                      resumeOperation();
                    }
                    closeAlertDetails();
                  }}
                  disabled={isLoading || errors.system}
                  className="px-4 py-2 bg-yellow-600 text-white rounded-md hover:bg-yellow-700 disabled:bg-yellow-400"
                >
                  {isLoading ? 'Resuming...' : errors.system ? 'System Error' : 'Resume Operation'}
                </button>
              )}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
