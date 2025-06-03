import React, { useEffect, useState } from 'react';
import useStore from './store';
import SystemControls from './components/SystemControls';
import OrderQueue from './components/OrderQueue';
import AlertPanel from './components/AlertPanel';
import LiveCameraFeed from './components/LiveCameraFeed';
import StatusBoard from './components/StatusBoard';
import LogsPanel from './components/LogsPanel';
import NewOrderPanel from './components/NewOrderPanel';
import InventoryPanel from './components/InventoryPanel';
import barnsLogo from './assets/BARNS-Logo.png';
import './index.css';

export default function App() {
  const { 
    fetchOrders, 
    fetchAlerts, 
    fetchSchedulerStatus,
    fetchInventoryStatus,
    checkSystemHealth,
    connectOrderWS, 
    connectAlertWS, 
    errors 
  } = useStore();
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const [activeTab, setActiveTab] = useState('dashboard');

  useEffect(() => {
    const setupConnections = async () => {
      try {
        // Initial data fetching
        await fetchOrders();
        await fetchAlerts();
        await fetchSchedulerStatus();
        await fetchInventoryStatus();
        
        // Check all services health
        await checkSystemHealth();
        
        // Set up WebSocket connections
        connectOrderWS();
        connectAlertWS();
      } catch (error) {
        console.error('Unexpected error during setup:', error);
      }
    };

    setupConnections();

    // Set up periodic health checks every 2 minutes
    const healthCheckInterval = setInterval(() => {
      checkSystemHealth();
    }, 120000);

    return () => clearInterval(healthCheckInterval);
  }, [fetchOrders, fetchAlerts, fetchSchedulerStatus, fetchInventoryStatus, checkSystemHealth, connectOrderWS, connectAlertWS]);

  // Mobile menu toggle
  const toggleSidebar = () => {
    setSidebarOpen(!sidebarOpen);
  };

  // Global connection status notification
  const hasErrors = Object.values(errors).some(err => err !== null);

  return (
    <div className="flex flex-col h-screen w-full overflow-hidden" style={{ background: 'linear-gradient(135deg, #f0fdf4 0%, #ecfdf5 100%)' }}>
      {/* Header with Modern BARNS Design */}
      <header className="header-gradient text-white shadow-xl z-10" style={{ 
        background: 'linear-gradient(135deg, #004029 0%, #00754a 50%, #008552 100%)',
        boxShadow: '0 8px 32px rgba(0, 64, 41, 0.3)'
      }}>
        <div className="container mx-auto px-6 py-4">
          <div className="flex justify-between items-center">
            {/* Logo and Brand */}
            <div className="flex items-center space-x-4">
              <div className="flex items-center bg-white bg-opacity-10 rounded-xl p-2 backdrop-blur-sm">
                <img 
                  src={barnsLogo} 
                  alt="BARNS Logo" 
                  className="h-10 w-10 object-contain"
                />
              </div>
              <div>
                <h1 className="text-2xl font-bold text-white">BARNS</h1>
                <p className="text-sm text-green-200 opacity-90 font-medium">Business Automation & Robotics</p>
              </div>
            </div>
            
            {/* Tab Navigation - Desktop */}
            <div className="hidden md:flex space-x-2 bg-black bg-opacity-20 rounded-xl p-2 backdrop-blur-sm">
              <button
                onClick={() => setActiveTab('dashboard')}
                className={`px-8 py-3 rounded-lg text-sm font-semibold transition-all duration-300 ${
                  activeTab === 'dashboard' 
                    ? 'bg-white text-green-700 shadow-lg transform scale-105' 
                    : 'text-green-100 hover:text-green-600 hover:bg-white hover:bg-opacity-15'
                }`}
              >
                <span className="flex items-center space-x-2">
                  <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
                    <path d="M3 4a1 1 0 011-1h12a1 1 0 011 1v2a1 1 0 01-1 1H4a1 1 0 01-1-1V4zM3 10a1 1 0 011-1h6a1 1 0 011 1v6a1 1 0 01-1 1H4a1 1 0 01-1-1v-6zM14 9a1 1 0 00-1 1v6a1 1 0 001 1h2a1 1 0 001-1v-6a1 1 0 00-1-1h-2z" />
                  </svg>
                  <span>Dashboard</span>
                </span>
              </button>
              <button
                onClick={() => setActiveTab('cameras')}
                className={`px-8 py-3 rounded-lg text-sm font-semibold transition-all duration-300 ${
                  activeTab === 'cameras' 
                    ? 'bg-white text-green-700 shadow-lg transform scale-105' 
                    : 'text-green-100 hover:text-green-600 hover:bg-white hover:bg-opacity-15'
                }`}
              >
                <span className="flex items-center space-x-2">
                  <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
                    <path fillRule="evenodd" d="M4 5a2 2 0 00-2 2v6a2 2 0 002 2h12a2 2 0 002-2V7a2 2 0 00-2-2h-1.586l-.707-.707A1 1 0 0013 4H7a1 1 0 00-.707.293L5.586 5H4zm6 9a3 3 0 100-6 3 3 0 000 6z" clipRule="evenodd" />
                  </svg>
                  <span>Live Cameras</span>
                </span>
              </button>
              <button
                onClick={() => setActiveTab('logs')}
                className={`px-8 py-3 rounded-lg text-sm font-semibold transition-all duration-300 ${
                  activeTab === 'logs' 
                    ? 'bg-white text-green-700 shadow-lg transform scale-105' 
                    : 'text-green-100 hover:text-green-600 hover:bg-white hover:bg-opacity-15'
                }`}
              >
                <span className="flex items-center space-x-2">
                  <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
                    <path d="M9 2a1 1 0 000 2h2a1 1 0 100-2H9z" />
                    <path fillRule="evenodd" d="M4 5a2 2 0 012-2v1a2 2 0 002 2h4a2 2 0 002-2V3a2 2 0 012 2v6a2 2 0 01-2 2H6a2 2 0 01-2-2V5zm3 3a1 1 0 000 2h.01a1 1 0 100-2H7zm3 0a1 1 0 000 2h3a1 1 0 100-2h-3zm-3 4a1 1 0 100 2h.01a1 1 0 100-2H7zm3 0a1 1 0 100 2h3a1 1 0 100-2h-3z" clipRule="evenodd" />
                  </svg>
                  <span>System Logs</span>
                </span>
              </button>
            </div>
            
            {/* Mobile menu button */}
            <button 
              onClick={toggleSidebar}
              className="p-3 rounded-lg md:hidden hover:bg-white hover:bg-opacity-15 transition-colors"
            >
              <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6h16M4 12h16M4 18h16" />
              </svg>
            </button>
          </div>
          
          {/* Mobile Tab Navigation */}
          <div className="md:hidden mt-4 pt-4 border-t border-green-600 border-opacity-30">
            <div className="flex space-x-2">
              <button
                onClick={() => setActiveTab('dashboard')}
                className={`px-4 py-2 rounded-lg text-sm font-medium transition-all duration-300 ${
                  activeTab === 'dashboard' 
                    ? 'bg-white text-green-700 shadow-md' 
                    : 'text-green-100 hover:text-green-600 hover:bg-white hover:bg-opacity-15'
                }`}
              >
                Dashboard
              </button>
              <button
                onClick={() => setActiveTab('cameras')}
                className={`px-4 py-2 rounded-lg text-sm font-medium transition-all duration-300 ${
                  activeTab === 'cameras' 
                    ? 'bg-white text-green-700 shadow-md' 
                    : 'text-green-100 hover:text-green-600 hover:bg-white hover:bg-opacity-15'
                }`}
              >
                Cameras
              </button>
              <button
                onClick={() => setActiveTab('logs')}
                className={`px-4 py-2 rounded-lg text-sm font-medium transition-all duration-300 ${
                  activeTab === 'logs' 
                    ? 'bg-white text-green-700 shadow-md' 
                    : 'text-green-100 hover:text-green-600 hover:bg-white hover:bg-opacity-15'
                }`}
              >
                Logs
              </button>
            </div>
          </div>
        </div>
      </header>

      {/* System Status Notification */}
      {hasErrors && (
        <div className="bg-gradient-to-r from-red-50 to-orange-50 border-l-4 border-red-400 p-4 shadow-sm z-10">
          <div className="flex items-center">
            <div className="flex-shrink-0">
              <svg className="h-5 w-5 text-red-400" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20" fill="currentColor">
                <path fillRule="evenodd" d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
              </svg>
            </div>
            <div className="ml-3">
              <div className="flex items-center space-x-2">
                <p className="text-sm font-medium text-red-800">
                  System Alert
                </p>
                <span className="badge badge-error">Connection Issues</span>
              </div>
              <p className="text-sm text-red-700 mt-1">
                Some BARNS services are experiencing connectivity issues. Check the System Logs tab for detailed information.
              </p>
            </div>
          </div>
        </div>
      )}

      {/* Main Content Area */}
      <div className="flex-1 overflow-hidden">
        {activeTab === 'dashboard' ? (
          <div className="h-full overflow-auto">
            <div className="container mx-auto px-6 py-8">
              <div className="flex flex-col lg:flex-row gap-8">
                {/* Main content */}
                <div className="flex-1 space-y-8">
                  
                  
                  {/* Order Queue */}
                  <OrderQueue />
                  
                  {/* New Order Panel */}
                  <NewOrderPanel />
                  {/* System controls */}
                  <SystemControls />
                </div>
                
                {/* Sidebar */}
                <div className={`lg:w-96 space-y-8 lg:block ${sidebarOpen ? 'block' : 'hidden'}`}>
                  {/* Alert panel */}
                  <AlertPanel />
                  
                  {/* Status board */}
                  <StatusBoard />
                  
                  {/* Inventory panel */}
                  <InventoryPanel />
                </div>
              </div>
            </div>
          </div>
        ) : activeTab === 'cameras' ? (
          <div className="h-full p-6">
            <LiveCameraFeed />
          </div>
        ) : (
          <div className="h-full p-6">
            <LogsPanel />
          </div>
        )}
      </div>
    </div>
  );
}
