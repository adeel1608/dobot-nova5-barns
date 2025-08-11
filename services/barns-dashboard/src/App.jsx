import React, { useEffect, useState } from "react";
import useStore from "./store";
import DashboardPage from "./pages/dashboard";
import AlertsPage from "./pages/alerts";
import InventoryPage from "./pages/inventory";
import CamerasPage from "./pages/cameras";
import LogsPage from "./pages/logs";
import barnsLogo from "./assets/barns.png";
import qssLogo from "./assets/qss.png";
import notification from "./assets/notification.png";
import profile from "./assets/profile.png";
import "./index.css";

export default function App() {
  const {
    fetchOrders,
    fetchAlerts,
    fetchSchedulerStatus,
    fetchInventoryStatus,
    checkSystemHealth,
    connectOrderWS,
    connectAlertWS,
    setNavigationHandler,
    errors,
  } = useStore();
  const [sidebarOpen, setSidebarOpen] = useState(false);
  // const [activeTab, setActiveTab] = useState("dashboard");
  const [activeTab, setActiveTab] = useState(() => {
    const hash = window.location.hash.replace("#/", "");
    return hash || "dashboard";
  });

  useEffect(() => {
    // Register navigation handler with store
    setNavigationHandler(setActiveTab);

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
        console.error("Unexpected error during setup:", error);
      }
    };

    setupConnections();

    // Set up periodic health checks every 2 minutes
    const healthCheckInterval = setInterval(() => {
      checkSystemHealth();
    }, 120000);

    return () => clearInterval(healthCheckInterval);
  }, [
      fetchOrders,
      fetchAlerts,
      fetchSchedulerStatus,
      fetchInventoryStatus,
      checkSystemHealth,
      connectOrderWS,
      connectAlertWS,
      setNavigationHandler,
    ]);

  // Mobile menu toggle
  const toggleSidebar = () => {
    setSidebarOpen(!sidebarOpen);
  };

  // Global connection status notification
  const hasErrors = Object.values(errors).some((err) => err !== null);

  return (
    <div
      className="flex flex-col min-h-screen w-full pb-2"

    >
      {/* Header with Modern BARNS Design */}
      <header
        className="bg-gray-100 text-gray-500 z-10 flex-shrink-0 px-6"
      >
        <div className="container-fluid">
          <div className="flex justify-between items-center">
            {/* Left Side - Logo, Brand, and Navigation */}
            <div className="flex items-center space-x-8">
              {/* Logo and Brand */}
              <div className="flex items-center space-x-3 flex-shrink-0">
                <img
                  src={barnsLogo}
                  alt="Barns Logo"
                  className=" w-auto object-contain"
                  style={{ height: '70px' }}
                />

              </div>

              {/* Tab Navigation - Horizontal */}
              <div className="hidden lg:flex items-center space-x-2">
                <button
                  onClick={() => {
                    setActiveTab("dashboard");
                    window.location.hash = "#/dashboard";
                  }}
                  style={{ boxShadow: 'none' }}
                  className={`px-4 py-1 rounded-lg font-semibold text-sm transition-all duration-300 shadow-none focus:shadow-none hover:shadow-none active:shadow-none ${
                    activeTab === "dashboard" 
                      ? "bg-green-800 text-white" 
                      : "text-gray-400 hover:text-green-800"
                  }`}
                >
                  Dashboard
                </button>
                <button
                  onClick={() => {
                    setActiveTab("alerts");
                    window.location.hash = "#/alerts";
                  }}
                  style={{ boxShadow: 'none' }}
                  className={`px-4 py-1 font-medium text-sm transition-all duration-300 shadow-none focus:shadow-none hover:shadow-none active:shadow-none ${
                    activeTab === "alerts" 
                      ? "bg-green-800 text-white" 
                      : "text-gray-400 hover:text-white hover:bg-green-800 "
                  }`}
                >
                  Notifications
                </button>
                <button
                  onClick={() => {
                    setActiveTab("inventory");
                    window.location.hash = "#/inventory";
                  }}
                  style={{ boxShadow: 'none' }}
                  className={`px-4 py-1 font-medium text-sm transition-all duration-300 shadow-none focus:shadow-none hover:shadow-none active:shadow-none ${
                    activeTab === "inventory" 
                      ? "bg-green-800 text-white" 
                      : "text-gray-400 hover:text-white hover:bg-green-800 "
                  }`}
                >
                  Inventory
                </button>
                <button
                  onClick={() => {
                    setActiveTab("cameras");
                    window.location.hash = "#/cameras";
                  }}
                  style={{ boxShadow: 'none' }}
                  className={`px-4 py-1 font-medium text-sm transition-all duration-300 shadow-none focus:shadow-none hover:shadow-none active:shadow-none ${
                    activeTab === "cameras" 
                      ? "bg-green-800 text-white" 
                      : "text-gray-400 hover:text-white hover:bg-green-800"
                  }`}
                >
                  Cameras
                </button>
                <button
                  onClick={() => {
                    setActiveTab("logs");
                    window.location.hash = "#/logs";
                  }}
                  style={{ boxShadow: 'none' }}
                  className={`px-4 py-1 font-medium text-sm transition-all duration-300 shadow-none focus:shadow-none hover:shadow-none active:shadow-none ${
                    activeTab === "logs" 
                      ? "bg-green-800 text-white" 
                      : "text-gray-400 hover:text-white hover:bg-green-800 "
                  }`}
                >
                  Logs
                </button>
              </div>
            </div>

            {/* Right Side - Branding and User Elements */}
            <div className="flex items-center space-x-6">
              
              {/* QSS Logo */}
              <img
                src={qssLogo}
                alt="QSS Logo"
                className="h-10 w-auto object-contain pt-2"
                style={{ height: '40px' }}
              />
              
              {/* Notification Bell */}
              <div className="relative">
                <img
                  src={notification}
                  alt="Notifications"
                  className="w-6 h-8 cursor-pointer"
                />
                <div className="absolute -top-1 -right-1 bg-green-800 text-white text-xs rounded-full w-5 h-5 flex items-center justify-center">
                  2
                </div>
              </div>
              
              {/* Profile Picture */}
              <img
                src={profile}
                alt="Profile"
                className="w-10 h-10 rounded-full cursor-pointer"
              />
            </div>

            {/* Mobile menu button */}
            <button
              onClick={toggleSidebar}
              className="p-2 rounded-lg lg:hidden hover:bg-gray-200 transition-colors flex-shrink-0"
            >
              <svg
                xmlns="http://www.w3.org/2000/svg"
                className="h-6 w-6"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M4 6h16M4 12h16M4 18h16"
                />
              </svg>
            </button>
          </div>

          {/* Mobile Tab Navigation */}
          <div className="lg:hidden mt-4 pt-4 border-t border-gray-300">
            <div className="flex flex-wrap gap-2">
              {[
                { key: "dashboard", label: "Dashboard" },
                { key: "alerts", label: "Notifications" },
                { key: "inventory", label: "Inventory" },
                { key: "cameras", label: "Cameras" },
                { key: "logs", label: "Logs" }
              ].map(({ key, label }) => (
                <button
                  key={key}
                  onClick={() => {
                    setActiveTab(key);
                    window.location.hash = `#/${key}`;
                  }}
                  style={{ boxShadow: 'none' }}
                  className={`px-4 py-2 rounded-lg text-sm font-medium transition-all duration-300 shadow-none focus:shadow-none hover:shadow-none active:shadow-none ${
                    activeTab === key
                      ? "bg-green-800 text-white"
                      : "text-gray-600 hover:text-green-800"
                  }`}
                >
                  {label}
                </button>
              ))}
            </div>
          </div>
        </div>
      </header>

      {/* System Status Notification */}
      {hasErrors && (
        <div className="bg-gradient-to-r from-red-50 to-orange-50 border-l-4 border-red-400 p-4 shadow-sm z-10" style={{display: hasErrors ? 'none' : 'none'}}>
          <div className="flex items-center">
            <div className="flex-shrink-0">
              <svg
                className="h-5 w-5 text-red-400"
                xmlns="http://www.w3.org/2000/svg"
                viewBox="0 0 20 20"
                fill="currentColor"
              >
                <path
                  fillRule="evenodd"
                  d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z"
                  clipRule="evenodd"
                />
              </svg>
            </div>
            <div className="ml-3">
              <div className="flex items-center space-x-2">
                <p className="text-sm font-medium text-red-800">System Alert</p>
                <span className="badge badge-error">Connection Issues</span>
              </div>
              <p className="text-sm text-red-700 mt-1">
                Some BARNS services are experiencing connectivity issues. Check
                the Logs tab for detailed information.
              </p>
            </div>
          </div>
        </div>
      )}

      {/* Main Content Area */}

      <div className={`flex-1 px-2 ${activeTab === 'inventory' ? 'overflow-y-auto' : ''}`}>
        {activeTab === "dashboard" && <DashboardPage />}
        {activeTab === "alerts" && <AlertsPage />}
        {activeTab === "inventory" && <InventoryPage />}
        {activeTab === "cameras" && <CamerasPage />}
        {activeTab === "logs" && <LogsPage />}
      </div>
    </div>
  );
}
