import React, { useEffect, useState } from "react";
import useStore from "./store";
import DashboardPage from "./pages/dashboard";
import AlertsPage from "./pages/alerts";
import InventoryPage from "./pages/inventory";
import CamerasPage from "./pages/cameras";
import LogsPage from "./pages/logs";
import barnsLogo from "./assets/barns.png";
import qssLogo from "./assets/qss.png";
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
      className="flex flex-col min-h-screen w-full bg-gray-50"
      style={{
        background: "linear-gradient(135deg, #f0fdf4 0%, #ecfdf5 100%)",
      }}
    >
      {/* Header with Modern BARNS Design */}
      <header
        className="header-gradient text-white shadow-xl z-10 flex-shrink-0 "
        style={{
          background:
            "linear-gradient(135deg, #004029 0%, #00754a 50%, #008552 100%)",
          boxShadow: "0 8px 32px rgba(0, 64, 41, 0.3)",
        }}
      >
        <div className="container-fluid   ">
          <div className="flex justify-between items-center  ">
            {/* Logo and Brand */}
            <div className="flex items-center space-x-3 sm:space-x-4 flex-shrink-0 ">
              <div className="flex items-center  bg-opacity-10 rounded-xl p-1.5 sm:p-2 backdrop-blur-sm">
                <img
                  src={qssLogo}
                  alt="Qss Logo"
                  className="sm:h-10 sm:w-10 object-contain"
                  style={{ height: "100%", width: "70%" }}
                />
              </div>
            </div>

            {/* Tab Navigation - Desktop */}
            <div className="hidden lg:flex space-x-1 xl:space-x-2 bg-opacity-20 rounded-xl p-1.5 xl:p-2 backdrop-blur-sm">
            {[
              { key: "dashboard", label: "Dashboard"
              },
              { key: "alerts", label: "Alerts"
              },
              { key: "inventory", label: "Inventory"
              },
              { key: "cameras", label: "Cameras"
              },
              { key: "logs", label: "Logs" 
              },
            ].map(({ key, label, icon }) => (
              <button
                key={key}
                onClick={() => {
                  setActiveTab(key);
                  window.location.hash = `#/${key}`;
                }}
                style={{ outline: "none", boxShadow: "none" }}
                className={`px-4 xl:px-6 py-2 xl:py-3 rounded-lg text-xs xl:text-sm font-semibold transition-all duration-300 transform
                  focus:outline-none focus:ring-0 focus:border-transparent
                  ${
                    activeTab === key
                      ? "bg-white/10 backdrop-blur-md text-white shadow-md scale-105 -translate-y-0.5 border-2 border-white"
                      : "text-white hover:bg-white/30 hover:text-white"
                  }`}
              >
                <span className="flex items-center space-x-2">
                  {/* {icon} */}
                  <span className="hidden xl:inline">{label}</span>
                </span>
              </button>
            ))}
            </div>

            {/* Logo and Brand */}
            <div className="flex items-center space-x-3 sm:space-x-4 flex-shrink-0 ">
              <div className="flex items-center  bg-opacity-10 rounded-xl p-4  sm:p-2 backdrop-blur-sm">
                <img
                  src={barnsLogo}
                  alt="Barns "
                  className="object-contain"
                  style={{ height: "60px", width: "60px" }}
                />
              </div>
            </div>
            {/* Mobile menu button */}
            <button
              onClick={toggleSidebar}
              className="p-2 sm:p-3 rounded-lg lg:hidden hover:bg-white hover:bg-opacity-15 transition-colors flex-shrink-0"
            >
              <svg
                xmlns="http://www.w3.org/2000/svg"
                className="h-5 w-5 sm:h-6 sm:w-6"
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
          <div className="lg:hidden mt-3 sm:mt-4 pt-3 sm:pt-4 border-t border-green-600 border-opacity-30">
            <div className="flex flex-wrap gap-1.5 sm:gap-2">
              {["dashboard", "alerts", "inventory", "cameras", "logs"].map(
                (tab) => (
                  <button
                    key={tab}
                    onClick={() => setActiveTab(tab)}
                    className={`px-3 sm:px-4 py-1.5 sm:py-2 rounded-lg text-xs sm:text-sm font-medium transition-all duration-300 capitalize ${
                      activeTab === tab
                        ? "bg-white text-green-700 shadow-md"
                        : "text-green-100 hover:text-green-600 hover:bg-white hover:bg-opacity-15"
                    }`}
                  >
                    {tab}
                  </button>
                )
              )}
            </div>
          </div>
        </div>
      </header>

      {/* System Status Notification */}
      {hasErrors && (
        <div className="bg-gradient-to-r from-red-50 to-orange-50 border-l-4 border-red-400 p-4 shadow-sm z-10">
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

      <div className={`flex-1 ${activeTab === 'inventory' ? 'overflow-y-auto' : ''}`}>
        {activeTab === "dashboard" && <DashboardPage />}
        {activeTab === "alerts" && <AlertsPage />}
        {activeTab === "inventory" && <InventoryPage />}
        {activeTab === "cameras" && <CamerasPage />}
        {activeTab === "logs" && <LogsPage />}
      </div>
    </div>
  );
}
