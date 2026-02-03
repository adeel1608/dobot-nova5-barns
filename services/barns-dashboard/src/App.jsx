import React, { useEffect, useState, useRef } from "react";
import useStore from "./store";
import { useTranslationsStore, useTranslation } from "./store/translationsStore";
import DashboardPage from "./pages/dashboard";
import AlertsPage from "./pages/alerts";
import InventoryPage from "./pages/inventory";
import CamerasPage from "./pages/cameras";
import SettingsPage from "./pages/settings";
import NewOrderPage from "./pages/newOrder";
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
    alerts,
  } = useStore();
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const [langDropdownOpen, setLangDropdownOpen] = useState(false);
  const langDropdownRef = useRef(null);
  const { languages, currentLocale, setCurrentLocale } = useTranslationsStore();
  const { t: tApp } = useTranslation("app");

  useEffect(() => {
    const handleClickOutside = (e) => {
      if (langDropdownRef.current && !langDropdownRef.current.contains(e.target)) {
        setLangDropdownOpen(false);
      }
    };
    document.addEventListener("mousedown", handleClickOutside);
    return () => document.removeEventListener("mousedown", handleClickOutside);
  }, []);

  const [activeTab, setActiveTab] = useState(() => {
    const hash = window.location.hash.replace("#/", "").trim();
    const main = hash ? hash.split("/")[0] : "";
    return main || "dashboard";
  });

  useEffect(() => {
    // Register navigation handler with store
    setNavigationHandler(setActiveTab);

    const setupConnections = async () => {
      try {
        // Set up WebSocket connections immediately (don't wait for API calls)
        connectOrderWS();
        connectAlertWS();

        // Load critical data first (with shorter timeout tolerance)
        const criticalData = [
          fetchOrders(),
          fetchAlerts(),
        ];

        // Load non-critical data in background (don't block UI)
        const backgroundData = [
          fetchSchedulerStatus(),
          fetchInventoryStatus(),
          checkSystemHealth()
        ];

        // Wait for critical data only
        await Promise.allSettled(criticalData);
        
        // Start background data loading (don't await - let it happen in background)
        Promise.allSettled(backgroundData).catch(err => 
          console.error("Background data loading error:", err)
        );
        
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
                  {tApp("navDashboard")}
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
                  {tApp("navNotifications")}
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
                  {tApp("navInventory")}
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
                  {tApp("navCameras")}
                </button>
                <button
                  onClick={() => {
                    setActiveTab("settings");
                    window.location.hash = "#/settings";
                  }}
                  style={{ boxShadow: 'none' }}
                  className={`px-4 py-1 font-medium text-sm transition-all duration-300 shadow-none focus:shadow-none hover:shadow-none active:shadow-none ${
                    activeTab === "settings" 
                      ? "bg-green-800 text-white" 
                      : "text-gray-400 hover:text-white hover:bg-green-800 "
                  }`}
                >
                  {tApp("navSettings")}
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

              {/* Language selector */}
              <div className="relative" ref={langDropdownRef}>
                <button
                  type="button"
                  onClick={() => setLangDropdownOpen(!langDropdownOpen)}
                  className="flex items-center justify-center w-9 h-9 rounded-lg text-gray-600 hover:bg-gray-200 hover:text-gray-900 transition-colors flex-shrink-0"
                  title="Change language"
                  aria-label="Change language"
                >
                  <svg className="w-5 h-5 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24" aria-hidden="true">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 01-9 9m9-9a9 9 0 00-9-9m9 9H3m9 9a9 9 0 01-9-9m9 9c1.657 0 3-4.03 3-9s-1.343-9-3-9m0 18c-1.657 0-3-4.03-3-9s1.343-9 3-9m-9 9a9 9 0 019-9" />
                  </svg>
                </button>
                {langDropdownOpen && (
                  <div className="absolute right-0 mt-1 w-44 bg-white rounded-lg shadow-lg border border-gray-200 py-1 z-50">
                    {languages.map((lang) => (
                      <button
                        key={lang.code}
                        type="button"
                        onClick={() => {
                          setCurrentLocale(lang.code);
                          setLangDropdownOpen(false);
                        }}
                        className={`w-full text-left px-4 py-2 text-sm transition-colors ${
                          currentLocale === lang.code
                            ? "bg-green-50 text-green-800 font-medium"
                            : "text-gray-700 hover:bg-gray-50"
                        }`}
                      >
                        {lang.name} ({lang.code})
                      </button>
                    ))}
                  </div>
                )}
              </div>
              
              {/* Notification Bell */}
              <div className="relative">
                <img
                  src={notification}
                  alt="Notifications"
                  className="w-6 h-8 cursor-pointer"
                  onClick={() => {
                    setActiveTab('alerts');
                    window.location.hash = '#/alerts';
                  }}
                />
                {alerts && alerts.length > 0 && (
                  <div className="absolute -top-1 -right-1 barns-dark-bg text-white text-xs font-semibold rounded-full w-5 h-5 flex items-center justify-center shadow-lg ring-2 ring-white">
                    {alerts.length}
                  </div>
                )}
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
                { key: "dashboard", labelKey: "navDashboard" },
                { key: "alerts", labelKey: "navNotifications" },
                { key: "inventory", labelKey: "navInventory" },
                { key: "cameras", labelKey: "navCameras" },
                { key: "settings", labelKey: "navSettings" }
              ].map(({ key, labelKey }) => (
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
                  {tApp(labelKey)}
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
                the Settings tab (Logs) for detailed information.
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
        {activeTab === "settings" && <SettingsPage />}
        {activeTab === "newOrder" && <NewOrderPage />}
      </div>
    </div>
  );
}
