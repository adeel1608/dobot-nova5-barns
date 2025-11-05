/**
 * Settings Page
 * Central configuration hub with multiple settings categories
 */

import React, { useState } from 'react';
import LogsPanel from './components/LogsPanel';
import IngredientSettings from './components/IngredientSettings';
import './styles.css';

export default function SettingsPage() {
  const [activeTab, setActiveTab] = useState('logs');

  const tabs = [
    { 
      id: 'logs', 
      name: 'System Logs', 
      icon: (
        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
        </svg>
      )
    },
    { 
      id: 'ingredients', 
      name: 'Ingredient Settings', 
      icon: (
        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4" />
        </svg>
      )
    }
  ];

  const renderContent = () => {
    switch (activeTab) {
      case 'logs':
        return <LogsPanel />;
      case 'ingredients':
        return <IngredientSettings />;
      default:
        return <LogsPanel />;
    }
  };

  return (
    <div className="container-fluid bg-gray-50">
      <div className="container-fluid">
        <div className="mt-3">
          
          {/* Settings Header with Tabs */}
          <div className="bg-white rounded-xl shadow-sm border border-gray-200 mb-3">
            <div className="px-6 py-4 border-b border-gray-200">
              <h1 className="text-2xl font-bold text-gray-900">Settings</h1>
              <p className="text-sm text-gray-600 mt-1">
                Manage system configuration and monitor operations
              </p>
            </div>
            
            {/* Tab Navigation */}
            <div className="px-6">
              <div className="flex space-x-1 border-b border-gray-200">
                {tabs.map((tab) => (
                  <button
                    key={tab.id}
                    onClick={() => setActiveTab(tab.id)}
                    className={`flex items-center space-x-2 px-4 py-3 font-medium text-sm transition-colors relative ${
                      activeTab === tab.id
                        ? 'text-blue-600 border-b-2 border-blue-600'
                        : 'text-gray-600 hover:text-gray-900'
                    }`}
                  >
                    {tab.icon}
                    <span>{tab.name}</span>
                  </button>
                ))}
              </div>
            </div>
          </div>
          
          {/* Tab Content */}
          <div>
            {renderContent()}
          </div>
          
        </div>
      </div>
    </div>
  );
}

