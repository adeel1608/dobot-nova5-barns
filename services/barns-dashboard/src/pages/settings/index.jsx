/**
 * Settings Page
 * Central configuration hub with multiple settings categories.
 * Tab switching is id-based (monitoring, logs, ingredients, translations); labels use translation.
 * Active sub-tab is synced with URL hash (#/settings/<tabId>) so it survives refresh.
 */

import React, { useState, useEffect } from 'react';
import { useTranslation } from '../../store/translationsStore';
import LogsPanel from './components/LogsPanel';
import IngredientSettings from './components/IngredientSettings';
import MonitoringPanel from './components/MonitoringPanel';
import TranslationsPanel from './components/TranslationsPanel';
import './styles.css';

const VALID_TAB_IDS = ['monitoring', 'logs', 'ingredients', 'translations'];

function getSettingsTabFromHash() {
  const hash = window.location.hash.replace("#/", "").trim();
  const parts = hash.split("/");
  if (parts[0] !== "settings") return "monitoring";
  const sub = parts[1];
  return sub && VALID_TAB_IDS.includes(sub) ? sub : "monitoring";
}

export default function SettingsPage() {
  const [activeTab, setActiveTab] = useState(getSettingsTabFromHash);
  const { t } = useTranslation('settings');

  useEffect(() => {
    const onHashChange = () => setActiveTab(getSettingsTabFromHash());
    window.addEventListener("hashchange", onHashChange);
    return () => window.removeEventListener("hashchange", onHashChange);
  }, []);

  const tabs = [
    { 
      id: 'monitoring', 
      nameKey: 'monitoring', 
      icon: (
        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z" />
        </svg>
      )
    },
    { 
      id: 'logs', 
      nameKey: 'logs', 
      icon: (
        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
        </svg>
      )
    },
    { 
      id: 'ingredients', 
      nameKey: 'ingredients', 
      icon: (
        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4" />
        </svg>
      )
    },
    { 
      id: 'translations', 
      nameKey: 'translations', 
      icon: (
        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 01-9 9m9-9a9 9 0 00-9-9m9 9H3m9 9a9 9 0 01-9-9m9 9c1.657 0 3-4.03 3-9s-1.343-9-3-9m0 18c-1.657 0-3-4.03-3-9s1.343-9 3-9m-9 9a9 9 0 019-9" />
        </svg>
      )
    }
  ];

  const renderContent = () => {
    switch (activeTab) {
      case 'monitoring':
        return <MonitoringPanel />;
      case 'logs':
        return <LogsPanel />;
      case 'ingredients':
        return <IngredientSettings />;
      case 'translations':
        return <TranslationsPanel />;
      default:
        return <MonitoringPanel />;
    }
  };

  return (
    <div className="container-fluid">
      <div className="container-fluid">
        <div className="mt-3">
          
          {/* Settings Header with Tabs */}
          <div className="bg-white rounded-xl shadow-sm border border-gray-200 mb-3">
            
            {/* Tab Navigation */}
            <div className="px-6" style={{ margin: '10px'}}>
              <div className="flex space-x-1">
                {tabs.map((tab) => (
                  <button
                    key={tab.id}
                    onClick={() => {
                      setActiveTab(tab.id);
                      window.location.hash = `#/settings/${tab.id}`;
                    }}
                    className={`flex items-center space-x-2 px-4 py-3 font-medium text-sm transition-colors relative ${
                      activeTab === tab.id
                        ? 'text-blue-600 border-b-2 border-blue-600'
                        : 'text-gray-600 hover:text-gray-900'
                    }`}
                  >
                    {tab.icon}
                    <span>{t(tab.nameKey)}</span>
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

