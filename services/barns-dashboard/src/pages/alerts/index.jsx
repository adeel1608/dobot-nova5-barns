/**
 * Alerts Page
 * Dedicated page for alert management and monitoring
 */

import React from 'react';
import AlertPanel from './components/AlertPanel';
import './styles.css';

export default function AlertsPage() {
  return (
    <div className="alerts-page">
      <div className="container mx-auto px-4 py-4">
        <div className="max-w-7xl mx-auto">
          
          {/* Page Header - Compact */}
          <div className="mb-4">
            <h1 className="text-2xl font-bold text-gray-900 mb-1">
              System Alerts
            </h1>
            <p className="text-sm text-gray-600">
              Monitor and manage system alerts and notifications
            </p>
          </div>
          
          {/* Main Alert Panel */}
          <AlertPanel />
          
        </div>
      </div>
    </div>
  );
} 