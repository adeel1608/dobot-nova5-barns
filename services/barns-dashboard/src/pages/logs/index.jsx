/**
 * Logs Page
 * System logs monitoring and analysis
 */

import React from 'react';
import LogsPanel from './components/LogsPanel';
import './styles.css';

export default function LogsPage() {
  return (
    <div className="container-fluid bg-gray-50">
      <div className="container-fluid">
        <div className="mt-3">
          
          {/* Minimal Page Header */}
          {/* <div className="mb-3">
            <h1 className="text-xl font-bold text-gray-900 mb-1">
              System Logs
            </h1>
            <p className="text-xs text-gray-600">
              Monitor system activity and troubleshoot issues
            </p>
          </div> */}
          
          {/* Main Logs Panel */}
          <LogsPanel />
          
        </div>
      </div>
    </div>
  );
} 