/**
 * Cameras Page
 * Live camera feeds and video monitoring - Optimized for maximum space utilization
 */

import React from 'react';
import UnifiedCameraPanel from './components/UnifiedCameraPanel';
import './styles.css';

export default function CamerasPage() {
  return (
    <div className="cameras-page min-h-screen">
      <div className="container mx-auto px-3 py-3">
        <div className="max-w-7xl mx-auto">
          
          {/* Minimal Page Header */}
          <div className="mb-3">
            <h1 className="text-xl font-bold text-gray-900 mb-1">
              Live Camera Feeds
            </h1>
            <p className="text-xs text-gray-600">
              Monitor live video feeds from the coffee brewing system
            </p>
          </div>
          
          {/* Unified Camera Panel */}
          <UnifiedCameraPanel />
          
        </div>
      </div>
    </div>
  );
} 