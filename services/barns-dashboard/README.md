# BARNS Dashboard - Developer Guide

## 🎯 Overview

The BARNS Dashboard is a React-based web application providing comprehensive monitoring and control interface for the coffee automation system. It serves as the primary operator interface for managing orders, monitoring system status, viewing camera feeds, and handling alerts.

## 📋 Table of Contents

- [🏗️ Architecture](#-architecture)
- [📂 Code Structure](#-code-structure)
- [🔌 API Structure](#-api-structure)
- [🎨 Frontend Structure](#-frontend-structure)
- [🧭 Navigation System](#-navigation-system)
- [🚀 Adding Features](#-adding-features)
- [⚙️ Development Setup](#-development-setup)
- [🎯 State Management](#-state-management)
- [📱 Responsive Design](#-responsive-design)
- [🎨 Styling Guide](#-styling-guide)
- [🔧 Configuration](#-configuration)
- [📖 API Integration Patterns](#-api-integration-patterns)
- [🛠️ Development Workflow](#-development-workflow)

## 🏗️ Architecture

### Technology Stack
- **React 19.1.0**: Modern React with hooks and functional components
- **Zustand 5.0.5**: Lightweight state management
- **Tailwind CSS 4.1.7**: Utility-first CSS framework
- **Axios 1.9.0**: HTTP client for API communication
- **Vite 6.3.5**: Fast build tool and development server
- **React DnD Kit**: Drag-and-drop functionality for order management

### System Components
```
┌─────────────────────────────────────────────────────────┐
│                    BARNS Dashboard                      │
├─────────────────────────────────────────────────────────┤
│ Pages: Dashboard | Alerts | Cameras | Inventory | Logs │
├─────────────────────────────────────────────────────────┤
│              Store Layer (Zustand)                     │
├─────────────────────────────────────────────────────────┤
│               API Layer (Axios)                        │
├─────────────────────────────────────────────────────────┤
│         Backend Services via Nginx Proxy               │
└─────────────────────────────────────────────────────────┘
```

## 📂 Code Structure

```
src/
├── 📄 main.jsx                    # App entry point
├── 📄 App.jsx                     # Main app component with routing
├── 📄 index.css                   # Global styles
├── 📄 App.css                     # App-specific styles
├── 📄 theme.js                    # Theme configuration
├── 📄 store.js                    # Legacy store (deprecated)
│
├── 📁 api/                        # API communication layer
│   ├── 📄 base.js                 # Base API client with error handling
│   ├── 📄 index.js                # API exports
│   ├── 📄 alerts.js               # Alerts API endpoints
│   ├── 📄 cameras.js              # Camera feeds API
│   ├── 📄 inventory.js            # Inventory management API
│   ├── 📄 logs.js                 # System logs API
│   ├── 📄 orders.js               # Order management API
│   └── 📄 system.js               # System status API
│
├── 📁 store/                      # State management (Zustand)
│   ├── 📄 index.js                # Combined store and WebSocket management
│   ├── 📄 dashboardStore.js       # Dashboard and orders state
│   ├── 📄 alertsStore.js          # Alerts management state
│   ├── 📄 camerasStore.js         # Camera feeds state
│   ├── 📄 inventoryStore.js       # Inventory management state
│   └── 📄 logsStore.js            # System logs state
│
├── 📁 utils/                      # Utility functions
│   ├── 📄 config.js               # App configuration constants
│   ├── 📄 websocket.js            # WebSocket management
│   ├── 📄 errorHandler.js         # Error handling utilities
│   └── 📄 inventoryData.js        # Mock inventory data
│
├── 📁 pages/                      # Page components
│   ├── 📁 dashboard/              # Main dashboard page
│   │   ├── 📄 index.jsx           # Dashboard page layout
│   │   ├── 📄 styles.css          # Dashboard-specific styles
│   │   └── 📁 components/         # Dashboard components
│   │       ├── 📄 AlertsPanel.jsx # Alerts display widget
│   │       ├── 📄 SystemPanel.jsx # System status widget
│   │       ├── 📄 OrderQueue.jsx  # Order management interface
│   │       ├── 📄 OrderDetails.jsx# Order details modal
│   │       └── 📄 IngredientsIndicator.jsx # Ingredient levels
│   │
│   ├── 📁 alerts/                 # Alerts management page
│   │   ├── 📄 index.jsx           # Alerts page layout
│   │   ├── 📄 styles.css          # Alerts-specific styles
│   │   └── 📁 components/
│   │       └── 📄 AlertPanel.jsx  # Main alerts interface
│   │
│   ├── 📁 cameras/                # Camera feeds page
│   │   ├── 📄 index.jsx           # Camera page layout
│   │   ├── 📄 styles.css          # Camera-specific styles
│   │   └── 📁 components/
│   │       └── 📄 UnifiedCameraPanel.jsx # Camera grid display
│   │
│   ├── 📁 inventory/              # Inventory management page
│   │   ├── 📄 index.jsx           # Inventory page layout
│   │   ├── 📄 styles.css          # Inventory-specific styles
│   │   └── 📁 components/
│   │       └── 📄 InventoryPanel.jsx # Inventory interface
│   │
│   └── 📁 logs/                   # System logs page
│       ├── 📄 index.jsx           # Logs page layout
│       ├── 📄 styles.css          # Logs-specific styles
│       └── 📁 components/
│           └── 📄 LogsPanel.jsx   # Logs viewer interface
│
├── 📁 shared/                     # Shared components (empty currently)
├── 📁 assets/                     # Static assets
└── 📁 components/                 # Global components (empty currently)
```

## 🔌 API Structure

### Base API Configuration

The API layer uses a centralized client with standardized error handling:

```javascript
// src/api/base.js
class APIClient {
  constructor(baseURL = '/api', defaultTimeout = 10000)
  
  // Standardized methods
  async get(url, params, options)
  async post(url, data, options)
  async put(url, data, options)
  async patch(url, data, options)
  async delete(url, options)
  
  // Helper methods
  async getList(url, params, itemName)
  async getById(url, id, itemName)
  async create(url, data, itemName)
  async update(url, id, data, itemName)
  async remove(url, id, itemName)
}
```

### API Modules

| Module | File | Purpose | Key Endpoints |
|--------|------|---------|---------------|
| **Orders** | `orders.js` | Order management | `/orders`, `/orders/{id}/start` |
| **Alerts** | `alerts.js` | Alert handling | `/alerts`, `/alerts/{id}/acknowledge` |
| **Cameras** | `cameras.js` | Video feeds | `/cameras`, `/cameras/{id}/stream` |
| **Inventory** | `inventory.js` | Stock management | `/inventory`, `/inventory/status` |
| **System** | `system.js` | System status | `/system/health`, `/system/status` |
| **Logs** | `logs.js` | System logging | `/logs` |

### API Response Format

All API responses follow a standardized format:

```javascript
{
  success: boolean,
  data: any,
  message: string,
  status?: number,
  error?: string,
  details?: {
    operation: string,
    status: number,
    technical_details: string,
    endpoint: string
  }
}
```

### API Configuration

```javascript
// src/utils/config.js
export const API_CONFIG = {
  API_BASE: '/api',                    // All API calls via nginx proxy
  WEBSOCKET_BASE: '/ws',               // WebSocket connections
  VIDEO_STREAM: 'http://localhost:8001' // Direct video stream
};
```

## 🎨 Frontend Structure

### Page Architecture

Each page follows a consistent structure:

```
pages/{page-name}/
├── index.jsx          # Page layout and data fetching
├── styles.css         # Page-specific styles
└── components/        # Page-specific components
    └── {PageName}Panel.jsx  # Main page component
```

### Component Pattern

```javascript
// Typical page structure
const PageIndex = () => {
  const store = usePageStore();
  
  useEffect(() => {
    store.fetchData();
  }, []);
  
  return (
    <div className="min-h-screen bg-white">
      <div className="max-w-7xl mx-auto px-3 py-3">
        <PagePanel />
      </div>
    </div>
  );
};
```

### Design System

- **Layout**: Consistent max-width containers (`max-w-7xl`)
- **Spacing**: Tailwind spacing classes (`px-3 py-3`)
- **Colors**: Light theme with white backgrounds
- **Typography**: Consistent heading and text sizes
- **Shadows**: Subtle shadows for depth (`shadow-sm`, `shadow-md`)
- **Rounded Corners**: Modern rounded corners (`rounded-lg`, `rounded-xl`)

## 🧭 Navigation System

### Navigation Structure

The main navigation is implemented in `App.jsx`:

```javascript
const navigationItems = [
  { name: 'Dashboard', href: '/', icon: '🏠' },
  { name: 'Alerts', href: '/alerts', icon: '🚨' },
  { name: 'Cameras', href: '/cameras', icon: '📹' },
  { name: 'Inventory', href: '/inventory', icon: '📦' },
  { name: 'Logs', href: '/logs', icon: '📋' }
];
```

### Adding New Navigation Items

1. **Add route in App.jsx**:
```javascript
const [currentPage, setCurrentPage] = useState('dashboard');

// Add new page to navigation
const navigationItems = [
  // ... existing items
  { name: 'NewPage', href: '/newpage', icon: '🆕' }
];

// Add route handling
const renderPage = () => {
  switch (currentPage) {
    // ... existing cases
    case 'newpage': return <NewPageIndex />;
    default: return <DashboardIndex />;
  }
};
```

2. **Create page directory**:
```
src/pages/newpage/
├── index.jsx
├── styles.css
└── components/
    └── NewPagePanel.jsx
```

## 🚀 Adding Features

### 1. Adding a New Page

**Step 1: Create page structure**
```bash
mkdir src/pages/analytics
mkdir src/pages/analytics/components
touch src/pages/analytics/index.jsx
touch src/pages/analytics/styles.css
touch src/pages/analytics/components/AnalyticsPanel.jsx
```

**Step 2: Create page component**
```javascript
// src/pages/analytics/index.jsx
import React, { useEffect } from 'react';
import { useAnalyticsStore } from '../../store/analyticsStore';
import AnalyticsPanel from './components/AnalyticsPanel';
import './styles.css';

const AnalyticsIndex = () => {
  const analyticsStore = useAnalyticsStore();

  useEffect(() => {
    analyticsStore.fetchData();
  }, []);

  return (
    <div className="min-h-screen bg-white">
      <div className="max-w-7xl mx-auto px-3 py-3">
        <AnalyticsPanel />
      </div>
    </div>
  );
};

export default AnalyticsIndex;
```

**Step 3: Create store**
```javascript
// src/store/analyticsStore.js
import { create } from 'zustand';
import { analyticsAPI } from '../api/analytics';

export const useAnalyticsStore = create((set, get) => ({
  data: [],
  loading: false,
  error: null,

  fetchData: async () => {
    set({ loading: true, error: null });
    try {
      const result = await analyticsAPI.getData();
      if (result.success) {
        set({ data: result.data, loading: false });
      } else {
        set({ error: result.error, loading: false });
      }
    } catch (error) {
      set({ error: error.message, loading: false });
    }
  }
}));
```

**Step 4: Add API module**
```javascript
// src/api/analytics.js
import { apiClient } from './base';

export const analyticsAPI = {
  getData: () => apiClient.getList('/analytics', {}, 'analytics data'),
  getMetrics: (timeRange) => apiClient.get('/analytics/metrics', { range: timeRange })
};
```

**Step 5: Add to navigation**
```javascript
// In App.jsx
const navigationItems = [
  // ... existing items
  { name: 'Analytics', href: '/analytics', icon: '📊' }
];

// Add import
import AnalyticsIndex from './pages/analytics';

// Add to renderPage()
case 'analytics': return <AnalyticsIndex />;
```

### 2. Adding Real-time Data Widgets

**Create widget component**:
```javascript
// src/components/widgets/MetricsWidget.jsx
import React, { useEffect, useState } from 'react';

const MetricsWidget = ({ title, endpoint, refreshInterval = 5000 }) => {
  const [data, setData] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch(endpoint);
        const result = await response.json();
        setData(result);
      } catch (error) {
        console.error('Widget fetch error:', error);
      } finally {
        setLoading(false);
      }
    };

    fetchData();
    const interval = setInterval(fetchData, refreshInterval);
    return () => clearInterval(interval);
  }, [endpoint, refreshInterval]);

  return (
    <div className="bg-white p-4 rounded-lg shadow-md">
      <h3 className="text-lg font-semibold mb-2">{title}</h3>
      {loading ? (
        <div className="animate-pulse bg-gray-200 h-16 rounded"></div>
      ) : (
        <div className="text-2xl font-bold text-green-600">
          {JSON.stringify(data)}
        </div>
      )}
    </div>
  );
};

export default MetricsWidget;
```

**Usage in pages**:
```javascript
<MetricsWidget 
  title="System Performance" 
  endpoint="/api/metrics/performance"
  refreshInterval={3000}
/>
```

### 3. Adding New Alert Types

**Step 1: Update alerts store**
```javascript
// In src/store/alertsStore.js
const ALERT_TYPES = {
  SYSTEM: 'system',
  INVENTORY: 'inventory', 
  ORDER: 'order',
  CUSTOM: 'custom'  // Add new type
};

// Add custom alert handling
handleCustomAlert: (alert) => {
  // Custom alert logic
  set(state => ({
    alerts: [alert, ...state.alerts]
  }));
}
```

**Step 2: Update alert panel**
```javascript
// In AlertPanel.jsx
const getAlertIcon = (type) => {
  switch (type) {
    case 'system': return '⚙️';
    case 'inventory': return '📦';
    case 'order': return '☕';
    case 'custom': return '🔔';  // Add new icon
    default: return '⚠️';
  }
};
```

### 4. Adding Drag-and-Drop Features

**Install dependencies** (already included):
```bash
npm install @dnd-kit/core @dnd-kit/sortable
```

**Implement sortable list**:
```javascript
import { DndContext, closestCenter } from '@dnd-kit/core';
import { SortableContext, verticalListSortingStrategy } from '@dnd-kit/sortable';

const SortableList = ({ items, onReorder }) => {
  const handleDragEnd = (event) => {
    const { active, over } = event;
    if (active.id !== over.id) {
      const oldIndex = items.findIndex(item => item.id === active.id);
      const newIndex = items.findIndex(item => item.id === over.id);
      onReorder(arrayMove(items, oldIndex, newIndex));
    }
  };
  
  return (
    <DndContext collisionDetection={closestCenter} onDragEnd={handleDragEnd}>
      <SortableContext items={items} strategy={verticalListSortingStrategy}>
        {items.map(item => (
          <SortableItem key={item.id} item={item} />
        ))}
      </SortableContext>
    </DndContext>
  );
};
```

## ⚙️ Development Setup

### Prerequisites
- Node.js 18+ 
- npm or yarn
- Access to BARNS backend services

### Installation
```bash
# Navigate to dashboard directory
cd services/barns-dashboard

# Install dependencies
npm install

# Start development server
npm run dev

# Build for production
npm run build

# Preview production build
npm run preview

# Lint code
npm run lint
```

### Development Server
```bash
npm run dev
# Runs on http://localhost:5173 (Vite default)
```

### Environment Configuration
Create `.env` file:
```env
VITE_API_BASE=http://localhost:8000/api
VITE_VIDEO_STREAM=http://localhost:8001
VITE_WS_BASE=ws://localhost:8000/ws
```

## 🎯 State Management

### Zustand Store Pattern

Each store follows a consistent pattern:

```javascript
// Store structure
export const useExampleStore = create((set, get) => ({
  // State
  data: [],
  loading: false,
  error: null,
  
  // Actions
  fetchData: async () => {
    set({ loading: true, error: null });
    try {
      const result = await api.getData();
      if (result.success) {
        set({ data: result.data, loading: false });
      } else {
        set({ error: result.error, loading: false });
      }
    } catch (error) {
      set({ error: error.message, loading: false });
    }
  },
  
  updateItem: (id, updates) => {
    set(state => ({
      data: state.data.map(item => 
        item.id === id ? { ...item, ...updates } : item
      )
    }));
  },
  
  reset: () => set({ data: [], loading: false, error: null })
}));
```

### Store Modules

| Store | Purpose | Key State | Key Actions |
|-------|---------|-----------|-------------|
| **dashboardStore** | Orders & system status | `orders`, `systemStatus` | `fetchOrders`, `startOrder` |
| **alertsStore** | Alert management | `alerts`, `acknowledged` | `fetchAlerts`, `acknowledgeAlert` |
| **camerasStore** | Video feeds | `cameras`, `streamStatus` | `getStreamStatus`, `refreshStream` |
| **inventoryStore** | Stock levels | `inventory`, `status` | `fetchInventoryStatus`, `updateStock` |
| **logsStore** | System logs | `logs` | `addLog`, `clearLogs` |

### WebSocket Integration

```javascript
// WebSocket management in store/index.js
export const useWebSocketStore = create((set, get) => ({
  connectionStatus: {
    orders: 'disconnected',
    alerts: 'disconnected'
  },
  
  connectOrderWS: () => {
    wsManager.connect('orders', '/orders', {
      onMessage: (data) => {
        if (data.type === 'order_update') {
          useDashboardStore.getState().fetchOrders();
        }
      }
    });
  },
  
  connectAlertWS: () => {
    wsManager.connect('alerts', '/alerts', {
      onMessage: (data) => {
        if (data.type === 'alert_update') {
          useAlertsStore.getState().fetchAlerts();
        }
      }
    });
  }
}));
```

## 📱 Responsive Design

### Breakpoint Strategy
- **Mobile First**: Base styles for mobile (320px+)
- **Tablet**: `md:` classes (768px+)
- **Desktop**: `lg:` classes (1024px+)
- **Large Desktop**: `xl:` classes (1280px+)

### Responsive Patterns

**Grid Layouts**:
```javascript
<div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
  {/* Responsive grid items */}
</div>
```

**Camera Grid**:
```javascript
<div className="grid grid-cols-1 md:grid-cols-2 gap-4">
  {/* 1 column mobile, 2 columns tablet+ */}
</div>
```

**Navigation**:
```javascript
<nav className="hidden md:flex md:space-x-8">
  {/* Hidden on mobile, flex on tablet+ */}
</nav>
```

## 🎨 Styling Guide

### Color Palette
```css
/* Primary theme colors */
--color-primary: #004029;
--color-primary-light: #00754a;
--color-success: #10b981;
--color-warning: #f59e0b;
--color-error: #ef4444;
--color-info: #3b82f6;
```

### Typography Scale
```css
/* Heading hierarchy */
.text-3xl    /* Main page titles */
.text-2xl    /* Section titles */
.text-xl     /* Subsection titles */
.text-lg     /* Component titles */
.text-base   /* Body text */
.text-sm     /* Secondary text */
.text-xs     /* Labels and meta text */
```

### Spacing System
```css
/* Consistent spacing */
.p-3    /* Page padding */
.p-4    /* Component padding */
.p-6    /* Section padding */
.gap-4  /* Grid gaps */
.space-y-4  /* Vertical spacing */
```

### Component Patterns
```css
/* Card pattern */
.bg-white .rounded-lg .shadow-md .p-4

/* Button pattern */
.px-4 .py-2 .rounded .font-medium .transition-colors

/* Form input pattern */
.border .rounded .px-3 .py-2 .focus:outline-none .focus:ring-2
```

## 🔧 Configuration

### Main Configuration Files

**API & UI Config** (`src/utils/config.js`):
```javascript
export const API_CONFIG = {
  API_BASE: '/api',
  WEBSOCKET_BASE: '/ws',
  VIDEO_STREAM: 'http://localhost:8001'
};

export const UI_CONFIG = {
  HEALTH_CHECK_INTERVAL: 120000,
  ORDER_REFRESH_INTERVAL: 30000,
  ALERT_REFRESH_INTERVAL: 60000,
  MAX_LOGS: 1000
};
```

**Tailwind Config** (`tailwind.config.js`):
```javascript
export default {
  content: ['./index.html', './src/**/*.{js,ts,jsx,tsx}'],
  theme: {
    extend: {
      colors: {
        'barns-primary': '#004029',
        'barns-success': '#10b981'
      }
    }
  }
}
```

**Vite Config** (`vite.config.js`):
```javascript
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api': 'http://localhost:8000',
      '/ws': {
        target: 'ws://localhost:8000',
        ws: true
      }
    }
  }
})
```

## 📖 API Integration Patterns

### Standard API Call Pattern
```javascript
// In store action
const fetchData = async () => {
  set({ loading: true, error: null });
  
  try {
    const result = await apiClient.getList('/endpoint', {}, 'items');
    
    if (result.success) {
      set({ data: result.data, loading: false });
      addLog('Store', 'info', `Fetched ${result.data.length} items`);
    } else {
      set({ error: result.error, loading: false });
      addLog('Store', 'error', `Failed to fetch data: ${result.error}`);
    }
  } catch (error) {
    set({ error: error.message, loading: false });
    addLog('Store', 'error', `Network error: ${error.message}`);
  }
};
```

### Error Handling Pattern
```javascript
// Centralized error handling
import { extractErrorMessage } from '../utils/errorHandler';

const handleApiError = (error, operation) => {
  const errorMessage = extractErrorMessage(error);
  
  return {
    success: false,
    error: errorMessage,
    details: {
      operation,
      status: error.response?.status,
      technical_details: error.message
    }
  };
};
```

### WebSocket Pattern
```javascript
// WebSocket connection management
import { wsManager } from '../utils/websocket';

const connectToUpdates = () => {
  wsManager.connect('orders', '/orders', {
    onOpen: () => console.log('Connected to orders'),
    onMessage: (data) => {
      if (data.type === 'order_update') {
        refreshOrders();
      }
    },
    onClose: () => console.log('Disconnected from orders')
  });
};
```

## 🛠️ Development Workflow

### Adding a Complete Feature

**Example: Adding Equipment Status Widget**

1. **Plan the feature**:
   - Identify data requirements
   - Design UI mockup
   - Plan API endpoints needed

2. **Create API module**:
```javascript
// src/api/equipment.js
import { apiClient } from './base';

export const equipmentAPI = {
  getStatus: () => apiClient.get('/equipment/status'),
  getHealth: () => apiClient.get('/equipment/health'),
  restart: (equipmentId) => apiClient.post(`/equipment/${equipmentId}/restart`)
};
```

3. **Create store**:
```javascript
// src/store/equipmentStore.js
import { create } from 'zustand';
import { equipmentAPI } from '../api/equipment';

export const useEquipmentStore = create((set, get) => ({
  equipment: [],
  health: {},
  loading: false,
  error: null,

  fetchStatus: async () => {
    set({ loading: true, error: null });
    try {
      const [statusResult, healthResult] = await Promise.all([
        equipmentAPI.getStatus(),
        equipmentAPI.getHealth()
      ]);
      
      if (statusResult.success && healthResult.success) {
        set({ 
          equipment: statusResult.data,
          health: healthResult.data,
          loading: false 
        });
      }
    } catch (error) {
      set({ error: error.message, loading: false });
    }
  },

  restartEquipment: async (equipmentId) => {
    const result = await equipmentAPI.restart(equipmentId);
    if (result.success) {
      get().fetchStatus(); // Refresh after restart
    }
    return result;
  }
}));
```

4. **Create component**:
```javascript
// src/pages/dashboard/components/EquipmentStatus.jsx
import React, { useEffect } from 'react';
import { useEquipmentStore } from '../../../store/equipmentStore';

const EquipmentStatus = () => {
  const { equipment, health, loading, error, fetchStatus, restartEquipment } = useEquipmentStore();

  useEffect(() => {
    fetchStatus();
    const interval = setInterval(fetchStatus, 30000); // Refresh every 30s
    return () => clearInterval(interval);
  }, []);

  const handleRestart = async (equipmentId) => {
    const result = await restartEquipment(equipmentId);
    if (result.success) {
      alert('Equipment restarted successfully');
    } else {
      alert(`Failed to restart: ${result.error}`);
    }
  };

  if (loading) return <div className="animate-pulse bg-gray-200 h-32 rounded"></div>;
  if (error) return <div className="text-red-600">Error: {error}</div>;
  
  return (
    <div className="bg-white rounded-lg shadow-md p-4">
      <h3 className="text-lg font-semibold mb-4">Equipment Status</h3>
      
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {equipment.map(item => (
          <div key={item.id} className="border rounded p-3">
            <div className="flex justify-between items-center">
              <h4 className="font-medium">{item.name}</h4>
              <span className={`px-2 py-1 rounded text-xs ${
                item.status === 'active' ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'
              }`}>
                {item.status}
              </span>
            </div>
            
            <div className="mt-2 text-sm text-gray-600">
              Health: {health[item.id]?.score || 'Unknown'}%
            </div>
            
            <button
              onClick={() => handleRestart(item.id)}
              className="mt-2 px-3 py-1 bg-blue-600 text-white rounded text-xs hover:bg-blue-700"
            >
              Restart
            </button>
          </div>
        ))}
      </div>
    </div>
  );
};

export default EquipmentStatus;
```

5. **Add to dashboard**:
```javascript
// In src/pages/dashboard/index.jsx
import EquipmentStatus from './components/EquipmentStatus';

// Add to layout
<div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
  <SystemPanel />
  <EquipmentStatus />  {/* Add here */}
  <AlertsPanel />
  <IngredientsIndicator />
</div>
```

6. **Add to main store initialization**:
```javascript
// In src/store/index.js
import { useEquipmentStore } from './equipmentStore';

// Add to initialize function
const equipmentStore = useEquipmentStore.getState();
await equipmentStore.fetchStatus();

// Export for use
export { useEquipmentStore };
```

### Testing New Features

**Component Testing**:
```javascript
// Test component with React Testing Library
import { render, screen, waitFor } from '@testing-library/react';
import EquipmentStatus from './EquipmentStatus';

test('renders equipment status', async () => {
  render(<EquipmentStatus />);
  
  await waitFor(() => {
    expect(screen.getByText('Equipment Status')).toBeInTheDocument();
  });
});
```

**Store Testing**:
```javascript
// Test store logic
import { useEquipmentStore } from './equipmentStore';

test('fetches equipment status', async () => {
  const store = useEquipmentStore.getState();
  await store.fetchStatus();
  
  expect(store.equipment.length).toBeGreaterThan(0);
  expect(store.loading).toBe(false);
});
```

### Debugging Tips

**1. Console Logging**:
```javascript
// Enable debug logs in development
if (ENV_CONFIG.FEATURES.ENABLE_DEBUG_LOGS) {
  console.log('Store state:', get());
}
```

**2. React DevTools**:
- Install React Developer Tools browser extension
- Use Components tab to inspect component state
- Use Profiler tab to identify performance issues

**3. Network Debugging**:
- Check Network tab in browser DevTools
- API calls are logged automatically by base client
- WebSocket connections visible in Network tab

**4. Store Debugging**:
```javascript
// Add to store for debugging
_debug: () => get(),
_reset: () => set(initialState)
```

### Performance Optimization

**1. Component Memoization**:
```javascript
import React, { memo } from 'react';

const ExpensiveComponent = memo(({ data }) => {
  // Component logic
});
```

**2. Store Selectors**:
```javascript
// Use selectors to prevent unnecessary re-renders
const orders = useStore(state => state.orders);
const loading = useStore(state => state.loading);
```

**3. Lazy Loading**:
```javascript
// Lazy load heavy components
const HeavyComponent = React.lazy(() => import('./HeavyComponent'));

// Use with Suspense
<Suspense fallback={<div>Loading...</div>}>
  <HeavyComponent />
</Suspense>
```

---

## 🎉 Quick Start Checklist

**For New Developers**:

- [ ] Clone repository and navigate to `/services/barns-dashboard`
- [ ] Run `npm install` to install dependencies
- [ ] Start development server with `npm run dev`
- [ ] Open browser to `http://localhost:5173`
- [ ] Explore the codebase starting with `src/App.jsx`
- [ ] Check `src/store/index.js` for state management
- [ ] Review `src/api/base.js` for API patterns
- [ ] Look at `src/pages/dashboard/` for component patterns
- [ ] Try making a simple change to see hot-reload

**For Adding Features**:

- [ ] Identify which page the feature belongs to
- [ ] Create API module if backend integration needed
- [ ] Create or update store for state management
- [ ] Create component following existing patterns
- [ ] Add component to appropriate page
- [ ] Test feature works as expected
- [ ] Add error handling and loading states
- [ ] Ensure responsive design works

---

This README provides a comprehensive guide for developers to understand, navigate, and extend the BARNS Dashboard codebase. The structure is designed to be maintainable, scalable, and consistent across all features.
