# BARNS Dashboard Service

## Purpose and Workflow

The BARNS Dashboard is a React-based web application that provides a comprehensive monitoring and control interface for the coffee automation system. It serves as the primary user interface for operators to manage orders, monitor system status, and respond to alerts.

### Core Responsibilities
- **Order Management**: Display, create, and manage coffee orders through an intuitive interface
- **Real-time Monitoring**: Show live system status, video feeds, and order progress
- **Queue Control**: Enable drag-and-drop reordering of the order queue
- **Alert Management**: Display and handle system alerts and notifications
- **System Control**: Provide controls for starting orders and emergency stops
- **Analytics Dashboard**: Show performance metrics and system statistics

### Workflow
1. **System Overview**: Dashboard displays current system status and active orders
2. **Order Creation**: Users can create new orders with drink specifications
3. **Queue Management**: Orders can be reordered via drag-and-drop interface
4. **Order Processing**: Real-time updates show order progress and completion
5. **Alert Handling**: System alerts are displayed and can be acknowledged
6. **Video Monitoring**: Live camera feeds provide visual system monitoring

### Dashboard Flow
```
User Interface → Order Management → Real-time Updates → System Control → Monitoring
```

## Architecture and Components

### Frontend Stack
- **React 18**: Modern React with hooks and functional components
- **TypeScript**: Type-safe development for better code quality
- **Zustand**: Lightweight state management for application state
- **Tailwind CSS**: Utility-first CSS framework for responsive design
- **React DnD**: Drag-and-drop functionality for order queue management

### Key Components

#### Order Management
- **OrderQueue**: Displays and manages the order queue with drag-and-drop
- **OrderCard**: Individual order display with status and controls
- **OrderDetailsModal**: Detailed view of order progress and timeline
- **NewOrderForm**: Form for creating new orders

#### System Monitoring
- **SystemStatus**: Overall system health and status indicators
- **VideoStreams**: Live camera feeds from the coffee-making process
- **AlertPanel**: System alerts and notifications display
- **PerformanceMetrics**: System performance and analytics

#### Real-time Features
- **WebSocket Integration**: Live updates from backend services
- **Auto-refresh**: Automatic data updates for current information
- **Event Handling**: Real-time response to system events

## API Integration

### Order Management Service (OMS) Integration
```typescript
// Order operations
const createOrder = async (order: NewOrder) => {
  const response = await fetch(`${OMS_URL}/orders/`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(order)
  });
  return response.json();
};

const startOrder = async (orderId: number) => {
  await fetch(`${OMS_URL}/orders/${orderId}/start`, {
    method: 'PATCH'
  });
};
```

### Real-time Updates
```typescript
// WebSocket connection for live updates
const connectWebSocket = () => {
  const ws = new WebSocket(`${WS_URL}/ws/orders`);
  
  ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    handleOrderUpdate(data);
  };
  
  return ws;
};
```

### Video Stream Integration
```typescript
// Video stream display
const VideoFeed = ({ cameraId }: { cameraId: string }) => {
  return (
    <img
      src={`${VIDEO_STREAM_URL}/stream/${cameraId}`}
      alt={`Camera ${cameraId}`}
      className="w-full h-auto"
    />
  );
};
```

## State Management

### Zustand Store Structure
```typescript
interface DashboardState {
  // Order management
  orders: Order[];
  selectedOrder: Order | null;
  
  // System status
  systemStatus: SystemStatus;
  alerts: Alert[];
  
  // UI state
  isLoading: boolean;
  currentTab: string;
  
  // Actions
  setOrders: (orders: Order[]) => void;
  updateOrder: (order: Order) => void;
  setSystemStatus: (status: SystemStatus) => void;
}
```

## Adding New Modules

### 1. Adding New Dashboard Tabs

**Step 1**: Create new component:
```typescript
// components/NewTab.tsx
import React from 'react';

const NewTab: React.FC = () => {
  return (
    <div className="p-6">
      <h2 className="text-2xl font-bold mb-4">New Feature</h2>
      {/* Tab content */}
    </div>
  );
};

export default NewTab;
```

**Step 2**: Add tab to main layout:
```typescript
// components/Dashboard.tsx
import NewTab from './NewTab';

const tabs = [
  { key: 'overview', label: 'Overview', component: <Overview /> },
  { key: 'orders', label: 'Orders', component: <OrderManagement /> },
  { key: 'new-tab', label: 'New Feature', component: <NewTab /> },  // Add here
];
```

### 2. Adding Real-time Data Widgets

**Step 1**: Create data widget component:
```typescript
// components/widgets/MetricsWidget.tsx
import React, { useEffect, useState } from 'react';

interface MetricsWidgetProps {
  title: string;
  endpoint: string;
  refreshInterval?: number;
}

const MetricsWidget: React.FC<MetricsWidgetProps> = ({ 
  title, 
  endpoint, 
  refreshInterval = 5000 
}) => {
  const [data, setData] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch(endpoint);
        const result = await response.json();
        setData(result);
      } catch (error) {
        console.error('Failed to fetch data:', error);
      } finally {
        setLoading(false);
      }
    };

    fetchData();
    const interval = setInterval(fetchData, refreshInterval);
    return () => clearInterval(interval);
  }, [endpoint, refreshInterval]);

  if (loading) return <div>Loading...</div>;

  return (
    <div className="bg-white p-4 rounded-lg shadow">
      <h3 className="text-lg font-semibold mb-2">{title}</h3>
      <pre className="text-sm">{JSON.stringify(data, null, 2)}</pre>
    </div>
  );
};

export default MetricsWidget;
```

**Step 2**: Add widget to dashboard:
```typescript
// In Overview component
<MetricsWidget 
  title="System Performance" 
  endpoint="/api/metrics/performance"
  refreshInterval={3000}
/>
```

### 3. Adding New Order Types

**Step 1**: Update order types:
```typescript
// types/Order.ts
export interface DrinkOption {
  type: 'Espresso' | 'Americano' | 'Latte' | 'Cappuccino' | 'Mocha' | 'NewDrink';
  size: 'small' | 'medium' | 'large';
  addons: string[];
}

export const DRINK_OPTIONS = [
  'Espresso', 
  'Americano', 
  'Latte', 
  'Cappuccino', 
  'Mocha',
  'NewDrink'  // Add new drink type
];
```

**Step 2**: Update order form:
```typescript
// components/NewOrderForm.tsx
const NewOrderForm = () => {
  const [selectedDrink, setSelectedDrink] = useState('Latte');
  
  return (
    <form>
      <select 
        value={selectedDrink} 
        onChange={(e) => setSelectedDrink(e.target.value)}
      >
        {DRINK_OPTIONS.map(drink => (
          <option key={drink} value={drink}>{drink}</option>
        ))}
      </select>
      {/* Additional form fields */}
    </form>
  );
};
```

### 4. Adding Advanced Analytics

**Step 1**: Create analytics service:
```typescript
// services/analytics.ts
export class AnalyticsService {
  private static instance: AnalyticsService;
  
  static getInstance(): AnalyticsService {
    if (!this.instance) {
      this.instance = new AnalyticsService();
    }
    return this.instance;
  }
  
  async getOrderMetrics(timeRange: string) {
    const response = await fetch(`/api/analytics/orders?range=${timeRange}`);
    return response.json();
  }
  
  async getPerformanceMetrics() {
    const response = await fetch('/api/analytics/performance');
    return response.json();
  }
  
  async getAlertMetrics() {
    const response = await fetch('/api/analytics/alerts');
    return response.json();
  }
}
```

**Step 2**: Create analytics dashboard:
```typescript
// components/Analytics.tsx
import React, { useEffect, useState } from 'react';
import { AnalyticsService } from '../services/analytics';

const Analytics: React.FC = () => {
  const [metrics, setMetrics] = useState(null);
  const analytics = AnalyticsService.getInstance();

  useEffect(() => {
    const loadMetrics = async () => {
      const [orders, performance, alerts] = await Promise.all([
        analytics.getOrderMetrics('24h'),
        analytics.getPerformanceMetrics(),
        analytics.getAlertMetrics()
      ]);
      
      setMetrics({ orders, performance, alerts });
    };

    loadMetrics();
  }, []);

  return (
    <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
      <MetricCard title="Orders Processed" value={metrics?.orders?.total} />
      <MetricCard title="Success Rate" value={`${metrics?.orders?.successRate}%`} />
      <MetricCard title="Active Alerts" value={metrics?.alerts?.active} />
    </div>
  );
};
```

### 5. Adding Custom Notifications

**Step 1**: Create notification system:
```typescript
// services/notifications.ts
export interface Notification {
  id: string;
  type: 'success' | 'warning' | 'error' | 'info';
  title: string;
  message: string;
  timestamp: Date;
  autoClose?: boolean;
}

export class NotificationService {
  private notifications: Notification[] = [];
  private listeners: ((notifications: Notification[]) => void)[] = [];

  addNotification(notification: Omit<Notification, 'id' | 'timestamp'>) {
    const newNotification: Notification = {
      ...notification,
      id: Math.random().toString(36),
      timestamp: new Date()
    };

    this.notifications.push(newNotification);
    this.notifyListeners();

    if (notification.autoClose !== false) {
      setTimeout(() => {
        this.removeNotification(newNotification.id);
      }, 5000);
    }
  }

  removeNotification(id: string) {
    this.notifications = this.notifications.filter(n => n.id !== id);
    this.notifyListeners();
  }

  subscribe(listener: (notifications: Notification[]) => void) {
    this.listeners.push(listener);
    return () => {
      this.listeners = this.listeners.filter(l => l !== listener);
    };
  }

  private notifyListeners() {
    this.listeners.forEach(listener => listener([...this.notifications]));
  }
}
```

**Step 2**: Create notification component:
```typescript
// components/NotificationContainer.tsx
import React, { useEffect, useState } from 'react';
import { NotificationService, Notification } from '../services/notifications';

const NotificationContainer: React.FC = () => {
  const [notifications, setNotifications] = useState<Notification[]>([]);
  const notificationService = new NotificationService();

  useEffect(() => {
    return notificationService.subscribe(setNotifications);
  }, []);

  return (
    <div className="fixed top-4 right-4 z-50 space-y-2">
      {notifications.map(notification => (
        <NotificationCard
          key={notification.id}
          notification={notification}
          onClose={() => notificationService.removeNotification(notification.id)}
        />
      ))}
    </div>
  );
};
```

### 6. Adding Keyboard Shortcuts

**Step 1**: Create keyboard handler:
```typescript
// hooks/useKeyboardShortcuts.ts
import { useEffect } from 'react';

interface ShortcutConfig {
  [key: string]: () => void;
}

export const useKeyboardShortcuts = (shortcuts: ShortcutConfig) => {
  useEffect(() => {
    const handleKeyPress = (event: KeyboardEvent) => {
      const key = `${event.ctrlKey ? 'ctrl+' : ''}${event.shiftKey ? 'shift+' : ''}${event.key.toLowerCase()}`;
      
      if (shortcuts[key]) {
        event.preventDefault();
        shortcuts[key]();
      }
    };

    window.addEventListener('keydown', handleKeyPress);
    return () => window.removeEventListener('keydown', handleKeyPress);
  }, [shortcuts]);
};
```

**Step 2**: Use shortcuts in components:
```typescript
// In main Dashboard component
const Dashboard = () => {
  const shortcuts = {
    'ctrl+n': () => openNewOrderDialog(),
    'ctrl+r': () => refreshData(),
    'escape': () => closeModals(),
    'ctrl+s': () => saveCurrentState(),
  };

  useKeyboardShortcuts(shortcuts);
  
  return (
    <div>
      {/* Dashboard content */}
    </div>
  );
};
```

## Environment Configuration

### Environment Variables
```env
# API Endpoints
REACT_APP_OMS_URL=http://localhost:8001
REACT_APP_VIDEO_STREAM_URL=http://localhost:8002
REACT_APP_WEBSOCKET_URL=ws://localhost:8001

# Feature Flags
REACT_APP_ENABLE_ANALYTICS=true
REACT_APP_ENABLE_VIDEO_RECORDING=false
REACT_APP_DEBUG_MODE=false

# UI Configuration
REACT_APP_REFRESH_INTERVAL=5000
REACT_APP_AUTO_REFRESH=true
```

## Development Setup

1. **Install Dependencies**:
   ```bash
   cd services/barns-dashboard
   npm install
   ```

2. **Development Server**:
   ```bash
   npm start
   # Runs on http://localhost:3000
   ```

3. **Type Checking**:
   ```bash
   npm run type-check
   ```

4. **Build for Production**:
   ```bash
   npm run build
   ```

## Testing

### Unit Testing
```bash
# Run unit tests
npm test

# Run with coverage
npm run test:coverage
```

### End-to-End Testing
```bash
# Run E2E tests with Cypress
npm run test:e2e
```

### Component Testing
```typescript
// Example component test
import { render, screen } from '@testing-library/react';
import OrderCard from '../components/OrderCard';

test('renders order card with correct information', () => {
  const mockOrder = {
    id: 1,
    status: 'queued',
    cups: [{ type: 'Latte', size: 'medium' }]
  };

  render(<OrderCard order={mockOrder} />);
  
  expect(screen.getByText('Order #1')).toBeInTheDocument();
  expect(screen.getByText('Latte')).toBeInTheDocument();
});
```

## Performance Considerations

- **Code Splitting**: Lazy load components to reduce initial bundle size
- **Memoization**: Use React.memo and useMemo for expensive operations
- **Virtual Scrolling**: For large lists of orders or data
- **WebSocket Management**: Efficient connection handling and cleanup
- **Image Optimization**: Optimize video stream display and loading
- **State Management**: Efficient state updates and subscription patterns

## Styling and Theming

### Tailwind CSS Configuration
```javascript
// tailwind.config.js
module.exports = {
  content: ['./src/**/*.{js,jsx,ts,tsx}'],
  theme: {
    extend: {
      colors: {
        'barns-primary': '#2563eb',
        'barns-secondary': '#64748b',
        'barns-success': '#059669',
        'barns-warning': '#d97706',
        'barns-error': '#dc2626',
      }
    }
  }
};
```

### Component Styling
- **Consistent Design System**: Use Tailwind utilities for consistency
- **Responsive Design**: Mobile-first approach with responsive breakpoints
- **Dark Mode Support**: Optional dark theme implementation
- **Accessibility**: ARIA labels and keyboard navigation support
