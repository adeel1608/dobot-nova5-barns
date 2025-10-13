# BARNS Dashboard

## Brief Overview

The BARNS Dashboard is a modern React web application providing real-time monitoring and control of the entire coffee brewing system with live order tracking, inventory management, camera feeds, system alerts, and comprehensive analytics.

## Key Features

- **Real-Time Order Tracking**: Live updates via WebSocket/Socket.IO
- **Inventory Management**: Visual stock levels with refill controls
- **Camera Feeds**: Live MJPEG streams from multiple cameras
- **Order Queue Management**: Drag-and-drop queue reordering
- **System Alerts**: Low inventory and system warnings
- **Analytics Dashboard**: Order statistics and performance metrics
- **Recipe Management**: View and manage drink recipes
- **POS Integration**: Direct order entry interface
- **Responsive Design**: Modern UI with Tailwind CSS
- **State Management**: Redux for predictable state updates

## Architecture

```
┌──────────────────────────────────────────────────┐
│          BARNS Dashboard (React)                 │
│                                                  │
│  ┌────────────────────────────────────┐         │
│  │  Pages                             │         │
│  │  - Dashboard (main view)           │         │
│  │  - Orders                          │         │
│  │  - Inventory                       │         │
│  │  - Analytics                       │         │
│  │  - Settings                        │         │
│  └────────────┬───────────────────────┘         │
│               │                                  │
│               ↓                                  │
│  ┌────────────────────────────────────┐         │
│  │  API Client (api/)                 │         │
│  │  - orders.js                       │         │
│  │  - inventory.js                    │         │
│  │  - system.js                       │         │
│  └────────────┬───────────────────────┘         │
│               │                                  │
│               ↓                                  │
│  ┌────────────────────────────────────┐         │
│  │  WebSocket/Socket.IO               │         │
│  │  - Real-time updates               │         │
│  └────────────────────────────────────┘         │
└───────────────┬──────────────────────────────────┘
                │
                ↓
       ┌────────────────┐
       │  API Bridge    │
       │  Service       │
       │  Port 8000     │
       └────────────────┘
```

## Setup & Installation

### Prerequisites

- Node.js 16+
- npm or yarn
- API Bridge service running

### Local Development

```bash
cd services/barns-dashboard

# Install dependencies
npm install

# Set API endpoint
echo "VITE_API_URL=http://localhost:8000" > .env

# Start dev server
npm run dev

# Access at http://localhost:5173
```

### Production Build

```bash
# Build static files
npm run build

# Output in dist/
ls -la dist/

# Serve with nginx (handled by Docker)
```

### Docker Deployment

```bash
docker-compose up -d dashboard
# Access at http://localhost:3000
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `VITE_API_URL` | `http://api-bridge:8000` | API Bridge URL |
| `VIDEO_STREAM_URL` | `http://video-stream-service:8000` | Video stream service URL |

### nginx Configuration

Served via nginx in production (port 80 internal, 3000 external).

## Key Components

### Pages

1. **Dashboard** (`src/pages/dashboard/`)
   - Order queue visualization
   - Live camera feeds
   - System status overview
   - Quick actions

2. **Orders** (`src/pages/orders/`)
   - Order creation interface
   - Order history
   - Order details view

3. **Inventory** (`src/pages/inventory/`)
   - Stock level visualization
   - Refill controls
   - Category summaries
   - Low stock alerts

4. **Analytics** (`src/pages/analytics/`)
   - Order statistics
   - Performance metrics
   - Trend visualizations

5. **Settings** (`src/pages/settings/`)
   - System configuration
   - User preferences

### API Integration

#### Order API (`src/api/orders.js`)

```javascript
import api from './base';

export const createOrder = async (order) => {
  const response = await api.post('/api/orders', order);
  return response.data;
};

export const startOrder = async (orderId) => {
  const response = await api.patch(`/api/orders/${orderId}/start`);
  return response.data;
};

export const getQueue = async () => {
  const response = await api.get('/api/queue');
  return response.data;
};
```

#### Inventory API (`src/api/inventory.js`)

```javascript
export const getInventoryStatus = async () => {
  const response = await api.get('/api/inventory/status');
  return response.data;
};

export const refillInventory = async (ingredientType, subtype) => {
  const response = await api.post(
    `/api/inventory/refill?ingredient_type=${ingredientType}&subtype=${subtype}`
  );
  return response.data;
};
```

### Real-Time Updates

#### WebSocket Connection

```javascript
// src/utils/websocket.js
const ws = new WebSocket('ws://localhost:8000/ws');

ws.onopen = () => {
  console.log('Connected to BARNS');
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  
  switch(data.type) {
    case 'order_update':
      dispatch(updateOrder(data.data));
      break;
    case 'inventory_update':
      dispatch(updateInventory(data.data));
      break;
  }
};
```

#### Socket.IO (Alternative)

```javascript
import io from 'socket.io-client';

const socket = io('http://localhost:8000');

socket.on('inventory.update', (data) => {
  console.log('Inventory updated:', data);
});
```

### State Management (Redux)

```javascript
// src/store/ordersSlice.js
import { createSlice } from '@reduxjs/toolkit';

const ordersSlice = createSlice({
  name: 'orders',
  initialState: {
    list: [],
    queue: [],
    loading: false
  },
  reducers: {
    setOrders: (state, action) => {
      state.list = action.payload;
    },
    updateOrderStatus: (state, action) => {
      const order = state.list.find(o => o.id === action.payload.id);
      if (order) {
        order.status = action.payload.status;
      }
    }
  }
});
```

## Usage Examples

### Create Order

```javascript
import { useDispatch } from 'react-redux';
import { createOrder } from '../api/orders';

function CreateOrderButton() {
  const dispatch = useDispatch();
  
  const handleCreate = async () => {
    const order = {
      cups: [
        { recipe: 'latte', size: 'medium' }
      ]
    };
    
    const result = await createOrder(order);
    dispatch(addOrder(result));
  };
  
  return <button onClick={handleCreate}>Create Order</button>;
}
```

### Monitor Inventory

```javascript
import { useEffect, useState } from 'react';
import { getInventoryStatus } from '../api/inventory';

function InventoryDashboard() {
  const [inventory, setInventory] = useState({});
  
  useEffect(() => {
    const fetchInventory = async () => {
      const data = await getInventoryStatus();
      setInventory(data.inventory);
    };
    
    fetchInventory();
    const interval = setInterval(fetchInventory, 10000);
    return () => clearInterval(interval);
  }, []);
  
  return (
    <div>
      {Object.entries(inventory).map(([category, items]) => (
        <CategoryCard key={category} category={category} items={items} />
      ))}
    </div>
  );
}
```

### Video Streaming

```javascript
function CameraFeed({ cameraId }) {
  return (
    <img 
      src={`${VIDEO_STREAM_URL}/stream/${cameraId}`}
      alt={`Camera ${cameraId}`}
      style={{ width: '100%' }}
    />
  );
}
```

## Dependencies

### Core Dependencies

- **React** (18.2.0): UI framework
- **Vite** (5.0.0): Build tool and dev server
- **Redux Toolkit** (@reduxjs/toolkit): State management
- **React Router** (react-router-dom): Routing
- **Axios**: HTTP client
- **Socket.IO Client** (socket.io-client): Real-time updates
- **Tailwind CSS** (3.4.0): Styling framework

### UI Components

- **Recharts**: Data visualization
- **React DnD**: Drag-and-drop for queue management
- **React Icons**: Icon library

## Integration Points

### Upstream Services

1. **API Bridge Service**
   - All HTTP API calls
   - WebSocket/Socket.IO connections
   - **Port**: 8000

2. **Video Stream Service**
   - Camera feeds
   - **Port**: 8001

## Development

### Project Structure

```
src/
├── api/           # API client modules
├── assets/        # Images, icons
├── components/    # Reusable components
├── pages/         # Page components
├── store/         # Redux store and slices
├── utils/         # Utility functions
├── theme.js       # Theme configuration
└── main.jsx       # Entry point
```

### Scripts

```bash
npm run dev        # Start dev server
npm run build      # Production build
npm run preview    # Preview production build
npm run lint       # Run ESLint
```

## Troubleshooting

### API Connection Failed

Check API Bridge:
```bash
curl http://localhost:8000/api/health
```

### WebSocket Not Connecting

1. Verify API Bridge WebSocket endpoint
2. Check browser console for errors
3. Test WebSocket manually:
   ```javascript
   const ws = new WebSocket('ws://localhost:8000/ws');
   ws.onopen = () => console.log('Connected');
   ```

### Video Feeds Not Loading

1. Check video-stream service:
   ```bash
   docker-compose ps video-stream-service
   ```

2. Test stream URL directly:
   ```
   http://localhost:8001/stream/webcam
   ```

## Performance Considerations

- **Bundle Size**: ~500KB gzipped
- **Initial Load**: <2s on local network
- **WebSocket Overhead**: ~10KB/s for real-time updates
- **Memory Usage**: ~50MB in browser

## Security Notes

- No authentication implemented (internal network)
- CORS configured for localhost development
- Production should add authentication layer
- WebSocket connections not encrypted (use WSS in production)

## Future Enhancements

- User authentication and roles
- Mobile app version
- Offline mode support
- Advanced analytics dashboards
- Custom alert rules
- Recipe builder interface
- Multi-language support
