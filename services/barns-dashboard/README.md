# BARNS Dashboard (React + Vite)

## Overview

The BARNS Dashboard is a React (Vite) single-page application for monitoring and operating the BARNS coffee system: orders, inventory, alerts, cameras, logs, and operational controls.

## Tech stack (as implemented)

- **React**: UI (`react`, `react-dom`)
- **Vite**: dev server + build (`vite`, `@vitejs/plugin-react`)
- **State**: Zustand (`zustand`) + per-feature stores under `src/store/`
- **HTTP**: Axios via a shared `APIClient` wrapper (`axios`, `src/api/base.js`)
- **Realtime**: Native WebSocket client (`src/utils/websocket.js`)
- **Charts**: Recharts (`recharts`)
- **Drag & drop**: dnd-kit (`@dnd-kit/core`, `@dnd-kit/sortable`)
- **Styling**: Tailwind + scoped CSS (`tailwindcss`, page `styles.css`)

## Runtime architecture

- **Development**
  - Vite dev server exposes the UI (default `5173`).
  - API calls go to `http://localhost:8000/api` (configured in `src/utils/config.js`).
  - WebSocket connects to `ws://localhost:8000/ws` (configured in `src/utils/config.js`).
  - InfluxDB requests to `/api/v2/*` are proxied to `http://localhost:8086` (configured in `vite.config.js`).

- **Production**
  - UI is served by Nginx from static `dist/` assets (see `nginx.conf` + `entrypoint.sh`).
  - The app expects relative API paths (`/api`, `/ws`) and relies on the reverse-proxy to route them.
  - `entrypoint.sh` generates `/env-config.js` at container start and injects it into `index.html` to provide runtime configuration via `window.env`.

## Project structure (React dashboard layout)

This codebase uses a feature-first layout: each page is a module with its own components and styles, while cross-cutting concerns live in `api/`, `store/`, `utils/`, and `constants/`.

```
services/barns-dashboard/
  src/
    api/                      # All HTTP clients for backend services
      base.js                 # axios wrapper + standardized response shape
      orders.js               # orders + queue + POS endpoints
      inventory.js            # inventory endpoints
      alerts.js               # alerts endpoints
      logs.js                 # logs endpoints
      cameras.js              # camera list + recording endpoints
      system.js               # system status + stop/resume
      recipes.js              # recipes endpoint
      index.js                # exports

    pages/                    # UI pages (tabs)
      dashboard/              # main operational view (orders + system + alerts summary)
      newOrder/               # POS-style order entry UI
      inventory/              # inventory panels + refill controls
      alerts/                 # alerts list + acknowledgement
      cameras/                # live streams + recordings controls
      settings/               # monitoring, logs, ingredient limits, translations

    store/                    # Zustand stores (per feature) + websocket integration
      dashboardStore.js
      inventoryStore.js
      alertsStore.js
      camerasStore.js
      logsStore.js
      translationsStore.js
      index.js                # combined facade + websocket wiring

    utils/
      config.js               # API/WS/video base URLs + UI timings
      websocket.js            # reconnecting WebSocket manager
      influxClient.js         # InfluxDB query helper (Flux -> CSV -> parsed)
      errorHandler.js         # shared error extraction

    constants/                # UI constants + translations
    App.jsx                   # tab navigation + layout
    main.jsx                  # React entry
```

## Configuration

### API / WebSocket / Video stream base URLs

The primary runtime configuration is centralized in `src/utils/config.js`:

- **HTTP API base**: `API_CONFIG.API_BASE`
  - Development: `http://localhost:8000/api`
  - Production: `/api` (relative; reverse-proxy must route this)
- **WebSocket base**: `API_CONFIG.WEBSOCKET_BASE`
  - Development: `ws://localhost:8000/ws`
  - Production: `/ws` (relative; reverse-proxy/ingress must route this)
- **Video stream base**: `API_CONFIG.VIDEO_STREAM`
  - Resolved from `window.env.VIDEO_STREAM_URL` or falls back to `http://localhost:30001`

### Runtime env injection (Nginx container)

`entrypoint.sh` writes `/env-config.js` as:

- `window.env.VIDEO_STREAM_URL`
- `window.env.API_BRIDGE_URL`
- `window.env.VITE_INFLUX_URL`
- `window.env.VITE_INFLUX_TOKEN`
- `window.env.VITE_INFLUX_ORG`
- `window.env.VITE_INFLUX_BUCKET`

Notes:
- The UI currently reads `VIDEO_STREAM_URL` (via `src/utils/config.js`) and InfluxDB values (via `src/utils/influxClient.js`).
- `API_BRIDGE_URL` is generated but is not currently used by `src/utils/config.js` for the main REST base URL.

## APIs: inbound and outbound

This section inventories all network interfaces the dashboard exposes (inbound) and all calls it makes (outbound), as implemented in this service.

### Inbound (into the dashboard service)

- **UI (SPA)**
  - `GET /` and static assets (served by Vite dev server or Nginx in prod)
  - `GET /env-config.js` (generated at container start in prod by `entrypoint.sh`)

- **Reverse-proxy paths (prod Nginx)**
  - `nginx.conf` currently proxies `location /api/` to `http://oms:8000/`
    - This makes the dashboard container an ingress point for `/api/*` in production deployments where Nginx is used.
  - WebSocket proxying for `/ws` is not configured in `nginx.conf` in this service (if you rely on `/ws` in prod, handle it in ingress or update Nginx).

### Outbound (from the dashboard to other services)

#### A) REST API (primary backend; base is `API_CONFIG.API_BASE`)

All of these are called via `src/api/base.js` → `apiClient` (Axios). In development the full URL resolves under `http://localhost:8000/api`.

- **Orders / Queue / POS** (`src/api/orders.js`)
  - `GET /orders` (optional `limit`, `offset`)
  - `POST /orders`
  - `PATCH /orders/{orderId}/start`
  - `POST /orders/{orderId}/stop`
  - `POST /orders/{orderId}/resume`
  - `DELETE /orders/{orderId}`
  - `PUT /queue/reorder` (body: `{ order_ids: string[] }`)
  - `POST /pos/process-order`
  - `GET /pos/menu-items`
  - `GET /pos/ingredients`
  - `GET /orders/stats/summary`

- **Inventory** (`src/api/inventory.js` + settings page)
  - `GET /inventory/category-info`
  - `GET /inventory/category-count`
  - `GET /inventory/stock-level`
  - `GET /inventory/status`
  - `GET /inventory/status/{item}`
  - `GET /inventory/category-summary`
  - `POST /inventory/refill` (body: `{ ingredient: string, amount: number }`)
  - `PUT /inventory/{item}/thresholds`
  - `POST /inventory/update-limits` (used by `src/pages/settings/components/IngredientSettings.jsx`)

- **Alerts** (`src/api/alerts.js`)
  - `GET /alerts/active`
  - `GET /alerts/acknowledged`
  - `POST /alerts/{alertId}/acknowledge`
  - `POST /alerts` (create)

- **Logs** (`src/api/logs.js`)
  - `GET /logs` (filters encoded as query string)
  - `DELETE /logs` (clear)
  - `GET /logs/export` (query: `format=csv|...`; response is a file/blob)

- **Cameras (control plane)** (`src/api/cameras.js`)
  - `GET /cameras`
  - `POST /cameras/{cameraId}/record/start`
  - `POST /cameras/{cameraId}/record/stop`
  - `GET /cameras/{cameraId}/recordings`

- **System control / status** (`src/api/system.js`)
  - `GET /system/status`
  - `POST /system/stop` (body: `{ reason?: string }`)
  - `POST /system/resume`

- **Recipes** (`src/api/recipes.js`)
  - `GET /recipes`

#### B) WebSocket (realtime events; base is `API_CONFIG.WEBSOCKET_BASE`)

- **Endpoint**
  - Development: `ws://localhost:8000/ws`
  - Production: `/ws` (relative)
  - Connection is managed by `src/utils/websocket.js` and wired in `src/store/index.js`.

- **Message types/events (handled by the UI)**
  - `type: "order_update"` with `event` values such as:
    - `scheduler.plan_built`
    - `scheduler.order_completed`
    - `scheduler.order_failed`
    - `scheduler.order_stopping` / `order_stopping`
    - `scheduler.order_stopped` / `order_stopped`
    - `order_resumed`
    - `scheduler.feedback_processed`
    - `scheduler.status_update`
  - `type: "inventory_update"`
  - Alerts/warnings: `type: "alert"` and/or `event` values including:
    - `validation_failed`
    - strings containing `threshold_warning`, `all_stations_occupied`, `retry_status`
  - Heartbeats:
    - client sends `{ type: "ping", timestamp: "..." }`
    - server may respond with `{ type: "pong" }`

#### C) Video stream service (data plane; base is `API_CONFIG.VIDEO_STREAM` and direct stream URLs)

- **Health/status** (used via `videoClient` in `src/api/system.js` and `src/api/cameras.js`)
  - `GET {VIDEO_STREAM_BASE}/status`

- **Live stream frames** (currently hard-coded in `src/pages/cameras/components/UnifiedCameraPanel.jsx`)
  - `GET http://localhost:30001/stream/{cameraId}?k={cacheBust}`

If you change the video-stream host/port, update both `window.env.VIDEO_STREAM_URL` (for API status calls) and the hard-coded stream URL usage.

#### D) InfluxDB (monitoring/analytics)

The dashboard issues Flux queries and expects CSV responses.

- **Query endpoint**
  - `POST {INFLUX_URL}/query?org={org}`
  - Default dev path is `/api/v2/query?org=barns` (Vite proxies `/api/v2` to `http://localhost:8086`).
  - Implementation: `src/utils/influxClient.js` (and debug helpers under `src/utils/`).

## Scripts

```bash
npm install
npm run dev
npm run build
npm run preview
npm run lint
```

## Troubleshooting (quick checks)

- **REST API not reachable**
  - Confirm the backend is serving at `http://localhost:8000/api` (dev default).
  - Check `/system/status` (used by the dashboard as its main health/status call).

- **WebSocket not connecting**
  - Confirm `ws://localhost:8000/ws` is reachable in dev.
  - In prod, ensure `/ws` is routed by your ingress/reverse-proxy (this service’s `nginx.conf` does not currently proxy it).

- **Camera streams not loading**
  - Confirm the stream server is reachable at `http://localhost:30001/stream/{cameraId}` (current UI default).
  - Confirm `{VIDEO_STREAM_BASE}/status` responds (used for stream service health).

## Security notes (current state)

- The dashboard assumes a trusted network; there is no authentication/authorization in this service.
- For production, terminate TLS at the edge and route REST and WebSocket endpoints over HTTPS/WSS.

