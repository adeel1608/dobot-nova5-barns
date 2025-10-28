# BARNS Grafana Setup

## Overview

Grafana is automatically configured to visualize logs from InfluxDB. The dashboard provides real-time monitoring of all BARNS services.

## Access

- **URL**: http://localhost:3006 (mapped from container port 3000)
- **Username**: `admin`
- **Password**: `barns_grafana_pass`

## Dashboard Features

The **BARNS Logs** dashboard includes:

1. **Service Filter** - Dropdown to select specific service (scheduler, oms, routine, etc.) or view All
2. **Log Level Filter** - Dropdown to filter by INFO, ERROR, DEBUG, or All
3. **Recent Logs Table** - Last 500 log entries with timestamps and filtering
4. **Log Levels Distribution** - Pie chart showing INFO/ERROR/DEBUG breakdown
5. **Logs by Service** - Bar chart of log volume per service
6. **Total Errors Gauge** - Quick view of error count
7. **Log Volume Over Time** - Time series graph showing log rates
8. **Recent Errors Table** - Last 100 errors with details and timestamps

## Usage Tips

### Filtering Logs by Service

1. Click on the **Service** dropdown at the top of the dashboard
2. Select a specific service (e.g., `scheduler`, `oms`, `routine`) or choose `All` to see everything
3. All panels will update to show only logs from the selected service
4. The dropdown auto-populates with all services currently sending logs

### Filtering by Log Level

- Use the **Log Level** dropdown to filter by INFO, ERROR, DEBUG, or All
- Combine with service filter to narrow down to specific service errors

### Viewing Timestamps

- All log tables display timestamps in the **_time** column (leftmost column)
- Timestamps are shown in your browser's local timezone
- Sort by clicking the _time column header

### Time Range Selection

- Use the **time range selector** (top right) to adjust the time window
- Quick ranges: Last 5m, 15m, 1h, 6h, 24h, 7d, 30d
- Or set a custom absolute time range
- Click on any table column header to sort
- Use table search to filter by text

### Common Queries

The dashboard uses Flux queries. To create custom panels:

**Filter by service:**
```flux
from(bucket: "logs")
  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
  |> filter(fn: (r) => r._measurement == "barns_logs")
  |> filter(fn: (r) => r.service == "scheduler")
```

**Filter by order_id:**
```flux
from(bucket: "logs")
  |> range(start: -1h)
  |> filter(fn: (r) => r._measurement == "barns_logs")
  |> filter(fn: (r) => r.order_id == "123")
```

**Count errors per service:**
```flux
from(bucket: "logs")
  |> range(start: -1h)
  |> filter(fn: (r) => r._measurement == "barns_logs")
  |> filter(fn: (r) => r.level == "ERROR")
  |> group(columns: ["service"])
  |> count()
```

## Auto-Refresh

The dashboard auto-refreshes every **5 seconds** to show real-time logs. You can adjust this using the refresh interval dropdown (top right).

## Datasource Configuration

The InfluxDB datasource is automatically provisioned with:
- **Name**: InfluxDB-BARNS
- **URL**: http://influxdb:8086
- **Organization**: barns
- **Bucket**: logs
- **Token**: barns-super-secret-token (configured via environment)

## Customization

All dashboard JSON files are located in `grafana/dashboards/`. You can:
1. Edit dashboards directly in Grafana UI
2. Export modified dashboards (JSON)
3. Save them back to `grafana/dashboards/` for persistence

## Troubleshooting

### Dashboard not showing data?

1. Check InfluxDB has data:
   ```bash
   docker exec -it barns-influxdb influx query --org barns --token barns-super-secret-token 'from(bucket: "logs") |> range(start: -1h) |> limit(n: 10)'
   ```

2. Verify Telegraf is receiving logs:
   ```bash
   docker-compose logs -f telegraf
   ```

3. Check Grafana datasource connection:
   - Go to Configuration > Data Sources > InfluxDB-BARNS
   - Click "Save & Test"

### No logs appearing?

Ensure services are using DEBUG=true to see all log levels:
```bash
docker-compose down
docker-compose up -d
```

## Directory Structure

```
grafana/
├── provisioning/
│   ├── datasources/
│   │   └── influxdb.yml       # Auto-configure InfluxDB
│   └── dashboards/
│       └── default.yml         # Auto-load dashboards
├── dashboards/
│   └── barns-logs.json         # Main logs dashboard
└── README.md                   # This file
```

