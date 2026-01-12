/**
 * InfluxDB Client Utility
 * Handles connections and queries to InfluxDB for monitoring data
 */

const INFLUX_CONFIG = {
  // Priority:
  // 1. Runtime env var (window.env) via entrypoint.sh
  // 2. Build-time env var (import.meta.env)
  // 3. Fallbacks
  url: (window.env && window.env.VITE_INFLUX_URL) || import.meta.env.VITE_INFLUX_URL || (import.meta.env && import.meta.env.DEV ? '/api/v2' : (window.location.hostname === 'localhost' ? '/api/v2' : 'http://influxdb:8086/api/v2')),
  token: (window.env && window.env.VITE_INFLUX_TOKEN) || import.meta.env.VITE_INFLUX_TOKEN || 'barns-super-secret-token',
  org: (window.env && window.env.VITE_INFLUX_ORG) || import.meta.env.VITE_INFLUX_ORG || 'barns',
  bucket: (window.env && window.env.VITE_INFLUX_BUCKET) || import.meta.env.VITE_INFLUX_BUCKET || 'logs'
};

/**
 * Parse CSV response from InfluxDB
 * InfluxDB CSV format: First column is often empty, starts with comma
 */
const parseCSV = (csv) => {
  if (!csv || csv.trim().length === 0) {
    console.warn('⚠️ Empty CSV response from InfluxDB');
    return [];
  }

  const lines = csv.trim().split('\n');
  const results = [];
  let headers = null;

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];

    // Skip empty lines and annotation lines (start with #)
    if (!line || line.startsWith('#')) {
      continue;
    }

    // Parse CSV line - don't trim individual values yet
    const values = line.split(',');

    // First non-comment line is the header
    if (!headers) {
      headers = values.map(v => v.trim());
      continue;
    }

    // Skip lines that don't match header length
    if (values.length !== headers.length) {
      continue;
    }

    // Parse data row
    const record = {};
    let hasData = false;

    headers.forEach((header, idx) => {
      let value = values[idx].trim();

      // Remove quotes if present
      if (value.startsWith('"') && value.endsWith('"')) {
        value = value.slice(1, -1);
      }

      // Skip empty header names (InfluxDB often has empty first column)
      if (header && header !== '') {
        record[header] = value;
        if (value) hasData = true;
      }
    });

    // Only add records that have some data
    if (hasData) {
      results.push(record);
    }
  }

  return results;
};

/**
 * Execute a Flux query and return results using fetch API
 */
export const queryInflux = async (fluxQuery) => {
  try {
    const queryUrl = `${INFLUX_CONFIG.url}/query?org=${INFLUX_CONFIG.org}`;

    // Debug logging (set to false in production)
    const DEBUG = true;

    if (DEBUG) {
      console.log('📡 InfluxDB Query URL:', queryUrl);
      console.log('📝 Query:', fluxQuery.substring(0, 200) + '...');
    }

    const response = await fetch(queryUrl, {
      method: 'POST',
      headers: {
        'Authorization': `Token ${INFLUX_CONFIG.token}`,
        'Content-Type': 'application/vnd.flux',
        'Accept': 'application/csv'
      },
      body: fluxQuery
    });

    if (DEBUG) {
      console.log('📊 Response status:', response.status, response.statusText);
    }

    if (!response.ok) {
      const errorText = await response.text();
      console.error('❌ InfluxDB error response:', errorText);
      throw new Error(`InfluxDB query failed: ${response.status} ${response.statusText}`);
    }

    const csv = await response.text();

    if (DEBUG) {
      console.log('📄 Raw CSV response:', csv.substring(0, 500));
      console.log('📄 CSV length:', csv.length, 'bytes');
    }

    const results = parseCSV(csv);

    if (DEBUG && results.length > 0) {
      console.log('✅ Parsed results:', results.length, 'records');
      console.log('📋 Sample record:', results[0]);
    }

    return results;
  } catch (error) {
    console.error('❌ Failed to query InfluxDB:', error);
    throw error;
  }
};

/**
 * Get recent logs with filters
 */
export const getRecentLogs = async (timeRange = '-30m', service = '.*', level = '.*', limit = 500) => {
  const query = `
    from(bucket: "${INFLUX_CONFIG.bucket}")
      |> range(start: ${timeRange})
      |> filter(fn: (r) => r["_measurement"] == "barns_logs")
      |> filter(fn: (r) => r["_field"] == "msg")
      |> filter(fn: (r) => r.service =~ /${service}/)
      |> filter(fn: (r) => r.level =~ /${level}/)
      |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")
      |> sort(columns: ["_time"], desc: true)
      |> limit(n: ${limit})
  `;
  return await queryInflux(query);
};

/**
 * Get log volume over time grouped by level
 */
export const getLogVolumeByLevel = async (timeRange = '-30m', service = '.*', level = '.*', window = '30s') => {
  const query = `
    from(bucket: "${INFLUX_CONFIG.bucket}")
      |> range(start: ${timeRange})
      |> filter(fn: (r) => r["_measurement"] == "barns_logs")
      |> filter(fn: (r) => r["_field"] == "msg")
      |> filter(fn: (r) => r.service =~ /${service}/)
      |> filter(fn: (r) => r.level =~ /${level}/)
      |> group(columns: ["level"])
      |> aggregateWindow(every: ${window}, fn: count, createEmpty: false)
      |> yield(name: "count")
  `;
  return await queryInflux(query);
};

/**
 * Get error distribution by service
 */
export const getErrorDistributionByService = async (timeRange = '-30m', service = '.*') => {
  const query = `
    from(bucket: "${INFLUX_CONFIG.bucket}")
      |> range(start: ${timeRange})
      |> filter(fn: (r) => r["_measurement"] == "barns_logs")
      |> filter(fn: (r) => r["_field"] == "msg")
      |> filter(fn: (r) => r["level"] == "ERROR")
      |> filter(fn: (r) => r.service =~ /${service}/)
      |> group(columns: ["service"])
      |> count()
      |> group()
      |> sort(columns: ["_value"], desc: true)
  `;
  return await queryInflux(query);
};

/**
 * Get top errors - Fixed query to match Grafana
 */
export const getTopErrors = async (timeRange = '-30m', service = '.*', limit = 20) => {
  const query = `
    from(bucket: "${INFLUX_CONFIG.bucket}")
      |> range(start: ${timeRange})
      |> filter(fn: (r) => r["_measurement"] == "barns_logs")
      |> filter(fn: (r) => r["_field"] == "msg")
      |> filter(fn: (r) => r["level"] == "ERROR")
      |> filter(fn: (r) => r.service =~ /${service}/)
      |> map(fn: (r) => ({ r with error_msg: r._value }))
      |> group(columns: ["service", "error_msg"])
      |> count()
      |> group()
      |> sort(columns: ["_value"], desc: true)
      |> limit(n: ${limit})
  `;
  return await queryInflux(query);
};

/**
 * Get error rate over time by service
 */
export const getErrorRateByService = async (timeRange = '-30m', service = '.*', window = '30s') => {
  const query = `
    from(bucket: "${INFLUX_CONFIG.bucket}")
      |> range(start: ${timeRange})
      |> filter(fn: (r) => r["_measurement"] == "barns_logs")
      |> filter(fn: (r) => r["_field"] == "msg")
      |> filter(fn: (r) => r["level"] == "ERROR")
      |> filter(fn: (r) => r.service =~ /${service}/)
      |> group(columns: ["service"])
      |> aggregateWindow(every: ${window}, fn: count, createEmpty: false)
  `;
  return await queryInflux(query);
};

/**
 * Get available services from logs
 */
export const getAvailableServices = async (timeRange = '-1h') => {
  const query = `
    import "influxdata/influxdb/schema"
    
    schema.tagValues(
      bucket: "${INFLUX_CONFIG.bucket}",
      tag: "service",
      predicate: (r) => r._measurement == "barns_logs",
      start: ${timeRange}
    )
  `;
  return await queryInflux(query);
};

/**
 * Get available log levels
 */
export const getAvailableLogLevels = async (timeRange = '-1h') => {
  const query = `
    import "influxdata/influxdb/schema"
    
    schema.tagValues(
      bucket: "${INFLUX_CONFIG.bucket}",
      tag: "level",
      predicate: (r) => r._measurement == "barns_logs",
      start: ${timeRange}
    )
  `;
  return await queryInflux(query);
};

/**
 * Get log statistics
 */
export const getLogStatistics = async (timeRange = '-30m', service = '.*') => {
  const query = `
    from(bucket: "${INFLUX_CONFIG.bucket}")
      |> range(start: ${timeRange})
      |> filter(fn: (r) => r["_measurement"] == "barns_logs")
      |> filter(fn: (r) => r["_field"] == "msg")
      |> filter(fn: (r) => r.service =~ /${service}/)
      |> group(columns: ["level"])
      |> count()
  `;
  return await queryInflux(query);
};

export default {
  getRecentLogs,
  getLogVolumeByLevel,
  getErrorDistributionByService,
  getTopErrors,
  getErrorRateByService,
  getAvailableServices,
  getAvailableLogLevels,
  getLogStatistics
};

