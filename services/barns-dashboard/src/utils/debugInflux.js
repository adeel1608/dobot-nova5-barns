/**
 * Debug utility for InfluxDB connection and data retrieval
 * Open browser console and run: window.debugInflux()
 */

export const debugInflux = async () => {
  console.log('🔍 Starting InfluxDB Debug...\n');

  // Test 1: Simple ping-style query
  console.log('📌 Test 1: Simple query to check connection');
  try {
    const url = '/api/v2/query?org=barns';
    const simpleQuery = `
      from(bucket: "logs")
        |> range(start: -24h)
        |> limit(n: 5)
    `;

    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Authorization': 'Token barns-super-secret-token',
        'Content-Type': 'application/vnd.flux',
        'Accept': 'application/csv'
      },
      body: simpleQuery
    });

    console.log('Status:', response.status, response.statusText);
    const data = await response.text();
    console.log('Response length:', data.length, 'bytes');
    console.log('First 1000 chars:', data.substring(0, 1000));
    console.log('\n');
  } catch (error) {
    console.error('❌ Test 1 failed:', error);
  }

  // Test 2: Check for barns_logs measurement
  console.log('📌 Test 2: Check barns_logs measurement');
  try {
    const url = '/api/v2/query?org=barns';
    const measurementQuery = `
      from(bucket: "logs")
        |> range(start: -24h)
        |> filter(fn: (r) => r["_measurement"] == "barns_logs")
        |> limit(n: 10)
    `;

    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Authorization': 'Token barns-super-secret-token',
        'Content-Type': 'application/vnd.flux',
        'Accept': 'application/csv'
      },
      body: measurementQuery
    });

    console.log('Status:', response.status, response.statusText);
    const data = await response.text();
    console.log('Response length:', data.length, 'bytes');
    console.log('Full response:');
    console.log(data);
    console.log('\n');
  } catch (error) {
    console.error('❌ Test 2 failed:', error);
  }

  // Test 3: Get all measurements
  console.log('📌 Test 3: List all measurements in bucket');
  try {
    const url = '/api/v2/query?org=barns';
    const listMeasurementsQuery = `
      import "influxdata/influxdb/schema"
      
      schema.measurements(bucket: "logs")
    `;

    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Authorization': 'Token barns-super-secret-token',
        'Content-Type': 'application/vnd.flux',
        'Accept': 'application/csv'
      },
      body: listMeasurementsQuery
    });

    console.log('Status:', response.status, response.statusText);
    const data = await response.text();
    console.log('Available measurements:', data);
    console.log('\n');
  } catch (error) {
    console.error('❌ Test 3 failed:', error);
  }

  console.log('✅ Debug complete! Check the output above.');
  console.log('\n💡 Next steps:');
  console.log('1. If Test 1 returns data, InfluxDB connection works');
  console.log('2. If Test 2 is empty, the measurement name might be different');
  console.log('3. Check Test 3 to see what measurements actually exist');
  console.log('4. Compare with Grafana queries to see what they use');
};

// Make available globally in dev mode
if (import.meta.env.DEV) {
  window.debugInflux = debugInflux;
  console.log('💡 Debug utility loaded. Run: debugInflux()');
}

export default debugInflux;

