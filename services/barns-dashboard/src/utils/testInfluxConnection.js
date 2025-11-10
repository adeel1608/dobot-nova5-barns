/**
 * InfluxDB Connection Test Utility
 * Run this in browser console to test InfluxDB connection
 */

export const testInfluxConnection = async () => {
  console.log('🔍 Testing InfluxDB Connection...');
  
  const testConfigs = [
    { name: 'Via Proxy', url: '/api/v2' },
    { name: 'Direct localhost', url: 'http://localhost:8086/api/v2' },
  ];

  for (const config of testConfigs) {
    console.log(`\n📡 Testing: ${config.name}`);
    console.log(`   URL: ${config.url}`);
    
    try {
      const queryUrl = `${config.url}/query?org=barns`;
      const testQuery = 'from(bucket: "logs") |> range(start: -1h) |> limit(n: 1)';
      
      const response = await fetch(queryUrl, {
        method: 'POST',
        headers: {
          'Authorization': 'Token barns-super-secret-token',
          'Content-Type': 'application/vnd.flux',
          'Accept': 'application/csv'
        },
        body: testQuery
      });

      console.log(`   Status: ${response.status} ${response.statusText}`);
      
      if (response.ok) {
        const data = await response.text();
        console.log(`   ✅ SUCCESS - Received ${data.length} bytes`);
        console.log(`   Sample response:`, data.substring(0, 200));
        return { success: true, config: config.name };
      } else {
        console.log(`   ❌ FAILED - ${response.status}`);
      }
    } catch (error) {
      console.log(`   ❌ ERROR:`, error.message);
    }
  }

  console.log('\n💡 Troubleshooting Tips:');
  console.log('   1. Check if InfluxDB is running: docker ps | grep influxdb');
  console.log('   2. Verify InfluxDB is accessible: curl http://localhost:8086/ping');
  console.log('   3. Check if dev server proxy is configured in vite.config.js');
  console.log('   4. Open DevTools Network tab to see the actual request');
  
  return { success: false };
};

// Auto-run if in development
if (import.meta.env.DEV) {
  window.testInfluxConnection = testInfluxConnection;
  console.log('💡 InfluxDB test utility loaded. Run: testInfluxConnection()');
}

