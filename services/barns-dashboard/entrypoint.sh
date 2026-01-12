#!/bin/sh

# Generate config.js with environment variables
cat <<EOF > /usr/share/nginx/html/env-config.js
window.env = {
  VIDEO_STREAM_URL: "${VIDEO_STREAM_URL}",
  API_BRIDGE_URL: "${API_BRIDGE_URL}",
  VITE_INFLUX_URL: "${VITE_INFLUX_URL}",
  VITE_INFLUX_TOKEN: "${VITE_INFLUX_TOKEN}",
  VITE_INFLUX_ORG: "${VITE_INFLUX_ORG}",
  VITE_INFLUX_BUCKET: "${VITE_INFLUX_BUCKET}"
};
EOF

# Inject script tag into index.html head if not already present
if ! grep -q "env-config.js" /usr/share/nginx/html/index.html; then
    sed -i '/<head>/a \    <script src="/env-config.js"></script>' /usr/share/nginx/html/index.html
fi

# Start Nginx
exec nginx -g "daemon off;"
