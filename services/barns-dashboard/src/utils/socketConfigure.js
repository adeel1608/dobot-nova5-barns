// utils/socketConfigure.js
import { io } from 'socket.io-client';

const SOCKET_URL = 'ws://192.168.100.68:8000'; // or your server URL

// Export a shared socket instance
const socket = io(SOCKET_URL, {
  transports: ['websocket'],  // force WebSocket only
  reconnection: true,
  reconnectionAttempts: 5,
  reconnectionDelay: 1000,
});

export default socket;
