// main.ts
import { startServer } from './server';

// Start the server
startServer();

// Handle graceful shutdown
process.on('SIGINT', () => {
  console.log('\nReceived SIGINT. Stopping the server gracefully.');
  process.exit(0);
});
