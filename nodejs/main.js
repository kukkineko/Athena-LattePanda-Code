"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
// main.ts
const server_1 = require("./server");
// Start the server
(0, server_1.startServer)();
// Handle graceful shutdown
process.on('SIGINT', () => {
    console.log('\nReceived SIGINT. Stopping the server gracefully.');
    process.exit(0);
});
