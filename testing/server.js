"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.startServer = void 0;
// server.ts
const express_1 = __importDefault(require("express"));
const body_parser_1 = __importDefault(require("body-parser"));
const app = (0, express_1.default)();
const port = 3000;
app.use(body_parser_1.default.json());
app.post('/processData', (req, res) => {
    // Handle data received from Python
    const inputData = req.body.data;
    console.log('Received data from Python:', inputData);
    // Process the data (you can perform any operations here)
    const processedData = inputData.toUpperCase();
    // Send data back to Python
    res.json({ result: processedData });
});
// Handle graceful shutdown
app.get('/shutdown', (req, res) => {
    console.log('Received shutdown request. Stopping the server gracefully.');
    res.send('Server is shutting down...');
    process.exit(0);
});
const startServer = () => {
    app.listen(port, () => {
        console.log(`Server is listening at http://localhost:${port}`);
    });
};
exports.startServer = startServer;
