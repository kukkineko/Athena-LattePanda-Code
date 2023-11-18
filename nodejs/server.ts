// server.ts
import express, { Request, Response } from 'express';
import bodyParser from 'body-parser';
// import rclnodejs
import rclnodejs from 'rclnodejs';

const app = express();
const port = 3000;

app.use(bodyParser.json());

app.post('/processData', (req: Request, res: Response) => {
  // Handle data received from Python
  const inputData: string = req.body.data;
  console.log('Received data from Python:', inputData);

  // Process the data (you can perform any operations here)
  const processedData: string = inputData.toUpperCase();

  // Send data back to Python
  res.json({ result: processedData });
});

// Handle graceful shutdown
app.get('/shutdown', (req: Request, res: Response) => {
  console.log('Received shutdown request. Stopping the server gracefully.');
  res.send('Server is shutting down...');
  process.exit(0);
});

export const startServer = () => {
  app.listen(port, () => {
    console.log(`Server is listening at http://localhost:${port}`);
  });
};
