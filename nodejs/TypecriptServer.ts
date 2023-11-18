// TypeScriptServerClient.ts

import * as net from 'net';

export class TypeScriptServerClient {
    private socket: net.Socket;

    constructor(private host: string = 'localhost', private port: number = 8080) {
        this.socket = new net.Socket();
        this.socket.connect(port, host);
    }

    public send_data(topic: string, data: any): void {
        const message = { topic, data };
        const jsonMessage = JSON.stringify(message);
        this.socket.write(jsonMessage);
    }

    public close(): void {
        this.socket.end();
    }
}
