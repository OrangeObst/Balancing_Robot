import threading
from flask import Flask, render_template
from flask_socketio import SocketIO

class WebSocketServer:
    def __init__(self, host='0.0.0.0', port=5000, constants_callback=None, constants_provider=None):
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app)
        self.host = host
        self.port = port
        self.server_thread = None
        self.running = False
        self.constants_callback = constants_callback
        self.constants_provider = constants_provider

        @self.app.route('/')
        def index():
            return render_template('index.html')
        
        @self.socketio.on('connect')
        def handle_connect():
            if self.constants_provider:
                constants = self.constants_provider()
                self.socketio.emit('current_constants', constants)

        @self.socketio.on('update_constants')
        def handle_update_constants(data):
            if self.constants_callback:
                self.constants_callback(data)

    def start(self):
        self.running = True
        self.server_thread = threading.Thread(target=self._run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def _run_server(self):
        while self.running:
            self.socketio.run(self.app, host=self.host, port=self.port)

    def stop(self):
        self.running = False

    def emit_data(self, data):
        self.socketio.emit('data', data)

if __name__ == '__main__':
    def on_constants_received(constants):
        print("Received constants:", constants)

    server = WebSocketServer(on_constants_received=on_constants_received)
    try:
        server.start()
        while True:
            pass
    except KeyboardInterrupt:
        print("Stopping server...")
        server.stop()