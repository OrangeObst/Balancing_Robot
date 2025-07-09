import threading
from flask import Flask, render_template
from flask_socketio import SocketIO

class WebSocketServer:
    def __init__(self, host='0.0.0.0', port=5000):
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app)
        self.host = host
        self.port = port
        self.server_thread = None
        self.running = False

        @self.app.route('/')
        def index():
            return render_template('index.html')

    def start(self):
        self.running = True
        self.server_thread = threading.Thread(target=self._run_server)
        self.server_thread.daemon = True  # Allows the program to exit even if the thread is still running
        self.server_thread.start()

    def _run_server(self):
        while self.running:
            self.socketio.run(self.app, host=self.host, port=self.port)

    def stop(self):
        self.running = False

    def emit_data(self, data):
        self.socketio.emit('data', data)

if __name__ == '__main__':
    server = WebSocketServer()
    try:
        server.start()
        while True:
            pass
    except KeyboardInterrupt:
        print("Stopping server...")
        server.stop()