import threading
from flask import Flask, render_template
from flask_socketio import SocketIO

class WebSocketServer:
    def __init__(self, host='0.0.0.0', port=5000):
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app)
        self.host = host
        self.port = port

        @self.app.route('/')
        def index():
            return render_template('index.html')

    def start(self):
        server_thread = threading.Thread(target=self._run_server)
        server_thread.daemon = True  # Allows the program to exit even if the thread is still running
        server_thread.start()

    def _run_server(self):
        self.socketio.run(self.app, host=self.host, port=self.port)

    def emit_data(self, data):
        self.socketio.emit('data', data)


if __name__ == '__main__':
    server = WebSocketServer()
    server.start()