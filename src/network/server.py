from flask import Flask, render_template
from flask_socketio import SocketIO
import subprocess

app = Flask(__name__)
socketio = SocketIO(app)

event_handlers = {}
namespace = {
    'robot': '/robot',
    'client': '/client',
}

class WebsocketServer:
    def __init__(self, host='0.0.0.0', port=5000):
        self.host = host
        self.port = port
        self.robot_process = None

    @socketio.on('connect', namespace='/client')
    def handle_client_connect(self):
        socketio.emit('get_constants', namespace='/robot')

    @socketio.on('pid_constants', namespace='/robot')
    def handle_pid_constants_from_robot(self, constants):
        socketio.emit('pid_constants', constants, namespace='/client')

    @socketio.on('update_constants', namespace='/client')
    def handle_update_constants(self, payload):
        socketio.emit('update_constants', payload, namespace='/robot')

    @socketio.on('start_robot', namespace='/client')
    def handle_start_robot(self):
            if self.robot_process is None or self.robot_process.poll() is not None:
                self.robot_process = subprocess.Popen(['python', 'main.py'])
                self.socketio.emit('robot_status', {'running': True})

    @socketio.on('stop_robot', namespace='/client')
    def handle_stop_robot(self):
        if self.robot_process and self.robot_process.poll() is None:
                self.robot_process.terminate()
                self.robot_process = None
                self.socketio.emit('robot_status', {'running': False})

    @app.route('/')
    def index(self):
        return render_template('index.html')

if __name__ == '__main__':
    socketio.run(app)