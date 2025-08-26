from flask import Flask, render_template
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app)
robot_process = None

@app.route('/')
def index():
    return render_template('index.html')

# ===== From Robot to Client ===== 

@socketio.on('pid_constants', namespace='/robot')
def handle_pid_constants_from_robot(payload):
    socketio.emit('pid_constants', payload, namespace='/client')

@socketio.on('data', namespace='/robot')
def handle_pid_constants_from_robot(payload):
    socketio.emit('data', payload, namespace='/client')


# ===== From Client to Robot ===== 

@socketio.on('connect', namespace='/client')
def handle_client_connect():
    socketio.emit('get_constants', namespace='/robot')

@socketio.on('update_constants', namespace='/client')
def handle_update_constants(payload):
    socketio.emit('update_constants', payload, namespace='/robot')

@socketio.on('start_robot', namespace='/client')
def handle_start_robot():
    socketio.emit('start_robot', namespace='/robot')

@socketio.on('stop_robot', namespace='/client')
def handle_stop_robot():
    socketio.emit('stop_robot', namespace='/robot')

@socketio.on('shutdown_robot', namespace='/client')
def handle_shutdown_robot():
    socketio.emit('shutdown_robot', namespace='/robot')

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0')