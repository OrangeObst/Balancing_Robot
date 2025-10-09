from flask import Flask, render_template
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app)
robot_process = None

@app.route('/')
def index():
    return render_template('index.html')

# ===== From Robot to Client ===== 

@socketio.on('data', namespace='/robot')
def handle_data(payload):
    socketio.emit('data', payload, namespace='/client')

@socketio.on('pid_constants', namespace='/robot')
def handle_pid_constants_from_robot(payload):
    socketio.emit('pid_constants', payload, namespace='/client')

@socketio.on('pos_pid_status', namespace='/robot')
def handle_pos_pid_status(payload):
    socketio.emit('pos_pid_status', payload, namespace='/client')

@socketio.on('robot_status', namespace='/robot')
def handle_robot_status(payload):
    socketio.emit('robot_status', payload, namespace='/client')

@socketio.on('robot_data', namespace='/robot')
def handle_robot_status(payload):
    socketio.emit('robot_data', payload, namespace='/client')

# ===== From Client to Robot ===== 

@socketio.on('connect', namespace='/client')
def handle_client_connect():
    socketio.emit('new_connection', namespace='/robot')

@socketio.on('update_constants', namespace='/client')
def handle_update_constants(payload):
    socketio.emit('update_constants', payload, namespace='/robot')

@socketio.on('calibrate_mpu', namespace='/client')
def handle_calibrate_mpu(payload=3):
    socketio.emit('calibrate_mpu', payload, namespace='/robot')

@socketio.on('start_robot', namespace='/client')
def handle_start_robot():
    socketio.emit('start_robot', namespace='/robot')

@socketio.on('stop_robot', namespace='/client')
def handle_stop_robot():
    socketio.emit('stop_robot', namespace='/robot')

@socketio.on('shutdown_robot', namespace='/client')
def handle_shutdown_robot():
    socketio.emit('shutdown_robot', namespace='/robot')

@socketio.on('save_settings', namespace='/client')
def handle_save_settings():
    socketio.emit('save_settings', namespace='/robot')

@socketio.on('switch_pos_pid', namespace='/client')
def handle_switch_pos_pid():
    socketio.emit('switch_pos_pid', namespace='/robot')

@socketio.on('start_motors', namespace='/client')
def handle_start_motors():
    socketio.emit('start_motors', namespace='/robot')

@socketio.on('stop_motors', namespace='/client')
def handle_stop_motors():
    socketio.emit('stop_motors', namespace='/robot')

@socketio.on('activate_motors', namespace='/client')
def handle_activate_motors():
    socketio.emit('activate_motors', namespace='/robot')

@socketio.on('deactivate_motors', namespace='/client')
def handle_deactivate_motors():
    socketio.emit('deactivate_motors', namespace='/robot')

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0')