from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit, join_room, leave_room

app = Flask(__name__)
socketio = SocketIO(app)
robot_connections = {}  # robot_id -> {'sid': socket_id, 'name': robot_name, ...}
client_subscriptions = {}  # client_sid -> robot_id

@app.route('/')
def index():
    return render_template('index.html')

# TODO: request SID part missing. Robot doesn't connect to server properly and commands don#t reach the robot

# ===== Robot Registration ===== 
@socketio.on('register_robot', namespace='/robot')
def register_robot(data):
    robot_id = data.get('id')  # Should be unique (e.g., MAC, UUID, or name)
    robot_name = data.get('name')
    robot_connections[robot_id] = {
        'sid': request.sid,
        'name': robot_name,
        # add more robot meta if needed
    }
    emit('robot_list', get_robot_list(), namespace='/client', broadcast=True)
    print(f"Robot registered: {robot_id} ({robot_name})")

@socketio.on('disconnect', namespace='/robot')
def robot_disconnect():
    # Remove robot from registry
    for robot_id, info in list(robot_connections.items()):
        if info['sid'] == request.sid:
            del robot_connections[robot_id]
            emit('robot_list', get_robot_list(), namespace='/client', broadcast=True)
            print(f"Robot disconnected: {robot_id}")
            break

def get_robot_list():
    return [{'id': rid, 'name': info['name']} for rid, info in robot_connections.items()]

# ===== Client Management =====

@socketio.on('get_robot_list', namespace='/client')
def handle_get_robot_list():
    emit('robot_list', get_robot_list())

@socketio.on('subscribe_robot', namespace='/client')
def subscribe_robot(data):
    robot_id = data.get('robot_id')
    prev_robot_id = client_subscriptions.get(request.sid)
    if prev_robot_id and prev_robot_id != robot_id:
        leave_room(prev_robot_id)
    client_subscriptions[request.sid] = robot_id
    join_room(robot_id)
    print(f"Client {request.sid} subscribed to robot {robot_id}")
    # emit('subscribed', {'robot_id': robot_id})

@socketio.on('disconnect', namespace='/client')
def client_disconnect():
    prev_robot_id = client_subscriptions.pop(request.sid, None)
    if prev_robot_id:
        leave_room(prev_robot_id)

# ===== From Robot to Client ===== 

@socketio.on('data', namespace='/robot')
def handle_data(payload):
    robot_id = None
    for rid, info in robot_connections.items():
        if info['sid'] == request.sid:
            robot_id = rid
            break
    if robot_id:
        socketio.emit('data', payload, room=robot_id, namespace='/client')

@socketio.on('pid_constants', namespace='/robot')
def handle_pid_constants_from_robot(payload):
    robot_id = None
    for rid, info in robot_connections.items():
        if info['sid'] == request.sid:
            robot_id = rid
            break
    if robot_id:
        socketio.emit('pid_constants', payload, room=robot_id, namespace='/client')

@socketio.on('pos_pid_status', namespace='/robot')
def handle_pos_pid_status(payload):
    robot_id = None
    for rid, info in robot_connections.items():
        if info['sid'] == request.sid:
            robot_id = rid
            break
    if robot_id:
        socketio.emit('pos_pid_status', payload, room=robot_id, namespace='/client')

@socketio.on('robot_status', namespace='/robot')
def handle_robot_status(payload):
    robot_id = None
    for rid, info in robot_connections.items():
        if info['sid'] == request.sid:
            robot_id = rid
            break
    if robot_id:
        socketio.emit('robot_status', payload, room=robot_id, namespace='/client')

@socketio.on('robot_data', namespace='/robot')
def handle_robot_data(payload):
    robot_id = None
    for rid, info in robot_connections.items():
        if info['sid'] == request.sid:
            robot_id = rid
            break
    if robot_id:
        # Room includes all clients subscribed to this robot
        socketio.emit('robot_data', payload, room=robot_id, namespace='/client')

# ===== From Client to Robot ===== 

@socketio.on('connect', namespace='/client')
def handle_client_connect():
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('new_connection', room=robot_sid, namespace='/robot')

@socketio.on('update_constants', namespace='/client')
def handle_update_constants(payload):
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('update_constants', payload, room=robot_sid, namespace='/robot')

@socketio.on('calibrate_mpu', namespace='/client')
def handle_calibrate_mpu(payload):
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('calibrate_mpu', payload, room=robot_sid, namespace='/robot')

@socketio.on('start_robot', namespace='/client')
def handle_start_robot():
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('start_robot', room=robot_sid, namespace='/robot')

@socketio.on('stop_robot', namespace='/client')
def handle_stop_robot():
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('stop_robot', room=robot_sid, namespace='/robot')

@socketio.on('shutdown_robot', namespace='/client')
def handle_shutdown_robot():
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('shutdown_robot', room=robot_sid, namespace='/robot')

@socketio.on('save_settings', namespace='/client')
def handle_save_settings():
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('save_settings', room=robot_sid, namespace='/robot')

@socketio.on('switch_pos_pid', namespace='/client')
def handle_switch_pos_pid():
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('switch_pos_pid', room=robot_sid, namespace='/robot')

@socketio.on('start_motors', namespace='/client')
def handle_start_motors():
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('start_motors', room=robot_sid, namespace='/robot')

@socketio.on('stop_motors', namespace='/client')
def handle_stop_motors():
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('stop_motors', room=robot_sid, namespace='/robot')

@socketio.on('activate_motors', namespace='/client')
def handle_activate_motors():
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('activate_motors', room=robot_sid, namespace='/robot')

@socketio.on('deactivate_motors', namespace='/client')
def handle_deactivate_motors():
    robot_id = client_subscriptions.get(request.sid)
    if robot_id and robot_id in robot_connections:
        robot_sid = robot_connections[robot_id]['sid']
        socketio.emit('deactivate_motors', room=robot_sid, namespace='/robot')

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0')