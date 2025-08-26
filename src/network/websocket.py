import socketio

class WebsocketClient:
    def __init__(self, set_pid_constants, get_pid_constants, start_robot, stop_robot, shutdown_robot, server_url='http://127.0.0.1:5000'):
        self.sio = socketio.Client()
        self.server_url = server_url

        @self.sio.on('connect')
        def connect():
            print("Robot connected to server")

        @self.sio.on('update_constants', namespace='/robot')
        def on_update_constants(constants):
            set_pid_constants(constants)

        @self.sio.on('get_constants', namespace='/robot')
        def on_get_constants():
            constants = get_pid_constants()
            self.sio.emit('pid_constants', constants, namespace='/robot')

        @self.sio.on('start_robot', namespace='/robot')
        def on_start_robot():
            start_robot()

        @self.sio.on('stop_robot', namespace='/robot')
        def on_stop_robot():
            stop_robot()

        @self.sio.on('shutdown_robot', namespace='/robot')
        def on_shutdown_robot():
            shutdown_robot()

    def connect(self):
        self.sio.connect(self.server_url, namespaces=['/robot'])

    def emit(self, event, data):
        self.sio.emit(event, data=data, namespace='/robot')