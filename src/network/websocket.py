import socketio

class WebsocketClient:
    def __init__(self, set_pid_constants, get_pid_constants, start_robot, stop_robot, server_url='http://127.0.0.1:5000'):
        self.sio = socketio.Client()
        self.server_url = server_url

        @self.sio.event
        def connect():
            print("Robot connected to server")

        @self.sio.on('update_constants')
        def on_update_constants(constants):
            set_pid_constants(constants)

        @self.sio.on('get_constants')
        def on_get_constants(constants):
            constants = get_pid_constants(constants)
            print(constants)
            self.sio.emit('pid_constants', constants)

        @self.sio.on('start_robot')
        def on_start_robot():
            start_robot()

        @self.sio.on('stop_robot')
        def on_stop_robot():
            stop_robot()

    def connect(self):
        self.sio.connect(self.server_url, namespaces=['/robot'])

    def emit(self, event, data):
        self.sio.emit(event, data=data, namespace='/robot')