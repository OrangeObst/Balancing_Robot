import socketio

class WebsocketClient:
    def __init__(self, set_pid_constants, get_pid_constants, server_url='http://localhost:5000'):
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
            get_pid_constants(constants)


    def connect(self):
        self.sio.connect(self.server_url, namespaces=['/robot'])