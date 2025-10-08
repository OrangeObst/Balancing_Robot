import socketio

class WebsocketClient:
    def __init__(self, set_pid_constants, get_pid_constants, calibrate_mpu, start_robot, stop_robot, shutdown_robot, save_settings, switch_pos_pid, server_url='http://127.0.0.1:5000'):
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
            get_pid_constants()

        @self.sio.on('calibrate_mpu', namespace='/robot')
        def on_calibrate_mpu(duration=3):
            calibrate_mpu(duration)

        @self.sio.on('start_robot', namespace='/robot')
        def on_start_robot():
            start_robot()

        @self.sio.on('stop_robot', namespace='/robot')
        def on_stop_robot():
            stop_robot()

        @self.sio.on('shutdown_robot', namespace='/robot')
        def on_shutdown_robot():
            shutdown_robot()

        @self.sio.on('save_settings', namespace='/robot')
        def on_save_settings():
            save_settings()

        @self.sio.on('switch_pos_pid', namespace='/robot')
        def switch_pos_pid():
            switch_pos_pid()

    def connect(self):
        self.sio.connect(self.server_url, namespaces=['/robot'])

    def emit(self, event, data):
        self.sio.emit(event, data=data, namespace='/robot')