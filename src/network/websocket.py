import socketio

class WebsocketClient:
    def __init__(self, 
                set_pid_constants,
                send_robot_specific_data,
                calibrate_mpu,
                start_robot,
                stop_robot, 
                shutdown_robot,
                save_settings, 
                switch_pos_pid, 
                start_motors, 
                stop_motors, 
                activate_motors, 
                deactivate_motors, 
                server_url='http://127.0.0.1:5000'):
        
        self.sio = socketio.Client()
        self.server_url = server_url

        @self.sio.on('connect')
        def connect():
            print("Robot connected to server")

        @self.sio.on('update_constants', namespace='/robot')
        def on_update_constants(constants):
            set_pid_constants(constants)

        @self.sio.on('new_connection', namespace='/robot')
        def on_new_connection():
            send_robot_specific_data()

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
        def on_switch_pos_pid():
            switch_pos_pid()

        @self.sio.on('start_motors', namespace='/robot')
        def on_start_motors():
            start_motors()

        @self.sio.on('stop_motors', namespace='/robot')
        def on_stop_motors():
            stop_motors()

        @self.sio.on('activate_motors', namespace='/robot')
        def on_activate_motors():
            activate_motors()

        @self.sio.on('deactivate_motors', namespace='/robot')
        def on_deactivate_motors():
            deactivate_motors()

    def connect(self):
        self.sio.connect(self.server_url, namespaces=['/robot'])

    def emit(self, event, data):
        self.sio.emit(event, data=data, namespace='/robot')