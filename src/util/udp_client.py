import socket
import time
import json

class UdpClient:
    def __init__(self, server_host, server_port):
        self.server_address = (server_host, server_port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.socket.setblocking(False)  # non-blocking adds about 2 ms to avg latency
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        print(f'host: {server_host} | port: {server_port} | add: {self.server_address}')

    def send(self, message):
        packed_message = json.dumps(message).encode('utf-8')
        self.socket.sendto(packed_message, self.server_address)

    def receive_messages(self):
        while True:
            data, server = self.socket.recvfrom(512)
            if data:
                data = json.loads(data.decode('utf-8'))
                print(f"Received: {data} from {server}")
                return data
        # try:
        #     data, server = self.socket.recvfrom(512)
        #     if data:
        #         data = json.loads(data.decode('utf-8'))
        #         print(f"Received: {data} from {server}")
        #     return data
        # except socket.error as e:
        #     if e.errno == socket.EAGAIN or e.errno == socket.EWOULDBLOCK:
        #         time.sleep(0.005)
        #     else:
        #         print(f"Error receiving message: {e}")
        # except Exception as e:
        #     print(f"General error: {e}")


    def close(self):
        self.socket.close()

if __name__ == '__main__':
    pass