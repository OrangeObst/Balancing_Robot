import socket
import threading
import time
import queue

class UdpClient:
    def __init__(self, server_host, server_port):
        self.server_address = (server_host, server_port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)  # non-blocking
        self.receiving = True
        self.message_queue = queue.Queue()

    def send(self, message):
        self.socket.sendto(message.encode(), self.server_address)

    def start_sending(self):
        threading.Thread(target=self.send_messages, daemon=True).start()

    def send_messages(self):
        while True:
            try:
                message = self.message_queue.get(timeout=1)
                self.send(message)
            except queue.Empty:
                pass

    def send_message(self, message):
        self.message_queue.put(message)

    def start_receiving(self):
        threading.Thread(target=self.receive_messages, daemon=True).start()

    def receive_messages(self):
        while self.receiving:
            try:
                data, server = self.socket.recvfrom(1024)
                print(f"Received: {data.decode()} from {server}")
                # React to the message here
            except BlockingIOError:
                # No data available, continue the loop
                pass
            except Exception as e:
                print(f"Error receiving message: {e}")
                break

    def stop_receiving(self):
        self.receiving = False

    def close(self):
        self.stop_receiving()
        self.socket.close()
