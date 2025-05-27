import socket
import time

# server and client adapted from https://realpython.com/python-sockets/

HOST = "172.20.10.2"  # The server's hostname or IP address
# HOST = "127.0.0.1"
PORT = 65432  # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    count = 1
    while True:
        message = f"Hello {count}"
        s.sendall(message.encode())
        time.sleep(0.5)
        count += 1
    # data = s.recv(1024)
