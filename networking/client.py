import socket

ip = "127.0.0.1"
port = 5001


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((ip, port))
    s.sendall(b"Hello, world")
    s.sendall(b"Hello, world again")

#     data = s.recv(1024)
#
# print(f"Received {data!r}")

