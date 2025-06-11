import socket

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

client_socket.connect(('127.0.0.1', 8080))
print("서버에 연결되었습니다.")

client_socket.sendall(b'Hello, Server!')
data = client_socket.recv(1024)
print(f"서버로부터 받은 데이터: {data.decode()}")

client_socket.close()