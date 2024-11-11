import socket
import json
import time

def receive_data():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('10.172.200.186', 65433))  # Adjust IP if needed

    try:
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            process_data(data)
    finally:
        client_socket.close()

def process_data(data):
    try:
        data = json.loads(data.decode())
        print(f"Linear Velocity: {data['linear']} m/s, Angular Velocity: {data['angular']} rad/s")
    except json.JSONDecodeError:
        print("Error decoding JSON data")

if __name__ == "__main__":
    receive_data()

