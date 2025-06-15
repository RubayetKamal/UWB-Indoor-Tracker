import socket
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
from queue import Queue

# Set up the server
server_ip = '0.0.0.0'
server_port = 8080
buffer_size = 1024

# Set up plotting
plt.style.use('fivethirtyeight')
fig, ax = plt.subplots()

# Queues for thread-safe communication
data_queue = Queue()
kalman_queue = Queue()

# Kalman Filter Initialization
x_hat = np.array([[0], [0], [0], [0]])  # Initial position and velocity estimate
P = np.eye(4)  # Initial covariance matrix
Q = np.array([[0.001, 0, 0, 0],
              [0, 0.001, 0, 0],
              [0, 0, 0.001, 0],
              [0, 0, 0, 0.001]])  # Process noise covariance
R = np.array([[0.1, 0],
              [0, 0.1]])  # Measurement noise covariance
delta_t = 1  # Time step

F_k = np.array([[1, 0, delta_t, 0],
                [0, 1, 0, delta_t],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])  # State transition matrix
H_k = np.array([[1, 0, 0, 0],
                [0, 1, 0, 0]])  # Measurement matrix

lock = threading.Lock()  # For thread-safe updates to Kalman filter variables

# Extract x and y from the incoming data
def extract_coordinates(data):
    try:
        pattern = r'Coordinates: x = ([\d.-]+), y = ([\d.-]+)'
        match = re.search(pattern, data)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            return x, y
    except Exception as e:
        print(f"Error in extract_coordinates: {e}")
    return None, None

# Kalman Filter update function
def kalman_update(z_k):
    global x_hat, P
    with lock:
        try:
            x_hat_pred = F_k @ x_hat
            P_pred = F_k @ P @ F_k.T + Q

            K = P_pred @ H_k.T @ np.linalg.inv(H_k @ P_pred @ H_k.T + R)
            x_hat = x_hat_pred + K @ (z_k - H_k @ x_hat_pred)
            P = (np.eye(4) - K @ H_k) @ P_pred

            return x_hat.flatten()
        except Exception as e:
            print(f"Error in Kalman update: {e}")
            return [0, 0, 0, 0]

# Thread to handle socket communication
def socket_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((server_ip, server_port))
    server_socket.listen(1)
    print(f"Server listening on {server_ip}:{server_port}")

    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    try:
        while True:
            data = conn.recv(buffer_size)
            if not data:
                break

            decoded_data = data.decode().strip()
            print(f"Received data: {decoded_data}")
            x, y = extract_coordinates(decoded_data)

            if x is not None and y is not None:
                if 0 <= x <= 4 and 0 <= y <= 4:
                    print(f"Raw: {x}, {y}")
                    data_queue.put((x, y))
                    z_k = np.array([[x], [y]])
                    kalman_pos = kalman_update(z_k)
                    kalman_queue.put((kalman_pos[0], kalman_pos[1]))
                    print(f"Kalman Filter: {kalman_pos[0]}, {kalman_pos[1]}")
                else:
                    print(f"Ignored: {x}, {y} (out of range)")

    except KeyboardInterrupt:
        print("Server closed")

    finally:
        conn.close()
        server_socket.close()

# Persistent storage for historical points
raw_history = {'x': [], 'y': []}
kalman_history = {'x': [], 'y': []}

# Update plot with new data
def update_plot(frame):
    while not data_queue.empty():
        x, y = data_queue.get()
        raw_history['x'].append(x)
        raw_history['y'].append(y)

    while not kalman_queue.empty():
        x, y = kalman_queue.get()
        kalman_history['x'].append(x)
        kalman_history['y'].append(y)

    ax.clear()
    if raw_history['x'] and raw_history['y']:
        ax.plot(raw_history['x'], raw_history['y'], marker='x', linestyle='-', color='b', label='Raw Data')
    if kalman_history['x'] and kalman_history['y']:
        ax.plot(kalman_history['x'], kalman_history['y'], marker='o', linestyle='-', color='r', label='Kalman Filter Output')

    ax.set_xlim(0, 4)
    ax.set_ylim(0, 4)
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_title("Real-Time Coordinate Visualization")
    if ax.get_legend_handles_labels()[1]:
        ax.legend()

# Run the socket server in a separate thread
server_thread = threading.Thread(target=socket_server)
server_thread.daemon = True
server_thread.start()

# Set up real-time plotting
ani = FuncAnimation(fig, update_plot, interval=1000, cache_frame_data=False)
plt.show()

# Wait for the server thread to finish
server_thread.join()
