import socket

HOST = '0.0.0.0'
PORT = 8765

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[TCP Debug Server] Listening on {HOST}:{PORT}")
        conn, addr = s.accept()
        print(f"[TCP Debug Server] Connected by {addr}")

        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    print("[TCP Debug Server] Connection closed.")
                    break
                message = data.decode('utf-8').strip()
                print(f"[TCP Debug Server] Received: {message}")

if __name__ == '__main__':
    main()
