import socket

def test_connection():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print(f"{sock.connect(("172.20.10.3", 1000))}")
        print("Connection successful!")
        sock.close()
    except Exception as e:
        print(f"Connection failed: {e}")


if __name__ == "__main__":
    test_connection()
