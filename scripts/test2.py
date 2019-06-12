import socket

HOST = ''  # Standard loopback interface address (localhost)
PORT = 65430       # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024)
            if not data:
                continue
            else:
                print(data)
                if data == b'1':
                    print("kitchen!")
                    conn.sendall(data)
                    

