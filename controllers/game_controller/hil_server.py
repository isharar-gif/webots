import socket
import json
import threading
import time

HOST = "127.0.0.1"
PORT = 5005

def handle_client(conn, addr):
    print("[HIL] client connected:", addr)
    buf = b""
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buf += data

            # протокол: newline-delimited JSON
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue

                try:
                    msg = json.loads(line.decode("utf-8"))
                except Exception as e:
                    print("[HIL] bad json:", e, "line:", line[:200])
                    continue

                # очакваме move пакет като от emitter-а
                if msg.get("type") == "move":
                    print("[HIL] MOVE:", msg.get("side"), msg.get("from"), "->", msg.get("to"),
                          "piece=", msg.get("piece"), "capture=", msg.get("capture"))

                    # тук по-късно ще извикаш реален robot SDK/ROS command
                    # за демо връщаме ACK
                    ack = {"type": "ack", "status": "ok", "ts": time.time()}
                    conn.sendall((json.dumps(ack) + "\n").encode("utf-8"))
                else:
                    print("[HIL] unknown msg type:", msg.get("type"))
    finally:
        conn.close()
        print("[HIL] client disconnected:", addr)

def main():
    print(f"[HIL] starting server on {HOST}:{PORT}")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(5)

    while True:
        conn, addr = s.accept()
        t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
        t.start()

if __name__ == "__main__":
    main()
