import socket
import json
import time

HOST = "127.0.0.1"
PORT = 5005

def send_move(sock, frm, to, side="White", piece="pawn", capture=False):
    msg = {
        "type": "move",
        "from": frm,
        "to": to,
        "side": side,
        "piece": piece,
        "capture": capture,
        "source": "MOCK",
        "eval": ""
    }
    sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))

def main():
    with socket.create_connection((HOST, PORT), timeout=2.0) as sock:
        print("[MOCK] connected")
        send_move(sock, "E2", "E4", side="White", piece="pawn")
        print("[MOCK] sent E2->E4")
        ack = sock.recv(4096)
        print("[MOCK] got:", ack.decode("utf-8", errors="ignore").strip())

if __name__ == "__main__":
    main()
