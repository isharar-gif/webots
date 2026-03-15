# game_controller.py
from controller import Supervisor, Keyboard

import chess
import chess.engine

import time
import re
import json
import os
import random
import sys
import socket
import traceback
import math

print("[debug] Python exe:", sys.executable)

# ========== SETTINGS ==========
DEMO_SPEED = 250
MAX_MOVES = 110

COMM_MODE = "WEBOTS"  # "WEBOTS" or "HIL"
HIL_HOST = "127.0.0.1"
HIL_PORT = 5005

RANDOM_OPENING_MOVES = 0
OPENING_RANDOM_PROB = 0

LOG_FILE = "game_log.txt"
DEBUG_MODE = False

WHITE_GRAVEYARD = [-0.6, -0.9, 0.04]
BLACK_GRAVEYARD = [-0.6,  0.9, 0.04]

# TCP follow offset: the TCP marker (URDF EE node in Webots) sits at hand_link,
# which is GRIPPER_TIP_OFFSET (0.06m) above the actual gripper fingertips.
# During follow, we offset the piece DOWN by this amount so it visually tracks
# the real tip, not the EE marker. Must match GRIPPER_TIP_OFFSET[2] in niryo_controller.py.
TCP_Z_ABOVE_PIECE_LEVEL = 0.01

GRAVE_COLS = 4
GRAVE_SPACING_X = 0.06
GRAVE_SPACING_Y = 0.06
GRAVE_LIFT_Z = 0.00

CALIB_FILE = "calib_board.json"


USE_STOCKFISH = True
STOCKFISH_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stockfish-linux-x86-64-avx2")


# ========== STOCKFISH ==========
engine = None
if USE_STOCKFISH:
    try:
        engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
        print("[SF] Stockfish engine initialised.")
    except Exception as e:
        print("[SF] Could not start Stockfish:", e)
        engine = None
        USE_STOCKFISH = False


# ========== VECTORS ==========
def add(a, b): return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
def sub(a, b): return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
def mul(s, v): return [s * v[0], s * v[1], s * v[2]]

def norm(v):
    n = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    return [v[0]/n, v[1]/n, v[2]/n] if n > 1e-9 else [0, 0, 1]


# ========== DEBUG HELPERS ==========
def wpos(sup: Supervisor, def_name: str):
    n = sup.getFromDef(def_name)
    if not n:
        print("[DBG] missing:", def_name)
        return None
    p = n.getPosition()
    return [float(p[0]), float(p[1]), float(p[2])]


# ========== GRID CALIBRATION ==========
class GridMapper:
    def __init__(self, sup: Supervisor):
        self.sup = sup

        ref_a1 = sup.getFromDef("REF_A1")
        ref_h1 = sup.getFromDef("REF_H1")
        ref_a8 = sup.getFromDef("REF_A8")
        ref_h8 = sup.getFromDef("REF_H8")

        rook_a1 = sup.getFromDef("ROOK_A1")
        rook_h1 = sup.getFromDef("ROOK_H1")
        rook_a8 = sup.getFromDef("ROOK_A8")
        rook_h8 = sup.getFromDef("ROOK_H8")

        if ref_a1 and ref_h1 and ref_a8 and ref_h8:
            A1 = list(ref_a1.getPosition())
            H1 = list(ref_h1.getPosition())
            A8 = list(ref_a8.getPosition())
            H8 = list(ref_h8.getPosition())
            print("[GridMapper]  Using REF corners (WORLD).")
        elif rook_a1 and rook_h1 and rook_a8 and rook_h8:
            A1 = list(rook_a1.getPosition())
            H1 = list(rook_h1.getPosition())
            A8 = list(rook_a8.getPosition())
            H8 = list(rook_h8.getPosition())
            print("[GridMapper]  Using ROOK corners (WORLD).")
        else:
            print("[GridMapper]  Corners missing! Using fallback hardcoded.")
            A1 = [-0.32, -0.44, 0.0]
            H1 = [ 0.56, -0.44, 0.0]
            A8 = [-0.32,  0.44, 0.0]
            H8 = [ 0.56,  0.44, 0.0]

        board_z = float(A1[2])
        A1[2] = board_z
        H1[2] = board_z
        A8[2] = board_z
        H8[2] = board_z

        self.A1, self.H1, self.A8, self.H8 = A1, H1, A8, H8
        self.origin = self.A1

        self.step_file = [
            (self.H1[0] - self.A1[0]) / 7.0,
            (self.H1[1] - self.A1[1]) / 7.0,
            0.0
        ]
        self.step_rank = [
            (self.A8[0] - self.A1[0]) / 7.0,
            (self.A8[1] - self.A1[1]) / 7.0,
            0.0
        ]

        self.up = [0.0, 0.0, 1.0]

        self.square_size = math.sqrt(
            (self.H1[0] - self.A1[0])**2 + (self.H1[1] - self.A1[1])**2
        ) / 7.0

        print("[GridMapper] square_size ≈", round(self.square_size, 4))
        print("[GridMapper] A1=", self.A1, "H1=", self.H1, "A8=", self.A8, "H8=", self.H8)

    def square_center_world(self, f: int, r: int):
        return [
            self.origin[0] + f * self.step_file[0] + r * self.step_rank[0],
            self.origin[1] + f * self.step_file[1] + r * self.step_rank[1],
            self.origin[2]
        ]


# ========== PARSE DEF NAMES ==========
def parse_def_square(name: str):
    m = re.search(r'_([A-H])([1-8])$', name)
    if not m:
        return None
    f = ord(m.group(1)) - ord('A')
    r = int(m.group(2)) - 1
    return f, r


PIECE_NAMES = {
    chess.PAWN:   "pawn",
    chess.KNIGHT: "knight",
    chess.BISHOP: "bishop",
    chess.ROOK:   "rook",
    chess.QUEEN:  "queen",
    chess.KING:   "king",
}

SNAP_PIECES_TO_GRID_ON_START = False
SNAP_Z_LIFT = 0.0

def snap_piece_to_square(node, f, r, mapper: GridMapper):
    center = mapper.square_center_world(f, r)
    center = [center[0], center[1], center[2] + SNAP_Z_LIFT]
    node.getField("translation").setSFVec3f(center)
    return center


# ========== LOG ==========
def log_line(text: str):
    if not LOG_FILE:
        return
    try:
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(text + "\n")
    except Exception as e:
        print("[log] ERROR while writing log:", e)

def init_log_file():
    if not LOG_FILE:
        return
    try:
        with open(LOG_FILE, "w", encoding="utf-8") as f:
            f.write("=== New game ===\n\n")
    except Exception as e:
        print("[log] Could not initialise log file:", e)

def log_move_to_file(move_index, side_str, move, eval_text, source):
    side_tag = "W" if side_str.lower().startswith("w") else "B"
    from_sq = chess.square_name(move.from_square)
    to_sq = chess.square_name(move.to_square)
    eval_short = (eval_text or "-").replace("\n", " ").strip()
    line = f"{move_index:02d} {side_tag}: {from_sq}->{to_sq} src={source} eval={eval_short}"
    log_line(line)


# ========== INDEX PIECES ==========
def index_pieces(sup: Supervisor, board: chess.Board, mapper: GridMapper):
    files = "ABCDEFGH"
    names = []
    for c in files:
        names.append(f"PAWN_{c}2")
        names.append(f"PAWN_{c}7")

    names += [
        "ROOK_A1", "KNIGHT_B1", "BISHOP_C1", "QUEEN_D1",
        "KING_E1", "BISHOP_F1", "KNIGHT_G1", "ROOK_H1",
        "ROOK_A8", "KNIGHT_B8", "BISHOP_C8", "QUEEN_D8",
        "KING_E8", "BISHOP_F8", "KNIGHT_G8", "ROOK_H8"
    ]

    nodes = {}
    occ = {}
    piece_offset = {}
    piece_rot0 = {}

    for n in names:
        node = sup.getFromDef(n)
        if not node:
            print("[warn] DEF not found:", n)
            continue

        idx = parse_def_square(n)
        if idx is None:
            print("[warn] cannot parse square from", n)
            continue

        f, r = idx
        sq = chess.square(f, r)

        center = mapper.square_center_world(f, r)

        if SNAP_PIECES_TO_GRID_ON_START:
            pos = snap_piece_to_square(node, f, r, mapper)
        else:
            pos = list(node.getField("translation").getSFVec3f())

        ALIGN_XY_TO_REF = False

        if ALIGN_XY_TO_REF:
            pos = list(node.getField("translation").getSFVec3f())
            new_pos = [center[0], center[1], pos[2]]
            node.getField("translation").setSFVec3f(new_pos)
            pos = new_pos

        rot0 = list(node.getField("rotation").getSFRotation())
        piece_rot0[n] = rot0

        offset = [pos[0] - center[0], pos[1] - center[1], pos[2] - center[2]]
        piece_offset[n] = offset

        nodes[n] = node
        occ[sq] = n

        sq_name = chess.square_name(sq)
        print(f"[init] {n} -> {sq_name} pos={pos}, center={center}, offset={offset}, rot0={rot0}")

    return nodes, occ, piece_offset, piece_rot0


# ========== SIMPLE ANIMATION ==========
def slide_piece_world(sup: Supervisor, node, target_pos, duration_ms=DEMO_SPEED):
    dt = int(sup.getBasicTimeStep())
    steps = max(1, int(duration_ms / dt))

    p0 = node.getField('translation').getSFVec3f()
    x0, y0, z0 = p0
    xt, yt, zt = target_pos

    for i in range(steps):
        t = (i + 1) / steps
        xi = x0 + (xt - x0) * t
        yi = y0 + (yt - y0) * t
        zi = z0 + (zt - z0) * t

        lift = 0.03 * (1.0 - abs(2.0 * t - 1.0))
        zi += lift

        node.getField('translation').setSFVec3f([xi, yi, zi])
        sup.step(dt)

    node.getField('translation').setSFVec3f(target_pos)


# ========== MOVE CHOOSER ==========
def choose_move(board: chess.Board, move_index: int):
    legal_moves = list(board.legal_moves)
    if not legal_moves:
        return None, "NONE"

    if USE_STOCKFISH and engine is not None:
        try:
            result = engine.play(board, chess.engine.Limit(time=0.1))
            mv = result.move
            if mv in legal_moves:
                print(f"[engine] STOCKFISH chose {mv.uci()}")
                return mv, "STOCKFISH"
        except Exception as e:
            print("[SF] Error:", e)

    mv = random.choice(legal_moves)
    print(f"[engine] RANDOM fallback chose {mv.uci()}")
    return mv, "RANDOM"


# ========== HIL ==========
hil_sock = None
def hil_connect():
    global hil_sock
    if hil_sock is not None:
        return hil_sock
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HIL_HOST, HIL_PORT))
    hil_sock = s
    print(f"[HIL] connected to {HIL_HOST}:{HIL_PORT}")
    return hil_sock

def hil_send_move(msg: dict):
    s = hil_connect()
    payload = json.dumps(msg) + "\n"
    s.sendall(payload.encode("utf-8"))


# ========== GRAVEYARD SLOTTER ==========
def make_graveyard_slotter():
    counts = {"White": 0, "Black": 0}

    def next_pos(side: str):
        base = WHITE_GRAVEYARD if side == "White" else BLACK_GRAVEYARD
        i = counts[side]
        counts[side] += 1

        col = i % GRAVE_COLS
        row = i // GRAVE_COLS

        return [
            base[0] + col * GRAVE_SPACING_X,
            base[1] + row * (GRAVE_SPACING_Y if side == "White" else -GRAVE_SPACING_Y),
            base[2] + GRAVE_LIFT_Z
        ]
    return next_pos


# ========== ACK HELPERS ==========
def setup_ack_receiver(sup, dt):
    r = sup.getDevice("ack_receiver")
    if r is None:
        print("[ACK] WARNING: ack_receiver device not found!")
        return None
    r.enable(dt)
    r.setChannel(2)
    print("[ACK] receiver enabled on channel 2")
    return r

def drain_ack(receiver, debug_print=False):
    last = None
    if receiver is None:
        return None

    while receiver.getQueueLength() > 0:
        try:
            s = receiver.getString()
            receiver.nextPacket()
            msg = json.loads(s)
            if debug_print:
                print("[ACK] RX:", msg)
            if msg.get("type") == "ack":
                last = msg
        except Exception:
            continue
    return last

def wait_for_ack_msg(sup, dt, receiver, side, stages, timeout_steps=800, debug_print=False):
    if receiver is None:
        return None

    stages = set(stages if isinstance(stages, (list, tuple, set)) else (stages,))
    for _ in range(timeout_steps):
        sup.step(dt)
        msg = drain_ack(receiver, debug_print=debug_print)
        if msg and msg.get("side") == side and msg.get("stage") in stages:
            return msg

    print(f"[ACK] TIMEOUT waiting side={side} stages={sorted(stages)}")
    return None


# ========== TCP FOLLOW HELPERS ==========
def get_translation(node):
    return node.getField("translation").getSFVec3f()

def set_translation(node, p):
    node.getField("translation").setSFVec3f([float(p[0]), float(p[1]), float(p[2])])

def set_rotation(node, r):
    node.getField("rotation").setSFRotation([float(r[0]), float(r[1]), float(r[2]), float(r[3])])

def set_locked(node, flag: bool):
    f = node.getField("locked")
    if f is not None:
        f.setSFBool(bool(flag))

def get_physics_field(node):
    if node is None:
        return None
    f = node.getField("physics")
    if f is None:
        return None
    return f.getSFNode()

def set_physics_mass(node, new_mass):
    phys = get_physics_field(node)
    if phys is None:
        return False
    mf = phys.getField("mass")
    if mf is None:
        return False
    mf.setSFFloat(float(new_mass))
    return True

def get_physics_mass(node, default=1.0):
    phys = get_physics_field(node)
    if phys is None:
        return default
    mf = phys.getField("mass")
    if mf is None:
        return default
    return float(mf.getSFFloat())


def follow_tcp_until_ack(sup, dt, piece_node, tcp_node, grasp_offset, receiver, side, stages, timeout_steps=2400, rot0=None):
    if receiver is None:
        return None

    stages = set(stages if isinstance(stages, (list, tuple, set)) else (stages,))
    for _ in range(timeout_steps):
        sup.step(dt)

        tcp_p = list(tcp_node.getPosition())
        set_translation(piece_node, [
            tcp_p[0] + grasp_offset[0],
            tcp_p[1] + grasp_offset[1],
            tcp_p[2] + grasp_offset[2]
        ])

        if rot0 is not None:
            set_rotation(piece_node, rot0)

        msg = drain_ack(receiver, debug_print=False)
        if msg and msg.get("side") == side and msg.get("stage") in stages:
            return msg

    print(f"[ACK] TIMEOUT (follow) waiting side={side} stages={sorted(stages)}")
    return None


# ========== BASE SENDER ==========
def send_robot_bases(sup, emitter):
    def pose(def_name):
        n = sup.getFromDef(def_name)
        if n is None:
            return None
        t = n.getField("translation").getSFVec3f()
        r = n.getField("rotation").getSFRotation()
        return {
            "t": [float(t[0]), float(t[1]), float(t[2])],
            "r": [float(r[0]), float(r[1]), float(r[2]), float(r[3])]
        }

    white_pose = pose("PANDA_WHITE") or pose("NIRYO_NED_WHITE") or pose("NED_WHITE")
    black_pose = pose("PANDA_BLACK") or pose("NIRYO_NED_BLACK") or pose("NED_BLACK")

    msg = {"type": "base", "white": white_pose, "black": black_pose}

    if emitter is not None:
        emitter.send(json.dumps(msg).encode("utf-8"))

    print("[base] sent:", msg)


# ========== SPECIAL MOVE OCC HANDLER ==========
def update_occ_for_special_moves(board, move, occ, nodes):
    """Sync occ map for castling and en passant. Call BEFORE chess_board.push(move)."""
    if board.is_en_passant(move):
        ep_rank = chess.square_rank(move.from_square)
        ep_file = chess.square_file(move.to_square)
        ep_sq   = chess.square(ep_file, ep_rank)
        ep_def  = occ.pop(ep_sq, None)
        if ep_def:
            print(f"[occ] en passant: removed {ep_def} from {chess.square_name(ep_sq)}")

    if board.is_castling(move):
        rank = chess.square_rank(move.from_square)
        if chess.square_file(move.to_square) == 6:  # kingside
            rook_from = chess.square(7, rank)
            rook_to   = chess.square(5, rank)
        else:                                         # queenside
            rook_from = chess.square(0, rank)
            rook_to   = chess.square(3, rank)
        rook_def = occ.pop(rook_from, None)
        if rook_def:
            occ[rook_to] = rook_def
            print(f"[occ] castling: {rook_def} {chess.square_name(rook_from)}->{chess.square_name(rook_to)}")


# ========== GAME LOOP ==========
def run_game_loop(sup, dt, emitter, ack_receiver, mapper, chess_board, nodes, occ, piece_offset, piece_rot0, tcp_white, tcp_black):
    grave_next = make_graveyard_slotter()
    moves = 0

    HOVER_Z_ABOVE_PIECE = 0.06

    while sup.step(dt) != -1 and moves < MAX_MOVES and not chess_board.is_game_over():
        try:
            move, source = choose_move(chess_board, moves)
            if move is None:
                print("[game] No legal moves.")
                break

            from_sq = move.from_square
            to_sq = move.to_square

            moving_piece = chess_board.piece_at(from_sq)
            if moving_piece is None:
                chess_board.push(move)
                moves += 1
                continue

            side_str = "White" if moving_piece.color == chess.WHITE else "Black"
            piece_name = PIECE_NAMES[moving_piece.piece_type]
            capture_flag = chess_board.is_capture(move)

            src_def = occ.get(from_sq)
            if src_def is None or src_def not in nodes:
                print(f"[warn] No DEF for {chess.square_name(from_sq)} -> push without anim.")
                chess_board.push(move)
                moves += 1
                continue

            node = nodes[src_def]

            ff = chess.square_file(from_sq)
            rr = chess.square_rank(from_sq)
            ft = chess.square_file(to_sq)
            rt = chess.square_rank(to_sq)

            off = piece_offset.get(src_def, [0.0, 0.0, 0.0])

            center_from = mapper.square_center_world(ff, rr)
            center_to   = mapper.square_center_world(ft, rt)

            grasp_from = [center_from[0] + off[0], center_from[1] + off[1], center_from[2] + off[2]]
            target_pos = [center_to[0]   + off[0], center_to[1]   + off[1], center_to[2]   + off[2]]

            pick_hover_world  = [grasp_from[0], grasp_from[1], grasp_from[2] + HOVER_Z_ABOVE_PIECE]
            pick_world        = [grasp_from[0], grasp_from[1], grasp_from[2]]

            place_hover_world = [target_pos[0], target_pos[1], target_pos[2] + HOVER_Z_ABOVE_PIECE]
            place_world       = [target_pos[0], target_pos[1], target_pos[2]]

            print(f"[Z-DEBUG] board_z={mapper.origin[2]:.4f}  piece_off_z={off[2]:.4f}"
                  f"  pick_world_z={pick_world[2]:.4f}  place_world_z={place_world[2]:.4f}"
                  f"  target_pos_z={target_pos[2]:.4f}")

            msg = {
                "type": "move",
                "from": chess.square_name(from_sq).upper(),
                "to": chess.square_name(to_sq).upper(),
                "capture": bool(capture_flag),
                "piece": piece_name,
                "side": side_str,
                "def": src_def,
                "source": source,
                "pick_hover_world":  pick_hover_world,
                "pick_world":        pick_world,
                "place_hover_world": place_hover_world,
                "place_world":       place_world,
            }

            if COMM_MODE == "WEBOTS":
                if emitter is not None:
                    print(f"[MOVE-PLAN] {side_str}: {piece_name} ({src_def}) {msg['from']} -> {msg['to']}")
                    emitter.send(json.dumps(msg).encode("utf-8"))
                else:
                    print("[WARN] COMM_MODE=WEBOTS but niryo_emitter missing")
            elif COMM_MODE == "HIL":
                hil_send_move(msg)

            captured_def  = occ.get(to_sq) if capture_flag else None
            captured_node = nodes[captured_def] if (captured_def and captured_def in nodes) else None
            captured_side = ("White" if side_str == "Black" else "Black") if capture_flag else None
            grave_pos     = grave_next(captured_side) if captured_node is not None else None

            if capture_flag:
                print(f"[CAPTURE] {captured_def} ({captured_side}) captured by {src_def} -> graveyard {grave_pos}")

            did_follow = False

            if tcp_white is not None and tcp_black is not None and ack_receiver is not None:
                side_tcp = tcp_white if side_str == "White" else tcp_black

                # Wait for gripper close ("grasped")
                grasp_msg = wait_for_ack_msg(
                    sup, dt, ack_receiver,
                    side=side_str,
                    stages=("grasped", "error"),
                    timeout_steps=1400,
                    debug_print=False
                )

                if grasp_msg and grasp_msg.get("stage") == "grasped":
                    rot0 = piece_rot0.get(src_def, None)

                    # Lock piece so physics doesn't drift it
                    set_locked(node, True)

                    # TCP marker sits at hand_link (URDF EE). The real gripper tip
                    # is TCP_Z_ABOVE_PIECE_LEVEL below that in world Z (when gripper
                    # points straight down). Offset the piece down accordingly.
                    grasp_offset = [0.0, 0.0, -TCP_Z_ABOVE_PIECE_LEVEL]

                    # Phase 1: Follow TCP grasped -> at_place_hover (arm in transit)
                    mid_msg = follow_tcp_until_ack(
                        sup, dt,
                        piece_node=node,
                        tcp_node=side_tcp,
                        grasp_offset=grasp_offset,
                        receiver=ack_receiver,
                        side=side_str,
                        stages=("at_place_hover", "error"),
                        timeout_steps=2500,
                        rot0=rot0
                    )

                    # Slide captured piece to graveyard NOW — before arm descends
                    if captured_node is not None and mid_msg and mid_msg.get("stage") == "at_place_hover":
                        slide_piece_world(sup, captured_node, grave_pos, 200)
                        captured_node = None  # mark handled

                    # Phase 2: Follow TCP at_place_hover -> released
                    end_msg = follow_tcp_until_ack(
                        sup, dt,
                        piece_node=node,
                        tcp_node=side_tcp,
                        grasp_offset=grasp_offset,
                        receiver=ack_receiver,
                        side=side_str,
                        stages=("released", "error"),
                        timeout_steps=700,
                        rot0=rot0
                    )

                    # Debug: log Z values at release moment
                    tcp_z_at_release = list(side_tcp.getPosition())[2]
                    piece_z_at_release = list(node.getField("translation").getSFVec3f())[2]
                    print(f"[Z-DEBUG] at 'released': TCP_world_z={tcp_z_at_release:.4f}"
                          f"  piece_z={piece_z_at_release:.4f}"
                          f"  target_pos_z={target_pos[2]:.4f}"
                          f"  snap_dz={target_pos[2] - piece_z_at_release:.4f}")

                    # Snap piece to exact target position and unlock
                    set_translation(node, target_pos)
                    set_locked(node, False)

                    if rot0 is not None:
                        set_rotation(node, rot0)

                    # Wait for arm to finish parking
                    done_msg = wait_for_ack_msg(
                        sup, dt, ack_receiver,
                        side=side_str,
                        stages=("done", "error"),
                        timeout_steps=1200,
                        debug_print=False
                    )

                    did_follow = bool(done_msg and done_msg.get("stage") == "done")

            # Fallback animation if follow didn't happen
            if not did_follow:
                slide_piece_world(sup, node, target_pos, duration_ms=DEMO_SPEED)
            else:
                slide_piece_world(sup, node, target_pos, duration_ms=120)

            # Restore rotation after slide animation
            rot0 = piece_rot0.get(src_def, None)
            if rot0 is not None:
                set_rotation(node, rot0)

            # Fallback: slide captured piece if it was not handled during at_place_hover
            if captured_node is not None:
                slide_piece_world(sup, captured_node, grave_pos, DEMO_SPEED // 2)

            # Sync occ map for castling / en passant BEFORE push
            update_occ_for_special_moves(chess_board, move, occ, nodes)

            chess_board.push(move)

            occ.pop(from_sq, None)
            occ.pop(to_sq, None)
            occ[to_sq] = src_def

            # Reset XY offset only; preserve Z (piece height above board)
            old_z = piece_offset.get(src_def, [0.0, 0.0, 0.0])[2]
            piece_offset[src_def] = [0.0, 0.0, old_z]

            moves += 1
            log_move_to_file(moves, side_str, move, "", source)

        except Exception as e:
            print("\n[ERROR] Exception inside game loop:", e)
            traceback.print_exc()
            break

    print(f"[end] Game over after {moves} moves. Result: {chess_board.result()}")


# ========== MAIN ==========
def main():
    sup = Supervisor()
    dt = int(sup.getBasicTimeStep())

    init_log_file()

    emitter = sup.getDevice("niryo_emitter")
    if emitter is None:
        print("[WARN] niryo_emitter device not found!")
    else:
        emitter.setChannel(1)
        print("[emit] niryo_emitter initialised on channel 1")

    # let world settle a tiny bit
    for _ in range(5):
        if sup.step(dt) == -1:
            break

    # DEBUG: print corners + one piece
    print("[DBG] REF_A1", wpos(sup, "REF_A1"))
    print("[DBG] REF_H1", wpos(sup, "REF_H1"))
    print("[DBG] REF_A8", wpos(sup, "REF_A8"))
    print("[DBG] REF_H8", wpos(sup, "REF_H8"))
    print("[DBG] ROOK_A1", wpos(sup, "ROOK_A1"))
    print("[DBG] ROOK_H8", wpos(sup, "ROOK_H8"))
    print("[DBG] PAWN_D2", wpos(sup, "PAWN_D2"))

    send_robot_bases(sup, emitter)
    # Step a few times to let robot controllers enable their receivers,
    # then re-send so neither robot misses the base message.
    for _ in range(10):
        if sup.step(dt) == -1:
            break
    send_robot_bases(sup, emitter)
    ack_receiver = setup_ack_receiver(sup, dt)

    mapper = GridMapper(sup)
    chess_board = chess.Board()

    nodes, occ, piece_offset, piece_rot0 = index_pieces(sup, chess_board, mapper)

    tcp_white = sup.getFromDef("TCP_WHITE")
    tcp_black = sup.getFromDef("TCP_BLACK")
    if tcp_white is None or tcp_black is None:
        print("[TCP] WARNING: TCP_WHITE or TCP_BLACK not found. Fake grasp follow disabled.")

    print("[game] starting loop...")
    run_game_loop(
        sup, dt,
        emitter=emitter,
        ack_receiver=ack_receiver,
        mapper=mapper,
        chess_board=chess_board,
        nodes=nodes,
        occ=occ,
        piece_offset=piece_offset,
        piece_rot0=piece_rot0,
        tcp_white=tcp_white,
        tcp_black=tcp_black
    )

    if engine is not None:
        try:
            engine.quit()
        except Exception:
            pass


if __name__ == "__main__":
    main()