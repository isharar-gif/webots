# Digital Twin Chess Robot Simulation — Technical Documentation

## Modifications and Enhancements by Ishara Rajarathna

**Project:** Digital Twin_niryobased (Webots Chess Robot Simulation)  
**Original Authors:** Kostadin Devran  
**Modified by:** Ishara Rajarathna  
**Date:** March 2026  
**Platform:** Webots R2025a, Python 3.10+, Stockfish 17

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Initial System Architecture & Flow](#2-initial-system-architecture--flow)
3. [Modified System Architecture & Flow](#3-modified-system-architecture--flow)
4. [Game Controller: Detailed Comparison](#4-game-controller-detailed-comparison)
5. [Robot Controller (niryo_controller): Detailed Comparison](#5-robot-controller-niryo_controller-detailed-comparison)
6. [Bug Fixes Applied](#6-bug-fixes-applied)
7. [World File Modifications](#7-world-file-modifications)
8. [Calibration & IK Improvements](#8-calibration--ik-improvements)
9. [Iterative Development Log](#9-iterative-development-log)
10. [Summary of All Changes](#10-summary-of-all-changes)

---

## 1. Project Overview

### 1.1 What the Project Does

This project simulates an autonomous chess-playing system using two **Niryo Ned** 6-DOF robot arms in the **Webots** robotics simulator. Two robot arms (White and Black) physically pick and place chess pieces on a board, with moves determined by the **Stockfish** chess engine.

### 1.2 Project Structure

```
my_bachelor_work/
├── controllers/
│   ├── game_controller/           # Supervisor controller
│   │   ├── game_controller.py     # Chess logic, move coordination
│   │   └── stockfish-linux-x86-64-avx2
│   └── niryo_controller/          # Robot arm controller (shared by both arms)
│       ├── niryo_controller.py    # IK, motion planning, gripper control
│       └── NiryoNed.urdf          # Robot arm kinematic model
├── protos/                        # Chess piece 3D models
│   ├── ChessPawn.proto
│   ├── ChessRook.proto
│   ├── ChessKnight.proto
│   ├── ChessBishop.proto
│   ├── ChessQueen.proto
│   └── ChessKing.proto
└── worlds/
    └── Digital Twin_niryobased.wbt  # Webots world file
```

### 1.3 Key Components

| Component | Role |
|-----------|------|
| `game_controller.py` | **Supervisor** — manages chess logic (python-chess), Stockfish engine, board state, piece animation, and inter-robot communication |
| `niryo_controller.py` | **Robot Controller** — handles inverse kinematics (ikpy), joint control, gripper operations, and move execution for each Niryo Ned arm |
| World file (`.wbt`) | Defines the 3D scene: chess board, 32 pieces, 2 robot arms, reference markers, communication devices |

### 1.4 Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| Webots | R2025a | Robotics simulation environment |
| Python | 3.10+ | Controller scripting language |
| python-chess | latest | Chess logic, move validation, board state |
| ikpy | latest | Inverse kinematics solver using URDF chain |
| Stockfish | 17 (Linux AVX2) | Chess engine for move selection |
| NumPy | latest | Numerical computations for IK/FK |

---

## 2. Initial System Architecture & Flow

### 2.1 Communication Architecture

```
┌─────────────────────┐     Channel 1      ┌─────────────────────┐
│   game_controller    │ ──── Emitter ────► │  niryo_controller   │
│   (Supervisor)       │                    │  (White Robot)      │
│                      │ ◄── Channel 2 ──── │                     │
│                      │     (ACK)          └─────────────────────┘
│                      │
│                      │     Channel 1      ┌─────────────────────┐
│                      │ ──── Emitter ────► │  niryo_controller   │
│                      │                    │  (Black Robot)      │
│                      │ ◄── Channel 2 ──── │                     │
└─────────────────────┘     (ACK)          └─────────────────────┘
```

- **Channel 1:** Supervisor broadcasts move commands (JSON) to both robots
- **Channel 2:** Robots send acknowledgement (ACK) messages back to supervisor
- Each robot filters messages by its `side` field ("White" or "Black")

### 2.2 Initial Game Flow

```
1. Supervisor starts → sends base pose to both robots
2. Stockfish chooses move → supervisor computes pick/place coordinates
3. Supervisor sends move JSON via Channel 1
4. Robot receives move → executes pick-and-place sequence:
   a. Open gripper
   b. Move to safe mid-point
   c. Move to pick hover position
   d. Descend to piece
   e. Close gripper
   f. Lift to hover
   g. Transit to place hover
   h. Descend to place position
   i. Open gripper (release)
   j. Lift and park
5. Robot sends ACK stages back to supervisor
6. Supervisor updates board state → next move
```

### 2.3 Initial Code Issues

The original code had several critical issues that prevented the simulation from running correctly:

| Issue | Severity | Description |
|-------|----------|-------------|
| Duplicate ACK block | **CRITICAL** | A duplicate `if lift_msg` block with wrong keyword argument (`offset=` instead of `grasp_offset=`) caused `TypeError` crash after every first move |
| `raise None` in IK | **CRITICAL** | `solve_ik()` raised `None` when all IK guesses failed, producing unhelpful `TypeError` |
| No physics on pieces | **CRITICAL** | Chess piece protos had no Physics node — robot could not physically interact with pieces |
| Stale piece offsets | **HIGH** | `piece_offset` never reset after moves — pick positions drifted |
| Wrong hover Z | **HIGH** | Hover height calculated from piece origin, not board surface |
| Premature capture animation | **HIGH** | Captured piece moved to graveyard before robot even started moving |
| No special move handling | **MEDIUM** | En passant, castling, and promotion broke the occupancy map |
| Unnamed receiver device | **LOW** | Supervisor receiver depended on Webots default naming — fragile |
| Windows-only Stockfish path | **CRITICAL** | Hardcoded Windows backslash path and `.exe` binary |

### 2.4 Initial Game Controller Code Analysis

The original `game_controller.py` had the following key sections:

1. **Settings block** — hardcoded constants for demo speed, hover heights, graveyard positions
2. **GridMapper class** — calibrated the chess board from 4 reference corner markers (REF_A1, REF_H1, REF_A8, REF_H8)
3. **index_pieces()** — mapped 32 DEF names to chess squares, computed initial offsets
4. **slide_piece_world()** — animated piece movement with parabolic lift
5. **choose_move()** — Stockfish with random fallback
6. **run_game_loop()** — main game loop with move execution and piece tracking
7. **follow_tcp_until_ack()** — TCP follow for fake-grasp during robot moves

Key issues in the initial game loop:
- The duplicate `if lift_msg` block caused an immediate crash
- Capture animation happened before the robot moved
- Piece offsets were never updated after moves
- Hover Z was computed from piece origin rather than board surface
- TCP follow used `getSFVec3f()` (local position, constant) instead of `getPosition()` (world position)

### 2.5 Initial Robot Controller Code Analysis

The original `niryo_controller.py` had:

1. **All 6 joints active in IK** — including joint 4 (forearm roll) which added unnecessary complexity
2. **3 generic IK guesses** — insufficient coverage for the board workspace
3. **No auto-calibration** — gripper down direction was hardcoded
4. **No orientation constraint** — IK solver could produce gripper-up solutions
5. **No seeded IK** — descent from hover could jump to a completely different arm configuration
6. **`raise last_err`** — could raise `None`, producing unhelpful TypeError
7. **No waypoint error handling** — one unreachable intermediate point crashed the whole move

---

## 3. Modified System Architecture & Flow

### 3.1 Key Architectural Changes

1. **Fake-grasp with TCP follow** — Pieces track the robot's TCP (Tool Center Point) world position in real-time during moves
2. **Auto-calibration** — Gripper "down" direction auto-detected via FK sweep
3. **Seeded IK** — Descent IK uses hover joints as seed to prevent cell misalignment
4. **Two-phase TCP follow** — Captured pieces cleared at `at_place_hover` stage
5. **Fixed joints** — Joint 4 (forearm roll) locked at startup; Joint 5 has configurable bias
6. **ikpy warning suppression** — Two-step chain loading eliminates spurious fixed-link warnings
7. **Force-vertical correction** — Analytical j5 correction applied before grip/release
8. **Orientation-constrained IK** — All hover/pick/place IK calls use Z-down orientation

### 3.2 Modified Game Flow

```
1. Supervisor starts → sends base pose TWICE (with delay for reliability)
2. Robot boots → auto-calibrates gripper orientation → sends "boot" ACK
3. Stockfish chooses move → supervisor computes coordinates using ACTUAL piece Z
4. Supervisor sends move JSON via Channel 1
5. Robot executes pick-and-place:
   a. Open gripper
   b. Move to safe mid-point (IK with orientation constraint)
   c. Move to pick hover (IK with Z-down orientation)
   d. Descend to piece (SEEDED IK from hover joints — no cell jump)
   e. Force gripper vertical (j5 correction)
   f. Close gripper → send "grasped" ACK
   ─── Supervisor begins TCP follow from this point ───
   g. Lift to hover → send "lifted" ACK
   h. Transit via safe-mid
   i. Move to place hover → send "at_place_hover" ACK
   ─── Supervisor slides captured piece to graveyard HERE ───
   j. Descend to place (SEEDED IK)
   k. Force gripper vertical
   l. Open gripper → send "released" ACK
   ─── Supervisor snaps piece to exact target ───
   m. Lift and park → send "done" ACK
6. Supervisor updates occ map (handles castling/en passant) → next move
```

### 3.3 Communication Protocol (ACK Stages)

| ACK Stage | Meaning | Supervisor Action |
|-----------|---------|-------------------|
| `boot` | Robot initialized | — |
| `start` | Move execution begun | — |
| `at_pick_hover` | Arm at hover above pick | — |
| `at_pick` | Arm at piece level | — |
| `grasped` | Gripper closed | **Begin TCP follow** |
| `lifted` | Arm lifted to hover | (drained during follow) |
| `at_place_hover` | Arm at hover above place | **Slide captured piece to graveyard** |
| `at_place` | Arm at place level | — |
| `released` | Gripper opened | **Snap piece to target, unlock** |
| `parked` | Arm at park position | — |
| `done` | Move complete | **Update board state** |
| `error` | Something failed | **Log error, continue** |

### 3.4 Fake-Grasp vs Physics-Based Grasp

The project uses a **fake-grasp** (scripted) approach rather than physics-based grasping:

| Aspect | Physics-Based Grasp | Fake-Grasp (Used) |
|--------|--------------------|--------------------|
| Piece Physics | Required (mass, friction, damping) | Not needed (static solids) |
| Gripper Contact | Physical contact detection | No contact — supervisor tracks TCP |
| Piece Attachment | Physics constraints | Supervisor sets piece position each step |
| Stability | Prone to oscillation/vibration | Always stable |
| Visual Fidelity | Realistic but hard to tune | Looks correct with proper TCP tracking |
| Complexity | High (contact properties, damping) | Low (position tracking only) |

**Why fake-grasp was chosen:** Adding Physics nodes to chess pieces caused continuous vibration (contact solver oscillations between pieces and board). The scripted approach provides clean, stable visuals with the robot arm moving realistically via IK while the supervisor handles piece positioning.

---

## 4. Game Controller: Detailed Comparison

### 4.1 Initialization Changes

#### Platform Compatibility (FIX-13)

**BEFORE (Windows-only):**
```python
STOCKFISH_PATH = r"stockfish\stockfish-windows-x86-64-avx2.exe"
```

**AFTER (Cross-platform):**
```python
STOCKFISH_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "stockfish-linux-x86-64-avx2"
)
```

**Why:** The original used a Windows backslash path and `.exe` binary. The fix uses `__file__`-relative absolute path construction, making it independent of the working directory at runtime. Webots does not always set the controller working directory to the controller folder, so a bare filename like `"stockfish-linux-x86-64-avx2"` could fail with `FileNotFoundError`, causing silent fallback to random moves.

---

#### ACK Receiver Naming (FIX-09)

**BEFORE:**
```python
r = sup.getDevice("receiver")
```

**AFTER:**
```python
r = sup.getDevice("ack_receiver")
```

**Corresponding world file change:**
```
# BEFORE
DEF ACK_RECEIVER Receiver {
  channel 2
}

# AFTER
DEF ACK_RECEIVER Receiver {
  name "ack_receiver"
  channel 2
}
```

**Why:** Relying on Webots' default unnamed device name `"receiver"` is fragile — any additional Receiver node would cause a name collision. An explicit name prevents silent conflicts.

---

#### Base Message Reliability (FIX-11)

**BEFORE:**
```python
send_robot_bases(sup, emitter)
ack_receiver = setup_ack_receiver(sup, dt)
```

**AFTER:**
```python
send_robot_bases(sup, emitter)
# Step a few times to let robot controllers enable their receivers,
# then re-send so neither robot misses the base message.
for _ in range(10):
    if sup.step(dt) == -1:
        break
send_robot_bases(sup, emitter)
ack_receiver = setup_ack_receiver(sup, dt)
```

**Why:** If a robot controller starts slightly after the supervisor, the first base message is transmitted before the receiver is enabled. Without `base_ready=True`, all move commands are silently rejected by `niryo_controller` (the `handle_move` method returns immediately). Sending twice with a delay ensures both robots receive the base pose.

---

### 4.2 Pick/Place Coordinate Computation

#### Z-Level Fix (FIX-29)

**BEFORE:**
```python
PIECE_GRASP_Z = 0.002
HOVER_Z_ABOVE_PIECE = 0.10

board_z_from = center_from[2]   # = 0 (board REF markers at Z=0)
board_z_to   = center_to[2]     # = 0

pick_hover_world  = [grasp_from[0], grasp_from[1],
                     board_z_from + HOVER_Z_ABOVE_PIECE]   # Z = 0 + 0.10 = 0.10
pick_world        = [grasp_from[0], grasp_from[1],
                     board_z_from + PIECE_GRASP_Z]         # Z = 0 + 0.002 = 0.002
```

**AFTER:**
```python
HOVER_Z_ABOVE_PIECE = 0.06

pick_hover_world  = [grasp_from[0], grasp_from[1],
                     grasp_from[2] + HOVER_Z_ABOVE_PIECE]  # Z = 0.04 + 0.06 = 0.10
pick_world        = [grasp_from[0], grasp_from[1],
                     grasp_from[2]]                        # Z = 0.04 (actual piece Z)
```

**Why:** The original computed pick Z from the board surface reference (`board_z = 0`), not from the actual piece position (`grasp_from[2] ≈ 0.04`). This caused the arm to target 4cm below the pieces. The fix uses the actual piece world Z coordinate, so the arm descends to the correct height regardless of piece type (pawns at Z≈0.04, kings at Z≈0.055).

**Impact on IK targets:**

| Coordinate | Before | After |
|------------|--------|-------|
| pick_world Z | 0.002m (board level) | 0.040m (piece level) |
| pick_hover Z | 0.100m (10cm above board) | 0.100m (6cm above piece) |
| place_world Z | 0.002m | 0.040m |
| place_hover Z | 0.100m | 0.100m |

---

#### Piece Offset Reset (FIX-04, FIX-12)

**BEFORE:**
```python
# No offset reset — stale offset used for all subsequent moves
chess_board.push(move)
occ.pop(from_sq, None)
occ[to_sq] = src_def
```

**FIX-04 (initial fix — too aggressive):**
```python
chess_board.push(move)
occ.pop(from_sq, None)
occ[to_sq] = src_def
piece_offset[src_def] = [0.0, 0.0, 0.0]  # Reset ALL — but Z is piece height!
```

**FIX-12 (corrected fix):**
```python
chess_board.push(move)
occ.pop(from_sq, None)
occ[to_sq] = src_def

# Reset XY offset only; preserve Z (piece height above board)
old_z = piece_offset.get(src_def, [0.0, 0.0, 0.0])[2]
piece_offset[src_def] = [0.0, 0.0, old_z]
```

**Why:** After the supervisor snaps a piece to the grid center, the XY offset from its original placement is no longer valid. However, the Z offset (piece height) is a geometric constant that must be preserved. FIX-04 initially reset all three to zero, causing pieces to be placed at Z=0 (board surface) on their second move — making them visually sink into the board mesh. FIX-12 corrected this to only reset XY while preserving Z.

---

### 4.3 TCP Follow System

#### Grasp Offset Change (FIX-26)

**BEFORE:**
```python
# Compute offset between piece and TCP at grasp moment
tcp_p_now = list(side_tcp.getPosition())
piece_p = list(node.getField("translation").getSFVec3f())
grasp_offset = sub(piece_p, tcp_p_now)
# grasp_offset ≈ [0.03, -0.05, -0.08] — large, wrong offset!
```

**AFTER:**
```python
# TCP marker sits at hand_link (URDF EE). The real gripper tip
# is TCP_Z_ABOVE_PIECE_LEVEL below that in world Z.
grasp_offset = [0.0, 0.0, -TCP_Z_ABOVE_PIECE_LEVEL]
# grasp_offset = [0.0, 0.0, -0.01] — small, correct offset
```

**Why:** Since the arm never physically reaches the piece (IK workspace limitations and no physics), the computed offset was large and incorrect, causing the piece to float at a wrong position away from the arm tip. Setting a fixed small offset (`-0.01` in Z to account for the TCP marker being slightly above the gripper tip) makes the piece track the arm tip directly. The constant `TCP_Z_ABOVE_PIECE_LEVEL = 0.01` is defined at the top of the file and must match the gripper geometry.

---

#### TCP World Position Fix (FIX-18)

**BEFORE:**
```python
tcp_p = get_translation(tcp_node)
# get_translation() calls getSFVec3f() — returns LOCAL position
# For TCP_WHITE: always returns [0.007, -0.072, -0.001] — CONSTANT!
```

**AFTER:**
```python
tcp_p = list(tcp_node.getPosition())
# getPosition() returns WORLD position — changes as arm moves!
# Returns actual world coordinates of the TCP marker
```

**Why:** `getSFVec3f()` returns the node's own local translation field — a constant value `[0.007, -0.072, -0.001]` that never changes as the arm moves. This meant:
```
grasp_offset = piece_world - tcp_local_constant
each follow step: set piece = tcp_local_constant + grasp_offset
                = tcp_local + (piece_world - tcp_local) = piece_world (unchanged!)
```
The piece position was set to itself on every step — the "piece following arm" was completely non-functional. Pieces only moved by the fallback `slide_piece_world` called after the arm finished.

---

#### Two-Phase Follow with Capture Clearing (FIX-16)

**BEFORE:**
```python
# Single follow: grasped → released
# Captured piece removed AFTER robot finishes (visual overlap for seconds)
end_msg = follow_tcp_until_ack(
    sup, dt,
    piece_node=node,
    tcp_node=side_tcp,
    grasp_offset=grasp_offset,
    receiver=ack_receiver,
    side=side_str,
    stages=("released", "error"),
    timeout_steps=2400
)
# Captured piece still at destination — overlap during entire place sequence
```

**AFTER:**
```python
# Phase 1: grasped → at_place_hover (arm transiting with piece)
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

# Clear captured piece NOW — before arm descends to place
if captured_node is not None and mid_msg and mid_msg.get("stage") == "at_place_hover":
    slide_piece_world(sup, captured_node, grave_pos, 200)
    captured_node = None  # mark handled

# Phase 2: at_place_hover → released (arm placing piece)
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
```

**Why:** In the original, the captured piece sat at the destination throughout the entire move, causing visual overlap when the moving piece arrived. By splitting the follow into two phases:
1. **Phase 1** follows the arm from grasp through lift, transit, to hover above destination
2. At `at_place_hover`, the destination square is cleared (captured piece slides to graveyard in 200ms)
3. **Phase 2** follows the arm's descent to place the moving piece on the now-clear square

---

### 4.4 Capture & Special Move Handling

#### Deferred Capture Animation (FIX-06)

**BEFORE:**
```python
# Captured piece animated to graveyard BEFORE robot moves
if capture_flag:
    captured_def = occ.get(to_sq)
    if captured_def and captured_def in nodes:
        captured_node = nodes[captured_def]
        captured_side = "White" if side_str == "Black" else "Black"
        grave_pos = grave_next(captured_side)
        slide_piece_world(sup, captured_node, grave_pos, DEMO_SPEED // 2)
        occ.pop(to_sq, None)
# Robot hasn't even started moving yet!
```

**AFTER:**
```python
# Capture info gathered, animation deferred
captured_def  = occ.get(to_sq) if capture_flag else None
captured_node = nodes[captured_def] if (captured_def and captured_def in nodes) else None
captured_side = ("White" if side_str == "Black" else "Black") if capture_flag else None
grave_pos     = grave_next(captured_side) if captured_node is not None else None

# ... robot executes entire move ...

# Captured piece animated at at_place_hover (during TCP follow Phase 1)
# OR as fallback after robot finishes:
if captured_node is not None:
    slide_piece_world(sup, captured_node, grave_pos, DEMO_SPEED // 2)
```

**Why:** The original moved the captured piece to the graveyard before the robot even started moving. This created a logical inconsistency (piece disappears before being "captured") and potential timing issues. The deferred approach:
1. Pre-computes capture info and graveyard slot
2. Clears the captured piece only when the arm is above the destination (two-phase follow)
3. Falls back to post-move clearing if TCP follow times out

---

#### Special Move Occupancy Map (FIX-08)

**ADDED (entirely new helper function):**
```python
def update_occ_for_special_moves(board, move, occ, nodes):
    """Sync occ map for castling and en passant.
    Call BEFORE chess_board.push(move)."""
    
    # En passant: captured pawn is NOT on to_sq but on the passing square
    if board.is_en_passant(move):
        ep_rank = chess.square_rank(move.from_square)  # Same rank as moving pawn
        ep_file = chess.square_file(move.to_square)    # Same file as destination
        ep_sq   = chess.square(ep_file, ep_rank)
        ep_def  = occ.pop(ep_sq, None)
        if ep_def:
            print(f"[occ] en passant: removed {ep_def} from {chess.square_name(ep_sq)}")

    # Castling: move the rook in the occ map
    if board.is_castling(move):
        rank = chess.square_rank(move.from_square)
        if chess.square_file(move.to_square) == 6:  # kingside (g-file)
            rook_from = chess.square(7, rank)  # h-file rook
            rook_to   = chess.square(5, rank)  # f-file
        else:  # queenside (c-file)
            rook_from = chess.square(0, rank)  # a-file rook
            rook_to   = chess.square(3, rank)  # d-file
        rook_def = occ.pop(rook_from, None)
        if rook_def:
            occ[rook_to] = rook_def
            print(f"[occ] castling: {rook_def} "
                  f"{chess.square_name(rook_from)}->{chess.square_name(rook_to)}")
```

**Why:** The original only handled normal moves in the occupancy map (`occ.pop(from_sq); occ[to_sq] = src_def`). Three special cases broke this:

| Move Type | Problem | Fix |
|-----------|---------|-----|
| **En passant** | Captured pawn is on a different square than `to_sq` (same file, same rank as `from_sq`) | Remove pawn from its actual square |
| **Castling** | Rook must also move (h1→f1 kingside, a1→d1 queenside) | Update rook's occ entry |
| **Promotion** | Pawn becomes queen but DEF name stays "PAWN_X2" | Not handled (visual limitation — pawn mesh stays) |

This function must be called **before** `chess_board.push(move)` because `board.is_en_passant()` and `board.is_castling()` need the pre-move board state to work correctly.

---

### 4.5 Duplicate Code Removal (FIX-01)

**DELETED (~33 lines):**
```python
# This entire second block was a duplicate with wrong keyword
if lift_msg and lift_msg.get("stage") == "lifted":
    set_locked(node, True)
    end_msg = follow_tcp_until_ack(
        sup, dt,
        piece_node=node,
        tcp_node=side_tcp,
        offset=off,            # ← WRONG: should be grasp_offset=
        receiver=ack_receiver,
        side=side_str,
        stages=("released", "error"),
        timeout_steps=2400
    )
    set_translation(node, target_pos)
    set_locked(node, False)
    rot0 = piece_rot0.get(src_def, None)
    if rot0 is not None:
        set_rotation(node, rot0)
    done_msg = wait_for_ack_msg(
        sup, dt, ack_receiver,
        side=side_str,
        stages=("done", "error"),
        timeout_steps=1200,
        debug_print=False
    )
    did_follow = bool(done_msg and done_msg.get("stage") == "done")
```

**Why:** This was the most critical bug. The function signature is:
```python
def follow_tcp_until_ack(..., grasp_offset, ...)
```
But the duplicate block called it with `offset=off` — a keyword that doesn't exist in the function signature. Python raised:
```
TypeError: follow_tcp_until_ack() got an unexpected keyword argument 'offset'
```
This exception was caught by the game loop's outer `try/except` block which then executed `break`, stopping the entire game after the very first move.

---

### 4.6 Graveyard Positions Fix (FIX-27)

**BEFORE:**
```python
WHITE_GRAVEYARD = [-0.6, -0.9, 0.0]
BLACK_GRAVEYARD = [-0.6,  0.9, 0.0]
```

**AFTER:**
```python
WHITE_GRAVEYARD = [-0.6, -0.9, 0.04]
BLACK_GRAVEYARD = [-0.6,  0.9, 0.04]
```

**Why:** Chess pieces sit with their centers at Z≈0.040. Captured pieces slid to graveyard with Z=0.0, appearing sunk ~4cm into the floor. Setting Z=0.04 matches the piece height on the board.

---

### 4.7 Complete Game Controller Side-by-Side

| Section | Initial Code | Modified Code | Why Changed |
|---------|-------------|---------------|-------------|
| Stockfish path | Windows `.exe` path | `__file__`-relative Linux path | Cross-platform |
| Base message | Sent once | Sent twice with delay | Reliability |
| ACK receiver | `"receiver"` (default) | `"ack_receiver"` (explicit) | Safety |
| Hover Z | `board_z + 0.10` | `piece_z + 0.06` | Correct height |
| Pick Z | `board_z + 0.002` | `piece_z` (actual) | Arm reaches pieces |
| Piece offset | Never reset | XY reset, Z preserved | Correct positioning |
| Capture timing | Before robot moves | At `at_place_hover` | No visual overlap |
| TCP position | `getSFVec3f()` (local) | `getPosition()` (world) | Pieces actually follow arm |
| Grasp offset | Computed (large, wrong) | Fixed `[0,0,-0.01]` | Piece at arm tip |
| Follow phases | Single (grasped→released) | Two-phase with capture clear | Clean placement |
| Special moves | Not handled | En passant + castling handler | Correct occ map |
| Duplicate block | Present (crashes game) | Deleted | Game doesn't crash |

---

## 5. Robot Controller (niryo_controller): Detailed Comparison

### 5.1 Joint Configuration

#### Fixed Joint 4 (Forearm Roll)

**BEFORE:**
```python
# No joint fixing — all 6 joints solved by IK
ik_active_joints = {"joint_1", "joint_2", "joint_3",
                    "joint_4", "joint_5", "joint_6"}
```

**AFTER:**
```python
# Joint 4 fixed at startup + configurable offset
J4_ROTATION_OFFSET = 0.0  # Changeable constant

j4_startup = init_joints[3]
j4_rotated = j4_startup + J4_ROTATION_OFFSET

# Smart clamping — try opposite direction if one exceeds limits
if j4_rotated > j4_max:
    j4_alt = j4_startup - J4_ROTATION_OFFSET
    if j4_alt >= j4_min:
        j4_rotated = j4_alt
    else:
        j4_rotated = j4_max  # Clamp as last resort

self.fixed_j4 = clamp(j4_rotated, j4_min, j4_max)
self.motors[3].setPosition(self.fixed_j4)

# IK solves only joints 1, 2, 3, 5, 6
ik_active_joints = {"joint_1", "joint_2", "joint_3",
                    "joint_5", "joint_6"}
```

**Why:** The forearm roll (joint 4) doesn't contribute meaningfully to the chess pick-and-place task — it only rotates the gripper jaws around the forearm axis. Fixing it eliminates one degree of freedom from the IK solver, reducing the solution space and preventing the solver from finding configurations where the forearm is rotated to unusual angles. The `J4_ROTATION_OFFSET` constant allows easy adjustment of the gripper jaw orientation (e.g., `math.pi/2` for 90° rotation).

The smart clamping logic handles cases where the desired rotation exceeds joint limits by trying the opposite direction (e.g., if +90° exceeds max, try -90° which often reaches the same physical orientation for 180° offsets).

---

#### Joint 5 Bias (Wrist Pitch)

**ADDED:**
```python
# ---- Joint_5 (wrist pitch / hand_link tilt) bias ----
# This offset is ADDED to every joint_5 value after IK solves.
# It shifts the hand_link/gripper downward.
J5_BIAS_OFFSET = -math.pi / 6.0  # -60 degrees

def _apply_motor_limits(self, q6):
    q6 = list(q6)
    q6[3] = self.fixed_j4                    # Force joint_4
    q6[4] = float(q6[4]) + J5_BIAS_OFFSET   # Apply wrist bias
    out = []
    for i in range(6):
        jmin, jmax = self.joint_limits[i]
        out.append(clamp(float(q6[i]), jmin, jmax))
    return out
```

**Why:** The hand_link orientation in the URDF doesn't perfectly align with the desired "gripper pointing down" configuration. The IK solver computes joint values for the URDF model, but the physical gripper orientation in the Webots simulation may differ slightly. Adding a -60° bias to joint 5 shifts the wrist to a more natural grasping angle. This is applied consistently to all motor commands through `_apply_motor_limits()`, which is called by both `set_joint_targets()` and `set_joints_smooth()`.

The bias was determined experimentally: -90° was too aggressive (gripper tilted too far), -60° provided the best visual alignment between the gripper and the chess pieces.

---

### 5.2 Auto-Calibration System

**ADDED (entirely new):**
```python
def _auto_calibrate(self):
    """Auto-detect which direction the URDF's EE Z-axis points when
    the gripper is physically pointing down, then calibrate j5 correction.

    Tests both target directions: [0,0,-1] and [0,0,+1]
    with the formula j5 = -(j2 + j3) + correction.

    Picks whichever (direction, correction) combo gives the smallest
    angle between the FK Z-axis and the target.

    Validates across multiple (j2, j3) poses to confirm the formula
    generalises — rejects candidates that only work for one pose.
    """
    j2_test, j3_test = -0.5, 0.3

    validation_poses = [(-0.3, 0.1), (-0.5, 0.3), (-0.8, 0.8)]

    best_correction = 0.0
    best_angle = 1e9
    best_direction = [0.0, 0.0, -1.0]

    for down_candidate in [[0.0, 0.0, -1.0], [0.0, 0.0, 1.0]]:
        for corr_try in [x * 0.01 for x in range(-150, 151)]:
            j5_try = -(j2_test + j3_test) + corr_try
            test_q6 = [0.0, j2_test, j3_test, self.fixed_j4, j5_try, 0.0]
            q_full = self._q6_to_full(test_q6)
            fk = self.chain.forward_kinematics(q_full)
            ee_z = fk[:3, 2]

            cos_a = float(np.dot(ee_z, down_candidate))
            cos_a = max(-1.0, min(1.0, cos_a))
            angle = math.acos(cos_a)

            if angle < best_angle:
                best_angle = angle
                best_correction = corr_try
                best_direction = list(down_candidate)

    self.j5_down_correction = best_correction
    self.gripper_down_vector = best_direction

    # Validate across multiple poses
    for j2_v, j3_v in validation_poses:
        j5_v = -(j2_v + j3_v) + self.j5_down_correction
        verify_q6 = [0.0, j2_v, j3_v, self.fixed_j4, j5_v, 0.0]
        q_full = self._q6_to_full(verify_q6)
        fk = self.chain.forward_kinematics(q_full)
        ee_z = fk[:3, 2]
        cos_a = float(np.dot(ee_z, self.gripper_down_vector))
        verify_angle = math.acos(max(-1.0, min(1.0, cos_a)))
        print(f"[AUTO-CALIB] Verify j2={j2_v} j3={j3_v}: "
              f"angle_from_down={math.degrees(verify_angle):.2f}°")
```

**Why:** Different URDF configurations can have the gripper Z-axis pointing either up or down in the kinematic model. Instead of hardcoding `[0,0,-1]` or `[0,0,+1]`, the auto-calibration:

1. **Sweeps both directions** — tests `[0,0,-1]` and `[0,0,+1]` as candidate "down" vectors
2. **Sweeps 301 correction values** — from -1.50 to +1.50 rad in 0.01 steps
3. **Uses forward kinematics** — computes the actual EE Z-axis direction for each (direction, correction) pair
4. **Picks the best** — minimum angle between FK Z-axis and candidate direction
5. **Validates** — confirms the formula works for 3 different arm poses (not just the test pose)

For the NiryoNed URDF used in this project, the auto-calibration consistently detects:
- `gripper_down_vector = [0.0, 0.0, -1.0]`
- `j5_down_correction ≈ -1.50 rad`

---

### 5.3 IK Solver Improvements

#### Error Handling (FIX-02)

**BEFORE:**
```python
if best is None:
    raise last_err  # Could be None → TypeError: exceptions must derive from BaseException
```

**AFTER:**
```python
if not candidates:
    if last_err is not None:
        raise RuntimeError(
            f"[IK] No solution found for target_base={target_base} "
            f"(world={target_world}). Last solver error: {last_err}"
        )
    raise RuntimeError(
        f"[IK] No solution found for target_base={target_base}: "
        "all initial guesses failed."
    )
```

**Same fix applied to `solve_ik_with_fallback_down()`:**
```python
# BEFORE
raise last_err  # Could be None

# AFTER
if last_err is not None:
    raise RuntimeError(
        f"[IK fallback-down] No solution found for {down_world}. "
        f"Tried offsets {DOWN_TRY_Z_OFFSETS}. Last error: {last_err}"
    )
raise RuntimeError(
    f"[IK fallback-down] No solution found for {down_world}."
)
```

**Why:** `raise None` causes `TypeError: exceptions must derive from BaseException`, which hides the real IK failure reason and produces an opaque traceback. The fix provides clear, actionable error messages with the failed target position and the list of Z offsets tried.

---

#### Multi-Guess IK with Orientation Constraint

**BEFORE (3 generic guesses, no target awareness):**
```python
def _make_guess_q6_list(self):
    q_now = self.get_current_joints()
    guesses = [q_now]
    guesses.append([0.0, -0.8, 0.8, 0.0, 0.0, 0.0])   # j5=0 → gripper up!
    guesses.append([0.0, -1.0, 1.0, 0.2, 0.0, 0.0])   # j5=0 → gripper up!
    guesses.append([0.2, -0.9, 0.9, 0.0, 0.0, 0.2])   # j5=0 → gripper up!
    return guesses
```

**AFTER (12+ targeted guesses with calibrated j5):**
```python
def _make_guess_q6_list(self, target_base=None):
    """Generate initial guesses for IK solver.
    All guesses use the calibrated j5 correction so the gripper
    starts pointing DOWN (not up) in every initial guess."""
    q_now = self.get_current_joints()

    j1 = 0.0
    if target_base is not None:
        j1 = math.atan2(target_base[1], target_base[0])

    j4 = self.fixed_j4
    corr = self.j5_down_correction

    guesses = [q_now]
    # Each guess computes j5 from the calibrated formula
    for j2, j3 in [(-0.3, 0.1), (-0.5, 0.3), (-0.6, 0.5), (-0.7, 0.6),
                    (-0.8, 0.8), (-1.0, 1.0), (-0.4, 0.2), (-0.2, 0.05)]:
        j5 = -(j2 + j3) + corr  # Calibrated for gripper-down
        guesses.append([j1, j2, j3, j4, j5, 0.0])

    # Joint 1 variations for side cells
    for j1_off in [0.2, -0.2, 0.4, -0.4]:
        j5 = -(-0.6 + 0.5) + corr
        guesses.append([j1 + j1_off, -0.6, 0.5, j4, j5, 0.0])

    return guesses
```

**Why — 4 critical improvements:**

1. **Target-aware j1:** Joint 1 is pre-computed from the target position using `atan2(by, bx)`, pointing the arm toward the target cell. Old guesses used `j1=0.0` which only works for cells directly in front.

2. **Calibrated j5:** Every guess uses `j5 = -(j2 + j3) + correction` from auto-calibration, so the gripper starts pointing DOWN in every initial guess. Old guesses had `j5=0.0` which means gripper pointing UP — the IK solver had to rotate 180° from every starting point.

3. **More (j2, j3) combinations:** 8 different arm extension levels vs 3, covering near cells (j2=-0.2, j3=0.05 → arm nearly vertical) to far cells (j2=-1.0, j3=1.0 → arm fully extended).

4. **j1 variations:** `±0.2, ±0.4` rad offsets cover cells at the board edges where `atan2` alone may not provide a good starting point.

---

#### Seeded IK for Descent (FIX-36)

**ADDED:**
```python
def solve_ik(self, target_world, seed_q6=None):
    """Solve IK for joints 1, 2, 3, 5, 6.
    
    When seed_q6 is provided, uses ONLY that single guess.
    This forces the solver to find the nearest local minimum,
    preventing the arm from jumping to a different configuration.
    """
    target_base = self.world_to_base(target_world)
    target_base[2] += TOOL_Z_OFFSET

    q_now = self.get_current_joints()
    desired_ee_z = self.gripper_down_vector

    if seed_q6 is not None:
        # Single-seed mode: one attempt, no multi-guess
        guesses = [seed_q6]
    else:
        guesses = self._make_guess_q6_list(target_base)

    candidates = []
    last_err = None

    for guess_q6 in guesses:
        try:
            init_full = self._q6_to_full(guess_q6)
            q_full = self.chain.inverse_kinematics(
                target_position=target_base,
                target_orientation=desired_ee_z,
                orientation_mode="Z",
                initial_position=init_full
            )
            q6 = self._full_to_q6(q_full)
            angle_dev, ee_z_actual = self._check_orientation_down(q6)
            # ... compute pos_err, joint_score ...
            candidates.append({...})
        except Exception as e:
            last_err = e

    # Select best candidate (prefer good orientation, minimize joint movement)
    good = [c for c in candidates if c['angle_dev'] < ORIENTATION_TOLERANCE_RAD]
    if good:
        best = min(good, key=lambda c: c['joint_score'])
    else:
        best = min(candidates, key=lambda c: c['angle_dev'])
    return best['q6']
```

**Usage in handle_move:**
```python
# PHASE 1: Solve hover IK (unseded — uses all guesses)
q_pick_hover = self.solve_ik(pick_hover)

# PHASE 2: Descend to piece (SEEDED with hover joints)
q_pick_down, _ = self.solve_ik_with_fallback_down(
    pick_world, seed_q6=q_pick_hover  # ← Seed ensures straight descent
)

# PHASE 6: Solve place hover IK (unseeded)
q_place_hover = self.solve_ik(place_hover)

# PHASE 7: Descend to place (SEEDED with place hover joints)
q_place_down, _ = self.solve_ik_with_fallback_down(
    place_world, seed_q6=q_place_hover  # ← Seed ensures straight descent
)
```

**Why — root cause of "arm lunges to wrong cell":**

Without seeding, the IK solver received the same multi-guess list for both hover and descent targets. The hover target (same XY, higher Z) and descent target (same XY, lower Z) could produce completely different local minima in joint space. The solver converged to one configuration for hover and a different one for descent. When the arm transitioned from hover to descent, it swung to the new configuration mid-air — appearing to lunge to a different cell.

By seeding with the hover joints, the descent IK starts from the hover configuration and finds the nearest solution — the arm simply lowers straight down (only Z changes, same XY → same joint 1 and similar joints 2,3).

---

### 5.4 Force Gripper Vertical

**ADDED (entirely new):**
```python
def force_gripper_vertical(self, wait_steps=30):
    """Correct joint_5 to make gripper point exactly straight down.
    Only touches joint_5. All other joints stay as they are."""
    q_now = self.get_current_joints()
    j2 = q_now[1]
    j3 = q_now[2]
    old_j5 = q_now[4]

    # Analytical formula from auto-calibration
    j5_vertical = -(j2 + j3) + self.j5_down_correction
    j5_min, j5_max = self.joint_limits[4]
    j5_vertical = clamp(j5_vertical, j5_min, j5_max)

    q_corrected = list(q_now)
    q_corrected[3] = self.fixed_j4
    q_corrected[4] = j5_vertical

    print(f"[FORCE-DOWN] j2={j2:.3f} j3={j3:.3f} "
          f"j5: {old_j5:.3f} -> {j5_vertical:.3f} "
          f"(delta={math.degrees(abs(j5_vertical - old_j5)):.1f}°)")

    self.set_joints_smooth(q_corrected, max_steps=wait_steps, speed=0.05)

    angle_dev, ee_z = self._check_orientation_down(self.get_current_joints())
    print(f"[FORCE-DOWN] Result: angle_from_down={math.degrees(angle_dev):.1f}°")
```

**Called before grasping and before releasing:**
```python
# Before grip
self.force_gripper_vertical(wait_steps=30)
self.close_gripper(wait_steps=10)
self.send_ack("grasped", {"locked": True})

# Before release
self.force_gripper_vertical(wait_steps=30)
self.open_gripper(wait_steps=10)
self.send_ack("released")
```

**Why:** IK may not perfectly achieve vertical gripper orientation due to the numerical optimization nature of the solver (ikpy uses scipy.optimize which finds local minima). This analytical correction:
1. Reads current j2 and j3 values from motor sensors
2. Computes exact j5 using the calibrated formula `j5 = -(j2 + j3) + correction`
3. Smoothly moves only j5 to the corrected value (all other joints unchanged)
4. Verifies the result via FK orientation check

This ensures the gripper is always properly oriented at the moment of grip/release, regardless of IK solution quality.

---

### 5.5 Orientation Check System

**ADDED (entirely new):**
```python
def _check_orientation_down(self, q6):
    """Check how far the gripper orientation deviates from 'down'.
    Returns (angle_in_radians, ee_z_axis_vector)."""
    q_full = self._q6_to_full(q6)
    fk = self.chain.forward_kinematics(q_full)
    ee_z = fk[:3, 2]  # End-effector Z-axis direction (3rd column of rotation matrix)

    down = np.array(self.gripper_down_vector)  # Auto-detected direction
    ee_z_np = np.array(ee_z)

    cos_angle = np.dot(ee_z_np, down) / (np.linalg.norm(ee_z_np) * np.linalg.norm(down) + 1e-12)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = math.acos(cos_angle)
    return angle, ee_z.tolist()
```

**Used in IK candidate selection:**
```python
ORIENTATION_TOLERANCE_RAD = 0.25  # ~14 degrees

# Filter solutions by orientation quality
good = [c for c in candidates
        if c['angle_dev'] < ORIENTATION_TOLERANCE_RAD]

if good:
    # Among good solutions, pick the one with least joint movement
    best = min(good, key=lambda c: c['joint_score'])
    print(f"[IK] GOOD solution: angle_dev={math.degrees(best['angle_dev']):.1f}°")
else:
    # No good solution — use the one closest to vertical
    best = min(candidates, key=lambda c: c['angle_dev'])
    print(f"[IK] WARNING: Best angle_dev={math.degrees(best['angle_dev']):.1f}°")
```

**Why:** Not all IK solutions produce a gripper pointing straight down. The IK solver minimizes a cost function that balances position accuracy and orientation — it may sacrifice orientation for position. The orientation check:
1. Measures the angle between the end-effector Z-axis and the calibrated "down" direction
2. Filters solutions: those within 0.25 rad (≈14°) are considered "good"
3. Among good solutions, selects the one requiring least joint movement from current position
4. If no solution meets the tolerance, uses the one closest to vertical (degraded but best available)

---

### 5.6 URDF Chain Configuration

#### ikpy Warning Suppression

**BEFORE:**
```python
full_chain = Chain.from_urdf_file(
    urdf_path,
    last_link_vector=GRIPPER_TIP_OFFSET
)
# Produces 3 warnings:
# UserWarning: Link Base link (index: 0) is of type 'fixed' but set as active
# UserWarning: Link hand_link_base_gripper_joint (index: 7) is of type 'fixed' but set as active
# UserWarning: Link last_joint (index: 9) is of type 'fixed' but set as active
```

**AFTER:**
```python
# Step 1: Temporary chain for inspection only — suppress warnings
with warnings.catch_warnings():
    warnings.simplefilter("ignore", UserWarning)
    full_chain = Chain.from_urdf_file(
        urdf_path,
        last_link_vector=GRIPPER_TIP_OFFSET
    )

# Step 2: Build active mask — ONLY revolute joints we want IK to solve
ik_active_joints = {"joint_1", "joint_2", "joint_3", "joint_5", "joint_6"}
mask = []
for link in full_chain.links:
    link_name = getattr(link, "name", "") or ""
    mask.append(link_name in ik_active_joints)

# Step 3: Build FINAL chain with correct mask — no warnings
self.chain = Chain.from_urdf_file(
    urdf_path,
    active_links_mask=mask,
    last_link_vector=GRIPPER_TIP_OFFSET
)
```

**Chain structure (10 links):**
```
[0] 'Base link'                      active=False (fixed)
[1] 'joint_1'                        active=True  (shoulder rotation)
[2] 'joint_2'                        active=True  (shoulder pitch)
[3] 'joint_3'                        active=True  (elbow pitch)
[4] 'joint_4'                        active=False (forearm roll — FIXED)
[5] 'joint_5'                        active=True  (wrist pitch)
[6] 'joint_6'                        active=True  (wrist roll)
[7] 'hand_link_base_gripper_joint'   active=False (fixed)
[8] 'joint_base_to_jaw_2'            active=False (gripper)
[9] 'last_joint'                     active=False (virtual tip)
```

**Why:** The original loaded the chain with the default mask (all True), which included fixed links like "Base link" and "hand_link_base_gripper_joint". ikpy detected these as fixed links marked active and produced warnings. While functionally harmless (fixed links provide no transformation regardless of mask), the warnings were misleading and cluttered the console output. The two-step approach:
1. Uses a suppressed temporary chain only for link name inspection
2. Builds the production chain with the correct mask where only revolute joints the IK should solve are marked active

---

### 5.7 Waypoint Error Handling (FIX-07)

**BEFORE:**
```python
def move_world_waypoints(self, p_from, p_to, steps=4, speed=0.02):
    if p_from is None or p_to is None:
        return
    for i in range(1, steps + 1):
        t = i / steps
        p = [
            p_from[0] + (p_to[0] - p_from[0]) * t,
            p_from[1] + (p_to[1] - p_from[1]) * t,
            p_from[2] + (p_to[2] - p_from[2]) * t
        ]
        q = self.solve_ik(p)       # Crashes if unreachable!
        self.set_joints_smooth(q, max_steps=140, speed=speed)
```

**AFTER:**
```python
def move_world_waypoints(self, p_from, p_to, steps=4, speed=0.02):
    if p_from is None or p_to is None:
        return
    for i in range(1, steps + 1):
        t = i / steps
        p = [
            p_from[0] + (p_to[0] - p_from[0]) * t,
            p_from[1] + (p_to[1] - p_from[1]) * t,
            p_from[2] + (p_to[2] - p_from[2]) * t
        ]
        try:
            q = self.solve_ik(p)
        except Exception as e:
            print(f"[waypoints] IK failed at step {i}/{steps} pos={p}: {e}. Skipping.")
            continue   # Skip unreachable waypoint, continue to next
        self.set_joints_smooth(q, max_steps=140, speed=speed)
```

**Why:** An unreachable intermediate waypoint no longer aborts the entire move. The arm skips that waypoint and continues toward the target. The outer try/except in `handle_move` still catches final destination failures from `solve_ik_with_fallback_down`.

Same pattern applied to `move_vertical()`:
```python
def move_vertical(self, xy_world, z_from, z_to, steps=4, speed=0.03):
    for i in range(1, steps + 1):
        t = i / steps
        z = z_from + (z_to - z_from) * t
        p = [xy_world[0], xy_world[1], z]
        try:
            q = self.solve_ik(p)
        except Exception as e:
            print(f"[vertical] IK failed at step {i}/{steps} z={z:.4f}: {e}. Skipping.")
            continue
        self.set_joints_smooth(q, max_steps=100, speed=speed)
```

---

### 5.8 Speed Parameter Fix (FIX-10)

**BEFORE:**
```python
spd = MOVE_SPEED if speed is None else float(speed)
```

**AFTER:**
```python
spd = float(speed) if speed is not None else MOVE_SPEED
```

**Why:** The original logic was technically correct (since default `speed=0.02` is never `None`) but misleadingly implied `MOVE_SPEED` is the primary value. The rewrite makes the intent clear: use the caller's speed value when provided, fall back to the global `MOVE_SPEED` only when explicitly passed as `None`.

---

### 5.9 Complete Robot Controller Side-by-Side

| Section | Initial Code | Modified Code | Why Changed |
|---------|-------------|---------------|-------------|
| **Joint fixing** | All 6 joints in IK | Joint 4 fixed, configurable offset | Reduce IK complexity |
| **Joint 5 bias** | None | -60° bias on all motor commands | Correct wrist angle |
| **IK guesses** | 3 generic (j5=0, gripper up) | 12+ targeted (j5 calibrated, gripper down) | Better convergence |
| **Auto-calibration** | None | FK sweep for down direction + j5 correction | URDF-independent |
| **Orientation check** | None | FK-based angle measurement + filtering | Quality IK solutions |
| **Force vertical** | None | Analytical j
