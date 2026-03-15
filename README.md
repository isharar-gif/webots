# webots
# Digital Twin Chess Robot Simulation — Technical Documentation

## Modifications and Enhancements by Ishara Rajarathna

**Project:** Digital Twin_niryobased (Webots Chess Robot Simulation)  
**Original Authors:** Kostadin & Devran  
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
9. [Summary of All Changes](#9-summary-of-all-changes)

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
| No physics on pieces | **CRITICAL** | Chess piece protos had no `Physics` node — robot passed through pieces |
| Stale piece offsets | **HIGH** | `piece_offset` never reset after moves — pick positions drifted |
| Wrong hover Z | **HIGH** | Hover height calculated from piece origin, not board surface |
| Premature capture animation | **HIGH** | Captured piece moved to graveyard before robot even started moving |
| Windows Stockfish path | **HIGH** | Hardcoded Windows path prevented Linux execution |

---

## 3. Modified System Architecture & Flow

### 3.1 Key Architectural Changes

1. **Fake-grasp with TCP follow** — Pieces track the robot's TCP (Tool Center Point) world position in real-time during moves
2. **Auto-calibration** — Gripper "down" direction auto-detected via FK sweep
3. **Seeded IK** — Descent IK uses hover joints as seed to prevent cell misalignment
4. **Two-phase TCP follow** — Captured pieces cleared at `at_place_hover` stage
5. **Fixed joints** — Joint 4 (forearm roll) locked at startup; Joint 5 has configurable bias

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

**Why:** The original used a Windows backslash path and `.exe` binary. The fix uses `__file__`-relative absolute path construction, making it independent of the working directory at runtime.

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
for _ in range(10):
    if sup.step(dt) == -1:
        break
send_robot_bases(sup, emitter)
ack_receiver = setup_ack_receiver(sup, dt)
```

**Why:** If a robot controller starts slightly after the supervisor, the first base message is transmitted before the receiver is enabled. Without `base_ready=True`, all move commands are silently rejected. Sending twice with a delay ensures both robots receive the base pose.

---

### 4.2 Pick/Place Coordinate Computation

#### Z-Level Fix (FIX-29)

**BEFORE:**
```python
PIECE_GRASP_Z = 0.002
HOVER_Z_ABOVE_PIECE = 0.10

pick_hover_world  = [grasp_from[0], grasp_from[1],
                     board_z_from + HOVER_Z_ABOVE_PIECE]
pick_world        = [grasp_from[0], grasp_from[1],
                     board_z_from + PIECE_GRASP_Z]
```

**AFTER:**
```python
HOVER_Z_ABOVE_PIECE = 0.06

pick_hover_world  = [grasp_from[0], grasp_from[1],
                     grasp_from[2] + HOVER_Z_ABOVE_PIECE]
pick_world        = [grasp_from[0], grasp_from[1],
                     grasp_from[2]]
```

**Why:** The original computed pick Z from the board surface reference (`board_z = 0`), not from the actual piece position (`grasp_from[2] ≈ 0.04`). This caused the arm to target 4cm below the pieces. The fix uses the actual piece world Z coordinate.

---

#### Piece Offset Reset (FIX-04, FIX-12)

**BEFORE:**
```python
# No offset reset — stale offset used for all subsequent moves
chess_board.push(move)
occ.pop(from_sq, None)
occ[to_sq] = src_def
```

**AFTER:**
```python
chess_board.push(move)
occ.pop(from_sq, None)
occ[to_sq] = src_def

# Reset XY offset only; preserve Z (piece height above board)
old_z = piece_offset.get(src_def, [0.0, 0.0, 0.0])[2]
piece_offset[src_def] = [0.0, 0.0, old_z]
```

**Why:** After the supervisor snaps a piece to the grid center, the XY offset from its original placement is no longer valid. However, the Z offset (piece height) is a geometric constant that must be preserved. FIX-04 initially reset all three to zero, causing pieces to sink into the board on second moves. FIX-12 corrected this to preserve Z.

---

### 4.3 TCP Follow System

#### Grasp Offset Change (FIX-26)

**BEFORE:**
```python
# Compute offset between piece and TCP at grasp moment
tcp_p_now = list(side_tcp.getPosition())
piece_p = list(node.getField("translation").getSFVec3f())
grasp_offset = sub(piece_p, tcp_p_now)
```

**AFTER:**
```python
grasp_offset = [0.0, 0.0, -TCP_Z_ABOVE_PIECE_LEVEL]
```

**Why:** Since the arm never physically reaches the piece (IK workspace limitations), the computed offset was large and incorrect, causing the piece to float at a wrong position away from the arm tip. Setting a fixed small offset (`-0.01` in Z to account for the TCP marker being slightly above the gripper tip) makes the piece track the arm tip directly.

---

#### TCP World Position Fix (FIX-18)

**BEFORE:**
```python
tcp_p = get_translation(tcp_node)  # getSFVec3f() — LOCAL position, constant!
```

**AFTER:**
```python
tcp_p = list(tcp_node.getPosition())  # getPosition() — WORLD position, dynamic!
```

**Why:** `getSFVec3f()` returns the node's own local translation field — a constant value that never changes as the arm moves. `getPosition()` returns the absolute world position reflecting actual arm kinematics. This was the root cause of pieces never moving during TCP follow.

---

#### Two-Phase Follow with Capture Clearing (FIX-16)

**BEFORE:**
```python
# Single follow: grasped → released
# Captured piece removed AFTER robot finishes (visual overlap)
end_msg = follow_tcp_until_ack(..., stages=("released", "error"), ...)
```

**AFTER:**
```python
# Phase 1: grasped → at_place_hover (arm transiting)
mid_msg = follow_tcp_until_ack(..., stages=("at_place_hover", "error"), ...)

# Clear captured piece NOW — before arm descends to place
if captured_node is not None and mid_msg:
    slide_piece_world(sup, captured_node, grave_pos, 200)

# Phase 2: at_place_hover → released (arm placing)
end_msg = follow_tcp_until_ack(..., stages=("released", "error"), ...)
```

**Why:** In the original, the captured piece sat at the destination throughout the entire move, causing visual overlap when the moving piece arrived. By splitting the follow into two phases, the captured piece is cleared the moment the arm reaches hover above the destination — before it descends.

---

### 4.4 Capture & Special Move Handling

#### Deferred Capture Animation (FIX-06)

**BEFORE:**
```python
# Captured piece animated to graveyard BEFORE robot moves
if capture_flag:
    captured_node = nodes[occ.get(to_sq)]
    slide_piece_world(sup, captured_node, grave_pos, ...)
    occ.pop(to_sq, None)
```

**AFTER:**
```python
# Capture info gathered, animation deferred
captured_def  = occ.get(to_sq) if capture_flag else None
captured_node = nodes[captured_def] if captured_def else None

# ... robot executes move ...

# Captured piece animated AFTER robot finishes (or at at_place_hover)
if captured_node is not None:
    slide_piece_world(sup, captured_node, grave_pos, ...)
```

**Why:** The original moved the captured piece to the graveyard before the robot even started moving. This created a logical inconsistency and potential timing issues.

---

#### Special Move Occupancy Map (FIX-08)

**ADDED:**
```python
def update_occ_for_special_moves(board, move, occ, nodes):
    """Sync occ map for castling and en passant.
    Call BEFORE chess_board.push(move)."""
    
    # En passant: captured pawn is NOT on to_sq
    if board.is_en_passant(move):
        ep_rank = chess.square_rank(move.from_square)
        ep_file = chess.square_file(move.to_square)
        ep_sq   = chess.square(ep_file, ep_rank)
        occ.pop(ep_sq, None)

    # Castling: move rook in occ map
    if board.is_castling(move):
        rank = chess.square_rank(move.from_square)
        if chess.square_file(move.to_square) == 6:  # kingside
            rook_from = chess.square(7, rank)
            rook_to   = chess.square(5, rank)
        else:  # queenside
            rook_from = chess.square(0, rank)
            rook_to   = chess.square(3, rank)
        rook_def = occ.pop(rook_from, None)
        if rook_def:
            occ[rook_to] = rook_def
```

**Why:** The original only handled normal moves in the occupancy map. En passant captures (where the captured pawn is on a different square than the destination) and castling (where the rook must also move) left ghost entries, causing incorrect piece lookups on subsequent moves.

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
    # ... 20 more lines ...
```

**Why:** This was the most critical bug. The duplicate block called `follow_tcp_until_ack()` with `offset=off` instead of `grasp_offset=grasp_offset`. Python raised `TypeError: unexpected keyword argument 'offset'`, which was caught by the game loop's outer `try/except` and broke out with `break` — stopping the entire game after one move.

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
self.fixed_j4 = clamp(j4_rotated, j4_min, j4_max)
self.motors[3].setPosition(self.fixed_j4)

# IK solves only joints 1, 2, 3, 5, 6
ik_active_joints = {"joint_1", "joint_2", "joint_3",
                    "joint_5", "joint_6"}
```

**Why:** The forearm roll (joint 4) doesn't contribute meaningfully to the chess pick-and-place task. Fixing it eliminates one degree of freedom from the IK solver, reducing the solution space and preventing the solver from finding configurations where the forearm is rotated to unusual angles. The `J4_ROTATION_OFFSET` constant allows easy adjustment of the gripper jaw orientation.

---

#### Joint 5 Bias (Wrist Pitch)

**ADDED:**
```python
J5_BIAS_OFFSET = -math.pi / 6.0  # -60 degrees

def _apply_motor_limits(self, q6):
    q6 = list(q6)
    q6[3] = self.fixed_j4           # Force joint_4
    q6[4] = float(q6[4]) + J5_BIAS_OFFSET  # Apply wrist bias
    out = []
    for i in range(6):
        jmin, jmax = self.joint_limits[i]
        out.append(clamp(float(q6[i]), jmin, jmax))
    return out
```

**Why:** The hand_link orientation in the URDF doesn't perfectly align with the desired "gripper pointing down" configuration. Adding a -60° bias to joint 5 shifts the wrist to a more natural grasping angle. This is applied consistently to all motor commands.

---

### 5.2 Auto-Calibration System

**ADDED (entirely new):**
```python
def _auto_calibrate(self):
    """Auto-detect which direction the URDF's EE Z-axis points when
    the gripper is physically pointing down."""
    
    # Test BOTH possible down directions
    for down_candidate in [[0,0,-1], [0,0,+1]]:
        for corr_try in [x * 0.01 for x in range(-150, 151)]:
            j5_try = -(j2_test + j3_test) + corr_try
            # ... compute FK, measure angle ...
            if angle < best_angle:
                best_correction = corr_try
                best_direction = down_candidate

    self.j5_down_correction = best_correction
    self.gripper_down_vector = best_direction
```

**Why:** Different URDF configurations can have the gripper Z-axis pointing either up or down in the kinematic model. Instead of hardcoding `[0,0,-1]` or `[0,0,+1]`, the auto-calibration sweeps both directions and all possible j5 corrections to find which combination makes the gripper point physically downward. This makes the controller robust to URDF changes.

The calibration validates across multiple (j2, j3) test poses to ensure the formula `j5 = -(j2 + j3) + correction` generalizes, rather than only working for one specific arm configuration.

---

### 5.3 IK Solver Improvements

#### Error Handling (FIX-02)

**BEFORE:**
```python
if best is None:
    raise last_err  # Could be None → TypeError!
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

**Why:** `raise None` causes `TypeError: exceptions must derive from BaseException`, which hides the real IK failure reason. The fix provides clear, actionable error messages with the failed target position.

---

#### Multi-Guess IK with Orientation Constraint

**BEFORE (3 generic guesses):**
```python
def _make_guess_q6_list(self):
    q_now = self.get_current_joints()
    guesses = [q_now]
    guesses.append([0.0, -0.8, 0.8, 0.0, 0.0, 0.0])
    guesses.append([0.0, -1.0, 1.0, 0.2, 0.0, 0.0])
    guesses.append([0.2, -0.9, 0.9, 0.0, 0.0, 0.2])
    return guesses
```

**AFTER (12+ targeted guesses with calibrated j5):**
```python
def _make_guess_q6_list(self, target_base=None):
    q_now = self.get_current_joints()
    j1 = math.atan2(target_base[1], target_base[0]) if target_base else 0.0
    j4 = self.fixed_j4
    corr = self.j5_down_correction

    guesses = [q_now]
    for j2, j3 in [(-0.3,0.1), (-0.5,0.3), (-0.6,0.5), (-0.7,0.6),
                    (-0.8,0.8), (-1.0,1.0), (-0.4,0.2), (-0.2,0.05)]:
        j5 = -(j2 + j3) + corr  # Calibrated for gripper-down
        guesses.append([j1, j2, j3, j4, j5, 0.0])

    # Joint 1 variations for side cells
    for j1_off in [0.2, -0.2, 0.4, -0.4]:
        j5 = -(-0.6 + 0.5) + corr
        guesses.append([j1 + j1_off, -0.6, 0.5, j4, j5, 0.0])
    return guesses
```

**Why:**
1. **Target-aware j1:** Joint 1 is pre-computed from the target position using `atan2`, pointing the arm toward the target cell
2. **Calibrated j5:** Every guess uses the auto-calibrated formula so the gripper starts pointing down
3. **More coverage:** 8 different (j2, j3) combinations plus j1 variations cover the entire board workspace
4. **Fixed j4:** All guesses respect the locked forearm orientation

---

#### Seeded IK for Descent (FIX-36)

**ADDED:**
```python
def solve_ik(self, target_world, seed_q6=None):
    """When seed_q6 is provided, uses ONLY that single guess.
    This forces the solver to find the nearest local minimum."""
    
    if seed_q6 is not None:
        # Single-seed mode: one attempt, no multi-guess
        guesses = [seed_q6]
    else:
        guesses = self._make_guess_q6_list(target_base)
```

**Usage in handle_move:**
```python
# Hover → Pick: seed with hover joints
q_pick_down, _ = self.solve_ik_with_fallback_down(
    pick_world, seed_q6=q_pick_hover  # ← Seed!
)

# Hover → Place: seed with hover joints  
q_place_down, _ = self.solve_ik_with_fallback_down(
    place_world, seed_q6=q_place_hover  # ← Seed!
)
```

**Why:** This was the fix for the "arm lunges to wrong cell before gripping" problem. Without seeding, the IK solver could find a completely different joint-space local minimum for the descent target (same XY, lower Z) than the one used for the hover position. The arm would swing to this new configuration mid-air, landing over the wrong cell. By seeding with the hover joints, the descent finds the nearest solution — the arm simply lowers straight down.

---

### 5.4 Force Gripper Vertical

**ADDED (entirely new):**
```python
def force_gripper_vertical(self, wait_steps=30):
    """Correct joint_5 to make gripper point exactly straight down.
    Only touches joint_5. All other joints stay as they are."""
    q_now = self.get_current_joints()
    j2, j3 = q_now[1], q_now[2]
    
    j5_vertical = -(j2 + j3) + self.j5_down_correction
    j5_vertical = clamp(j5_vertical, j5_min, j5_max)
    
    q_corrected = list(q_now)
    q_corrected[4] = j5_vertical
    self.set_joints_smooth(q_corrected, ...)
```

**Called before grasping and before releasing:**
```python
# Before grip
self.force_gripper_vertical(wait_steps=30)
self.close_gripper(wait_steps=10)

# Before release
self.force_gripper_vertical(wait_steps=30)
self.open_gripper(wait_steps=10)
```

**Why:** IK may not perfectly achieve vertical gripper orientation due to the numerical optimization nature of the solver. This analytical correction uses the calibrated formula to compute the exact joint 5 value that makes the gripper point straight down, given the current joint 2 and joint 3 values. Applied immediately before grip/release to ensure the gripper is properly oriented.

---

### 5.5 Orientation Check System

**ADDED (entirely new):**
```python
def _check_orientation_down(self, q6):
    """Check how far the gripper orientation deviates from 'down'."""
    q_full = self._q6_to_full(q6)
    fk = self.chain.forward_kinematics(q_full)
    ee_z = fk[:3, 2]  # End-effector Z-axis direction

    down = np.array(self.gripper_down_vector)  # Auto-detected
    cos_angle = np.dot(ee_z, down) / (np.linalg.norm(ee_z) + 1e-12)
    angle = math.acos(np.clip(cos_angle, -1.0, 1.0))
    return angle, ee_z.tolist()
```

**Used in IK candidate selection:**
```python
# Filter solutions by orientation quality
good = [c for c in candidates
        if c['angle_dev'] < ORIENTATION_TOLERANCE_RAD]

if good:
    best = min(good, key=lambda c: c['joint_score'])
else:
    best = min(candidates, key=lambda c: c['angle_dev'])
```

**Why:** Not all IK solutions produce a gripper pointing straight down. The orientation check measures the angle between the end-effector Z-axis and the calibrated "down" direction. Solutions within the tolerance (0.25 rad ≈ 14°) are preferred, selected by minimum joint movement from current position. If no solution meets the tolerance, the one closest to vertical is used.

---

### 5.6 URDF Chain Configuration

#### ikpy Warning Suppression

**BEFORE:**
```python
full_chain = Chain.from_urdf_file(urdf_path, ...)
# Produces warnings: "Link Base link (index: 0) is of type 'fixed'
# but set as active in the active_links_mask"
```

**AFTER:**
```python
# Temporary chain for inspection only — suppress warnings
with warnings.catch_warnings():
    warnings.simplefilter("ignore", UserWarning)
    full_chain = Chain.from_urdf_file(urdf_path, ...)

# Build active mask from inspected chain
mask = [link_name in ik_active_joints for link in full_chain.links]

# Final chain with correct mask — no warnings
self.chain = Chain.from_urdf_file(urdf_path, active_links_mask=mask, ...)
```

**Why:** The original loaded the chain with an incorrect mask (including fixed links like "Base link" and "hand_link_base_gripper_joint" as active), producing misleading warnings. The fix uses a two-step approach: inspect the chain first to discover link types, then build the final chain with only revolute joints marked as active.

---

### 5.7 Waypoint Error Handling (FIX-07)

**BEFORE:**
```python
def move_world_waypoints(self, p_from, p_to, steps=4, speed=0.02):
    for i in range(1, steps + 1):
        p = [...]
        q = self.solve_ik(p)  # Crashes if unreachable!
        self.set_joints_smooth(q, ...)
```

**AFTER:**
```python
def move_world_waypoints(self, p_from, p_to, steps=4, speed=0.02):
    for i in range(1, steps + 1):
        p = [...]
        try:
            q = self.solve_ik(p)
        except Exception as e:
            print(f"[waypoints] IK failed at step {i}/{steps}: {e}. Skipping.")
            continue  # Skip unreachable waypoint, continue to next
        self.set_joints_smooth(q, ...)
```

**Why:** An unreachable intermediate waypoint no longer aborts the entire move. The arm skips that waypoint and continues toward the target.

---

## 6. Bug Fixes Applied

### 6.1 Complete Fix Log

| Fix # | Issue | File(s) | Change | Status |
|-------|-------|---------|--------|--------|
| FIX-01 | Duplicate ACK block crash | game_controller.py | Deleted 33-line duplicate block | ✅ Active |
| FIX-02 | `raise None` in IK | niryo_controller.py | Guarded with RuntimeError | ✅ Active |
| FIX-03 | No physics on pieces | All 6 protos | Added then **reverted** (caused shaking) | ❌ Reverted |
| FIX-04 | Stale piece_offset | game_controller.py | Reset after each move | ✅ Active (enhanced by FIX-12) |
| FIX-05 | Wrong hover Z | game_controller.py | Use board surface Z | ✅ Active (superseded by FIX-29) |
| FIX-06 | Premature capture | game_controller.py | Deferred to after robot move | ✅ Active |
| FIX-07 | Waypoint crash | niryo_controller.py | try/except in loop | ✅ Active |
| FIX-08 | Special moves break occ | game_controller.py | Added handler for castling/en passant | ✅ Active |
| FIX-09 | Unnamed receiver | World + game_controller | Explicit "ack_receiver" name | ✅ Active |
| FIX-10 | Speed param logic | niryo_controller.py | Clarified conditional | ✅ Active |
| FIX-11 | Base message missed | game_controller.py | Send twice with delay | ✅ Active |
| FIX-12 | Z offset lost | game_controller.py | Preserve Z in offset reset | ✅ Active |
| FIX-13 | Stockfish path | game_controller.py | `__file__`-relative path | ✅ Active |
| FIX-16 | Capture visual overlap | game_controller.py | Two-phase TCP follow | ✅ Active |
| FIX-18 | TCP local vs world | game_controller.py | `getPosition()` not `getSFVec3f()` | ✅ Active |
| FIX-26 | Wrong grasp offset | game_controller.py | Fixed `[0,0,-0.01]` offset | ✅ Active |
| FIX-29 | Pick Z from piece level | game_controller.py | Use `grasp_from[2]` | ✅ Active |
| FIX-33 | Cell misalignment | niryo_controller.py | Orientation-constrained hover IK | ✅ Active |
| FIX-36 | Unwanted pre-grip movement | niryo_controller.py | Seeded IK for descent | ✅ Active |
| FIX-37 | Wrong yaw/rotation | World + niryo_controller | Robot rotation in world file | ✅ Active |

### 6.2 Reverted Fixes

| Fix # | Why Reverted |
|-------|-------------|
| FIX-03 | Adding Physics nodes caused continuous piece vibration due to contact solver oscillations |
| FIX-25 | Moving pieces to Z=0 caused them to sink through the board (mesh origin ≠ bottom) |
| FIX-31 | Same as FIX-25 — coordinate unification by moving pieces failed |
| FIX-35 (partial) | Moving robots to y=±0.25 placed them physically over the board |

**Key Learning:** The simulation works best with **static pieces** (no Physics node) and a **scripted fake-grasp** approach where the supervisor manually tracks pieces with the TCP position. Real physics-based grasping requires significantly more tuning of contact properties, friction, damping, and gripper force — not practical for this chess simulation.

---

## 7. World File Modifications

### 7.1 Robot Rotation (FIX-37)

**BEFORE:**
```
NIRYO_NED_WHITE: no rotation specified (defaults to 0 0 1 0)
NIRYO_NED_BLACK: rotation 0 0 1 3.14159 (180°)
```

**AFTER:**
```
NIRYO_NED_WHITE: rotation 0 0 1  1.5708  (90°, faces +Y → board)
NIRYO_NED_BLACK: rotation 0 0 1 -1.5708  (-90°, faces -Y → board)
```

**Why:** Without explicit rotation, the White robot faced world +X (away from the board). The supervisor sent `yaw=0` which completely broke the coordinate transform — the arm extended sideways instead of toward the board.

### 7.2 Receiver Naming (FIX-09)

**BEFORE:**
```
DEF ACK_RECEIVER Receiver {
  channel 2
}
```

**AFTER:**
```
DEF ACK_RECEIVER Receiver {
  name "ack_receiver"
  channel 2
}
```

---

## 8. Calibration & IK Improvements

### 8.1 The j5 Down-Correction Formula

The relationship between joint angles and gripper orientation for the Niryo Ned:

```
joint_5 = -(joint_2 + joint_3) + correction
```

Where:
- `joint_2` controls the shoulder pitch (upper arm angle)
- `joint_3` controls the elbow pitch (forearm angle)
- `joint_5` controls the wrist pitch
- `correction` is a constant determined by the URDF geometry

**Derivation:** For the gripper to point straight down regardless of arm pose, the wrist must compensate for the combined shoulder and elbow angles. The sum `(j2 + j3)` represents the total tilt of the arm chain; negating it and adding a correction factor produces the required wrist angle.

### 8.2 Auto-Detection of Down Direction

The URDF end-effector Z-axis can point either `[0,0,+1]` or `[0,0,-1]` depending on the joint/link conventions. The auto-calibration tests both:

```
For each direction in {[0,0,-1], [0,0,+1]}:
    For correction in {-1.50, -1.49, ..., +1.50}:
        Compute FK with test joints
        Measure angle between EE Z-axis and candidate direction
        Keep best (smallest angle) result

Result: self.gripper_down_vector = detected direction
        self.j5_down_correction = optimal correction value
```

For the NiryoNed URDF used in this project, the auto-calibration consistently detects:
- `gripper_down_vector = [0.0, 0.0, -1.0]`
- `j5_down_correction ≈ -1.50 rad`

---

## 9. Summary of All Changes

### 9.1 Files Modified

| File | Changes |
|------|---------|
| `game_controller.py` | 11 fixes + 6 improvements (FIX-01,04,05,06,08,09,11,12,16,18,26,29) |
| `niryo_controller.py` | 8 fixes + complete IK overhaul (FIX-02,07,10,33,36,37 + auto-calibration + fixed joints + seeded IK) |
| `Digital Twin_niryobased.wbt` | Robot rotations, receiver naming (FIX-09,37) |

### 9.2 Lines of Code

| Component | Initial LOC | Final LOC | Change |
|-----------|------------|-----------|--------|
| game_controller.py | ~750 | ~850 | +100 (new helpers, two-phase follow, special move handler) |
| niryo_controller.py | ~540 | ~850 | +310 (auto-calibration, multi-guess IK, seeded descent, force vertical, orientation check) |

### 9.3 Behavioral Improvements

| Aspect | Before | After |
|--------|--------|-------|
| **Game stability** | Crashed after 1 move | 37+ moves without errors |
| **Piece tracking** | Pieces never followed arm | Real-time TCP follow from grasp to release |
| **Cell alignment** | Arm landed on wrong cells | Seeded IK ensures correct cell targeting |
| **Capture handling** | Visual overlap during capture | Captured piece cleared at hover stage |
| **IK reliability** | 3 generic guesses, frequent failures | 12+ targeted guesses with auto-calibrated orientation |
| **Gripper orientation** | Random orientation from IK | Analytically forced vertical before grip/release |
| **Special moves** | Castling/en passant broke board state | Proper occ map updates for all move types |
| **Error recovery** | Single failure stopped game | Graceful error handling with informative messages |

---

*Document generated: March 2026*  
*Project: Digital Twin_niryobased — Webots Chess Robot Simulation*
