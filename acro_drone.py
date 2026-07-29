
Aerobatic mission for ArduCopter SITL (Gazebo Harmonic + ArduPilot plugin, vehicle: iris_stanoff)

Sequence:
  1. Arm + takeoff to 20 m         (GUIDED)
  2. Flip x2 ("salto")             (ALT_HOLD -> FLIP -> ALT_HOLD -> GUIDED)
  3. Climb another 20 m -> 40 m    (GUIDED)
  4. Flip x1 ("jungkir balik")     (ALT_HOLD -> FLIP -> ALT_HOLD -> GUIDED)
  5. Land

IMPORTANT — why we cannot flip directly from GUIDED:
ArduCopter's FLIP mode can only be *entered* from STABILIZE, ACRO or ALT_HOLD
(see ArduCopter/mode_flip.cpp -> ModeFlip::init() -> allows_flip()). GUIDED is
not in that list, so this script always steps through ALT_HOLD as a bridge:
GUIDED (climb) -> ALT_HOLD (bridge) -> FLIP (maneuver, auto-returns to ALT_HOLD
when done) -> GUIDED (resume autonomous control).

Because FLIP/ALT_HOLD are pilot-input modes, the script sends neutral
RC_CHANNELS_OVERRIDE values (mid-stick = 1500) before switching, so the
vehicle behaves as if a pilot were quietly holding a centered hover stick.
With roll centered, ArduPilot's default flip direction is roll-right.
"""

import time
import math
import threading
from pymavlink import mavutil

# ============================== CONFIG ===============================
# Adjust to whatever your SITL / MAVProxy output line actually is, e.g.:
#   'udp:127.0.0.1:14551'   (typical MAVProxy --out endpoint)
#   'tcp:127.0.0.1:5762'    (typical direct SITL tcp port)
CONNECTION_STRING = 'tcp:127.0.0.1:5762'

TARGET_SYSTEM = 1
TARGET_COMPONENT = 1

CLIMB_1_ALT_M = 20.0     # first climb target (relative to home)
CLIMB_2_ALT_M = 40.0     # second climb target = +20 m more
ALT_TOLERANCE_M = 1.0    # acceptable altitude error to consider "reached"
CLIMB_TIMEOUT_S = 60.0

FLIP_ENTRY_MODE = 'ALT_HOLD'   # FLIP may only be entered from STABILIZE / ACRO / ALT_HOLD
FLIP_COUNT_STAGE_1 = 2          # "salto sebanyak 2 kali"
FLIP_COUNT_STAGE_2 = 1          # "jungkir balik"
FLIP_TIMEOUT_S = 5.0             # ArduPilot's internal FLIP_TIMEOUT_MS is 2.5s; this adds margin
FLIP_SETTLE_S = 3.0              # pause after each flip so attitude/altitude settle

HOVER_RC_PWM = 1500       # neutral stick (roll/pitch/yaw centered, throttle = zero climb rate)
RC_OVERRIDE_IGNORE = 65535

MODE_CHANGE_TIMEOUT_S = 10.0
# =======================================================================


class TelemetryState:
    """Shared vehicle state. Only telemetry_reader() writes to this; everything
    else only reads it, always through the lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.relative_alt_m = None
        self.custom_mode = None
        self.armed = False

    def update_from_global_position(self, msg):
        with self.lock:
            self.relative_alt_m = msg.relative_alt / 1000.0

    def update_from_heartbeat(self, msg):
        with self.lock:
            self.custom_mode = msg.custom_mode
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def snapshot(self):
        with self.lock:
            return {
                'relative_alt_m': self.relative_alt_m,
                'custom_mode': self.custom_mode,
                'armed': self.armed,
            }


def telemetry_reader(master, state, stop_event):
    """Single thread that owns recv_match(); nothing else should call it."""
    while not stop_event.is_set():
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        msg_type = msg.get_type()
        if msg_type == 'GLOBAL_POSITION_INT':
            state.update_from_global_position(msg)
        elif msg_type == 'HEARTBEAT':
            state.update_from_heartbeat(msg)


def mode_id(master, mode_name):
    mapping = master.mode_mapping()
    if mapping is None or mode_name not in mapping:
        raise ValueError(f"Unknown/unsupported mode '{mode_name}'")
    return mapping[mode_name]


def set_mode(master, state, mode_name, timeout=MODE_CHANGE_TIMEOUT_S):
    """Request a mode change and block until HEARTBEAT confirms it actually happened."""
    target = mode_id(master, mode_name)
    master.mav.set_mode_send(
        TARGET_SYSTEM,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        target
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        if state.snapshot()['custom_mode'] == target:
            print(f"[MODE] Confirmed: {mode_name}")
            return
        time.sleep(0.1)
    raise TimeoutError(f"Timed out waiting for mode {mode_name}")


def wait_flip_complete(master, state, timeout=FLIP_TIMEOUT_S):
    """FLIP auto-reverts to the previous mode once the maneuver finishes/aborts.
    We detect completion as: mode is no longer FLIP."""
    flip_id = mode_id(master, 'FLIP')
    deadline = time.time() + timeout
    while state.snapshot()['custom_mode'] == flip_id:
        if time.time() > deadline:
            raise TimeoutError("FLIP never completed (vehicle stuck in FLIP mode)")
        time.sleep(0.05)


def rc_override(master, roll=RC_OVERRIDE_IGNORE, pitch=RC_OVERRIDE_IGNORE,
                 throttle=RC_OVERRIDE_IGNORE, yaw=RC_OVERRIDE_IGNORE):
    master.mav.rc_channels_override_send(
        TARGET_SYSTEM, TARGET_COMPONENT,
        roll, pitch, throttle, yaw,
        RC_OVERRIDE_IGNORE, RC_OVERRIDE_IGNORE, RC_OVERRIDE_IGNORE, RC_OVERRIDE_IGNORE
    )


def clear_rc_override(master):
    rc_override(master)  # all channels = ignore -> release manual override


def arm(master, state, timeout=10.0):
    master.mav.command_long_send(
        TARGET_SYSTEM, TARGET_COMPONENT,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        if state.snapshot()['armed']:
            print("[ARM] Armed")
            return
        time.sleep(0.1)
    raise TimeoutError("Arming failed / timed out")


def wait_for_altitude(state, target_alt_m, timeout=CLIMB_TIMEOUT_S):
    deadline = time.time() + timeout
    while time.time() < deadline:
        alt = state.snapshot()['relative_alt_m']
        if alt is not None and abs(alt - target_alt_m) <= ALT_TOLERANCE_M:
            print(f"[ALT] Reached {alt:.1f} m (target {target_alt_m:.1f} m)")
            return
        time.sleep(0.2)
    raise TimeoutError(f"Never reached {target_alt_m} m (timeout)")


def takeoff(master, state, target_alt_m):
    master.mav.command_long_send(
        TARGET_SYSTEM, TARGET_COMPONENT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_alt_m
    )
    wait_for_altitude(state, target_alt_m)


def climb_to(master, state, target_alt_m, timeout=CLIMB_TIMEOUT_S):
    """Hold a stationary GUIDED position target directly above the takeoff point,
    climbing/descending until the target altitude is reached. Must already be
    in GUIDED mode."""
    type_mask = 0b0000_1101_1111_1000  # use position x,y,z; ignore vel/accel/yaw/yaw_rate
    deadline = time.time() + timeout
    while time.time() < deadline:
        master.mav.set_position_target_local_ned_send(
            0, TARGET_SYSTEM, TARGET_COMPONENT,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, -target_alt_m,   # x, y, z (NED: negative z = up)
            0, 0, 0,               # vx, vy, vz (ignored)
            0, 0, 0,               # ax, ay, az (ignored)
            0, 0                   # yaw, yaw_rate (ignored)
        )
        alt = state.snapshot()['relative_alt_m']
        if alt is not None and abs(alt - target_alt_m) <= ALT_TOLERANCE_M:
            print(f"[CLIMB] Reached {alt:.1f} m")
            return
        time.sleep(0.2)
    raise TimeoutError(f"Climb to {target_alt_m} m timed out")


def do_flip(master, state, restore_altitude_m=None):
    """One full FLIP cycle: neutral sticks -> ALT_HOLD -> FLIP -> (auto back to
    ALT_HOLD) -> release override -> back to GUIDED -> optionally re-climb to
    restore_altitude_m to undo any altitude lost during the maneuver."""
    print("[FLIP] Preparing (neutral stick override)...")
    rc_override(master, roll=HOVER_RC_PWM, pitch=HOVER_RC_PWM,
                throttle=HOVER_RC_PWM, yaw=HOVER_RC_PWM)
    time.sleep(0.5)  # let ALT_HOLD settle on the neutral stick before flipping

    set_mode(master, state, FLIP_ENTRY_MODE)
    time.sleep(0.5)

    print("[FLIP] Triggering FLIP")
    set_mode(master, state, 'FLIP')
    wait_flip_complete(master, state)
    print("[FLIP] Completed, vehicle returned to previous mode")

    time.sleep(FLIP_SETTLE_S)
    clear_rc_override(master)
    set_mode(master, state, 'GUIDED')

    if restore_altitude_m is not None:
        climb_to(master, state, restore_altitude_m)


def land(master, state, timeout=60.0):
    set_mode(master, state, 'LAND')
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not state.snapshot()['armed']:
            print("[LAND] Disarmed, landed.")
            return
        time.sleep(0.5)
    raise TimeoutError("Landing timed out")


def main():
    master = mavutil.mavlink_connection(CONNECTION_STRING)
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from system {master.target_system}, component {master.target_component}")

    state = TelemetryState()
    stop_event = threading.Event()
    reader = threading.Thread(target=telemetry_reader, args=(master, state, stop_event), daemon=True)
    reader.start()

    master.mav.request_data_stream_send(
        TARGET_SYSTEM, TARGET_COMPONENT,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )

    try:
        set_mode(master, state, 'GUIDED')
        arm(master, state)

        print(f"=== Stage 1: takeoff to {CLIMB_1_ALT_M} m ===")
        takeoff(master, state, CLIMB_1_ALT_M)
        time.sleep(2)  # settle before acrobatics

        print(f"=== Stage 2: {FLIP_COUNT_STAGE_1}x salto ===")
        for i in range(FLIP_COUNT_STAGE_1):
            print(f"--- Flip {i + 1}/{FLIP_COUNT_STAGE_1} ---")
            do_flip(master, state, restore_altitude_m=CLIMB_1_ALT_M)

        print(f"=== Stage 3: climb to {CLIMB_2_ALT_M} m ===")
        climb_to(master, state, CLIMB_2_ALT_M)
        time.sleep(2)

        print(f"=== Stage 4: jungkir balik ({FLIP_COUNT_STAGE_2}x) ===")
        for i in range(FLIP_COUNT_STAGE_2):
            do_flip(master, state, restore_altitude_m=CLIMB_2_ALT_M)

        print("=== Stage 5: landing ===")
        land(master, state)

    finally:
        clear_rc_override(master)
        stop_event.set()
        reader.join(timeout=2)


if __name__ == '__main__':
    main()
