from flask import Flask, jsonify, request, Response
import threading
import time
import random
import math
import socket
import struct
from collections import deque

try:
    import cv2
except ImportError:
    cv2 = None

app = Flask(__name__)

SIMULATION_MODE = False
USE_CAMERA = True
CAMERA_INDEX = 0

# OpenCV is used only by AUTONOMOUS mode to choose the visually clearer side.
# Ultrasonic distance remains the authority for actual obstacle proximity.
VISION_UPDATE_INTERVAL_S = 0.10
VISION_STALE_S = 0.75
VISION_TIE_MARGIN = 0.015

AUTO_OBSTACLE_TRIGGER_CM = 32.0
AUTO_BACKUP_S = 0.35
AUTO_TURN_S = 0.55
AUTO_BACKUP_SPEED = 25
AUTO_TURN_SPEED = 38

# Linux SocketCAN interface. The interface must already be configured at 500 kbit/s.
CAN_ENABLED = True
CAN_INTERFACE = "can0"

# ============================================================
# CAN protocol
# ============================================================
# Existing, already implemented Battery ECU frame:
CAN_ID_BATTERY_STATUS = 0x100

# Sensor ECU telemetry frames used by this integration build:
CAN_ID_SENSOR_STATUS = 0x200      # range + servo + RFID/status
CAN_ID_SENSOR_ACCEL = 0x201       # ax/ay/az
CAN_ID_SENSOR_GYRO = 0x202        # gx/gy/gz
CAN_ID_SENSOR_ORIENTATION = 0x203 # roll/pitch/yaw
CAN_ID_SENSOR_SCAN = 0x204        # left/center/right scan ranges

# Motor ECU status/heartbeat:
CAN_ID_MOTOR_STATUS = 0x300

# Raspberry Pi supervisory drive command. Sensor ECU also listens to this
# for OLED drive state and MAX7219 direction arrows.
CAN_ID_PI_DRIVE_COMMAND = 0x400

# Browser -> Sensor ECU lock/unlock command.
CAN_ID_PI_LOCK_COMMAND = 0x401

# Linux CAN frame constants.
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_SFF_MASK = 0x000007FF

CONTROL_TIMEOUT_S = 0.50
ECU_TIMEOUT_S = 0.50
CONTROL_LOOP_INTERVAL_S = 0.02

WALL_SLOW_CM = 40.0
WALL_STOP_CM = 20.0
WALL_ESTOP_CM = 12.0

CURRENT_WARN_A = 4.75
CURRENT_LIMIT_A = 5.25
CURRENT_SHUTDOWN_A = 5.50
OVERCURRENT_COOLDOWN_S = 1.5

DEFAULT_SPEED_LIMIT = 50

state_lock = threading.RLock()

vehicle = {
    "battery": {
        "voltage": 8.20,
        "current": 0.35,
        "temperature": 27.0,
        "online": True,
        "last_seen": 0.0,
    },
    "sensor": {
        "distance_cm": 120.0,
        "servo_angle_deg": 0.0,
        "imu": {
            "ax": 0.0,
            "ay": 0.0,
            "az": 1.0,
            "gx": 0.0,
            "gy": 0.0,
            "gz": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "rfid_unlocked": False,
        "scan": {
            "left_cm": 100.0,
            "center_cm": 120.0,
            "right_cm": 95.0,
        },
        "online": True,
        "last_seen": 0.0,
    },
    "motor": {
        "requested_left": 0,
        "requested_right": 0,
        "actual_left": 0,
        "actual_right": 0,
        "enabled": True,
        "online": True,
        "last_seen": 0.0,
    },
    "system": {
        "mode": "ASSISTED",
        "speed_limit": DEFAULT_SPEED_LIMIT,
        "estop": False,
        "overcurrent_fault": False,
        "overcurrent_until": 0.0,
        "wall_intervention": False,
        "current_intervention": False,
        "last_control_time": time.time(),
        "control_connected": True,
        # Browser command freshness protection. A newly loaded dashboard gets a
        # newer client_started_ms value and starts its sequence counter at 0.
        "control_client_started_ms": 0,
        "last_drive_sequence": -1,
        "simulation": SIMULATION_MODE,
        "camera_enabled": USE_CAMERA,
    },
}

event_log = deque(maxlen=100)


def log_event(message):
    stamp = time.strftime("%H:%M:%S")
    entry = f"{stamp}  {message}"
    with state_lock:
        event_log.appendleft(entry)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def now():
    return time.time()


# ============================================================
# Safety manager
# ============================================================

def apply_safety(requested_left, requested_right, wall_mode="full"):
    with state_lock:
        s = vehicle["system"]
        battery = vehicle["battery"]
        sensor = vehicle["sensor"]

        left = int(clamp(requested_left, -100, 100))
        right = int(clamp(requested_right, -100, 100))

        s["wall_intervention"] = False
        s["current_intervention"] = False

        if s["estop"]:
            return 0, 0

        if not sensor["rfid_unlocked"]:
            return 0, 0

        if now() - battery["last_seen"] > ECU_TIMEOUT_S:
            return 0, 0
        if now() - sensor["last_seen"] > ECU_TIMEOUT_S:
            return 0, 0

        if s["overcurrent_fault"]:
            if now() < s["overcurrent_until"]:
                return 0, 0
            s["overcurrent_fault"] = False
            log_event("OVERCURRENT cooldown complete")

        distance = sensor["distance_cm"]
        forward_requested = left > 0 or right > 0

        if wall_mode != "off" and forward_requested:
            if distance < WALL_ESTOP_CM:
                s["wall_intervention"] = True
                return 0, 0

            if distance < WALL_STOP_CM:
                s["wall_intervention"] = True
                if left > 0:
                    left = 0
                if right > 0:
                    right = 0

            elif wall_mode == "full" and distance < WALL_SLOW_CM:
                s["wall_intervention"] = True
                scale = (distance - WALL_STOP_CM) / (WALL_SLOW_CM - WALL_STOP_CM)
                scale = clamp(scale, 0.0, 1.0)
                if left > 0:
                    left = int(left * scale)
                if right > 0:
                    right = int(right * scale)

        current = battery["current"]

        if current > CURRENT_SHUTDOWN_A:
            s["current_intervention"] = True
            s["overcurrent_fault"] = True
            s["overcurrent_until"] = now() + OVERCURRENT_COOLDOWN_S
            log_event(f"OVERCURRENT shutdown at {current:.2f} A")
            return 0, 0

        if current > CURRENT_LIMIT_A:
            s["current_intervention"] = True
            left = int(left * 0.45)
            right = int(right * 0.45)

        elif current > CURRENT_WARN_A:
            s["current_intervention"] = True
            left = int(left * 0.75)
            right = int(right * 0.75)

        return left, right


# ============================================================
# Drive controllers
# ============================================================

def manual_requested_output():
    with state_lock:
        return vehicle["motor"]["requested_left"], vehicle["motor"]["requested_right"]

auto_recovery_phase = "DRIVE"
auto_phase_until = 0.0
auto_turn_direction = "RIGHT"

vision_left_obstruction = 0.0
vision_right_obstruction = 0.0
vision_last_seen = 0.0
vision_available = False


def autonomous_requested_output():
    global auto_recovery_phase, auto_phase_until, auto_turn_direction

    with state_lock:
        distance = vehicle["sensor"]["distance_cm"]
        scan = dict(vehicle["sensor"]["scan"])
        limit = vehicle["system"]["speed_limit"]

        left_obstruction = vision_left_obstruction
        right_obstruction = vision_right_obstruction
        vision_age = now() - vision_last_seen
        camera_choice_available = vision_available and vision_age <= VISION_STALE_S

    cruise = min(limit, 45)
    turn_speed = min(limit, AUTO_TURN_SPEED)
    backup_speed = min(limit, AUTO_BACKUP_SPEED)
    t = now()

    # Once an obstacle is encountered, back straight up briefly before turning
    if auto_recovery_phase == "BACKUP":
        if t < auto_phase_until:
            return -backup_speed, -backup_speed

        auto_recovery_phase = "TURN"
        auto_phase_until = t + AUTO_TURN_S

    if auto_recovery_phase == "TURN":
        if t < auto_phase_until:
            if auto_turn_direction == "LEFT":
                return -turn_speed, turn_speed
            return turn_speed, -turn_speed

        auto_recovery_phase = "DRIVE"

    # Normal autonomous cruising.
    if distance > 50:
        return cruise, cruise

    if AUTO_OBSTACLE_TRIGGER_CM < distance <= 50:
        return int(cruise * 0.50), int(cruise * 0.50)

    # Obstacle close enough to require a recovery maneuver.
    # Prefer OpenCV when it has a recent, decisive left/right result.
    if camera_choice_available and abs(left_obstruction - right_obstruction) >= VISION_TIE_MARGIN:
        # Higher obstruction score means that half of the image is visually
        # busier, so turn toward the half with the LOWER obstruction score.
        if left_obstruction > right_obstruction:
            auto_turn_direction = "RIGHT"
        else:
            auto_turn_direction = "LEFT"
        choice_source = "camera"
    else:
        # If the camera is unavailable or both halves look almost equal, retain
        # the existing ultrasonic left/right scan as a deterministic fallback.
        if scan["left_cm"] > scan["right_cm"]:
            auto_turn_direction = "LEFT"
        else:
            auto_turn_direction = "RIGHT"
        choice_source = "ultrasonic"

    auto_recovery_phase = "BACKUP"
    auto_phase_until = t + AUTO_BACKUP_S

    if choice_source == "camera":
        log_event(
            f"AUTO obstacle -> backup then {auto_turn_direction} "
            f"(vision L {left_obstruction:.3f} / R {right_obstruction:.3f})"
        )
    else:
        log_event(
            f"AUTO obstacle -> backup then {auto_turn_direction} "
            f"(ultrasonic fallback)"
        )

    return -backup_speed, -backup_speed


# ============================================================
# Simulation
# ============================================================

def simulation_loop():
    obstacle_distance = 120.0
    battery_voltage = 8.20
    battery_temp = 27.0
    yaw = 0.0
    sweep = -70
    sweep_dir = 1

    while True:
        time.sleep(0.05)

        with state_lock:
            actual_left = vehicle["motor"]["actual_left"]
            actual_right = vehicle["motor"]["actual_right"]

        avg_forward = max(0.0, (actual_left + actual_right) / 2.0)
        turning = actual_right - actual_left

        if avg_forward > 0:
            obstacle_distance -= avg_forward * 0.015
        elif actual_left < 0 and actual_right < 0:
            obstacle_distance += abs((actual_left + actual_right) / 2.0) * 0.02

        if abs(turning) > 35:
            obstacle_distance += abs(turning) * 0.02

        obstacle_distance = clamp(obstacle_distance, 8, 180)

        effort = (abs(actual_left) + abs(actual_right)) / 200.0
        simulated_current = 0.35 + effort * 4.2
        simulated_current += (abs(actual_left - actual_right) / 200.0) * 1.4
        simulated_current += random.uniform(-0.10, 0.10)
        simulated_current = max(0.25, simulated_current)

        battery_voltage = 8.20 - simulated_current * 0.035 + random.uniform(-0.01, 0.01)
        target_temp = 27.0 + simulated_current * 1.5
        battery_temp += (target_temp - battery_temp) * 0.01

        pitch = clamp((avg_forward / 100.0) * -5.0 + random.uniform(-0.8, 0.8), -10, 10)
        roll = clamp((turning / 100.0) * 7.0 + random.uniform(-0.8, 0.8), -18, 18)
        yaw = (yaw + turning * 0.05) % 360

        ax = math.sin(math.radians(roll)) * 0.15 + random.uniform(-0.02, 0.02)
        ay = math.sin(math.radians(pitch)) * 0.15 + random.uniform(-0.02, 0.02)
        az = 1.0 + random.uniform(-0.02, 0.02)
        gz = turning * 0.8 + random.uniform(-1.0, 1.0)

        sweep += sweep_dir * 5
        if sweep >= 70:
            sweep = 70
            sweep_dir = -1
        elif sweep <= -70:
            sweep = -70
            sweep_dir = 1

        left_scan = clamp(obstacle_distance + 28 + random.uniform(-8, 28), 8, 200)
        right_scan = clamp(obstacle_distance + 24 + random.uniform(-8, 28), 8, 200)

        with state_lock:
            vehicle["battery"]["voltage"] = round(battery_voltage, 2)
            vehicle["battery"]["current"] = round(simulated_current, 2)
            vehicle["battery"]["temperature"] = round(battery_temp, 1)
            vehicle["battery"]["last_seen"] = now()

            vehicle["sensor"]["distance_cm"] = round(obstacle_distance, 1)
            vehicle["sensor"]["servo_angle_deg"] = sweep
            vehicle["sensor"]["imu"] = {
                "ax": round(ax, 3),
                "ay": round(ay, 3),
                "az": round(az, 3),
                "gx": 0.0,
                "gy": 0.0,
                "gz": round(gz, 2),
                "roll": round(roll, 1),
                "pitch": round(pitch, 1),
                "yaw": round(yaw, 1),
            }
            vehicle["sensor"]["scan"] = {
                "left_cm": round(left_scan, 1),
                "center_cm": round(obstacle_distance, 1),
                "right_cm": round(right_scan, 1),
            }
            vehicle["sensor"]["last_seen"] = now()
            vehicle["motor"]["last_seen"] = now()



# ============================================================
# SocketCAN
# ============================================================

can_socket = None
can_socket_lock = threading.Lock()
can_tx_counter = 0
can_lock_tx_counter = 0


def initialize_can():
    """Open the already-configured Linux SocketCAN interface."""
    global can_socket

    if not CAN_ENABLED:
        return False

    try:
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

        # Give SocketCAN plenty of receive headroom for the Battery, Sensor,
        # and Motor ECU telemetry streams. This helps prevent Linux RX drops
        # during short scheduling hiccups on the Pi.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)

        sock.bind((CAN_INTERFACE,))
        sock.settimeout(0.20)
        can_socket = sock
        log_event(f"CAN ONLINE -> {CAN_INTERFACE} @ 500 kbps")
        return True
    except OSError as exc:
        can_socket = None
        log_event(f"CAN ERROR -> {exc}")
        return False


def decode_i8(value):
    return value - 256 if value > 127 else value


def encode_i8(value):
    return int(clamp(int(value), -128, 127)) & 0xFF


def u16_le(data, offset):
    return int.from_bytes(data[offset:offset + 2], "little", signed=False)


def i16_le(data, offset):
    return int.from_bytes(data[offset:offset + 2], "little", signed=True)


def send_can_frame(identifier, payload):
    """Send one standard 11-bit CAN frame."""
    if not CAN_ENABLED or can_socket is None:
        return False

    payload = bytes(payload[:8])
    padded = payload.ljust(8, b"\x00")
    frame = struct.pack("=IB3x8s", identifier & CAN_SFF_MASK, len(payload), padded)

    try:
        with can_socket_lock:
            can_socket.send(frame)
        return True
    except OSError as exc:
        log_event(f"CAN TX ERROR -> {exc}")
        return False


def send_drive_can_command(left, right):
    """
    Pi -> Motor ECU / supervisory command, ID 0x400

    byte 0: left motor command, signed int8, -100..100 %
    byte 1: right motor command, signed int8, -100..100 %
    byte 2: mode, 0=MANUAL, 1=ASSISTED, 2=AUTONOMOUS
    byte 3: speed limit, uint8, 0..100 %
    byte 4: flags
            bit0 = E-stop active
            bit1 = vehicle/RFID permission unlocked
    byte 5: safety flags
            bit0 = wall intervention active
            bit1 = current intervention active
            bit2 = overcurrent fault active
    byte 6-7: rolling uint16 command counter, little-endian
    """
    global can_tx_counter

    with state_lock:
        system = vehicle["system"]
        sensor = vehicle["sensor"]

        mode_code = {
            "MANUAL": 0,
            "ASSISTED": 1,
            "AUTONOMOUS": 2,
        }.get(system["mode"], 1)

        flags = 0
        if system["estop"]:
            flags |= 0x01
        if sensor["rfid_unlocked"]:
            flags |= 0x02

        safety_flags = 0
        if system["wall_intervention"]:
            safety_flags |= 0x01
        if system["current_intervention"]:
            safety_flags |= 0x02
        if system["overcurrent_fault"]:
            safety_flags |= 0x04

        speed_limit = int(clamp(system["speed_limit"], 0, 100))

    counter = can_tx_counter & 0xFFFF
    payload = bytes([
        encode_i8(left),
        encode_i8(right),
        mode_code,
        speed_limit,
        flags,
        safety_flags,
        counter & 0xFF,
        (counter >> 8) & 0xFF,
    ])

    if send_can_frame(CAN_ID_PI_DRIVE_COMMAND, payload):
        can_tx_counter = (can_tx_counter + 1) & 0xFFFF



def send_lock_can_command(unlocked):
    """
    Pi -> Sensor ECU vehicle lock command, ID 0x401

    byte 0: desired state, 0=LOCKED, 1=UNLOCKED
    byte 1: command source, 1=browser
    bytes 2-5: reserved
    bytes 6-7: rolling uint16 counter, little-endian

    Sensor ECU applies the command and confirms the resulting state through
    CAN 0x200. The Pi uses only that confirmed state for drive permission.
    """
    global can_lock_tx_counter

    counter = can_lock_tx_counter & 0xFFFF
    payload = bytes([
        1 if unlocked else 0,
        1,
        0,
        0,
        0,
        0,
        counter & 0xFF,
        (counter >> 8) & 0xFF,
    ])

    if send_can_frame(CAN_ID_PI_LOCK_COMMAND, payload):
        can_lock_tx_counter = (can_lock_tx_counter + 1) & 0xFFFF
        return True

    return False

def process_can_frame(identifier, data):
    """Decode one received ECU frame into the dashboard state."""
    dlc = len(data)
    timestamp = now()

    # --------------------------------------------------------
    # Battery ECU, ID 0x100
    # Existing firmware format, preserved exactly.
    # --------------------------------------------------------
    if identifier == CAN_ID_BATTERY_STATUS and dlc == 8:
        voltage_mv = u16_le(data, 0)
        temperature_centi_c = i16_le(data, 2)
        current_ma = i16_le(data, 4)

        voltage_v = None if voltage_mv == 0xFFFF else voltage_mv / 1000.0
        temperature_c = None if temperature_centi_c == -32768 else temperature_centi_c / 100.0
        current_a = None if current_ma == -32768 else current_ma / 1000.0

        with state_lock:
            if voltage_v is not None:
                vehicle["battery"]["voltage"] = voltage_v
            if temperature_c is not None:
                vehicle["battery"]["temperature"] = temperature_c
            if current_a is not None:
                vehicle["battery"]["current"] = current_a

            vehicle["battery"]["online"] = True
            vehicle["battery"]["last_seen"] = timestamp
        return

    # --------------------------------------------------------
    # Sensor status, ID 0x200
    # bytes 0-1: forward distance in mm, uint16
    # bytes 2-3: servo angle in 0.1 deg, int16
    # byte 4: flags, bit0 = RFID/unlock state
    # byte 5: sensor status
    #         bit0 = center ultrasonic valid
    #         bit1 = IMU connected
    #         bit2 = servo scan enabled
    #         bit3 = Sensor ECU CAN initialized
    #         bit4 = MFRC522 RFID reader connected
    #         bit7 = Sensor ECU lock state is authoritative
    # bytes 6-7: rolling counter
    # --------------------------------------------------------
    if identifier == CAN_ID_SENSOR_STATUS and dlc == 8:
        distance_mm = u16_le(data, 0)
        servo_deci_deg = i16_le(data, 2)
        flags = data[4]
        sensor_status = data[5]
        rfid_source_valid = bool(sensor_status & 0x80)

        with state_lock:
            if distance_mm != 0xFFFF:
                vehicle["sensor"]["distance_cm"] = distance_mm / 10.0

            if servo_deci_deg != -32768:
                vehicle["sensor"]["servo_angle_deg"] = servo_deci_deg / 10.0

            # Sensor ECU is authoritative for vehicle lock state.
            # RFID and browser 0x401 commands both change the same state.
            if rfid_source_valid:
                old_rfid_state = vehicle["sensor"]["rfid_unlocked"]
                new_rfid_state = bool(flags & 0x01)
                vehicle["sensor"]["rfid_unlocked"] = new_rfid_state

                if old_rfid_state != new_rfid_state:
                    log_event(
                        "VEHICLE -> UNLOCKED" if new_rfid_state else "VEHICLE -> LOCKED"
                    )

            vehicle["sensor"]["online"] = True
            vehicle["sensor"]["last_seen"] = timestamp
        return

    # --------------------------------------------------------
    # Sensor acceleration, ID 0x201
    # ax/ay/az = signed int16 in milli-g, bytes 0-5
    # bytes 6-7 = rolling counter
    # --------------------------------------------------------
    if identifier == CAN_ID_SENSOR_ACCEL and dlc == 8:
        ax = i16_le(data, 0) / 1000.0
        ay = i16_le(data, 2) / 1000.0
        az = i16_le(data, 4) / 1000.0

        with state_lock:
            vehicle["sensor"]["imu"]["ax"] = ax
            vehicle["sensor"]["imu"]["ay"] = ay
            vehicle["sensor"]["imu"]["az"] = az
            vehicle["sensor"]["online"] = True
            vehicle["sensor"]["last_seen"] = timestamp
        return

    # --------------------------------------------------------
    # Sensor gyro, ID 0x202
    # gx/gy/gz = signed int16 in 0.01 deg/s, bytes 0-5
    # bytes 6-7 = rolling counter
    # --------------------------------------------------------
    if identifier == CAN_ID_SENSOR_GYRO and dlc == 8:
        gx = i16_le(data, 0) / 100.0
        gy = i16_le(data, 2) / 100.0
        gz = i16_le(data, 4) / 100.0

        with state_lock:
            vehicle["sensor"]["imu"]["gx"] = gx
            vehicle["sensor"]["imu"]["gy"] = gy
            vehicle["sensor"]["imu"]["gz"] = gz
            vehicle["sensor"]["online"] = True
            vehicle["sensor"]["last_seen"] = timestamp
        return

    # --------------------------------------------------------
    # Sensor orientation, ID 0x203
    # roll/pitch/yaw = signed int16 in 0.01 deg, bytes 0-5
    # bytes 6-7 = rolling counter
    # --------------------------------------------------------
    if identifier == CAN_ID_SENSOR_ORIENTATION and dlc == 8:
        roll = i16_le(data, 0) / 100.0
        pitch = i16_le(data, 2) / 100.0
        yaw = i16_le(data, 4) / 100.0

        with state_lock:
            vehicle["sensor"]["imu"]["roll"] = roll
            vehicle["sensor"]["imu"]["pitch"] = pitch
            vehicle["sensor"]["imu"]["yaw"] = yaw % 360.0
            vehicle["sensor"]["online"] = True
            vehicle["sensor"]["last_seen"] = timestamp
        return

    # --------------------------------------------------------
    # Sensor scan, ID 0x204
    # left/center/right range = uint16 mm, bytes 0-5
    # bytes 6-7 = rolling counter
    # --------------------------------------------------------
    if identifier == CAN_ID_SENSOR_SCAN and dlc == 8:
        left_mm = u16_le(data, 0)
        center_mm = u16_le(data, 2)
        right_mm = u16_le(data, 4)

        with state_lock:
            if left_mm != 0xFFFF:
                vehicle["sensor"]["scan"]["left_cm"] = left_mm / 10.0
            if center_mm != 0xFFFF:
                vehicle["sensor"]["scan"]["center_cm"] = center_mm / 10.0
            if right_mm != 0xFFFF:
                vehicle["sensor"]["scan"]["right_cm"] = right_mm / 10.0

            vehicle["sensor"]["online"] = True
            vehicle["sensor"]["last_seen"] = timestamp
        return

    # --------------------------------------------------------
    # Motor ECU status, ID 0x300
    # byte 0: actual left output, signed int8 %
    # byte 1: actual right output, signed int8 %
    # byte 2: enabled, 0/1
    # byte 3: motor fault flags
    # bytes 4-5: rolling counter
    # bytes 6-7: reserved
    # --------------------------------------------------------
    if identifier == CAN_ID_MOTOR_STATUS and dlc >= 4:
        actual_left = decode_i8(data[0])
        actual_right = decode_i8(data[1])
        enabled = bool(data[2])

        with state_lock:
            vehicle["motor"]["actual_left"] = int(clamp(actual_left, -100, 100))
            vehicle["motor"]["actual_right"] = int(clamp(actual_right, -100, 100))
            vehicle["motor"]["enabled"] = enabled
            vehicle["motor"]["online"] = True
            vehicle["motor"]["last_seen"] = timestamp
        return


def can_receive_loop():
    """Continuously receive SocketCAN frames without blocking Flask."""
    while True:
        if can_socket is None:
            time.sleep(0.25)
            continue

        try:
            frame = can_socket.recv(16)
        except socket.timeout:
            continue
        except OSError as exc:
            log_event(f"CAN RX ERROR -> {exc}")
            time.sleep(0.25)
            continue

        if len(frame) != 16:
            continue

        raw_id, dlc, payload = struct.unpack("=IB3x8s", frame)

        if raw_id & CAN_EFF_FLAG:
            continue
        if raw_id & CAN_RTR_FLAG:
            continue

        identifier = raw_id & CAN_SFF_MASK
        process_can_frame(identifier, payload[:min(dlc, 8)])



def control_loop():
    while True:
        time.sleep(CONTROL_LOOP_INTERVAL_S)

        with state_lock:
            mode = vehicle["system"]["mode"]
            last_control = vehicle["system"]["last_control_time"]

        if mode in ("MANUAL", "ASSISTED"):
            if now() - last_control > CONTROL_TIMEOUT_S:
                requested_left, requested_right = 0, 0
                with state_lock:
                    vehicle["system"]["control_connected"] = False
            else:
                requested_left, requested_right = manual_requested_output()
                with state_lock:
                    vehicle["system"]["control_connected"] = True
        else:
            requested_left, requested_right = autonomous_requested_output()

        # MANUAL still has a hard forward collision stop.
        # ASSISTED and AUTONOMOUS additionally slow down as a wall approaches.
        wall_mode = "hard" if mode == "MANUAL" else "full"
        final_left, final_right = apply_safety(
            requested_left, requested_right, wall_mode=wall_mode
        )

        # In live mode, "actual" motor values are feedback from CAN ID 0x300.
        # The Pi only transmits the final safety-filtered command here.
        send_drive_can_command(final_left, final_right)


# ============================================================
# Optional OpenCV camera
# ============================================================

camera = None
camera_lock = threading.Lock()
latest_camera_frame = None


def get_camera():
    global camera
    if not USE_CAMERA or cv2 is None:
        return None

    with camera_lock:
        if camera is None:
            camera = cv2.VideoCapture(CAMERA_INDEX)

            # Keep latency down when the backend/driver supports this property.
            try:
                camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

        return camera


def _edge_density(binary_image):
    if binary_image is None or binary_image.size == 0:
        return 0.0
    return float(cv2.countNonZero(binary_image)) / float(binary_image.size)


def calculate_visual_obstruction(frame):
    """
    Return (left_score, right_score) for the camera image.

    The score is based on Canny edge density in the useful lower portion of
    the image. The lower half of that region is weighted more heavily because
    it is more likely to contain obstacles in the vehicle's path.

    This is intentionally only a LEFT-versus-RIGHT chooser. It does not replace
    the ultrasonic sensor for deciding whether an obstacle is actually close.
    """
    if frame is None or cv2 is None:
        return 0.0, 0.0

    h, w = frame.shape[:2]
    if h < 40 or w < 80:
        return 0.0, 0.0

    # Ignore most of the ceiling/sky and the extreme image borders.
    y0 = int(h * 0.28)
    y1 = int(h * 0.95)
    x_margin = int(w * 0.06)
    center_gap = int(w * 0.03)
    center = w // 2

    roi = frame[y0:y1, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(gray, 55, 140)

    # Slight dilation makes clusters of nearby edges count as visual occupancy
    # rather than as isolated one-pixel lines.
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    busy = cv2.dilate(edges, kernel, iterations=1)

    left_half = busy[:, x_margin:center - center_gap]
    right_half = busy[:, center + center_gap:w - x_margin]

    def weighted_half_score(half):
        if half is None or half.size == 0:
            return 0.0

        split = max(1, half.shape[0] // 2)
        upper = half[:split, :]
        lower = half[split:, :]

        # Nearby/path-level visual structure matters more than the upper image.
        return 0.35 * _edge_density(upper) + 0.65 * _edge_density(lower)

    return weighted_half_score(left_half), weighted_half_score(right_half)


def camera_capture_loop():
    """Capture one shared camera stream for both OpenCV autonomy and the UI."""
    global camera, latest_camera_frame
    global vision_left_obstruction, vision_right_obstruction
    global vision_last_seen, vision_available

    while True:
        cam = get_camera()

        if cam is None:
            with state_lock:
                vision_available = False
            time.sleep(0.5)
            continue

        with camera_lock:
            ok, frame = cam.read()

        if not ok or frame is None:
            with state_lock:
                vision_available = False

            # Re-open the device on the next pass in case the USB camera was
            # unplugged/replugged.
            with camera_lock:
                try:
                    cam.release()
                except Exception:
                    pass
                if camera is cam:
                    camera = None

            time.sleep(0.5)
            continue

        left_score, right_score = calculate_visual_obstruction(frame)

        with camera_lock:
            latest_camera_frame = frame.copy()

        with state_lock:
            vision_left_obstruction = left_score
            vision_right_obstruction = right_score
            vision_last_seen = now()
            vision_available = True

        time.sleep(VISION_UPDATE_INTERVAL_S)


def camera_frames():
    while True:
        with camera_lock:
            frame = None if latest_camera_frame is None else latest_camera_frame.copy()

        if frame is None:
            time.sleep(0.1)
            continue

        with state_lock:
            mode = vehicle["system"]["mode"]
            distance = vehicle["sensor"]["distance_cm"]
            left = vehicle["motor"]["actual_left"]
            right = vehicle["motor"]["actual_right"]

        cv2.putText(frame, f"MODE: {mode}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"Distance: {distance:.1f} cm", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Motors: L {left}%  R {right}%", (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        ok, jpg = cv2.imencode(".jpg", frame)
        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpg.tobytes() +
            b"\r\n"
        )


# ============================================================
# Flask API
# ============================================================

@app.route("/")
def index():
    return HTML


@app.route("/api/state")
def api_state():
    with state_lock:
        snapshot = {
            "battery": dict(vehicle["battery"]),
            "sensor": {
                **vehicle["sensor"],
                "imu": dict(vehicle["sensor"]["imu"]),
                "scan": dict(vehicle["sensor"]["scan"]),
            },
            "motor": dict(vehicle["motor"]),
            "system": dict(vehicle["system"]),
            "events": list(event_log),
        }

    t = now()
    snapshot["battery"]["age_ms"] = int((t - snapshot["battery"]["last_seen"]) * 1000)
    snapshot["sensor"]["age_ms"] = int((t - snapshot["sensor"]["last_seen"]) * 1000)
    snapshot["motor"]["age_ms"] = int((t - snapshot["motor"]["last_seen"]) * 1000)

    return jsonify(snapshot)


@app.route("/api/drive", methods=["POST"])
def api_drive():
    data = request.get_json(force=True)
    left = int(clamp(int(data.get("left", 0)), -100, 100))
    right = int(clamp(int(data.get("right", 0)), -100, 100))

    try:
        client_started_ms = int(data["client_started_ms"])
        sequence = int(data["seq"])
    except (KeyError, TypeError, ValueError):
        return jsonify({
            "ok": False,
            "error": "Drive command is missing freshness metadata"
        }), 400

    with state_lock:
        system = vehicle["system"]
        active_client_started_ms = int(system["control_client_started_ms"])

        # A request from an older dashboard page is stale and must never regain
        # control after a newer page has started sending commands.
        if client_started_ms < active_client_started_ms:
            return jsonify({
                "ok": True,
                "accepted": False,
                "stale": True,
                "reason": "older_dashboard",
                "seq": sequence,
            })

        # A newly loaded dashboard becomes the current controller and starts a
        # fresh sequence space.
        if client_started_ms > active_client_started_ms:
            system["control_client_started_ms"] = client_started_ms
            system["last_drive_sequence"] = -1

        # Reject duplicate or out-of-order requests from the current dashboard.
        if sequence <= int(system["last_drive_sequence"]):
            return jsonify({
                "ok": True,
                "accepted": False,
                "stale": True,
                "reason": "old_sequence",
                "seq": sequence,
            })

        system["last_drive_sequence"] = sequence

        limit = int(system["speed_limit"])
        left = int(clamp(left, -limit, limit))
        right = int(clamp(right, -limit, limit))
        vehicle["motor"]["requested_left"] = left
        vehicle["motor"]["requested_right"] = right
        system["last_control_time"] = now()
        mode = system["mode"]

    # Low-latency path: when a fresh MANUAL/ASSISTED command reaches Flask,
    # apply the normal safety rules and put it onto CAN immediately instead of
    # waiting for the next background control-loop tick.
    if mode in ("MANUAL", "ASSISTED"):
        wall_mode = "hard" if mode == "MANUAL" else "full"
        final_left, final_right = apply_safety(left, right, wall_mode=wall_mode)
        send_drive_can_command(final_left, final_right)

    return jsonify({
        "ok": True,
        "accepted": True,
        "stale": False,
        "seq": sequence,
    })


@app.route("/api/mode", methods=["POST"])
def api_mode():
    data = request.get_json(force=True)
    mode = str(data.get("mode", "ASSISTED")).upper()

    if mode not in ("MANUAL", "ASSISTED", "AUTONOMOUS"):
        return jsonify({"ok": False, "error": "Invalid mode"}), 400

    with state_lock:
        old_mode = vehicle["system"]["mode"]
        vehicle["system"]["mode"] = mode
        vehicle["motor"]["requested_left"] = 0
        vehicle["motor"]["requested_right"] = 0
        vehicle["system"]["last_control_time"] = now()

    if old_mode != mode:
        log_event(f"MODE -> {mode}")

    return jsonify({"ok": True})


@app.route("/api/speed", methods=["POST"])
def api_speed():
    data = request.get_json(force=True)
    speed = int(clamp(int(data.get("speed", DEFAULT_SPEED_LIMIT)), 0, 100))

    with state_lock:
        old_speed = vehicle["system"]["speed_limit"]
        vehicle["system"]["speed_limit"] = speed
        vehicle["system"]["last_control_time"] = now()

    if old_speed != speed:
        log_event(f"SPEED LIMIT -> {speed}%")

    return jsonify({"ok": True})


@app.route("/api/estop", methods=["POST"])
def api_estop():
    data = request.get_json(force=True)
    enabled = bool(data.get("enabled", True))

    with state_lock:
        old = vehicle["system"]["estop"]
        vehicle["system"]["estop"] = enabled
        vehicle["motor"]["requested_left"] = 0
        vehicle["motor"]["requested_right"] = 0

    if old != enabled:
        log_event("E-STOP ENGAGED" if enabled else "E-STOP RELEASED")

    return jsonify({"ok": True})



@app.route("/api/lock", methods=["POST"])
@app.route("/api/rfid", methods=["POST"])
def api_vehicle_lock():
    data = request.get_json(force=True)
    unlocked = bool(data.get("unlocked", False))

    if SIMULATION_MODE:
        with state_lock:
            old_state = vehicle["sensor"]["rfid_unlocked"]
            vehicle["sensor"]["rfid_unlocked"] = unlocked

        if old_state != unlocked:
            log_event("BROWSER -> UNLOCKED" if unlocked else "BROWSER -> LOCKED")

        return jsonify({"ok": True, "confirmed": True})

    if not CAN_ENABLED or can_socket is None:
        return jsonify({
            "ok": False,
            "error": "CAN interface is not available"
        }), 503

    if not send_lock_can_command(unlocked):
        return jsonify({
            "ok": False,
            "error": "Could not transmit lock command"
        }), 503

    log_event("BROWSER REQUEST -> UNLOCK" if unlocked else "BROWSER REQUEST -> LOCK")

    # Sensor ECU confirms the final state in its next 0x200 status frame.
    return jsonify({"ok": True, "confirmed": False})


@app.route("/video_feed")
def video_feed():
    if not USE_CAMERA or cv2 is None:
        return ("Camera disabled", 503)

    return Response(
        camera_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# ============================================================
# Dashboard
# ============================================================

HTML = r"""
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LGARI Control</title>

<style>
    :root {
        --bg: #0b0d0f;
        --surface: #111418;
        --surface-2: #171b20;
        --line: #252a31;
        --text: #e7e9ec;
        --muted: #8a919a;
        --accent: #d8dee7;
        --good: #7fbf8c;
        --warn: #d2b36c;
        --bad: #d07b72;
    }

    * { box-sizing: border-box; }

    body {
        margin: 0;
        background: var(--bg);
        color: var(--text);
        font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }

    .page {
        max-width: 1540px;
        margin: 0 auto;
        padding: 22px;
    }

    .header {
        display: flex;
        align-items: end;
        justify-content: space-between;
        gap: 18px;
        padding: 2px 2px 18px;
        border-bottom: 1px solid var(--line);
        margin-bottom: 18px;
    }

    .brand {
        letter-spacing: .18em;
        font-size: 14px;
        font-weight: 700;
    }

    .header-title {
        margin-top: 6px;
        font-size: 27px;
        font-weight: 650;
        letter-spacing: -0.02em;
    }

    .header-right {
        display: flex;
        gap: 8px;
        flex-wrap: wrap;
        justify-content: end;
    }

    .tag {
        font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
        font-size: 11px;
        color: var(--muted);
        border: 1px solid var(--line);
        padding: 7px 9px;
        border-radius: 4px;
        background: var(--surface);
    }

    .layout {
        display: grid;
        grid-template-columns: minmax(0, 1.55fr) minmax(330px, .85fr);
        gap: 16px;
    }

    .stack {
        display: grid;
        gap: 16px;
    }

    .module {
        background: var(--surface);
        border: 1px solid var(--line);
        border-radius: 7px;
        overflow: hidden;
    }

    .module-head {
        padding: 11px 13px;
        border-bottom: 1px solid var(--line);
        display: flex;
        justify-content: space-between;
        align-items: center;
    }

    .module-title {
        font-size: 12px;
        text-transform: uppercase;
        letter-spacing: .12em;
        color: var(--muted);
        font-weight: 700;
    }

    .module-body {
        padding: 14px;
    }

    .camera {
        height: 430px;
        position: relative;
        background:
            linear-gradient(rgba(255,255,255,.025) 1px, transparent 1px),
            linear-gradient(90deg, rgba(255,255,255,.025) 1px, transparent 1px),
            #0f1216;
        background-size: 28px 28px;
        display: flex;
        align-items: center;
        justify-content: center;
    }

    .camera img {
        width: 100%;
        height: 100%;
        object-fit: cover;
    }

    .camera-placeholder {
        color: var(--muted);
        font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
        font-size: 12px;
        text-align: center;
        line-height: 1.8;
    }

    .camera-overlay {
        position: absolute;
        left: 14px;
        bottom: 14px;
        font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
        font-size: 11px;
        line-height: 1.55;
        background: rgba(5,7,9,.72);
        border: 1px solid rgba(255,255,255,.08);
        padding: 9px 11px;
        border-radius: 4px;
    }

    .control-row {
        display: grid;
        grid-template-columns: 1.1fr 1fr;
        gap: 16px;
    }

    .mode-switch {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        gap: 6px;
        margin-bottom: 18px;
    }

    button {
        color: var(--text);
        background: var(--surface-2);
        border: 1px solid var(--line);
        border-radius: 4px;
        padding: 10px 12px;
        cursor: pointer;
        font: inherit;
        font-size: 12px;
        font-weight: 650;
    }

    button:hover { background: #1d2229; }

    button.active {
        background: var(--accent);
        color: #111418;
        border-color: var(--accent);
    }

    .drive-keys {
        display: grid;
        grid-template-columns: repeat(3, 58px);
        gap: 7px;
        justify-content: center;
        margin: 10px 0 14px;
    }

    .key {
        height: 48px;
        border: 1px solid var(--line);
        border-radius: 4px;
        display: flex;
        align-items: center;
        justify-content: center;
        background: #12161b;
        color: var(--muted);
        font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
        font-weight: 700;
    }

    .key.active {
        color: #0b0d0f;
        background: var(--accent);
        border-color: var(--accent);
    }

    .speed-wrap {
        margin-top: 14px;
        display: grid;
        grid-template-columns: 1fr 58px;
        align-items: center;
        gap: 12px;
    }

    input[type="range"] {
        width: 100%;
        accent-color: #d8dee7;
    }

    .speed-number {
        font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
        text-align: right;
    }

    .action-row {
        margin-top: 14px;
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 8px;
    }

    .estop {
        border-color: #66322e;
        background: #251614;
        color: #e4a09a;
    }

    .estop.engaged {
        background: #7d2f28;
        border-color: #9a4038;
        color: white;
    }

    .metrics {
        display: grid;
        grid-template-columns: repeat(2, 1fr);
        gap: 1px;
        background: var(--line);
        border: 1px solid var(--line);
    }

    .metric {
        background: var(--surface);
        padding: 14px;
        min-height: 88px;
    }

    .metric-label {
        color: var(--muted);
        font-size: 11px;
        text-transform: uppercase;
        letter-spacing: .1em;
    }

    .metric-value {
        margin-top: 9px;
        font-size: 25px;
        font-weight: 550;
        letter-spacing: -0.03em;
    }

    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }

    .status-list {
        display: grid;
        gap: 0;
    }

    .status {
        padding: 9px 0;
        border-bottom: 1px solid var(--line);
        display: flex;
        justify-content: space-between;
        gap: 18px;
        font-size: 12px;
    }

    .status:last-child { border-bottom: 0; }

    .ok { color: var(--good); }
    .warn { color: var(--warn); }
    .bad { color: var(--bad); }

    .visual-grid {
        display: grid;
        grid-template-columns: 1.1fr 1fr;
        gap: 16px;
    }

    .radar-wrap {
        position: relative;
        height: 270px;
        overflow: hidden;
    }

    .visual-grid > .module {
        min-width: 0;
    }

    #radarCanvas {
        width: 100%;
        height: 100%;
        display: block;
    }

    .imu-stage {
        height: 270px;
        display: flex;
        align-items: center;
        justify-content: center;
        position: relative;
        overflow: hidden;
        background:
            linear-gradient(rgba(255,255,255,.022) 1px, transparent 1px),
            linear-gradient(90deg, rgba(255,255,255,.022) 1px, transparent 1px);
        background-size: 24px 24px;
    }

    .imu-car {
        width: 130px;
        height: 72px;
        position: relative;
        transition: transform .12s linear;
        transform-origin: 50% 50%;
        will-change: transform;
    }

    .imu-car .body {
        position: absolute;
        inset: 12px 14px;
        border: 1px solid #8e959d;
        background: #1b2026;
        box-shadow: inset 0 0 0 1px rgba(255,255,255,.03);
    }

    .imu-car .nose {
        position: absolute;
        width: 34px;
        height: 24px;
        border: 1px solid #8e959d;
        border-left: none;
        right: -9px;
        top: 24px;
        background: #1b2026;
    }

    .imu-car .wheel {
        position: absolute;
        width: 24px;
        height: 8px;
        background: #747b84;
    }

    .w1 { left: 14px; top: 4px; }
    .w2 { right: 14px; top: 4px; }
    .w3 { left: 14px; bottom: 4px; }
    .w4 { right: 14px; bottom: 4px; }

    .imu-readout {
        position: absolute;
        left: 12px;
        bottom: 10px;
        font-size: 11px;
        color: var(--muted);
        line-height: 1.5;
    }

    .warning-banner {
        display: none;
        margin-bottom: 14px;
        padding: 10px 12px;
        border: 1px solid #684239;
        background: #211513;
        color: #d9a199;
        border-radius: 4px;
        font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
        font-size: 11px;
    }

    .warning-banner.show { display: block; }

    .telemetry-metrics {
        grid-template-columns: repeat(2, 1fr);
    }

    .imu-divider {
        color: #4b5159;
    }

    .right-stack {
        align-content: start;
    }

    .event-module {
        height: 560px;
        min-height: 560px;
        max-height: 560px;
        display: flex;
        flex-direction: column;
        overflow: hidden;
    }

    .event-body {
        flex: 1 1 auto;
        min-height: 0;
        overflow: hidden;
        display: flex;
    }

    .log {
        width: 100%;
        height: 100%;
        min-height: 0;
        max-height: 100%;
        flex: 1 1 auto;
        font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
        font-size: 11px;
        line-height: 1.65;
        color: #b9bec5;
        overflow-x: hidden;
        overflow-y: auto;
        scrollbar-gutter: stable;
        padding-right: 4px;
    }

    .subtle {
        color: var(--muted);
        font-size: 11px;
    }

    @media(max-width: 1000px) {
        .layout { grid-template-columns: 1fr; }
    }

    @media(max-width: 720px) {
        .control-row,
        .visual-grid {
            grid-template-columns: 1fr;
        }
        .camera { height: 320px; }
    }
</style>
</head>

<body>
<div class="page">

    <div class="header">
        <div>
            <div class="brand">LGARI</div>
            <div class="header-title">Vehicle Control Interface</div>
        </div>

        <div class="header-right">
            <div class="tag" id="mode-tag">ASSISTED</div>
            <div class="tag" id="sim-tag">SIMULATION</div>
            <div class="tag">PI CTRL / V0.5.5</div>
        </div>
    </div>

    <div id="warning-banner" class="warning-banner"></div>

    <div class="layout">

        <div class="stack">

            <div class="module">
                <div class="module-head">
                    <div class="module-title">Forward Camera</div>
                    <div class="subtle">Perception feed</div>
                </div>
                <div class="camera" id="camera-area">
                    <div class="camera-placeholder" id="camera-placeholder">
                        CAMERA DISABLED IN SIMULATION<br>
                        ENABLE WHEN USB CAMERA IS CONNECTED
                    </div>
                    <div class="camera-overlay">
                        <div id="cam-mode">MODE ASSISTED</div>
                        <div id="cam-distance">RANGE -- CM</div>
                        <div id="cam-motors">MOTOR L 0 / R 0</div>
                    </div>
                </div>
            </div>

            <div class="control-row">

                <div class="module">
                    <div class="module-head">
                        <div class="module-title">Driver Input</div>
                        <div class="subtle">WASD</div>
                    </div>
                    <div class="module-body">
                        <div class="mode-switch">
                            <button id="manual-btn" onclick="setMode('MANUAL')">Manual</button>
                            <button id="assisted-btn" onclick="setMode('ASSISTED')">Assisted</button>
                            <button id="auto-btn" onclick="setMode('AUTONOMOUS')">Autonomous</button>
                        </div>

                        <div class="drive-keys">
                            <div></div>
                            <div class="key" id="key-w">W</div>
                            <div></div>
                            <div class="key" id="key-a">A</div>
                            <div class="key" id="key-s">S</div>
                            <div class="key" id="key-d">D</div>
                        </div>

                        <div class="speed-wrap">
                            <input id="speed-slider"
                                   type="range"
                                   min="0"
                                   max="100"
                                   value="50"
                                   step="1">
                            <div class="speed-number"><span id="speed-value">50</span>%</div>
                        </div>

                        <div class="action-row">
                            <button class="estop" id="estop-btn" onclick="toggleEstop()">E-STOP</button>
                            <button id="lock-btn" onclick="toggleVehicleLock()">Unlock Vehicle</button>
                        </div>
                    </div>
                </div>

                <div class="module">
                    <div class="module-head">
                        <div class="module-title">Drive Output</div>
                        <div class="subtle">requested / final</div>
                    </div>
                    <div class="module-body">
                        <div class="metrics">
                            <div class="metric">
                                <div class="metric-label">Requested L</div>
                                <div class="metric-value mono"><span id="req-left">0</span>%</div>
                            </div>
                            <div class="metric">
                                <div class="metric-label">Requested R</div>
                                <div class="metric-value mono"><span id="req-right">0</span>%</div>
                            </div>
                            <div class="metric">
                                <div class="metric-label">Actual L</div>
                                <div class="metric-value mono"><span id="act-left">0</span>%</div>
                            </div>
                            <div class="metric">
                                <div class="metric-label">Actual R</div>
                                <div class="metric-value mono"><span id="act-right">0</span>%</div>
                            </div>
                        </div>

                        <div class="status-list" style="margin-top: 12px;">
                            <div class="status"><span>Wall intervention</span><strong id="wall-status">OFF</strong></div>
                            <div class="status"><span>Current limiting</span><strong id="current-limit-status">OFF</strong></div>
                            <div class="status"><span>RFID state</span><strong id="rfid-status">LOCKED</strong></div>
                        </div>
                    </div>
                </div>

            </div>

            <div class="visual-grid">

                <div class="module">
                    <div class="module-head">
                        <div class="module-title">Ultrasonic Sweep</div>
                        <div class="subtle">servo scan</div>
                    </div>
                    <div class="radar-wrap">
                        <canvas id="radarCanvas"></canvas>
                    </div>
                </div>

                <div class="module">
                    <div class="module-head">
                        <div class="module-title">IMU Orientation</div>
                        <div class="subtle">roll / pitch / yaw</div>
                    </div>
                    <div class="imu-stage">
                        <div class="imu-car" id="imu-car">
                            <div class="body"></div>
                            <div class="nose"></div>
                            <div class="wheel w1"></div>
                            <div class="wheel w2"></div>
                            <div class="wheel w3"></div>
                            <div class="wheel w4"></div>
                        </div>

                        <div class="imu-readout">
                            ROLL <span id="imu-roll">0.0</span>°<br>
                            PITCH <span id="imu-pitch">0.0</span>°<br>
                            YAW <span id="imu-yaw">0.0</span>°<br>
                            <span class="imu-divider">────────</span><br>
                            AX <span id="imu-ax">0.000</span> g<br>
                            AY <span id="imu-ay">0.000</span> g<br>
                            AZ <span id="imu-az">1.000</span> g
                        </div>
                    </div>
                </div>

            </div>
        </div>

        <div class="stack right-stack">

            <div class="module">
                <div class="module-head">
                    <div class="module-title">Vehicle Telemetry</div>
                    <div class="subtle">live state</div>
                </div>
                <div class="metrics telemetry-metrics">
                    <div class="metric">
                        <div class="metric-label">Pack Voltage</div>
                        <div class="metric-value mono"><span id="voltage">--</span> V</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Current</div>
                        <div class="metric-value mono"><span id="current">--</span> A</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Drive Speed</div>
                        <div class="metric-value mono"><span id="drive-speed">0</span>%</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Battery Temp</div>
                        <div class="metric-value mono"><span id="temperature">--</span>°C</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Forward Range</div>
                        <div class="metric-value mono"><span id="distance">--</span> cm</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Speed Limit</div>
                        <div class="metric-value mono"><span id="telemetry-speed-limit">50</span>%</div>
                    </div>
                </div>
            </div>

            <div class="module">
                <div class="module-head">
                    <div class="module-title">Network Health</div>
                    <div class="subtle">ECU heartbeat</div>
                </div>
                <div class="module-body status-list">
                    <div class="status"><span>Battery ECU</span><strong id="battery-online">...</strong></div>
                    <div class="status"><span>Sensor ECU</span><strong id="sensor-online">...</strong></div>
                    <div class="status"><span>Motor ECU</span><strong id="motor-online">...</strong></div>
                    <div class="status"><span>Browser control</span><strong id="control-online">...</strong></div>
                    <div class="status"><span>E-stop</span><strong id="estop-status">...</strong></div>
                    <div class="status"><span>Overcurrent</span><strong id="oc-status">...</strong></div>
                </div>
            </div>

            <div class="module event-module">
                <div class="module-head">
                    <div class="module-title">System Event Log</div>
                    <div class="subtle">latest first</div>
                </div>
                <div class="module-body event-body">
                    <div class="log" id="event-log">Waiting for events...</div>
                </div>
            </div>

        </div>
    </div>
</div>

<script>
let pressed = new Set();
let estop = false;
let vehicleUnlocked = false;
let currentMode = "ASSISTED";
let speedLimit = 50;
let sliderDragging = false;
let speedCommitTimer = null;

// Continuous display heading for the IMU graphic.
// We unwrap 0/360 transitions so CSS never animates a 359-degree jump.
let imuVisualYaw = 0;
let imuLastRawYaw = null;

const DRIVE_HEARTBEAT_MS = 150;
const STATE_REFRESH_DELAY_MS = 250;

// Freshness metadata for the drive channel. A newly loaded dashboard gets a
// newer client timestamp, and every command from this page gets a higher
// sequence number. The Pi rejects delayed/out-of-order commands.
const CONTROL_CLIENT_STARTED_MS = Date.now();
let driveSequence = 0;

// Only the background heartbeat is single-flight. Actual driver input is sent
// immediately and is allowed to bypass a slow heartbeat request.
let heartbeatInFlight = false;

function postJSON(url, data, signal=null) {
    return fetch(url, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(data),
        signal,
        cache: "no-store"
    });
}

function setMode(mode) {
    currentMode = mode;
    pressed.clear();
    updateKeyDisplay();

    // Publish zero input immediately while the mode request is processed.
    sendDriveImmediate();
    postJSON("/api/mode", {mode});
}

function toggleEstop() {
    estop = !estop;
    postJSON("/api/estop", {enabled: estop});

    if (estop) {
        pressed.clear();
        updateKeyDisplay();
        sendDriveImmediate();
    }
}

function toggleVehicleLock() {
    const desiredUnlocked = !vehicleUnlocked;

    postJSON("/api/lock", {unlocked: desiredUnlocked})
        .catch(err => console.error("Lock command failed:", err));
}

function computeDrive() {
    if (!["MANUAL", "ASSISTED"].includes(currentMode)) return [0, 0];

    let fwd = 0;
    let turn = 0;

    if (pressed.has("w")) fwd += speedLimit;
    if (pressed.has("s")) fwd -= speedLimit;
    if (pressed.has("a")) turn -= speedLimit;
    if (pressed.has("d")) turn += speedLimit;

    let left = fwd + turn;
    let right = fwd - turn;

    left = Math.max(-speedLimit, Math.min(speedLimit, left));
    right = Math.max(-speedLimit, Math.min(speedLimit, right));

    return [left, right];
}

function makeDrivePayload() {
    const [left, right] = computeDrive();

    return {
        left,
        right,
        seq: ++driveSequence,
        client_started_ms: CONTROL_CLIENT_STARTED_MS
    };
}

function sendDriveImmediate() {
    // Key transitions never wait behind the heartbeat. If an older heartbeat
    // eventually reaches the Pi after this request, its lower sequence number
    // causes it to be discarded.
    const payload = makeDrivePayload();

    postJSON("/api/drive", payload)
        .then(response => {
            if (!response.ok) {
                console.error("Immediate drive request failed with HTTP", response.status);
            }
        })
        .catch(err => console.error("Immediate drive request failed:", err));
}

async function sendDriveHeartbeat() {
    // The heartbeat exists only to keep the Pi's 0.5 s control watchdog alive.
    // Never queue another heartbeat while one is already outstanding.
    if (heartbeatInFlight) return;
    if (!["MANUAL", "ASSISTED"].includes(currentMode)) return;

    heartbeatInFlight = true;
    const payload = makeDrivePayload();

    try {
        const response = await postJSON("/api/drive", payload);
        if (!response.ok) {
            console.error("Drive heartbeat failed with HTTP", response.status);
        }
    } catch (err) {
        console.error("Drive heartbeat failed:", err);
    } finally {
        heartbeatInFlight = false;
    }
}

function updateKeyDisplay() {
    ["w","a","s","d"].forEach(k => {
        document.getElementById("key-" + k)
            .classList.toggle("active", pressed.has(k));
    });
}

window.addEventListener("keydown", e => {
    let k = e.key.toLowerCase();

    if (["w","a","s","d"].includes(k)) {
        e.preventDefault();

        // Ignore Windows/Chrome keyboard auto-repeat.
        if (e.repeat) return;

        pressed.add(k);
        updateKeyDisplay();
        sendDriveImmediate();
    }

    if (e.code === "Space") {
        e.preventDefault();
        if (!estop) toggleEstop();
    }
});

window.addEventListener("keyup", e => {
    let k = e.key.toLowerCase();

    if (["w","a","s","d"].includes(k)) {
        e.preventDefault();
        pressed.delete(k);
        updateKeyDisplay();

        // Release/STOP is sent immediately even if a heartbeat is still slow.
        sendDriveImmediate();
    }
});

window.addEventListener("blur", () => {
    pressed.clear();
    updateKeyDisplay();
    sendDriveImmediate();
});

// Background watchdog heartbeat. It never queues and driver input can always
// bypass it.
setInterval(() => {
    sendDriveHeartbeat();
}, DRIVE_HEARTBEAT_MS);

const speedSlider = document.getElementById("speed-slider");

speedSlider.addEventListener("pointerdown", () => {
    sliderDragging = true;
});

speedSlider.addEventListener("pointerup", () => {
    sliderDragging = false;
    commitSpeed();
});

speedSlider.addEventListener("pointercancel", () => {
    sliderDragging = false;
    commitSpeed();
});

speedSlider.addEventListener("input", () => {
    speedLimit = Number(speedSlider.value);
    document.getElementById("speed-value").textContent = speedLimit;

    clearTimeout(speedCommitTimer);
    speedCommitTimer = setTimeout(commitSpeed, 120);
});

speedSlider.addEventListener("change", commitSpeed);

function commitSpeed() {
    speedLimit = Number(speedSlider.value);
    document.getElementById("speed-value").textContent = speedLimit;
    postJSON("/api/speed", {speed: speedLimit});
    sendDriveImmediate();
}

function statusText(el, ok, goodText="ONLINE", badText="OFFLINE") {
    el.textContent = ok ? goodText : badText;
    el.className = ok ? "ok" : "bad";
}

function drawRadar(sensor) {
    const canvas = document.getElementById("radarCanvas");
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;

    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;

    const ctx = canvas.getContext("2d");
    ctx.scale(dpr, dpr);

    const w = rect.width;
    const h = rect.height;
    const cx = w / 2;
    const cy = h - 18;
    const maxR = Math.min(w * 0.46, h * 0.88);

    ctx.clearRect(0, 0, w, h);

    ctx.strokeStyle = "rgba(216,222,231,.16)";
    ctx.lineWidth = 1;

    [0.25, 0.5, 0.75, 1].forEach(scale => {
        ctx.beginPath();
        ctx.arc(cx, cy, maxR * scale, Math.PI, 2 * Math.PI);
        ctx.stroke();
    });

    [-60, -30, 0, 30, 60].forEach(deg => {
        const a = (deg - 90) * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(
            cx + Math.cos(a) * maxR,
            cy + Math.sin(a) * maxR
        );
        ctx.stroke();
    });

    const readings = [
        {deg: -60, dist: sensor.scan.left_cm},
        {deg: 0, dist: sensor.scan.center_cm},
        {deg: 60, dist: sensor.scan.right_cm}
    ];

    readings.forEach(r => {
        const ratio = Math.min(r.dist, 200) / 200;
        const rr = ratio * maxR;
        const a = (r.deg - 90) * Math.PI / 180;
        const x = cx + Math.cos(a) * rr;
        const y = cy + Math.sin(a) * rr;

        ctx.fillStyle = "#d8dee7";
        ctx.beginPath();
        ctx.arc(x, y, 3, 0, Math.PI * 2);
        ctx.fill();
    });

    const sweepDeg = sensor.servo_angle_deg || 0;
    const sweepA = (sweepDeg - 90) * Math.PI / 180;

    ctx.strokeStyle = "rgba(216,222,231,.8)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(
        cx + Math.cos(sweepA) * maxR,
        cy + Math.sin(sweepA) * maxR
    );
    ctx.stroke();

    ctx.fillStyle = "#d8dee7";
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = "#8a919a";
    ctx.font = "11px ui-monospace, monospace";
    ctx.fillText("200 cm", cx + maxR - 40, cy - 6);
    ctx.fillText("L", 12, cy - 4);
    ctx.fillText("R", w - 20, cy - 4);
}

async function refreshState() {
    const res = await fetch("/api/state");
    const s = await res.json();

    currentMode = s.system.mode;
    estop = s.system.estop;
    vehicleUnlocked = s.sensor.rfid_unlocked;

    if (!sliderDragging) {
        speedLimit = s.system.speed_limit;
        speedSlider.value = speedLimit;
        document.getElementById("speed-value").textContent = speedLimit;
    }

    document.getElementById("mode-tag").textContent = s.system.mode;
    document.getElementById("sim-tag").textContent = s.system.simulation ? "SIMULATION" : "LIVE";

    document.getElementById("manual-btn").classList.toggle("active", s.system.mode === "MANUAL");
    document.getElementById("assisted-btn").classList.toggle("active", s.system.mode === "ASSISTED");
    document.getElementById("auto-btn").classList.toggle("active", s.system.mode === "AUTONOMOUS");

    document.getElementById("voltage").textContent = s.battery.voltage.toFixed(2);
    document.getElementById("current").textContent = s.battery.current.toFixed(2);
    document.getElementById("temperature").textContent = s.battery.temperature.toFixed(1);
    document.getElementById("distance").textContent = s.sensor.distance_cm.toFixed(1);

    // No wheel encoders yet, so this is motor-output-based estimated speed.
    const driveSpeed = Math.round(
        (Math.abs(s.motor.actual_left) + Math.abs(s.motor.actual_right)) / 2
    );
    document.getElementById("drive-speed").textContent = driveSpeed;
    document.getElementById("telemetry-speed-limit").textContent = s.system.speed_limit;

    document.getElementById("req-left").textContent = s.motor.requested_left;
    document.getElementById("req-right").textContent = s.motor.requested_right;
    document.getElementById("act-left").textContent = s.motor.actual_left;
    document.getElementById("act-right").textContent = s.motor.actual_right;

    document.getElementById("cam-mode").textContent = "MODE " + s.system.mode;
    document.getElementById("cam-distance").textContent = "RANGE " + s.sensor.distance_cm.toFixed(1) + " CM";
    document.getElementById("cam-motors").textContent = "MOTOR L " + s.motor.actual_left + " / R " + s.motor.actual_right;

    document.getElementById("imu-roll").textContent = s.sensor.imu.roll.toFixed(1);
    document.getElementById("imu-pitch").textContent = s.sensor.imu.pitch.toFixed(1);
    document.getElementById("imu-yaw").textContent = s.sensor.imu.yaw.toFixed(1);
    document.getElementById("imu-ax").textContent = s.sensor.imu.ax.toFixed(3);
    document.getElementById("imu-ay").textContent = s.sensor.imu.ay.toFixed(3);
    document.getElementById("imu-az").textContent = s.sensor.imu.az.toFixed(3);

    const car = document.getElementById("imu-car");

    const rawYaw = s.sensor.imu.yaw;

    if (imuLastRawYaw === null) {
        imuLastRawYaw = rawYaw;
        imuVisualYaw = rawYaw;
    } else {
        let delta = rawYaw - imuLastRawYaw;

        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        imuVisualYaw += delta;
        imuLastRawYaw = rawYaw;
    }

    // CSS positive rotation is clockwise. Negating makes A turn the
    // vehicle graphic left and D turn it right.
    car.style.transform = `rotate(${-imuVisualYaw}deg)`;

    drawRadar(s.sensor);

    const wall = document.getElementById("wall-status");
    wall.textContent = s.system.wall_intervention ? "ACTIVE" : "OFF";
    wall.className = s.system.wall_intervention ? "warn" : "ok";

    const ci = document.getElementById("current-limit-status");
    ci.textContent = s.system.current_intervention ? "ACTIVE" : "OFF";
    ci.className = s.system.current_intervention ? "warn" : "ok";

    const rfid = document.getElementById("rfid-status");
    rfid.textContent = s.sensor.rfid_unlocked ? "UNLOCKED" : "LOCKED";
    rfid.className = s.sensor.rfid_unlocked ? "ok" : "bad";

    statusText(document.getElementById("battery-online"), s.battery.age_ms < 500);
    statusText(document.getElementById("sensor-online"), s.sensor.age_ms < 500);
    statusText(document.getElementById("motor-online"), s.motor.age_ms < 500);
    statusText(document.getElementById("control-online"), s.system.control_connected);

    const es = document.getElementById("estop-status");
    es.textContent = s.system.estop ? "ENGAGED" : "READY";
    es.className = s.system.estop ? "bad" : "ok";

    const oc = document.getElementById("oc-status");
    oc.textContent = s.system.overcurrent_fault ? "FAULT" : "CLEAR";
    oc.className = s.system.overcurrent_fault ? "bad" : "ok";

    const estopBtn = document.getElementById("estop-btn");
    estopBtn.classList.toggle("engaged", s.system.estop);
    estopBtn.textContent = s.system.estop ? "RELEASE E-STOP" : "E-STOP";

    const lockBtn = document.getElementById("lock-btn");
    lockBtn.textContent = s.sensor.rfid_unlocked ? "Lock Vehicle" : "Unlock Vehicle";

    document.getElementById("event-log").innerHTML =
        s.events.length ? s.events.join("<br>") : "No events yet.";

    const warning = document.getElementById("warning-banner");
    let warnings = [];

    if (s.system.estop) warnings.push("EMERGENCY STOP ACTIVE");
    if (!s.sensor.rfid_unlocked) warnings.push("VEHICLE LOCKED");
    if (s.system.overcurrent_fault) warnings.push("OVERCURRENT FAULT");
    if (s.system.wall_intervention) warnings.push("WALL SAFETY INTERVENTION");
    if (s.system.current_intervention) warnings.push("CURRENT LIMITING ACTIVE");

    warning.textContent = warnings.join("   /   ");
    warning.classList.toggle("show", warnings.length > 0);

    if (s.system.camera_enabled) {
        const area = document.getElementById("camera-area");
        if (!document.getElementById("camera-stream")) {
            const img = document.createElement("img");
            img.id = "camera-stream";
            img.src = "/video_feed";
            area.insertBefore(img, area.firstChild);
            document.getElementById("camera-placeholder").style.display = "none";
        }
    }
}

async function stateRefreshLoop() {
    try {
        await refreshState();
    } catch (err) {
        console.error("State refresh failed:", err);
    } finally {
        // Schedule the next poll only after this one has completely finished.
        // This prevents /api/state requests from ever piling up.
        setTimeout(stateRefreshLoop, STATE_REFRESH_DELAY_MS);
    }
}

stateRefreshLoop();
</script>

</body>
</html>
"""

if __name__ == "__main__":

    if SIMULATION_MODE:
        log_event("SIMULATION MODE ENABLED")
        threading.Thread(target=simulation_loop, daemon=True).start()
    else:
        log_event("LIVE HARDWARE MODE ENABLED")

    if CAN_ENABLED:
        if initialize_can():
            threading.Thread(target=can_receive_loop, daemon=True).start()

    if USE_CAMERA and cv2 is not None:
        threading.Thread(target=camera_capture_loop, daemon=True).start()

    threading.Thread(target=control_loop, daemon=True).start()

    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
