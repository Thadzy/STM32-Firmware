"""
protocol.py
-----------
Modbus RTU register map and command translator.
Maps WebSocket JSON commands from UI to Modbus register writes,
and reads status registers back to UI-friendly JSON.

Register Map (from Base System documentation):
  WRITE registers:
    0x00 - Heartbeat (reply HI=18537 when robot sends YA=22881)
    0x01 - Operating mode (1=Home, 2=Jog, 4=Auto, 8=SetHome, 16=Test)
    0x02 - Gripper manual (0=Up, 1=Down, 2=Open, 4=Close)
    0x03 - Gripper sequence (1=Pick, 2=Place)
    0x04 - Gripper enable in AUTO (0=disable, 1=enable)
    0x05 - Jog step size in degrees (signed, +CCW, -CW)
    0x06 - Test type (0=Precision, 1=Performance)
    0x07 - Performance test velocity
    0x08 - Performance test acceleration
    0x09 - Precision test initial position
    0x10 - Precision test final position
    0x11 - Precision test repeat count
    0x12-0x21 - Pick and place sequence slots
    0x22 - Number of pick-place pairs
    0x23 - P2P unit (0=degree, 1=index)
    0x24 - P2P target value (signed)
    0x25 - Soft stop (0=run, 1=stop)

  READ registers:
    0x00 - Heartbeat
    0x26 - Reed sensors (gripper state)
    0x27 - Current task
    0x28 - Position (raw / 10 = degrees)
    0x29 - Velocity (raw / 10)
    0x30 - Acceleration (raw / 10)
    0x31 - Emergency state
"""

import logging
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ConnectionException

log = logging.getLogger("protocol")

# Modbus slave address
SLAVE_ID = 21

# Heartbeat values
HB_YA = 22881   # Robot sends this
HB_HI = 18537   # PC replies this

# Register addresses
REG_HEARTBEAT       = 0x00
REG_MODE            = 0x01
REG_GRIPPER_MANUAL  = 0x02
REG_GRIPPER_SEQ     = 0x03
REG_GRIPPER_AUTO    = 0x04
REG_JOG             = 0x05
REG_TEST_TYPE       = 0x06
REG_PERF_VEL        = 0x07
REG_PERF_ACCEL      = 0x08
REG_PREC_INIT       = 0x09
REG_PREC_FINAL      = 0x10
REG_PREC_COUNT      = 0x11
REG_SEQUENCE_START  = 0x12
REG_SEQUENCE_COUNT  = 0x22
REG_P2P_UNIT        = 0x23
REG_P2P_TARGET      = 0x24
REG_SOFT_STOP       = 0x25
REG_REED            = 0x26
REG_TASK            = 0x27
REG_POSITION        = 0x28
REG_VELOCITY        = 0x29
REG_ACCELERATION    = 0x30
REG_EMERGENCY       = 0x31

# Mode values
MODE_HOME     = 0x01
MODE_JOG      = 0x02
MODE_AUTO     = 0x04
MODE_SET_HOME = 0x08
MODE_TEST     = 0x10

# Gripper manual values
GRIPPER_UP    = 0x00
GRIPPER_DOWN  = 0x01
GRIPPER_OPEN  = 0x02
GRIPPER_CLOSE = 0x04

# Gripper sequence values
GRIPPER_PICK  = 0x01
GRIPPER_PLACE = 0x02

# Task names mapped from register bits
TASK_MAP = {
    0x01: "Homing",
    0x02: "Go Pick",
    0x04: "Go Place",
    0x08: "Go Point",
    0x00: "Idle",
}

# Reed sensor interpretation
def decode_reed(raw):
    """Decode reed sensor register to gripper state strings."""
    bit0 = bool(raw & 0x01)  # Reed 1
    bit1 = bool(raw & 0x02)  # Reed 2
    bit2 = bool(raw & 0x04)  # Reed 3 (jaw)

    if bit0 and not bit1:
        height = "Up"
    elif not bit0 and bit1:
        height = "Down"
    else:
        height = "Idle"

    jaw = "Closed" if bit2 else "Open"
    return f"{height} / {jaw}"


def decode_task(raw):
    """Decode task register to task name string."""
    for bit_val, name in TASK_MAP.items():
        if bit_val != 0x00 and (raw & bit_val):
            return name
    return "Idle"


def to_signed16(value):
    """Convert unsigned 16-bit value to signed int16."""
    if value >= 32768:
        return value - 65536
    return value


def to_unsigned16(value):
    """Convert signed value to unsigned 16-bit for Modbus wire."""
    # Ensure value is within int16 range
    if value < -32768: value = -32768
    if value > 32767: value = 32767
    
    if value < 0:
        return value + 65536
    return value


class ModbusProtocol:
    """
    Handles all Modbus RTU communication with the STM32 robot controller.
    Provides high-level methods that map to UI actions.
    """

    def __init__(self, port: str, baudrate: int = 19200):
        self.port = port
        self.baudrate = baudrate
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity="E",
            stopbits=1,
            timeout=1,
        )
        self.connected = False

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Connect to the Modbus serial port."""
        try:
            result = self.client.connect()
            self.connected = result
            if result:
                log.info(f"Connected to {self.port} at {self.baudrate} baud")
            else:
                log.error(f"Failed to connect to {self.port}")
            return result
        except Exception as e:
            log.error(f"Connection error: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from the Modbus serial port."""
        if self.client:
            self.client.close()
        self.connected = False
        log.info("Disconnected from serial port")

    # ------------------------------------------------------------------
    # Low-level register access
    # ------------------------------------------------------------------

    def _write(self, address: int, value: int, label: str = "") -> bool:
        """Write a single holding register. Returns True on success."""
        if not self.connected:
            log.warning(f"[WRITE] Not connected — skipping {label}")
            return False
        try:
            value_u16 = to_unsigned16(value)
            result = self.client.write_register(
                address, value_u16, slave=SLAVE_ID
            )
            ok = not result.isError()
            status = "OK" if ok else "ERROR"
            signed = to_signed16(value_u16)
            log.info(
                f"[WRITE] {label:20s} | Slave:{SLAVE_ID} | "
                f"Addr:{address} (0x{address:02X}) | "
                f"Raw:{value_u16} | Signed:{signed} | "
                f"Hex:0x{value_u16:04X} | Status:{status}"
            )
            if not ok:
                log.error(f"[WRITE] Modbus error: {result}")
            return ok
        except Exception as e:
            log.error(f"[WRITE] Connection error on {label}: {e}")
            self.connected = False
            return False

    def _read_block(self, start: int, count: int) -> list | None:
        """Read a block of holding registers. Returns list of raw values or None."""
        if not self.connected:
            return None
        try:
            result = self.client.read_holding_registers(
                start, count, slave=SLAVE_ID
            )
            if result.isError():
                log.error(f"[READ] Modbus error at address {start}: {result}")
                return None
            return result.registers
        except Exception as e:
            log.error(f"[READ] Connection error at address {start}: {e}")
            self.connected = False
            return None

    # ------------------------------------------------------------------
    # Status polling
    # ------------------------------------------------------------------

    def read_status(self) -> dict:
        """
        Read all status registers and return a STATS dict for the UI.
        Covers 0x26 to 0x31 (12 registers including gaps).
        Also handles heartbeat at 0x00.
        """
        if not self.connected:
            return self._empty_stats(heartbeat=False)

        # 1. Heartbeat check (Task 5: handled inside read_status)
        hb_regs = self._read_block(REG_HEARTBEAT, 1)
        heartbeat_ok = False
        if hb_regs is not None:
            if hb_regs[0] == HB_YA:
                # Robot sent YA, we reply HI
                self._write(REG_HEARTBEAT, HB_HI, "heartbeat reply")
                heartbeat_ok = True
            elif hb_regs[0] == HB_HI:
                # Already replied or pending robot update
                heartbeat_ok = True

        if not self.connected:
            return self._empty_stats(heartbeat=heartbeat_ok)

        # 2. Status block read (Task 3: address 0x26, count 12)
        regs = self._read_block(REG_REED, 12)
        if regs is None:
            return self._empty_stats(heartbeat=heartbeat_ok)

        # Map registers based on count=12 from 0x26
        # 0:0x26, 1:0x27, 2:0x28, 3:0x29, ... 10:0x30, 11:0x31
        reed_raw        = regs[0]   # 0x26
        task_raw        = regs[1]   # 0x27
        pos_raw         = regs[2]   # 0x28
        vel_raw         = regs[3]   # 0x29
        # Gap at 0x2A-0x2F (regs[4] to regs[9])
        accel_raw       = regs[10]  # 0x30
        emergency_raw   = regs[11]  # 0x31

        pos   = to_signed16(pos_raw) / 10.0
        vel   = to_signed16(vel_raw) / 10.0
        accel = to_signed16(accel_raw) / 10.0

        return {
            "type": "STATS",
            "pos": f"{pos:.1f}",
            "speed": f"{vel:.1f}",
            "accel": f"{accel:.1f}",
            "gripper": decode_reed(reed_raw),
            "mode": decode_task(task_raw),
            "emergency": "Active" if emergency_raw else "Idle",
            "heartbeat": heartbeat_ok,
            "connected": self.connected,
        }

    def _empty_stats(self, heartbeat: bool = False) -> dict:
        """Return a default stats structure when disconnected or error occurs."""
        return {
            "type": "STATS",
            "pos": "--",
            "speed": "--",
            "accel": "--",
            "gripper": "Idle / Idle",
            "mode": "Idle",
            "emergency": "Idle",
            "heartbeat": heartbeat,
            "connected": self.connected,
        }

    # ------------------------------------------------------------------
    # Command handlers — mapped from WebSocket JSON actions
    # ------------------------------------------------------------------

    def cmd_go_home(self):
        self._write(REG_MODE, MODE_HOME, "go_home")

    def cmd_set_home(self, offset_angle: int = 0):
        # We write to REG_MODE with MODE_SET_HOME first, 
        # but usually set_home might also use a register for offset?
        # Register map doesn't show an offset register, so we just trigger the mode.
        self._write(REG_MODE, MODE_SET_HOME, "set_home")

    def cmd_stop(self):
        self._write(REG_SOFT_STOP, 1, "soft_stop")

    def cmd_jog(self, value: int, direction: str):
        """Jog by value degrees. direction: 'CW' or 'CCW'."""
        self._write(REG_MODE, MODE_JOG, "jog_mode")
        step = value if direction == "CCW" else -value
        self._write(REG_JOG, step, f"jog_{direction}")

    def cmd_gripper_up(self):
        self._write(REG_GRIPPER_MANUAL, GRIPPER_UP, "gripper_up")

    def cmd_gripper_down(self):
        self._write(REG_GRIPPER_MANUAL, GRIPPER_DOWN, "gripper_down")

    def cmd_gripper_open(self):
        self._write(REG_GRIPPER_MANUAL, GRIPPER_OPEN, "gripper_open")

    def cmd_gripper_close(self):
        self._write(REG_GRIPPER_MANUAL, GRIPPER_CLOSE, "gripper_close")

    def cmd_gripper_pick(self):
        self._write(REG_GRIPPER_SEQ, GRIPPER_PICK, "gripper_pick")

    def cmd_gripper_place(self):
        self._write(REG_GRIPPER_SEQ, GRIPPER_PLACE, "gripper_place")

    def cmd_set_auto(self, use_gripper: bool = True):
        self._write(REG_MODE, MODE_AUTO, "set_auto")
        self._write(REG_GRIPPER_AUTO, 1 if use_gripper else 0, "gripper_auto")

    def cmd_pick_place(self, sequence: list, directions: list, use_gripper: bool):
        """Write pick-place sequence to registers 0x12-0x21 (up to 16 slots)."""
        self._write(REG_MODE, MODE_AUTO, "auto_mode")
        self._write(REG_GRIPPER_AUTO, 1 if use_gripper else 0, "gripper_auto")

        for i, slot in enumerate(sequence):
            if i >= 16:  # Max 16 slots (0x12 to 0x21)
                break
            
            # Use provided direction or default to CCW
            dir_str = directions[i] if i < len(directions) else "CCW"
            signed_slot = slot if dir_str == "CCW" else -slot
            self._write(REG_SEQUENCE_START + i, signed_slot, f"seq_slot_{i}")

        pairs = len(sequence) // 2
        self._write(REG_SEQUENCE_COUNT, pairs, "seq_count")

    def cmd_point_to_point(self, value: int, unit: str):
        """Move to target. unit: 'degree' or 'index'."""
        self._write(REG_MODE, MODE_AUTO, "auto_mode")
        unit_val = 0 if unit == "degree" else 1
        self._write(REG_P2P_UNIT, unit_val, "p2p_unit")
        self._write(REG_P2P_TARGET, value, "p2p_target")

    def cmd_set_test(self):
        self._write(REG_MODE, MODE_TEST, "set_test")

    def cmd_performance_test(self, speed: int, accel: int):
        self._write(REG_TEST_TYPE, 1, "test_performance")
        self._write(REG_PERF_VEL, speed, "perf_vel")
        self._write(REG_PERF_ACCEL, accel, "perf_accel")

    def cmd_precision_test(self, init_pos: int, final_pos: int, count: int):
        self._write(REG_TEST_TYPE, 0, "test_precision")
        self._write(REG_PREC_INIT, init_pos, "prec_init")
        self._write(REG_PREC_FINAL, final_pos, "prec_final")
        self._write(REG_PREC_COUNT, count, "prec_count")
