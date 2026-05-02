"""
main.py
-------
WebSocket server that bridges the browser UI and the STM32 robot
controller via Modbus RTU.

Usage:
  python main.py [--port PORT] [--ws-port WS_PORT] [--list-ports]

  PORT          Serial port (default: auto-detect)
                macOS example: /dev/tty.usbmodem1234
                Windows example: COM3

  WS_PORT       WebSocket port (default: 8765)

  --list-ports  Print available serial ports and exit

The server:
  1. Accepts WebSocket connections from the browser UI
  2. Translates JSON commands to Modbus RTU writes
  3. Periodically reads status registers and pushes STATS to the UI
  4. Handles heartbeat exchange with STM32 (within status loop)
"""

import asyncio
import json
import logging
import argparse
import sys
import serial.tools.list_ports
import websockets

from protocol import ModbusProtocol

# ------------------------------------------------------------------
# Logging
# ------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("main")

# ------------------------------------------------------------------
# Serial port management (Task 2 & 6)
# ------------------------------------------------------------------

def find_serial_port() -> str | None:
    """
    Try to find the STM32 ST-Link VCP port automatically.
    Checks common keywords and OS-specific patterns.
    """
    ports = list(serial.tools.list_ports.comports())

    # Priority 1: Hardware identifiers/descriptions
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ["stlink", "stm32", "virtual com"]):
            log.info(f"Auto-detected ST-Link port: {p.device}")
            return p.device

    # Priority 2: OS-specific patterns (Task 2)
    for p in ports:
        dev = p.device
        if sys.platform == "darwin" and "tty.usbmodem" in dev:
            log.info(f"Auto-detected macOS modem port: {dev}")
            return dev
        if sys.platform == "win32" and dev.upper().startswith("COM"):
            log.info(f"Auto-detected Windows COM port: {dev}")
            return dev

    # Fallback: first available port
    if ports:
        log.warning(f"No ST-Link pattern found, using first available: {ports[0].device}")
        return ports[0].device

    return None


def list_ports():
    """Print available serial ports and exit (Task 6)."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    
    print(f"{'Device':<25} | {'Description':<35} | {'HWID'}")
    print("-" * 80)
    for p in ports:
        print(f"{p.device:<25} | {p.description or 'N/A':<35} | {p.hwid}")

# ------------------------------------------------------------------
# WebSocket handler
# ------------------------------------------------------------------

class BaseSystemServer:
    """
    Manages the WebSocket server and coordinates between
    the browser UI and the Modbus protocol handler.
    """

    def __init__(self, serial_port: str | None, ws_port: int = 8765):
        self.serial_port = serial_port
        self.ws_port = ws_port
        self.modbus: ModbusProtocol | None = None
        self.clients: set = set()
        self.status_interval = 0.5  # seconds between status polls

    # ------------------------------------------------------------------
    # Broadcast to all connected UI clients
    # ------------------------------------------------------------------

    async def broadcast(self, message: dict):
        if not self.clients:
            return
        data = json.dumps(message)
        await asyncio.gather(
            *[client.send(data) for client in self.clients],
            return_exceptions=True,
        )

    # ------------------------------------------------------------------
    # Status polling loop (Task 4 & 5)
    # ------------------------------------------------------------------

    async def status_loop(self):
        """Periodically read robot status and push to UI."""
        last_connected = False
        while True:
            await asyncio.sleep(self.status_interval)
            
            if self.modbus:
                # Modbus calls are blocking, run in executor
                stats = await asyncio.get_event_loop().run_in_executor(
                    None, self.modbus.read_status
                )
                
                # Graceful error handling: detect drop (Task 4)
                if last_connected and not self.modbus.connected:
                    log.error("Modbus connection lost!")
                    stats["message"] = "Serial connection lost"
                
                last_connected = self.modbus.connected
                await self.broadcast(stats)
            else:
                # If no modbus instance yet, just broadcast a 'Not Connected' state
                if self.clients:
                    await self.broadcast({
                        "type": "STATS",
                        "connected": False,
                        "heartbeat": False,
                        "mode": "Not Connected",
                        "pos": "--",
                        "speed": "--",
                        "accel": "--"
                    })

    # ------------------------------------------------------------------
    # JSON command router
    # ------------------------------------------------------------------

    async def handle_command(self, message: dict) -> dict | None:
        """
        Route a JSON command from the UI to the correct Modbus call.
        Returns a response dict to send back, or None.
        """
        mode   = message.get("mode", "")
        action = message.get("action", "")

        # ---- Connect ----
        if mode == "Connect" and action == "connect_port":
            port_num = message.get("port")
            if port_num is not None:
                if sys.platform == "win32":
                    port_str = f"COM{port_num}"
                else:
                    port_str = self.serial_port or find_serial_port()

            if port_str is None:
                return {
                    "mode": "Connect",
                    "action": "connect_port",
                    "status": "error",
                    "message": "No serial port found",
                }

            # Disconnect existing connection first
            if self.modbus:
                await asyncio.get_event_loop().run_in_executor(
                    None, self.modbus.disconnect
                )

            self.modbus = ModbusProtocol(port=port_str)
            ok = await asyncio.get_event_loop().run_in_executor(
                None, self.modbus.connect
            )

            if ok:
                return {
                    "mode": "Connect",
                    "action": "connect_port",
                    "status": "success",
                    "message": f"Connected to {port_str} (slave 21)",
                }
            else:
                return {
                    "mode": "Connect",
                    "action": "connect_port",
                    "status": "error",
                    "message": f"Failed to connect to {port_str}",
                }

        # Guard: all other commands require active connection
        if not self.modbus or not self.modbus.connected:
            log.warning(f"Command ignored — not connected: {mode}/{action}")
            return None

        loop = asyncio.get_event_loop()

        # ---- Home ----
        if mode == "Home" and action == "go_home":
            await loop.run_in_executor(None, self.modbus.cmd_go_home)

        elif mode == "Home" and action == "set_home":
            offset = message.get("offset_angle", 0)
            await loop.run_in_executor(
                None, self.modbus.cmd_set_home, offset
            )

        # ---- Stop ----
        elif mode == "Stop" and action == "stop":
            await loop.run_in_executor(None, self.modbus.cmd_stop)

        # ---- Manual jog ----
        elif mode == "Manual" and action == "jog":
            value     = message.get("value", 10)
            direction = message.get("direction", "CW")
            await loop.run_in_executor(
                None, self.modbus.cmd_jog, value, direction
            )

        # ---- Gripper manual ----
        elif mode == "Manual" and action == "gripper_up":
            await loop.run_in_executor(None, self.modbus.cmd_gripper_up)

        elif mode == "Manual" and action == "gripper_down":
            await loop.run_in_executor(None, self.modbus.cmd_gripper_down)

        elif mode == "Manual" and action == "gripper_open":
            await loop.run_in_executor(None, self.modbus.cmd_gripper_open)

        elif mode == "Manual" and action == "gripper_close":
            await loop.run_in_executor(None, self.modbus.cmd_gripper_close)

        elif mode == "Manual" and action == "gripper_pick":
            await loop.run_in_executor(None, self.modbus.cmd_gripper_pick)

        elif mode == "Manual" and action == "gripper_place":
            await loop.run_in_executor(None, self.modbus.cmd_gripper_place)

        # ---- Auto ----
        elif mode == "Auto" and action == "set_auto":
            use_gripper = message.get("use_gripper", True)
            await loop.run_in_executor(
                None, self.modbus.cmd_set_auto, use_gripper
            )

        elif mode == "Auto" and action == "pick_place":
            sequence    = message.get("sequence", [])
            directions  = message.get("directions", [])
            use_gripper = message.get("use_gripper", True)
            await loop.run_in_executor(
                None, self.modbus.cmd_pick_place,
                sequence, directions, use_gripper
            )

        elif mode == "Auto" and action == "point_to_point":
            value = message.get("value", 0)
            unit  = message.get("unit", "degree")
            await loop.run_in_executor(
                None, self.modbus.cmd_point_to_point, value, unit
            )

        # ---- Test ----
        elif mode == "Test" and action == "set_test":
            await loop.run_in_executor(None, self.modbus.cmd_set_test)

        elif mode == "Test" and action == "performance":
            speed = message.get("speed", 10)
            accel = message.get("accel", 10)
            await loop.run_in_executor(
                None, self.modbus.cmd_performance_test, speed, accel
            )

        elif mode == "Test" and action == "precision":
            init_pos  = message.get("init_pos", 0)
            final_pos = message.get("final_pos", 0)
            count     = message.get("count", 1)
            await loop.run_in_executor(
                None, self.modbus.cmd_precision_test,
                init_pos, final_pos, count
            )

        else:
            log.warning(f"Unknown command: mode={mode} action={action}")

        return None

    # ------------------------------------------------------------------
    # WebSocket connection handler
    # ------------------------------------------------------------------

    async def handler(self, websocket):
        self.clients.add(websocket)
        client_addr = websocket.remote_address
        log.info(f"Client connected: {client_addr}")

        # Send initial connection status
        await websocket.send(json.dumps({
            "type": "STATS",
            "message": "Connected to Python Backend",
            "pos": "--",
            "speed": "--",
            "accel": "--",
            "gripper": "Idle / Idle",
            "mode": "Idle",
            "emergency": "Idle",
            "connected": self.modbus.connected if self.modbus else False,
        }))

        try:
            async for raw in websocket:
                try:
                    message = json.loads(raw)
                    log.debug(f"Received: {message}")
                    response = await self.handle_command(message)
                    if response:
                        await websocket.send(json.dumps(response))
                except json.JSONDecodeError:
                    log.error(f"Invalid JSON: {raw}")
                except Exception as e:
                    log.error(f"Command error: {e}")

        except websockets.exceptions.ConnectionClosedOK:
            pass
        except Exception as e:
            log.error(f"WebSocket error: {e}")
        finally:
            self.clients.discard(websocket)
            log.info(f"Client disconnected: {client_addr}")

    # ------------------------------------------------------------------
    # Start server
    # ------------------------------------------------------------------

    async def run(self):
        log.info(f"WebSocket Server running ws://localhost:{self.ws_port}")

        # Start status polling loop as background task
        asyncio.create_task(self.status_loop())

        async with websockets.serve(self.handler, "0.0.0.0", self.ws_port):
            await asyncio.Future()  # run forever

# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Base System WebSocket-to-Modbus bridge"
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Serial port (e.g. /dev/tty.usbmodem1234 or COM3). "
             "Auto-detected if not specified.",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=8765,
        help="WebSocket server port (default: 8765)",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List available serial ports and exit",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Task 6: List ports and exit
    if args.list_ports:
        list_ports()
        sys.exit(0)

    serial_port = args.port
    if serial_port is None:
        serial_port = find_serial_port()
        if serial_port:
            log.info(f"Using auto-detected port: {serial_port}")
        else:
            log.warning(
                "No serial port detected. "
                "Connect STM32 and use --port to specify manually, "
                "or connect via the UI."
            )

    server = BaseSystemServer(
        serial_port=serial_port,
        ws_port=args.ws_port,
    )

    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        log.info("Server stopped by user")
