# Base System

WebSocket-to-Modbus bridge for the FRA502 Circular Pick and Place Robot.

```
Browser UI (port 3000) ─── WebSocket ──► Backend (port 8765) ─── Modbus RTU ──► STM32
```

---

## Files

```
basesystem/
├── docker-compose.yml      # Start frontend container
├── .env.example            # Copy to .env and set serial port
├── frontend-image.tar      # Web UI Docker image
└── backend/
    ├── Dockerfile
    ├── main.py             # WebSocket server
    ├── protocol.py         # Modbus register map
    └── requirements.txt
```

---

## Quick Start

### Step 1 — Load frontend image

```bash
docker load -i frontend-image.tar
```

### Step 2 — Install backend dependencies (once)

```bash
pip install -r backend/requirements.txt
```

### Step 3 — Start frontend

```bash
docker compose up -d frontend
```

### Step 4 — Start backend (see platform section below)

### Step 5 — Open browser

```
http://localhost:3000
```

---

## Platform-specific connection guide

### macOS (Apple Silicon / Intel)

Docker cannot pass serial ports into containers on macOS.
Run the backend natively instead.

**Find your serial port:**
```bash
ls /dev/tty.usb*
# Example output: /dev/tty.usbmodem1203
```

**Run backend:**
```bash
python backend/main.py --port /dev/tty.usbmodem1203
```

**Or auto-detect port:**
```bash
python backend/main.py
```

**Checklist before connecting:**
- STM32 USB plugged into this Mac (not Windows)
- STM32CubeIDE Debugger stopped and disconnected
- Serial port visible in `ls /dev/tty.usb*`

---

### Windows

Docker Desktop on Windows cannot pass serial ports into containers directly.
Run the backend natively.

**Find your serial port:**
1. Open Device Manager
2. Expand Ports (COM & LPT)
3. Find STMicroelectronics STLink Virtual COM Port (COMx)
4. Note the COM number e.g. COM3

**Run backend:**
```bash
pip install -r backend/requirements.txt
docker compose up -d frontend
python backend/main.py --port COM3
```

**Select the same COM number in the browser UI when connecting.**

**Checklist before connecting:**
- STM32 USB plugged into this Windows machine
- STM32CubeIDE Debugger stopped and disconnected
- Only one instance of main.py running:
  ```bash
  taskkill /F /IM main.py
  python backend/main.py --port COM3
  ```

---

### Linux

**Find your serial port:**
```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
# Example output: /dev/ttyACM0
```

**Option A — Run backend natively:**
```bash
python backend/main.py --port /dev/ttyACM0
```

**Option B — Run everything in Docker:**

If your user is not in the `dialout` group, add it first:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for this to take effect
```

Set serial port in `.env`:
```bash
cp .env.example .env
# Edit .env: SERIAL_PORT=/dev/ttyACM0
```

Then start everything:
```bash
docker compose up -d
```

**Checklist before connecting:**
- STM32 USB plugged into this machine
- STM32CubeIDE Debugger stopped and disconnected
- Serial port visible in `ls /dev/ttyACM*`

---

## Verify everything is running

Run these commands to confirm all services are up:

```bash
# Serial port visible
ls /dev/tty.usb*          # macOS
ls /dev/ttyACM*           # Linux

# Frontend container running
docker ps | grep frontend

# Backend process running
ps aux | grep main.py

# Ports in use
lsof -i :3000             # macOS/Linux
lsof -i :8765             # macOS/Linux
```

Expected results:
- Port 3000 — nginx (Frontend)
- Port 8765 — python (Backend)
- Browser shows Connection dialog at http://localhost:3000

---

## Common commands

| Action | Command |
|--------|---------|
| Start frontend | `docker compose up -d frontend` |
| Stop frontend | `docker compose down` |
| View frontend logs | `docker compose logs -f frontend` |
| List serial ports | `python backend/main.py --list-ports` |
| Restart backend | Kill and re-run `python backend/main.py` |

---

## Troubleshooting

**Browser shows "Server is offline"**
- Backend is not running — start `python backend/main.py`
- Port 8765 already in use — kill existing process first

**Connected but Status ERROR on all commands**
- STM32 Debugger is still running — stop it in STM32CubeIDE
- Wrong COM port — check Device Manager (Windows) or `ls /dev/tty.usb*` (macOS)
- printf redirected to wrong UART — ensure `_write()` uses `huart3` not `hlpuart1`

**macOS: No serial port found**
- STM32 USB is plugged into Windows, not Mac
- Try unplugging and replugging the USB cable

**Windows: Address already in use**
```bash
taskkill /F /IM main.exe
taskkill /F /IM python.exe
python backend/main.py --port COM3
```

---

## STM32 firmware settings

| Setting    | Value                   |
|-----------|-------------------------|
| Baud rate  | 19200                   |
| Data bits  | 8                       |
| Parity     | Even                    |
| Stop bits  | 1                       |
| Slave ID   | 21                      |
| UART       | LPUART1 via ST-Link VCP |
| Modbus     | RTU Slave mode          |