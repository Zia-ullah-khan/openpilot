# On Comma Device (Server)
# Basic - control only (no video)
python tools/joystick/remoteControl.py

# With video streaming (road camera)
python tools/joystick/remoteControl.py --video

# With wide camera
python tools/joystick/remoteControl.py --video --camera wide

# With driver camera at 15 fps
python tools/joystick/remoteControl.py --video --camera driver --fps 15

# Custom ports
python tools/joystick/remoteControl.py --video --control-port 6969 --video-port 6970

# On Your PC (Client)
# Calibration - interactive axis/button mapping saved to JSON
python tools/joystick/wheel_calibration.py

# Connect to comma device (replace with your comma's IP)
python TestRemotecontrol.py --host 192.168.1.X --no-server

# Connect to comma with explicit calibration file
python TestRemotecontrol.py --host 192.168.1.X --calibration-file tools/joystick/wheel_calibration.json --no-server

# Connect with video
python TestRemotecontrol.py --host 192.168.1.X --video --no-server

# Local testing (starts local server too)
python TestRemotecontrol.py

# Local with video display (no actual video without comma)
python TestRemotecontrol.py --video

## Best Commands:
# Server (comma device)
python remoteControl.py --video --telemetry

# Client (PC)
python TestRemotecontrol.py --host <comma_ip> --video --telemetry --no-server

# To enable camera when comma is not connected to car:
cd /data/openpilot
python -c "from openpilot.common.params import Params; p=Params(); p.put_bool('IsDriverViewEnabled', True); p.put_bool('JoystickDebugMode', True); print('enabled')"