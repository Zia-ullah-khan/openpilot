#!/usr/bin/env python3

#Receive data from remote server like the following (Steering, Gas/Brake, Buttons)
#and publish to joystickd. Optionally stream video back to client.

import sys
import os
import socket
import struct
import threading
import time
import argparse
import traceback

# Button bit positions (must match TestRemotecontrol.py)
BTN_CANCEL = 0
BTN_LEFT_BLINKER = 1
BTN_RIGHT_BLINKER = 2
BTN_GEAR_UP = 3
BTN_GEAR_DOWN = 4
BTN_CRUISE_UP = 5
BTN_CRUISE_DOWN = 6
BTN_CRUISE_MAIN = 7

# Telemetry wire format (little-endian, standard sizes, no native padding)
# speed_mph, steering_angle, engaged, gear, left_blinker, right_blinker,
# brake_pressed, gas_pressed, cruise_speed, lat_active, long_active, current_speed
TELEMETRY_FORMAT = '<ffBBBBBBfBBf'

# Import messaging on comma only
messaging = None
VisionIpcClient = None
VisionStreamType = None

if sys.platform != 'win32':
  sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
  from cereal import messaging
  try:
    from msgq.visionipc import VisionIpcClient, VisionStreamType
  except ImportError:
    pass


def video_stream_thread(host, port, camera='road', fps=20, width=640, height=360, jpeg_quality=60):
  """Stream video frames via TCP socket."""
  if VisionIpcClient is None:
    print("VisionIpcClient not available, video streaming disabled")
    return

  import importlib
  import numpy as np

  try:
    cv2 = importlib.import_module('cv2')
  except Exception:
    print("OpenCV (cv2) is required for high-FPS streaming. Install opencv-python on this runtime.")
    return

  def c_contig(arr):
    return arr if arr.flags.c_contiguous else np.ascontiguousarray(arr)

  def vipc_buf_to_bgr(buf):
    """Convert VisionIPC YUV buffer to BGR quickly, handling stride padding."""
    if not hasattr(buf, 'uv_offset') or not hasattr(buf, 'stride'):
      # Fallback path for legacy/simple I420 buffers (packed, no stride metadata).
      h, w = int(buf.height), int(buf.width)
      flat = np.frombuffer(bytes(buf.data), dtype=np.uint8)
      yuv = flat.reshape((h * 3 // 2, w))
      return c_contig(cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420))

    h, w = int(buf.height), int(buf.width)
    stride = int(buf.stride)
    raw = np.frombuffer(bytes(buf.data), dtype=np.uint8)

    # Build compact I420 from stride-padded Y + interleaved UV (NV12-like) planes.
    y_plane = raw[:buf.uv_offset].reshape((-1, stride))[:h, :w].copy()
    uv_interleaved = raw[buf.uv_offset:].reshape((-1, stride))[:h // 2, :w]
    u_plane = uv_interleaved[:, 0::2].copy()
    v_plane = uv_interleaved[:, 1::2].copy()

    i420 = np.empty((h * 3 // 2, w), dtype=np.uint8)
    i420[:h, :] = y_plane
    i420[h:h + h // 4, :] = u_plane.reshape(-1, w)
    i420[h + h // 4:, :] = v_plane.reshape(-1, w)
    return c_contig(cv2.cvtColor(i420, cv2.COLOR_YUV2BGR_I420))

  # Map camera name to stream type
  stream_map = {
    'road': VisionStreamType.VISION_STREAM_ROAD,
    'wide': VisionStreamType.VISION_STREAM_WIDE_ROAD,
    'driver': VisionStreamType.VISION_STREAM_DRIVER,
  }

  if camera not in stream_map:
    print(f"Unknown camera: {camera}, using 'road'")
    camera = 'road'

  stream_type = stream_map[camera]

  print(f"Starting video stream server on {host}:{port} (camera: {camera}, {fps} fps)...")

  # Connect to vision IPC
  vipc = VisionIpcClient("camerad", stream_type, True)
  if not vipc.connect(True):
    print("Failed to connect to camerad")
    return

  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    print(f"Video stream waiting for connection on {port}...")

    while True:
      try:
        conn, addr = server.accept()
        print(f"Video client connected: {addr}")

        with conn:
          frame_interval = 1.0 / fps
          while True:
            start = time.time()

            # Get frame from vision IPC
            buf = vipc.recv()
            if buf is None:
              time.sleep(0.01)
              continue

            # Convert YUV -> BGR and encode with OpenCV for significantly better FPS.
            frame = vipc_buf_to_bgr(buf)
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            if not ok:
              continue
            jpeg_bytes = jpeg.tobytes()

            # Send frame: 4-byte length prefix + JPEG data
            try:
              conn.sendall(struct.pack('>I', len(jpeg_bytes)))
              conn.sendall(jpeg_bytes)
            except (BrokenPipeError, ConnectionResetError):
              print("Video client disconnected")
              break

            # Rate limit
            elapsed = time.time() - start
            if elapsed < frame_interval:
              time.sleep(frame_interval - elapsed)

      except Exception as e:
        print(f"Video stream error: {e}")
        print(traceback.format_exc())
        time.sleep(1)


def telemetry_stream_thread(host, port, rate=20):
  """Stream car telemetry data via TCP socket."""
  if messaging is None:
    print("Messaging not available, telemetry streaming disabled")
    return

  sm = messaging.SubMaster(['carState', 'selfdriveState', 'controlsState', 'carControl'])

  print(f"Starting telemetry stream server on {host}:{port} ({rate} Hz)...")

  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    print(f"Telemetry stream waiting for connection on {port}...")

    while True:
      try:
        conn, addr = server.accept()
        print(f"Telemetry client connected: {addr}")

        with conn:
          interval = 1.0 / rate
          while True:
            start = time.time()

            sm.update(0)
            cs = sm['carState']
            ss = sm['selfdriveState']

            # Pack telemetry data
            # Format: speed_mph(f), steering_angle(f), engaged(B), gear(B),
            #         left_blinker(B), right_blinker(B), brake_pressed(B), gas_pressed(B),
            #         cruise_speed(f), lat_active(B), long_active(B), current_speed(f)
            speed_mph = cs.vEgo * 2.237  # m/s to mph
            steering_angle = cs.steeringAngleDeg
            engaged = ss.enabled

            # Gear: 0=unknown, 1=park, 2=drive, 3=neutral, 4=reverse
            gear_map = {'park': 1, 'drive': 2, 'neutral': 3, 'reverse': 4}
            gear = gear_map.get(str(cs.gearShifter).split('.')[-1].lower(), 0)

            left_blinker = cs.leftBlinker
            right_blinker = cs.rightBlinker
            brake_pressed = cs.brakePressed
            gas_pressed = cs.gasPressed
            current_speed = cs.vEgo * 2.237 # m/s to mph
            cruise_speed = cs.cruiseState.speed * 2.237
            lat_active = ss.active
            long_active = ss.enabled

            data = struct.pack(TELEMETRY_FORMAT,
                              speed_mph, steering_angle,
                              engaged, gear, left_blinker, right_blinker,
                              brake_pressed, gas_pressed, cruise_speed,
                              lat_active, long_active, current_speed)

            try:
              conn.sendall(data)
            except (BrokenPipeError, ConnectionResetError):
              print("Telemetry client disconnected")
              break

            # Rate limit
            elapsed = time.time() - start
            if elapsed < interval:
              time.sleep(interval - elapsed)

      except Exception as e:
        print(f"Telemetry stream error: {e}")
        time.sleep(1)


def remote_control_thread(control_port=6969):
  pm = None
  if messaging is not None:
    sm = messaging.SubMaster(['testJoystick'], frequency=1. / 0.1)
    pm = messaging.PubMaster(['testJoystick'])

  def recv_exact(sock, size):
    """Receive exactly size bytes from TCP stream or return None on disconnect."""
    data = b''
    while len(data) < size:
      chunk = sock.recv(size - len(data))
      if not chunk:
        return None
      data += chunk
    return data

  HOST = '0.0.0.0'
  last_print_t = 0.0
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, control_port))
    s.listen()
    print(f"Listening for remote control data on {HOST}:{control_port}...")
    while True:
      conn, addr = s.accept()
      with conn:
        print(f"Connected by {addr}")
        while True:
          # New format: 2 floats + 1 byte = 9 bytes
          data = recv_exact(conn, 9)
          if data is None:
            print("Control client disconnected")
            break

          steering, gas_brake, button_mask = struct.unpack('ffB', data)

          # Decode button states
          cancel = bool(button_mask & (1 << BTN_CANCEL))
          left_blinker = bool(button_mask & (1 << BTN_LEFT_BLINKER))
          right_blinker = bool(button_mask & (1 << BTN_RIGHT_BLINKER))
          gear_up = bool(button_mask & (1 << BTN_GEAR_UP))
          gear_down = bool(button_mask & (1 << BTN_GEAR_DOWN))
          cruise_up = bool(button_mask & (1 << BTN_CRUISE_UP))
          cruise_down = bool(button_mask & (1 << BTN_CRUISE_DOWN))
          cruise_main = bool(button_mask & (1 << BTN_CRUISE_MAIN))

          # Build button status string
          btn_status = []
          if cancel: btn_status.append("CANCEL")
          if left_blinker: btn_status.append("LEFT")
          if right_blinker: btn_status.append("RIGHT")
          if gear_up: btn_status.append("GEAR_UP")
          if gear_down: btn_status.append("GEAR_DOWN")
          if cruise_up: btn_status.append("CRUISE+")
          if cruise_down: btn_status.append("CRUISE-")
          if cruise_main: btn_status.append("MAIN")

          now = time.time()
          if now - last_print_t > 0.25:
            btn_str = f" [{', '.join(btn_status)}]" if btn_status else ""
            print(f"Received: steer={steering:+.2f}, gas_brake={gas_brake:+.2f}{btn_str}")
            last_print_t = now

          if pm is not None:
            # Create and send the joystick message
            joystick_msg = messaging.new_message('testJoystick')
            joystick_msg.valid = True
            joystick_msg.testJoystick.axes = [gas_brake, steering]
            # Send buttons: [cancel, left_blinker, right_blinker, gear_up, gear_down, cruise_up, cruise_down, cruise_main]
            joystick_msg.testJoystick.buttons = [cancel, left_blinker, right_blinker,
                                                  gear_up, gear_down, cruise_up, cruise_down, cruise_main]
            pm.send('testJoystick', joystick_msg)


def main():
  parser = argparse.ArgumentParser(description='Remote Control Server')
  parser.add_argument('--video', action='store_true', help='Enable video streaming')
  parser.add_argument('--video-port', type=int, default=6970, help='Video stream port (default: 6970)')
  parser.add_argument('--video-width', type=int, default=640, help='Video width (default: 640)')
  parser.add_argument('--video-height', type=int, default=360, help='Video height (default: 360)')
  parser.add_argument('--jpeg-quality', type=int, default=60, help='JPEG quality 1-100 (default: 60)')
  parser.add_argument('--telemetry', action='store_true', help='Enable telemetry streaming')
  parser.add_argument('--telemetry-port', type=int, default=6971, help='Telemetry stream port (default: 6971)')
  parser.add_argument('--control-port', type=int, default=6969, help='Control port (default: 6969)')
  parser.add_argument('--camera', choices=['road', 'wide', 'driver'], default='road', help='Camera to stream')
  parser.add_argument('--fps', type=int, default=20, help='Video FPS (default: 20)')
  args = parser.parse_args()

  # Start video stream thread if enabled
  if args.video:
    video_thread = threading.Thread(
      target=video_stream_thread,
      args=('0.0.0.0', args.video_port, args.camera, args.fps, args.video_width, args.video_height, args.jpeg_quality),
      daemon=True
    )
    video_thread.start()

  # Start telemetry stream thread if enabled
  if args.telemetry:
    telemetry_thread = threading.Thread(
      target=telemetry_stream_thread,
      args=('0.0.0.0', args.telemetry_port, 20),
      daemon=True
    )
    telemetry_thread.start()

  # Run control thread
  remote_control_thread(control_port=args.control_port)


if __name__ == "__main__":
  main()