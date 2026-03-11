#test remote control
#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import threading
import socket
import struct
import time
import io
import json
from pathlib import Path

try:
  import pygame
except ImportError:
  print("Please install pygame: pip install pygame")
  sys.exit(1)

try:
  import cv2
  import numpy as np
  CV2_AVAILABLE = True
except ImportError:
  CV2_AVAILABLE = False

#send data to remotecontrol server that is on port 6969 over the network

from tools.joystick.remoteControl import remote_control_thread

# Default controls (used when no calibration JSON is available)
DEFAULT_AXIS_MAP = {
  'steering': 0,
  'gas': 2,
  'brake': 3,
}

DEFAULT_BUTTON_MAP = {
  'cancel': 0,
  'left_blinker': 1,
  'right_blinker': 2,
  'gear_up': 3,
  'gear_down': 4,
  'cruise_up': 5,
  'cruise_down': 6,
  'cruise_main': 7,
}

DEFAULT_CALIBRATION_FILE = Path(__file__).with_name('wheel_calibration.json')

# Keyboard control settings
KEYBOARD_STEER_SPEED = 2.0  # How fast steering moves with A/D (per second)
KEYBOARD_STEER_RETURN = 3.0  # How fast steering returns to center (per second)
BLINKER_PULSE_SECONDS = 0.35  # Keep blinker command active briefly on button tap

# Global for video frame sharing
latest_frame = None
frame_lock = threading.Lock()

# Global for telemetry sharing
telemetry_data = None
telemetry_lock = threading.Lock()


class TelemetryReceiver:
  """Receives telemetry stream from remote control server."""

  def __init__(self, host, port):
    self.host = host
    self.port = port
    self.running = False
    self.thread = None
    self.connected = False

    # Telemetry values
    self.speed_mph = 0.0
    self.steering_angle = 0.0
    self.engaged = False
    self.gear = 0  # 0=unknown, 1=park, 2=drive, 3=neutral, 4=reverse
    self.left_blinker = False
    self.right_blinker = False
    self.brake_pressed = False
    self.gas_pressed = False
    self.cruise_speed = 0.0
    self.lat_active = False
    self.long_active = False

  def start(self):
    self.running = True
    self.thread = threading.Thread(target=self._receive_loop, daemon=True)
    self.thread.start()

  def stop(self):
    self.running = False

  def get_gear_string(self):
    gear_names = {0: '?', 1: 'P', 2: 'D', 3: 'N', 4: 'R'}
    return gear_names.get(self.gear, '?')

  def _receive_loop(self):
    global telemetry_data

    while self.running:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
          s.settimeout(5)
          print(f"Connecting to telemetry stream at {self.host}:{self.port}...")
          s.connect((self.host, self.port))
          self.connected = True
          print("Telemetry stream connected!")

          while self.running:
            # Receive telemetry packet (18 bytes)
            data = self._recv_exact(s, 18)
            if not data:
              break

            # Unpack: 2 floats + 8 bytes + 1 float + 2 bytes
            (self.speed_mph, self.steering_angle,
             self.engaged, self.gear, self.left_blinker, self.right_blinker,
             self.brake_pressed, self.gas_pressed, self.cruise_speed,
             self.lat_active, self.long_active) = struct.unpack('ffBBBBBBfBB', data)

            with telemetry_lock:
              telemetry_data = self

      except socket.timeout:
        print("Telemetry stream connection timed out, retrying...")
      except ConnectionRefusedError:
        print(f"Telemetry stream refused on {self.host}:{self.port}, retrying in 2s...")
        time.sleep(2)
      except Exception as e:
        print(f"Telemetry stream error: {e}")
        time.sleep(1)
      finally:
        self.connected = False

  def _recv_exact(self, sock, n):
    """Receive exactly n bytes."""
    data = b''
    while len(data) < n:
      try:
        chunk = sock.recv(n - len(data))
        if not chunk:
          return None
        data += chunk
      except:
        return None
    return data


class VideoReceiver:
  """Receives video stream from remote control server."""

  def __init__(self, host, port):
    self.host = host
    self.port = port
    self.running = False
    self.thread = None
    self.frame = None
    self.fps = 0
    self.connected = False

  def start(self):
    self.running = True
    self.thread = threading.Thread(target=self._receive_loop, daemon=True)
    self.thread.start()

  def stop(self):
    self.running = False

  def _receive_loop(self):
    global latest_frame
    frame_count = 0
    fps_start = time.time()

    while self.running:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
          s.settimeout(5)
          print(f"Connecting to video stream at {self.host}:{self.port}...")
          s.connect((self.host, self.port))
          self.connected = True
          print("Video stream connected!")

          while self.running:
            # Read 4-byte length prefix
            length_data = self._recv_exact(s, 4)
            if not length_data:
              break

            frame_length = struct.unpack('>I', length_data)[0]

            # Read JPEG data
            jpeg_data = self._recv_exact(s, frame_length)
            if not jpeg_data:
              break

            # Decode JPEG
            if CV2_AVAILABLE:
              nparr = np.frombuffer(jpeg_data, np.uint8)
              frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
              if frame is not None:
                # Convert BGR to RGB for pygame
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                with frame_lock:
                  latest_frame = frame
            else:
              # Use pygame to decode JPEG
              frame_io = io.BytesIO(jpeg_data)
              try:
                surface = pygame.image.load(frame_io)
                with frame_lock:
                  latest_frame = surface
              except:
                pass

            # Calculate FPS
            frame_count += 1
            elapsed = time.time() - fps_start
            if elapsed >= 1.0:
              self.fps = frame_count / elapsed
              frame_count = 0
              fps_start = time.time()

      except socket.timeout:
        print("Video stream connection timed out, retrying...")
      except ConnectionRefusedError:
        print(f"Video stream refused on {self.host}:{self.port}, retrying in 2s...")
        time.sleep(2)
      except Exception as e:
        print(f"Video stream error: {e}")
        time.sleep(1)
      finally:
        self.connected = False

  def _recv_exact(self, sock, n):
    """Receive exactly n bytes."""
    data = b''
    while len(data) < n:
      try:
        chunk = sock.recv(n - len(data))
        if not chunk:
          return None
        data += chunk
      except:
        return None
    return data


class MozaR5Wheel:
  """MOZA R5 Racing Wheel controller."""

  def __init__(self, calibration_config=None):
    pygame.init()
    pygame.joystick.init()

    self.wheel = None
    self.steering = 0.0
    self.gas = 0.0
    self.brake = 0.0
    self.buttons = []
    self.calibration_config = calibration_config or {}

    # Find the MOZA wheel
    for i in range(pygame.joystick.get_count()):
      joy = pygame.joystick.Joystick(i)
      joy.init()
      name = joy.get_name().lower()
      print(f"Found controller {i}: {joy.get_name()}")
      if 'moza' in name or 'r5' in name or 'wheel' in name:
        self.wheel = joy
        print(f"Using: {joy.get_name()}")
        break

    if self.wheel is None and pygame.joystick.get_count() > 0:
      # Use first available controller
      self.wheel = pygame.joystick.Joystick(0)
      self.wheel.init()
      print(f"Using first controller: {self.wheel.get_name()}")

    if self.wheel is None:
      raise RuntimeError("No controller found! Please connect your MOZA R5 wheel.")

    print(f"  Axes: {self.wheel.get_numaxes()}")
    print(f"  Buttons: {self.wheel.get_numbuttons()}")
    print(f"  Hats: {self.wheel.get_numhats()}")

    axis_map = self.calibration_config.get('axes', {})
    self.steering_axis = int(axis_map.get('steering', DEFAULT_AXIS_MAP['steering']))
    self.gas_axis = int(axis_map.get('gas', DEFAULT_AXIS_MAP['gas']))
    self.brake_axis = int(axis_map.get('brake', DEFAULT_AXIS_MAP['brake']))

    button_map = self.calibration_config.get('buttons', {})
    self.button_map = {
      key: int(button_map.get(key, default_idx))
      for key, default_idx in DEFAULT_BUTTON_MAP.items()
    }

    # Initialize buttons list
    self.buttons = [False] * self.wheel.get_numbuttons()

  def update(self):
    """Poll wheel state and update values."""
    pygame.event.pump()

    if self.wheel:
      # Steering: -1 (left) to 1 (right)
      self.steering = self.wheel.get_axis(self.steering_axis)

      # Gas pedal: typically 0 to 1, but may be -1 to 1
      if self.gas_axis < self.wheel.get_numaxes():
        raw_gas = self.wheel.get_axis(self.gas_axis)
        # Convert from -1..1 to 0..1 if needed
        self.gas = (raw_gas + 1) / 2

      # Brake pedal
      if self.brake_axis < self.wheel.get_numaxes():
        raw_brake = self.wheel.get_axis(self.brake_axis)
        self.brake = (raw_brake + 1) / 2

      # Read all buttons
      for i in range(self.wheel.get_numbuttons()):
        self.buttons[i] = self.wheel.get_button(i)

  def get_gas_brake(self):
    """Return combined gas/brake value: positive = gas, negative = brake."""
    return self.gas - self.brake

  def get_button_by_name(self, control_name):
    """Get button state from logical control name."""
    idx = self.button_map.get(control_name, -1)
    if 0 <= idx < len(self.buttons):
      return self.buttons[idx]
    return False

  def close(self):
    pygame.quit()


def load_calibration(calibration_file):
  path = Path(calibration_file)
  if not path.exists():
    print(f"Calibration file not found: {path}. Using default mapping.")
    return {}

  try:
    with path.open('r', encoding='utf-8') as f:
      config = json.load(f)
    print(f"Loaded calibration from {path}")
    return config
  except Exception as e:
    print(f"Failed to load calibration from {path}: {e}")
    print("Using default mapping.")
    return {}


def send_commands(host='127.0.0.1', port=6969, video_port=None, telemetry_port=None,
                  use_keyboard=False, calibration_file=DEFAULT_CALIBRATION_FILE):
  """Send steering, gas/brake, and button commands to the remote control server."""
  global latest_frame, telemetry_data

  time.sleep(0.5)  # Wait for server to start

  # Initialize pygame (needed for keyboard mode even without wheel)
  pygame.init()

  wheel = None
  calibration_config = load_calibration(calibration_file)
  if not use_keyboard:
    try:
      wheel = MozaR5Wheel(calibration_config=calibration_config)
    except RuntimeError as e:
      print(f"Warning: {e}")
      print("Falling back to keyboard control (WASD + Q/E)")
      use_keyboard = True

  # Keyboard state
  kb_steering = 0.0
  kb_gas = 0.0
  kb_brake = 0.0
  kb_left_blinker = False
  kb_right_blinker = False
  kb_cancel = False
  last_time = time.time()
  left_blinker_until = 0.0
  right_blinker_until = 0.0
  prev_wheel_left = False
  prev_wheel_right = False

  # Start video receiver if enabled
  video_receiver = None
  if video_port:
    video_receiver = VideoReceiver(host, video_port)
    video_receiver.start()

  # Start telemetry receiver if enabled
  telemetry_receiver = None
  if telemetry_port:
    telemetry_receiver = TelemetryReceiver(host, telemetry_port)
    telemetry_receiver.start()

  # Setup pygame display for video/telemetry/keyboard
  screen = None
  clock = None
  if video_port or telemetry_port or use_keyboard:
    title = "Remote Control"
    if use_keyboard:
      title += " - Keyboard Mode (WASD + Q/E)"
    pygame.display.set_caption(title)
    screen = pygame.display.set_mode((640, 360), pygame.RESIZABLE)
    clock = pygame.time.Clock()

  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((host, port))
    print(f"Connected to {host}:{port}")
    if use_keyboard:
      print("Keyboard controls:")
      print("  W/S = Gas/Brake")
      print("  A/D = Steer left/right")
      print("  Q/E = Left/Right blinker")
      print("  ESC = Cancel/Disengage")
      print("  Close window or Ctrl+C to exit")
    else:
      print("Use your MOZA R5 wheel to control. Press Ctrl+C or close window to exit.")
      print("\nButton mappings loaded from calibration JSON (or defaults if missing).")

    try:
      running = True
      while running:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Handle pygame events
        for event in pygame.event.get():
          if event.type == pygame.QUIT:
            running = False
          elif event.type == pygame.VIDEORESIZE and screen:
            screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
          elif event.type == pygame.KEYDOWN and use_keyboard:
            if event.key == pygame.K_q:
              kb_left_blinker = True
              left_blinker_until = current_time + BLINKER_PULSE_SECONDS
              right_blinker_until = 0.0
            elif event.key == pygame.K_e:
              kb_right_blinker = True
              right_blinker_until = current_time + BLINKER_PULSE_SECONDS
              left_blinker_until = 0.0
            elif event.key == pygame.K_ESCAPE:
              kb_cancel = True
          elif event.type == pygame.KEYUP and use_keyboard:
            if event.key == pygame.K_q:
              kb_left_blinker = False
            elif event.key == pygame.K_e:
              kb_right_blinker = False
            elif event.key == pygame.K_ESCAPE:
              kb_cancel = False

        # Get keyboard state for continuous inputs
        if use_keyboard:
          keys = pygame.key.get_pressed()

          # Gas/Brake (W/S)
          kb_gas = 1.0 if keys[pygame.K_w] else 0.0
          kb_brake = 1.0 if keys[pygame.K_s] else 0.0

          # Steering (A/D) - gradual movement
          if keys[pygame.K_a]:
            kb_steering -= KEYBOARD_STEER_SPEED * dt
          elif keys[pygame.K_d]:
            kb_steering += KEYBOARD_STEER_SPEED * dt
          else:
            # Return to center
            if kb_steering > 0:
              kb_steering = max(0, kb_steering - KEYBOARD_STEER_RETURN * dt)
            elif kb_steering < 0:
              kb_steering = min(0, kb_steering + KEYBOARD_STEER_RETURN * dt)

          # Clamp steering
          kb_steering = max(-1.0, min(1.0, kb_steering))

          steering = kb_steering
          gas_brake = kb_gas - kb_brake
        else:
          wheel.update()
          steering = wheel.steering
          gas_brake = wheel.get_gas_brake()

        # Pack buttons into a bitmask (up to 8 buttons in one byte)
        button_mask = 0
        if use_keyboard:
          if kb_cancel:
            button_mask |= (1 << 0)
          left_blinker_active = kb_left_blinker or (current_time < left_blinker_until)
          right_blinker_active = kb_right_blinker or (current_time < right_blinker_until)
          if left_blinker_active:
            button_mask |= (1 << 1)
          if right_blinker_active:
            button_mask |= (1 << 2)
        else:
          wheel_left = wheel.get_button_by_name('left_blinker')
          wheel_right = wheel.get_button_by_name('right_blinker')

          # Convert a short tap into a minimum pulse so the receiver always sees it.
          if wheel_left and not prev_wheel_left:
            left_blinker_until = current_time + BLINKER_PULSE_SECONDS
            right_blinker_until = 0.0
          if wheel_right and not prev_wheel_right:
            right_blinker_until = current_time + BLINKER_PULSE_SECONDS
            left_blinker_until = 0.0

          prev_wheel_left = wheel_left
          prev_wheel_right = wheel_right

          if wheel.get_button_by_name('cancel'): button_mask |= (1 << 0)
          left_blinker_active = wheel_left or (current_time < left_blinker_until)
          right_blinker_active = wheel_right or (current_time < right_blinker_until)
          if left_blinker_active: button_mask |= (1 << 1)
          if right_blinker_active: button_mask |= (1 << 2)
          if wheel.get_button_by_name('gear_up'): button_mask |= (1 << 3)
          if wheel.get_button_by_name('gear_down'): button_mask |= (1 << 4)
          if wheel.get_button_by_name('cruise_up'): button_mask |= (1 << 5)
          if wheel.get_button_by_name('cruise_down'): button_mask |= (1 << 6)
          if wheel.get_button_by_name('cruise_main'): button_mask |= (1 << 7)

        # Pack: steering (float), gas_brake (float), buttons (byte)
        data = struct.pack('ffB', steering, gas_brake, button_mask)
        s.sendall(data)

        # Display video if available
        if screen:
          frame_surface = None
          if video_receiver:
            with frame_lock:
              frame = latest_frame

            if frame is not None:
              if CV2_AVAILABLE and isinstance(frame, np.ndarray):
                # Convert numpy array to pygame surface
                frame_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
              elif isinstance(frame, pygame.Surface):
                frame_surface = frame

          if frame_surface is not None:
            # Scale to fit window
            scaled = pygame.transform.scale(frame_surface, screen.get_size())
            screen.blit(scaled, (0, 0))
          else:
            # Always clear when no frame is available, otherwise text overlays accumulate.
            screen.fill((30, 30, 40))

          # Draw HUD overlay
          font = pygame.font.Font(None, 36)

          # Steering bar
          bar_width = 200
          bar_height = 20
          bar_x = screen.get_width() // 2 - bar_width // 2
          bar_y = screen.get_height() - 60
          pygame.draw.rect(screen, (50, 50, 50), (bar_x, bar_y, bar_width, bar_height))
          steer_pos = bar_x + bar_width // 2 + int(steering * bar_width // 2)
          pygame.draw.rect(screen, (0, 255, 0), (steer_pos - 5, bar_y, 10, bar_height))

          # Gas/brake indicator
          gb_text = f"Gas/Brake: {gas_brake:+.2f}"
          gb_surface = font.render(gb_text, True, (255, 255, 255))
          screen.blit(gb_surface, (10, screen.get_height() - 40))

          # FPS counter or keyboard mode indicator
          if video_receiver:
            fps_text = f"Video FPS: {video_receiver.fps:.1f}"
            fps_surface = font.render(fps_text, True, (255, 255, 0))
            screen.blit(fps_surface, (10, 10))
          elif use_keyboard:
            mode_surface = font.render("KEYBOARD MODE", True, (255, 200, 0))
            screen.blit(mode_surface, (10, 10))

          # Keyboard WASD display
          if use_keyboard:
            keys = pygame.key.get_pressed()
            wasd_x = screen.get_width() - 120
            wasd_y = screen.get_height() - 120
            key_size = 30

            # W key
            w_color = (0, 200, 0) if keys[pygame.K_w] else (80, 80, 80)
            pygame.draw.rect(screen, w_color, (wasd_x + key_size + 5, wasd_y, key_size, key_size))
            screen.blit(font.render("W", True, (255,255,255)), (wasd_x + key_size + 12, wasd_y + 5))

            # A key
            a_color = (0, 200, 0) if keys[pygame.K_a] else (80, 80, 80)
            pygame.draw.rect(screen, a_color, (wasd_x, wasd_y + key_size + 5, key_size, key_size))
            screen.blit(font.render("A", True, (255,255,255)), (wasd_x + 7, wasd_y + key_size + 10))

            # S key
            s_color = (200, 0, 0) if keys[pygame.K_s] else (80, 80, 80)
            pygame.draw.rect(screen, s_color, (wasd_x + key_size + 5, wasd_y + key_size + 5, key_size, key_size))
            screen.blit(font.render("S", True, (255,255,255)), (wasd_x + key_size + 12, wasd_y + key_size + 10))

            # D key
            d_color = (0, 200, 0) if keys[pygame.K_d] else (80, 80, 80)
            pygame.draw.rect(screen, d_color, (wasd_x + 2*(key_size + 5), wasd_y + key_size + 5, key_size, key_size))
            screen.blit(font.render("D", True, (255,255,255)), (wasd_x + 2*(key_size + 5) + 7, wasd_y + key_size + 10))

            # Q/E blinker keys
            q_color = (255, 165, 0) if kb_left_blinker else (80, 80, 80)
            pygame.draw.rect(screen, q_color, (wasd_x - key_size - 10, wasd_y, key_size, key_size))
            screen.blit(font.render("Q", True, (255,255,255)), (wasd_x - key_size - 3, wasd_y + 5))

            e_color = (255, 165, 0) if kb_right_blinker else (80, 80, 80)
            pygame.draw.rect(screen, e_color, (wasd_x + 3*(key_size + 5), wasd_y, key_size, key_size))
            screen.blit(font.render("E", True, (255,255,255)), (wasd_x + 3*(key_size + 5) + 7, wasd_y + 5))

          # Button indicators
          btn_y = 50
          if button_mask & 0x01:
            screen.blit(font.render("CANCEL", True, (255, 0, 0)), (10, btn_y))
          if button_mask & 0x02:
            screen.blit(font.render("< LEFT", True, (0, 255, 0)), (10, btn_y + 30))
          if button_mask & 0x04:
            screen.blit(font.render("RIGHT >", True, (0, 255, 0)), (screen.get_width() - 100, btn_y + 30))

          # Telemetry display
          with telemetry_lock:
            telem = telemetry_data
          if telem is not None:
            large_font = pygame.font.Font(None, 72)
            med_font = pygame.font.Font(None, 48)

            # Speed (large, center-top)
            speed_text = f"{telem.speed_mph:.0f}"
            speed_surface = large_font.render(speed_text, True, (255, 255, 255))
            speed_rect = speed_surface.get_rect(center=(screen.get_width() // 2, 50))
            screen.blit(speed_surface, speed_rect)
            mph_surface = font.render("MPH", True, (200, 200, 200))
            screen.blit(mph_surface, (speed_rect.right + 5, speed_rect.centery - 10))

            # Gear indicator (right side)
            gear_str = telem.get_gear_string()
            gear_color = (255, 255, 0) if gear_str == 'R' else (0, 255, 0)
            gear_surface = med_font.render(gear_str, True, gear_color)
            screen.blit(gear_surface, (screen.get_width() - 60, 50))

            # Engaged status
            status_color = (0, 255, 0) if telem.engaged else (255, 100, 100)
            status_text = "ENGAGED" if telem.engaged else "STANDBY"
            status_surface = font.render(status_text, True, status_color)
            screen.blit(status_surface, (screen.get_width() // 2 - 50, 100))

            # Lat/Long active indicators
            if telem.lat_active:
              screen.blit(font.render("LAT", True, (0, 200, 255)), (screen.get_width() - 100, 100))
            if telem.long_active:
              screen.blit(font.render("LONG", True, (0, 200, 255)), (screen.get_width() - 100, 130))

            # Blinker indicators from telemetry
            if telem.left_blinker:
              pygame.draw.polygon(screen, (0, 255, 100), [(30, 150), (10, 165), (30, 180)])
            if telem.right_blinker:
              rw = screen.get_width()
              pygame.draw.polygon(screen, (0, 255, 100), [(rw-30, 150), (rw-10, 165), (rw-30, 180)])

            # Brake/Gas indicators
            if telem.brake_pressed:
              pygame.draw.rect(screen, (255, 50, 50), (10, screen.get_height() - 100, 30, 30))
              screen.blit(font.render("B", True, (255, 255, 255)), (17, screen.get_height() - 95))
            if telem.gas_pressed:
              pygame.draw.rect(screen, (50, 255, 50), (50, screen.get_height() - 100, 30, 30))
              screen.blit(font.render("G", True, (255, 255, 255)), (57, screen.get_height() - 95))

            # Cruise speed (if set)
            if telem.cruise_speed > 0:
              cruise_text = f"SET: {telem.cruise_speed:.0f}"
              screen.blit(font.render(cruise_text, True, (100, 200, 255)), (screen.get_width() // 2 - 40, 130))

            # Steering angle from car
            steer_angle_text = f"Angle: {telem.steering_angle:.1f}°"
            screen.blit(font.render(steer_angle_text, True, (200, 200, 200)), (screen.get_width() - 150, screen.get_height() - 40))

          pygame.display.flip()
          clock.tick(60)
        else:
          # Console-only mode
          btn_str = ""
          if button_mask & 0x01: btn_str += " [CANCEL]"
          if button_mask & 0x02: btn_str += " [LEFT]"
          if button_mask & 0x04: btn_str += " [RIGHT]"
          print(f"\rSteer: {steering:+.2f} | Gas/Brake: {gas_brake:+.2f}{btn_str:<30}", end='', flush=True)
          time.sleep(0.05)  # 20 Hz update rate

    except KeyboardInterrupt:
      print("\nStopping...")
    finally:
      if wheel:
        wheel.close()
      else:
        pygame.quit()
      if video_receiver:
        video_receiver.stop()
      if telemetry_receiver:
        telemetry_receiver.stop()


def main():
  import argparse
  parser = argparse.ArgumentParser(description='MOZA R5 Wheel Remote Control')
  parser.add_argument('--host', default='127.0.0.1', help='Remote server host (default: 127.0.0.1)')
  parser.add_argument('--port', type=int, default=6969, help='Remote server port (default: 6969)')
  parser.add_argument('--video', action='store_true', help='Enable video streaming')
  parser.add_argument('--video-port', type=int, default=6970, help='Video stream port (default: 6970)')
  parser.add_argument('--telemetry', action='store_true', help='Enable telemetry streaming')
  parser.add_argument('--telemetry-port', type=int, default=6971, help='Telemetry stream port (default: 6971)')
  parser.add_argument('--keyboard', action='store_true', help='Use keyboard (WASD+QE) instead of wheel')
  parser.add_argument('--no-server', action='store_true', help='Connect to existing server (don\'t start local)')
  parser.add_argument('--calibration-file', default=str(DEFAULT_CALIBRATION_FILE),
                      help=f'Wheel calibration JSON path (default: {DEFAULT_CALIBRATION_FILE.name})')
  args = parser.parse_args()

  if not args.no_server:
    # Start server in background thread
    remote_thread = threading.Thread(target=remote_control_thread, daemon=True)
    remote_thread.start()

  # Send commands from main thread
  video_port = args.video_port if args.video else None
  telemetry_port = args.telemetry_port if args.telemetry else None
  send_commands(host=args.host, port=args.port, video_port=video_port,
                telemetry_port=telemetry_port, use_keyboard=args.keyboard,
                calibration_file=args.calibration_file)

  print("Done sending commands")


if __name__ == "__main__":
  main()