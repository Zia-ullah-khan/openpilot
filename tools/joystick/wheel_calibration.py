#!/usr/bin/env python3

import argparse
import json
import sys
import time
from pathlib import Path

try:
  import pygame
except ImportError:
  print("Please install pygame: pip install pygame")
  sys.exit(1)

DEFAULT_OUTPUT = Path(__file__).with_name("wheel_calibration.json")

AXIS_PROMPTS = [
  ("steering", "Move axis for steering left/right"),
  ("gas", "Press and release GAS pedal"),
  ("brake", "Press and release BRAKE pedal"),
]

BUTTON_PROMPTS = [
  ("cancel", "Press button for CANCEL"),
  ("left_blinker", "Press button for LEFT BLINKER"),
  ("right_blinker", "Press button for RIGHT BLINKER"),
  ("gear_up", "Press button for GEAR UP"),
  ("gear_down", "Press button for GEAR DOWN"),
  ("cruise_up", "Press button for CRUISE UP"),
  ("cruise_down", "Press button for CRUISE DOWN"),
  ("cruise_main", "Press button for CRUISE MAIN"),
]


def choose_wheel():
  pygame.init()
  pygame.joystick.init()

  count = pygame.joystick.get_count()
  if count == 0:
    raise RuntimeError("No controller found. Connect your wheel and try again.")

  preferred = None
  for i in range(count):
    joy = pygame.joystick.Joystick(i)
    joy.init()
    name = joy.get_name()
    lname = name.lower()
    print(f"Found controller {i}: {name}")
    if preferred is None and ("moza" in lname or "r5" in lname or "wheel" in lname):
      preferred = joy

  if preferred is not None:
    print(f"Using wheel: {preferred.get_name()}")
    return preferred

  joy = pygame.joystick.Joystick(0)
  joy.init()
  print(f"Using first controller: {joy.get_name()}")
  return joy


def pump():
  pygame.event.pump()
  time.sleep(0.01)


def read_axes(joy):
  return [joy.get_axis(i) for i in range(joy.get_numaxes())]


def render_axis_line(values):
  pairs = [f"A{i}:{v:+.3f}" for i, v in enumerate(values)]
  return " | ".join(pairs)


def sample_axis_ranges(joy, seconds):
  min_vals = [1.0] * joy.get_numaxes()
  max_vals = [-1.0] * joy.get_numaxes()
  start = time.time()
  last_print = 0.0

  while time.time() - start < seconds:
    pump()
    vals = read_axes(joy)
    now = time.time()

    for i, v in enumerate(vals):
      if v < min_vals[i]:
        min_vals[i] = v
      if v > max_vals[i]:
        max_vals[i] = v

    if now - last_print > 0.12:
      print("\r" + render_axis_line(vals) + " " * 10, end="", flush=True)
      last_print = now

  print()
  spans = [max_vals[i] - min_vals[i] for i in range(joy.get_numaxes())]
  return min_vals, max_vals, spans


def confirm_axis(joy, axis_idx, control_name):
  print(f"Now verify axis {axis_idx} for {control_name} (move it for 3 seconds)...")
  min_vals, max_vals, spans = sample_axis_ranges(joy, 3.0)
  min_v = min_vals[axis_idx]
  max_v = max_vals[axis_idx]
  span = spans[axis_idx]
  print(f"Axis {axis_idx}: min={min_v:+.3f}, max={max_v:+.3f}, span={span:.3f}")
  ans = input("Is this correct? [Y/n]: ").strip().lower()
  return ans != "n"


def detect_axis_by_motion(joy, control_name, prompt, seconds, required_span, require_both_sides=False):
  print(f"\n{prompt}")
  print("Move only this control while values update below.")

  min_vals, max_vals, spans = sample_axis_ranges(joy, seconds)
  axis_idx = max(range(len(spans)), key=lambda i: spans[i])

  min_v = min_vals[axis_idx]
  max_v = max_vals[axis_idx]
  span = spans[axis_idx]
  both_sides_ok = (min_v <= -0.6 and max_v >= 0.6) if require_both_sides else True

  print(f"Detected axis {axis_idx} for {control_name}")
  print(f"Axis {axis_idx}: min={min_v:+.3f}, max={max_v:+.3f}, span={span:.3f}")

  if span < required_span:
    raise RuntimeError(
      f"Movement too small (span {span:.3f}). Move through a larger range and retry."
    )
  if require_both_sides and not both_sides_ok:
    raise RuntimeError(
      "Did not see full left and right motion. Turn full left then full right and retry."
    )

  if not confirm_axis(joy, axis_idx, control_name):
    raise RuntimeError("User rejected detected axis.")

  return axis_idx


def detect_button(joy, prompt, timeout=15.0):
  print(f"\n{prompt}")
  print("Press and release one button...")

  prev = [joy.get_button(i) for i in range(joy.get_numbuttons())]
  start = time.time()

  while time.time() - start < timeout:
    pump()
    for i in range(joy.get_numbuttons()):
      cur = joy.get_button(i)
      if cur and not prev[i]:
        print(f"Detected button {i}")
        ans = input("Is this correct? [Y/n]: ").strip().lower()
        if ans != "n":
          return i
        print("Rejected. Press the correct button...")
      prev[i] = cur

  raise RuntimeError(f"Timed out detecting button for prompt: {prompt}")


def run_calibration(output_path: Path):
  joy = choose_wheel()
  print("\nCalibration starting...")
  print("Keep other controls untouched while each prompt is active.")

  axis_map = {}
  button_map = {}

  while True:
    try:
      axis_map["steering"] = detect_axis_by_motion(
        joy,
        "steering",
        "Move wheel full LEFT then full RIGHT for steering detection",
        seconds=8.0,
        required_span=1.4,
        require_both_sides=True,
      )
      break
    except RuntimeError as e:
      print(e)
      ans = input("Retry steering axis detection? [Y/n]: ").strip().lower()
      if ans == "n":
        raise

  for key, prompt in AXIS_PROMPTS:
    if key == "steering":
      continue

    while True:
      try:
        axis_map[key] = detect_axis_by_motion(
          joy,
          key,
          f"{prompt} fully, then release",
          seconds=7.0,
          required_span=0.55,
          require_both_sides=False,
        )
        break
      except RuntimeError as e:
        print(e)
        ans = input("Retry axis detection? [Y/n]: ").strip().lower()
        if ans == "n":
          raise

  for key, prompt in BUTTON_PROMPTS:
    while True:
      try:
        button_map[key] = detect_button(joy, prompt)
        break
      except RuntimeError as e:
        print(e)
        ans = input("Retry button detection? [Y/n]: ").strip().lower()
        if ans == "n":
          raise

  result = {
    "controller_name": joy.get_name(),
    "created_at_unix": int(time.time()),
    "axes": axis_map,
    "buttons": button_map,
  }

  output_path.parent.mkdir(parents=True, exist_ok=True)
  with output_path.open("w", encoding="utf-8") as f:
    json.dump(result, f, indent=2)
    f.write("\n")

  print(f"\nSaved calibration to: {output_path}")
  print("\nUse it with TestRemotecontrol.py automatically or set --calibration-file.")


def main():
  parser = argparse.ArgumentParser(description="Interactive wheel calibration for remote control")
  parser.add_argument("--output", default=str(DEFAULT_OUTPUT),
                      help=f"Output JSON path (default: {DEFAULT_OUTPUT.name})")
  args = parser.parse_args()

  try:
    run_calibration(Path(args.output))
  except KeyboardInterrupt:
    print("\nCalibration cancelled.")
  finally:
    pygame.quit()


if __name__ == "__main__":
  main()
