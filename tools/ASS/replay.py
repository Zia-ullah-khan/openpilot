#!/usr/bin/env python3
#replay the saved json file of steering, gas/break, gear, and blinker states
import json
import time
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
def replay_thread():
  params = Params()
  cloudlog.info("replay is waiting for CarParams")
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), messaging.car.CarParams)

  sm = messaging.SubMaster(['carState', 'selfdriveState'], frequency=1. / 20)
  pm = messaging.PubMaster(['carControl'])
  rk = messaging.Ratekeeper(20, print_delay_threshold=None)

  with open("record.json", "r") as f:
    records = json.load(f)

  start_time = time.time()
  for record in records:
    sm.update(0)
    cs = sm['carState']
    ss = sm['selfdriveState']

    # Create carControl message
    cc_msg = messaging.new_message('carControl')
    cc_msg.valid = True
    CC = cc_msg.carControl

    # Set controls based on record
    CC.gas = float(record["gasPressed"])
    CC.brake = float(record["brakePressed"])
    CC.steeringAngleDeg = record["steeringAngle"]
    CC.gearShifter = record["gearShifter"]
    CC.leftBlinker = record["leftBlinker"]
    CC.rightBlinker = record["rightBlinker"]

    # Send control message
    pm.send('carControl', cc_msg)

    rk.keep_time()