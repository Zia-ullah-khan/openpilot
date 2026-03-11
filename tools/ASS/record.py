#!/usr/bin/env python3

#get all steering, gas/break, gear, and blinker states and save to a json file

import json
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
def record_thread():
  params = Params()
  cloudlog.info("record is waiting for CarParams")
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), messaging.car.CarParams)

  sm = messaging.SubMaster(['carState', 'selfdriveState'], frequency=1. / 20)
  rk = messaging.Ratekeeper(20, print_delay_threshold=None)

  records = []
  while True:
    sm.update(0)
    cs = sm['carState']
    ss = sm['selfdriveState']

    record = {
      "vEgo": cs.vEgo,
      "steeringAngle": cs.steeringAngle,
      "engaged": ss.enabled,
      "gearShifter": cs.gearShifter,
      "leftBlinker": cs.leftBlinker,
      "rightBlinker": cs.rightBlinker,
      "brakePressed": cs.brakePressed,
      "gasPressed": cs.gasPressed,
      "cruiseSpeed": cs.cruiseState.speed,
      "latActive": ss.active,
      "longActive": ss.enabled
    }
    records.append(record)

    rk.keep_time()

  with open("record.json", "w") as f:
    json.dump(records, f, indent=2)