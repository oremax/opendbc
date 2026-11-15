"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Adapted for this fork: only the MADS multi-finger screen button is ported.
sunnypilot's speed limit readout (CarStateSP) is not part of this port, and
the vehicle bus parser is the one the base CarState already creates for
HW4 gen2 (Bus.adas); the button is read only when that parser exists.
"""
from opendbc.car import Bus, create_button_events, structs
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type


class CarStateExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.active_touch_points = 0

  def update(self, ret: structs.CarState, can_parsers) -> None:
    if not (self.CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS):
      return
    if Bus.adas not in can_parsers:
      return
    cp_adas = can_parsers[Bus.adas]

    prev_active_touch_points = self.active_touch_points
    self.active_touch_points = int(cp_adas.vl["UI_status2"]["UI_activeTouchPoints"])

    finger_count = None
    if self.CP_SP.flags & TeslaFlagsSP.MADS_SCREEN_BUTTON_3_FINGER:
      finger_count = 3
    elif self.CP_SP.flags & TeslaFlagsSP.MADS_SCREEN_BUTTON_4_FINGER:
      finger_count = 4
    elif self.CP_SP.flags & TeslaFlagsSP.MADS_SCREEN_BUTTON_5_FINGER:
      finger_count = 5

    if finger_count is not None:
      ret.buttonEvents = [*create_button_events(self.active_touch_points, prev_active_touch_points,
                                                {finger_count: ButtonType.lkas})]
