import os
import capnp
from opendbc.car.common.basedir import BASEDIR

capnp.remove_import_hook()
car = capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR])

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule

# sunnypilot structs (MADS port): plain dataclass mirror of cereal custom.CarParamsSP.
# Unlike sunnypilot's auto_dataclass version this stays a simple dataclass; the fields
# are the ones the MADS helpers and the Tesla screen button actually use.
from dataclasses import dataclass


@dataclass
class CarParamsSP:
  flags: int = 0        # flags for car specific quirks in sunnypilot
  safetyParam: int = 0  # flags for sunnypilot's custom safety flags
  pcmCruiseSpeed: bool = True
  intelligentCruiseButtonManagementAvailable: bool = False
  enableGasInterceptor: bool = False
