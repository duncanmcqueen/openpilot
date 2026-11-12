"""Lateral shaping for a GEN1 Mazda with the Torque Interceptor (TI1).

Ported from ladelisandwich/openpilot mazda-long-ti1-testing (95c3fda, afe2088, 0d42e5c), where it was fitted on
a TI1 CX-9's drives (his routes 00000022-35 and 00000067/68). The TI adds its request to the EPS torque-sensor
signal, so the whole power-steering assist answers it: openpilot's low-speed gains, written for EPS-limited cars,
are far too stiff, and the plant gain and friction change with speed.

His controller differs from this one: it multiplies the error by (1 + low_speed_factor / kp) on top of the
KP_INTERP gains, and his TI scale was tuned on top of that. This controller has no low-speed factor, so the error
factor here is that whole product. With the same KP_INTERP table (identical to ours below 15 m/s) it gives the
P gain and friction input of his tuned controller. The I gain and the 30 m/s KP stay this controller's.

Above 10 m/s the TI1 tune is softened from Lade's (HIGHWAY_SOFTEN below): on a TI1 CX-8 (Konik route
8762fabfa41efd82/00000010, 2026-11-12) his highway gain left a 0.5-0.65 Hz weave at 14-17 m/s that the model's path did
not ask for. The tester also found his NNFF mode, a flat P of 1.0, smoother there.

Two TI1-only settings adjust this tune; each is read when the drive starts and falls back to the tune when unset,
out of range, or unknown to an older params build:
  MazdaTI1SteerResponse  percent applied to the error factor (P, I and the friction input); 100 is the tune.
  MazdaTI1ActuatorDelay  seconds of actuator delay, as Lade's slider; 0 keeps the learned delay. Published by lagd
                         as actuator delay + 0.2 s, the same convention as its initial lag, so modeld and controls
                         both use it.

Applies only to GEN1 cars with TORQUE_INTERCEPTOR; GEN2/GEN3 TI2 cars never set that flag.
"""
import math

import numpy as np

from opendbc.car.mazda.values import CarControllerParams, MazdaSafetyFlags

# Lade's controller, which the error scale below was tuned against (latcontrol_torque there).
LADE_KP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]   # m/s
LADE_KP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, 0.6]
LADE_LOW_SPEED_X = [0, 10, 20, 30]                         # m/s
LADE_LOW_SPEED_Y = [12, 10.5, 8, 5]
LADE_MIN_SPEED = 1.0                                       # drive_helpers.MIN_SPEED

# Lade's MAZDA_TI_ERROR_SCALE: takes most of the low-speed factor back out below 30 mph, exactly 1.0 from
# 13.5 m/s up (P, I and the friction input).
ERROR_SCALE_BP = [5.0, 7.5, 10.0, 13.5]  # m/s
ERROR_SCALE_V = [0.2, 0.4, 0.65, 1.0]

# Lade's fitted plant shape, applied to the live or tune latAccelFactor and friction as the highway base.
LAT_ACCEL_FACTOR_SCALE_BP = [8.75, 11.75, 15.0, 18.75]  # m/s
LAT_ACCEL_FACTOR_SCALE_V = [1.3, 1.1, 0.9, 1.0]
# Lade's friction scale reached 1.0 only at 21 m/s; here it does by 13.5 m/s, the highway weave's lower edge.
FRICTION_SCALE_BP = [5.0, 11.0, 13.5]  # m/s
FRICTION_SCALE_V = [1.6, 1.5, 1.0]

# Multiplies the error factor; 1.0 (Lade's tune) up to 10 m/s. Effective P with this controller's KP table:
# 3.0 at 10 m/s, 2.3 at 13.5, 1.3 at 16, about 1.15 from 18 to 26 (near NNFF's flat 1.0), and the unsoftened
# tune again from 27 m/s (1.1, falling to 0.84 at 30).
HIGHWAY_SOFTEN_BP = [10.0, 13.5, 16.0, 20.0, 27.0]  # m/s
HIGHWAY_SOFTEN_V = [1.0, 0.8, 0.6, 0.65, 1.0]

RESPONSE_KEY = "MazdaTI1SteerResponse"
RESPONSE_RANGE = (50.0, 150.0)  # percent
DELAY_KEY = "MazdaTI1ActuatorDelay"
DELAY_RANGE = (0.0, 0.8)        # s; 0 = learned
MIN_DELAY = 0.05                # s; below this the setting means "learned"
LAGD_DELAY_OFFSET = 0.2         # lagd's initial lag is steerActuatorDelay + 0.2


def is_ti1(CP) -> bool:
  return CP.brand == "mazda" and bool(CP.flags & MazdaSafetyFlags.GEN1) and bool(CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR)


def steer_max(CP) -> float:
  """The TI's authority ends at TI_CEILING, where the car controller clamps: the PID limits and the saturation
  test sit there too, so a pinned TI is reported instead of hidden behind the command/output mismatch."""
  ccp = CarControllerParams(CP)
  return ccp.TI_CEILING / ccp.STEER_MAX


def error_factor(v_ego: float) -> float:
  low_speed_factor = (np.interp(v_ego, LADE_LOW_SPEED_X, LADE_LOW_SPEED_Y) / max(v_ego, LADE_MIN_SPEED)) ** 2
  kp = np.interp(v_ego, LADE_KP_SPEEDS, LADE_KP)
  return float((1 + low_speed_factor / max(kp, 1e-3)) * np.interp(v_ego, ERROR_SCALE_BP, ERROR_SCALE_V) *
               np.interp(v_ego, HIGHWAY_SOFTEN_BP, HIGHWAY_SOFTEN_V))


def shaped_params(base_lat_accel_factor: float, base_friction: float, v_ego: float) -> tuple[float, float]:
  return (base_lat_accel_factor * float(np.interp(v_ego, LAT_ACCEL_FACTOR_SCALE_BP, LAT_ACCEL_FACTOR_SCALE_V)),
          base_friction * float(np.interp(v_ego, FRICTION_SCALE_BP, FRICTION_SCALE_V)))


def _read_setting(params, key: str, low: float, high: float) -> float | None:
  """A saved setting in range, else None. Never raises: a stale params build without the key is unset."""
  try:
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    raw = params.get(key)
    if raw is None:
      return None
    value = float(raw.decode() if isinstance(raw, bytes) else raw)
  except Exception:
    return None
  return value if math.isfinite(value) and low <= value <= high else None


def response_scale(params=None) -> float:
  value = _read_setting(params, RESPONSE_KEY, *RESPONSE_RANGE)
  return 1.0 if value is None else value / 100.0


def actuator_delay(params=None) -> float | None:
  """The chosen actuator delay in seconds, or None to keep the learned delay."""
  value = _read_setting(params, DELAY_KEY, *DELAY_RANGE)
  return value if value is not None and value >= MIN_DELAY else None


class DelayPublication:
  """lagd hook: replace the published lateral delay with the chosen one. Estimator state and cache are untouched."""
  def __init__(self, CP, params=None):
    delay = actuator_delay(params) if is_ti1(CP) else None
    self.lateral_delay = None if delay is None else delay + LAGD_DELAY_OFFSET

  def delay(self, original):
    if self.lateral_delay is None:
      return original
    message = original.as_reader().as_builder()
    message.lateralDelay.lateralDelay = self.lateral_delay
    return message
