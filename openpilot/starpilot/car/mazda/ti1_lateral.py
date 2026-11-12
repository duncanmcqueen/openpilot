"""Lateral shaping for a GEN1 Mazda with the Torque Interceptor (TI1).

Ported from ladelisandwich/openpilot mazda-long-ti1-testing (95c3fda, afe2088, 0d42e5c), where it was fitted on
a TI1 CX-9's drives (his routes 00000022-35 and 00000067/68). The TI adds its request to the EPS torque-sensor
signal, so the whole power-steering assist answers it: openpilot's low-speed gains, written for EPS-limited cars,
are far too stiff, and the plant gain and friction change with speed.

His controller differs from this one: it multiplies the error by (1 + low_speed_factor / kp) on top of the
KP_INTERP gains, and his TI scale was tuned on top of that. This controller has no low-speed factor, so the error
factor here is that whole product. With the same KP_INTERP table (identical to ours below 15 m/s) it gives the
P gain and friction input of his tuned controller. The I gain and the 30 m/s KP stay this controller's.

Applies only to GEN1 cars with TORQUE_INTERCEPTOR; GEN2/GEN3 TI2 cars never set that flag.
"""
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
FRICTION_SCALE_BP = [5.0, 13.5, 21.0]  # m/s
FRICTION_SCALE_V = [1.6, 1.5, 1.0]


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
  return float((1 + low_speed_factor / max(kp, 1e-3)) * np.interp(v_ego, ERROR_SCALE_BP, ERROR_SCALE_V))


def shaped_params(base_lat_accel_factor: float, base_friction: float, v_ego: float) -> tuple[float, float]:
  return (base_lat_accel_factor * float(np.interp(v_ego, LAT_ACCEL_FACTOR_SCALE_BP, LAT_ACCEL_FACTOR_SCALE_V)),
          base_friction * float(np.interp(v_ego, FRICTION_SCALE_BP, FRICTION_SCALE_V)))
