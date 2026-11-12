"""TI1 lateral shaping (ti1_lateral.py) and its use in LatControlTorque."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car import structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.values import CAR, MazdaSafetyFlags
from openpilot.selfdrive.controls.lib import latcontrol_torque
from openpilot.starpilot.car.mazda import ti1_lateral

TI1 = int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR)


def make_cp(flags=TI1, fingerprint=CAR.MAZDA_CX8, brand="mazda"):
  cp = structs.CarParams(carFingerprint=fingerprint, brand=brand, flags=flags)
  cp.lateralTuning.init("torque")
  cp.lateralTuning.torque.latAccelFactor = 1.76
  cp.lateralTuning.torque.friction = 0.18
  return cp


def lade_effective_p(v):
  """P gain of the controller the TI1 tune was fitted on: (kp + low_speed_factor) * TI error scale."""
  lsf = (np.interp(v, ti1_lateral.LADE_LOW_SPEED_X, ti1_lateral.LADE_LOW_SPEED_Y) / max(v, 1.0)) ** 2
  kp = np.interp(v, ti1_lateral.LADE_KP_SPEEDS, ti1_lateral.LADE_KP)
  return (kp + lsf) * np.interp(v, ti1_lateral.ERROR_SCALE_BP, ti1_lateral.ERROR_SCALE_V)


@pytest.mark.parametrize("v", [1.0, 3.0, 5.0, 6.0, 7.5, 10.0, 12.0, 13.5, 15.0])
def test_error_factor_reproduces_the_tuned_p_gain(v):
  # This controller's KP_INTERP equals the tuned one up to 15 m/s, so kp * factor is the tuned P gain there.
  our_kp = np.interp(v, latcontrol_torque.INTERP_SPEEDS, latcontrol_torque.KP_INTERP)
  assert our_kp * ti1_lateral.error_factor(v) == pytest.approx(lade_effective_p(v))


def test_low_speed_gain_is_softened_and_highway_is_near_unity():
  assert ti1_lateral.error_factor(5.0) < 0.35   # about 0.29: far softer than the stock low-speed gain
  assert ti1_lateral.error_factor(7.5) < 0.6
  assert 1.0 <= ti1_lateral.error_factor(30.0) < 1.1


def test_plant_shape():
  assert ti1_lateral.shaped_params(2.0, 0.2, 5.0) == pytest.approx((2.6, 0.32))
  assert ti1_lateral.shaped_params(2.0, 0.2, 15.0) == pytest.approx((1.8, 0.2 * np.interp(15.0, [5, 13.5, 21], [1.6, 1.5, 1.0])))
  assert ti1_lateral.shaped_params(2.0, 0.2, 25.0) == pytest.approx((2.0, 0.2))


@pytest.mark.parametrize("flags, brand, expected", [
  (TI1, "mazda", True),
  (TI1 | int(MazdaSafetyFlags.NO_FSC), "mazda", True),
  (int(MazdaSafetyFlags.GEN1), "mazda", False),       # stock GEN1
  (int(MazdaSafetyFlags.GEN2), "mazda", False),       # TI2 cars never set TORQUE_INTERCEPTOR
  (int(MazdaSafetyFlags.TORQUE_INTERCEPTOR), "mazda", False),
  (TI1, "toyota", False),
])
def test_scope(flags, brand, expected):
  assert ti1_lateral.is_ti1(make_cp(flags=flags, brand=brand)) == expected


def _params_available():
  try:
    import openpilot.common.params  # noqa: F401  (loads the native params library)
    return True
  except OSError:
    return False


# Constructing the controller imports the car interfaces, which load the native params library.
needs_params = pytest.mark.skipif(not _params_available(), reason="native params library unavailable on this host")


def _controller(cp):
  # The base class's linear conversions (the CX-8 is not in NON_LINEAR_TORQUE_PARAMS).
  ci = SimpleNamespace(torque_from_lateral_accel=lambda: lambda a, p: CarInterfaceBase.torque_from_lateral_accel_linear(None, a, p),
                       lateral_accel_from_torque=lambda: lambda t, p: CarInterfaceBase.lateral_accel_from_torque_linear(None, t, p))
  return latcontrol_torque.LatControlTorque(cp.as_reader(), ci, 0.01, controller_mode=latcontrol_torque.ControllerMode.STANDARD)


@needs_params
def test_controller_limits_and_shape_for_ti1():
  lac = _controller(make_cp())
  assert lac.steer_max == pytest.approx(700 / 800)
  lac._apply_mazda_ti1_shape(5.0)
  assert float(lac.torque_params.latAccelFactor) == pytest.approx(1.76 * 1.3, rel=1e-6)
  assert float(lac.torque_params.friction) == pytest.approx(0.18 * 1.6, rel=1e-6)
  assert lac.pid.pos_limit == pytest.approx(0.875 * 1.76 * 1.3, rel=1e-6)
  # A new base keeps the speed shape on top.
  lac.update_torque_parameters(2.0, 0.0, 0.2)
  assert float(lac.torque_params.latAccelFactor) == pytest.approx(2.6, rel=1e-6)
  assert lac.mazda_ti1_base == pytest.approx((2.0, 0.2))
  lac._apply_mazda_ti1_shape(25.0)
  assert float(lac.torque_params.latAccelFactor) == pytest.approx(2.0, rel=1e-6)


@needs_params
def test_controller_is_untouched_for_other_mazdas():
  for flags in (int(MazdaSafetyFlags.GEN1), int(MazdaSafetyFlags.GEN2)):
    lac = _controller(make_cp(flags=flags, fingerprint=CAR.MAZDA_CX9_2021))
    assert not lac.mazda_ti1 and lac.steer_max == 1.0
    assert not math.isclose(float(lac.torque_params.latAccelFactor), 1.76 * 1.3)


@needs_params
def test_update_scales_the_error_only_for_ti1():
  vm = SimpleNamespace(calc_curvature=lambda angle, v, roll: 0.0)   # measured lateral accel 0
  params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  cs = structs.CarState(vEgo=5.0, steeringAngleDeg=0.0, steeringPressed=False)
  errors = {}
  for name, flags in (("ti1", TI1), ("stock", int(MazdaSafetyFlags.GEN1))):
    lac = _controller(make_cp(flags=flags))
    for _ in range(150):  # fill the 1 s request buffer with a constant 0.5 m/s^2 request
      _, _, log = lac.update_standard(True, cs, vm, params, False, 0.5 / 5.0**2, False, 0.2)
    errors[name] = log.error
  assert errors["stock"] == pytest.approx(0.5, rel=1e-3)
  assert errors["ti1"] == pytest.approx(0.5 * ti1_lateral.error_factor(5.0), rel=1e-3)
