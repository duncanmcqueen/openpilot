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


@pytest.mark.parametrize("v", [1.0, 3.0, 5.0, 6.0, 7.5, 10.0])
def test_error_factor_reproduces_the_tuned_p_gain(v):
  # This controller's KP_INTERP equals the tuned one up to 15 m/s, so kp * factor is the tuned P gain there.
  # Up to 10 m/s that is Lade's gain unchanged; above, HIGHWAY_SOFTEN takes it down.
  our_kp = np.interp(v, latcontrol_torque.INTERP_SPEEDS, latcontrol_torque.KP_INTERP)
  assert our_kp * ti1_lateral.error_factor(v) == pytest.approx(lade_effective_p(v))


def test_low_speed_gain_is_softened_and_highway_is_near_unity():
  assert ti1_lateral.error_factor(5.0) < 0.35   # about 0.29: far softer than the stock low-speed gain
  assert ti1_lateral.error_factor(7.5) < 0.6
  assert 1.0 <= ti1_lateral.error_factor(30.0) < 1.1


@pytest.mark.parametrize("v, soften", [(12.0, 1.0 - 0.2 * 2 / 3.5), (13.5, 0.8), (15.0, 0.68)])
def test_highway_gain_is_softened(v, soften):
  our_kp = np.interp(v, latcontrol_torque.INTERP_SPEEDS, latcontrol_torque.KP_INTERP)
  assert our_kp * ti1_lateral.error_factor(v) == pytest.approx(lade_effective_p(v) * soften)


def test_highway_effective_p_is_near_nnff():
  # Route 00000010 weaved at 14-17 m/s with P about 2.2; NNFF, which the tester found smoother, runs a flat 1.0.
  def p(v):
    return np.interp(v, latcontrol_torque.INTERP_SPEEDS, latcontrol_torque.KP_INTERP) * ti1_lateral.error_factor(v)
  assert 1.2 < p(16.0) < 1.45
  assert 0.95 < p(20.0) < 1.25
  assert p(10.0) == pytest.approx(lade_effective_p(10.0))
  speeds = np.arange(10.0, 30.0, 0.25)
  gains = [p(v) for v in speeds]
  assert all(np.diff(gains) <= 0.01)                     # no gain bump in or after the softened band
  assert max(gains[speeds.tolist().index(18.0):]) < 1.25


def test_plant_shape():
  assert ti1_lateral.shaped_params(2.0, 0.2, 5.0) == pytest.approx((2.6, 0.32))
  assert ti1_lateral.shaped_params(2.0, 0.2, 11.0) == pytest.approx((2.0 * np.interp(11.0, [8.75, 11.75], [1.3, 1.1]), 0.3))
  assert ti1_lateral.shaped_params(2.0, 0.2, 15.0) == pytest.approx((1.8, 0.2))   # friction unscaled from 13.5 m/s
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
  assert errors["ti1"] == pytest.approx(0.5 * 0.28804348, rel=1e-3)  # literal: see test_error_factor_is_the_tuned_value_at_5_mps


def test_error_factor_is_the_tuned_value_at_5_mps():
  # Literal, not computed by the helper under test: (1 + (11.25 / 5)^2 / 11.5) * 0.2.
  assert ti1_lateral.error_factor(5.0) == pytest.approx(0.28804348, rel=1e-6)


def _selection_cp(flags):
  cp = make_cp(flags=flags)
  cp.steerControlType = structs.CarParams.SteerControlType.torque
  cp.lateralTuning.torque.steeringAngleDeadzoneDeg = 0.0
  return cp.as_reader()


def test_ti1_cannot_select_the_starpilot_controller():
  # Independent review P1: the StarPilot torque law bypasses the TI1 error shaping.
  from openpilot.starpilot.lateral import controller_selection as sel
  stock = _selection_cp(int(MazdaSafetyFlags.GEN1))
  assert sel.policy_for(stock) == 'generic'  # other Mazdas keep their choice
  ti1 = _selection_cp(TI1)
  assert sel.policy_for(ti1) is None
  assert sel.default_selection(ti1).mode == sel.ControllerMode.STANDARD
  with pytest.raises(ValueError):
    sel.replace_mode(None, ti1, sel.ControllerMode.STARPILOT)
  saved = sel.replace_mode(None, stock, sel.ControllerMode.STARPILOT)  # a choice saved before the TI1 was fitted
  assert sel.selection_from_bytes(ti1, saved).mode == sel.ControllerMode.STANDARD


def test_ti1_is_excluded_from_automatic_steering_learning():
  # Independent review P2: speed shaping is not representable by the scalar fit or native validation.
  from openpilot.starpilot.flm.learning_source import automatic_capability
  assert automatic_capability(_selection_cp(TI1), 'standard') is None
  assert automatic_capability(_selection_cp(int(MazdaSafetyFlags.GEN1)), 'standard') is not None


@needs_params
def test_native_learning_baseline_records_the_ti1_limits():
  lac = _controller(make_cp())
  assert lac.native_learning_baseline[0] == pytest.approx(0.875)
  assert lac.native_learning_baseline == tuple(__import__('openpilot.starpilot.flm.runtime_adapter_registry',
                                                          fromlist=['baseline_values']).baseline_values(lac))


@needs_params
def test_public_update_shapes_every_frame():
  # Review test gap: the shape must be applied through update(), not only through the private helper.
  vm = SimpleNamespace(calc_curvature=lambda angle, v, roll: 0.0)
  params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  lac = _controller(make_cp())
  for v, factor in ((5.0, 1.76 * 1.3), (25.0, 1.76)):
    lac.update(True, structs.CarState(vEgo=v, steeringAngleDeg=0.0, steeringPressed=False), vm, params, False, 0.0, False, 0.2)
    assert float(lac.torque_params.latAccelFactor) == pytest.approx(factor, rel=1e-6)


class _FakeParams:
  def __init__(self, values=None, error=None):
    self.values, self.error = values or {}, error

  def get(self, key):
    if self.error is not None:
      raise self.error
    return self.values.get(key)


@pytest.mark.parametrize("raw, expected", [
  (None, 1.0), (100.0, 1.0), (70.0, 0.7), (b"120.0", 1.2), (150.0, 1.5), (50.0, 0.5),
  (49.0, 1.0), (151.0, 1.0), (float("nan"), 1.0), (b"junk", 1.0),
])
def test_response_scale(raw, expected):
  assert ti1_lateral.response_scale(_FakeParams({ti1_lateral.RESPONSE_KEY: raw})) == pytest.approx(expected)


@pytest.mark.parametrize("raw, expected", [
  (None, None), (0.0, None), (0.04, None), (0.05, 0.05), (0.3, 0.3), (b"0.45", 0.45), (0.8, 0.8), (0.81, None), (-0.1, None),
])
def test_actuator_delay(raw, expected):
  value = ti1_lateral.actuator_delay(_FakeParams({ti1_lateral.DELAY_KEY: raw}))
  assert value == (None if expected is None else pytest.approx(expected))


def test_settings_survive_a_params_build_without_the_keys():
  # A device whose libparams predates these keys raises UnknownKeyName; the tune must still load.
  broken = _FakeParams(error=Exception("UnknownKeyName"))
  assert ti1_lateral.response_scale(broken) == 1.0
  assert ti1_lateral.actuator_delay(broken) is None


def _lag_message(value):
  from openpilot.cereal import log
  msg = log.Event.new_message()
  msg.init('lateralDelay')
  msg.lateralDelay.lateralDelay = value
  msg.lateralDelay.validBlocks = 7
  return msg


def test_delay_publication_overrides_only_the_published_delay():
  pub = ti1_lateral.DelayPublication(make_cp(), _FakeParams({ti1_lateral.DELAY_KEY: 0.3}))
  original = _lag_message(0.535)
  out = pub.delay(original)
  assert out.lateralDelay.lateralDelay == pytest.approx(0.5)   # 0.3 s actuator delay + lagd's 0.2 s
  assert out.lateralDelay.validBlocks == 7
  assert original.lateralDelay.lateralDelay == pytest.approx(0.535)   # the estimator's own message is untouched


@pytest.mark.parametrize("cp, values", [
  (make_cp(), {}),                                                          # unset: learned delay
  (make_cp(), {ti1_lateral.DELAY_KEY: 0.0}),                                # 0 means learned
  (make_cp(flags=int(MazdaSafetyFlags.GEN1)), {ti1_lateral.DELAY_KEY: 0.3}),  # stock GEN1 ignores it
  (make_cp(flags=int(MazdaSafetyFlags.GEN2)), {ti1_lateral.DELAY_KEY: 0.3}),  # TI2 ignores it
])
def test_delay_publication_is_inert(cp, values):
  original = _lag_message(0.535)
  assert ti1_lateral.DelayPublication(cp, _FakeParams(values)).delay(original) is original


@needs_params
def test_steering_response_scales_the_ti1_error(monkeypatch):
  monkeypatch.setattr(ti1_lateral, "response_scale", lambda params=None: 0.7)
  vm = SimpleNamespace(calc_curvature=lambda angle, v, roll: 0.0)
  params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  cs = structs.CarState(vEgo=5.0, steeringAngleDeg=0.0, steeringPressed=False)
  lac = _controller(make_cp())
  for _ in range(150):
    _, _, log = lac.update_standard(True, cs, vm, params, False, 0.5 / 5.0**2, False, 0.2)
  assert log.error == pytest.approx(0.5 * 0.28804348 * 0.7, rel=1e-3)
