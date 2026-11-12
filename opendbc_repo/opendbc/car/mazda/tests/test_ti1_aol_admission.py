"""GEN1 Torque Interceptor (TI1) AOL admission.

The profile is exactly GEN1 | TORQUE_INTERCEPTOR (camera present) on validated platforms, behind evidence gates
that stay open until a capture confirms the GEN1 AOL sources on the car. Panda enforces native permission
independently (safety/tests/test_mazda_ti1_aol.py).
"""
import pytest

from opendbc.car import structs
from opendbc.car.mazda.stock_aol import (TI1_AOL_SAFETY_PARAM, TI1_CARS, TI1_EVIDENCE_GATES, admitted, qualified,
                                         gen2_qualified, ti1_configuration, ti1_qualified)
from opendbc.car.mazda.values import CAR, MazdaSafetyFlags

CLOSED_GATES = {name: True for name in TI1_EVIDENCE_GATES}
TI1 = int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR)


def make_cp(fingerprint=CAR.MAZDA_CX8, flags=TI1, param=TI1, alternative_experience=32):
  cp = structs.CarParams(carFingerprint=fingerprint, brand="mazda", flags=flags, pcmCruise=True,
                         openpilotLongitudinalControl=False, passive=False, notCar=False, dashcamOnly=False,
                         minSteerSpeed=0.0, transmissionType=structs.CarParams.TransmissionType.automatic,
                         fingerprintSource=structs.CarParams.FingerprintSource.can)
  cp.alternativeExperience = alternative_experience
  cp.init("safetyConfigs", 1)
  cp.safetyConfigs[0].safetyModel = structs.CarParams.SafetyModel.mazda
  cp.safetyConfigs[0].safetyParam = param
  return cp


def test_param_is_the_exact_ti1_word():
  assert TI1_AOL_SAFETY_PARAM == 9


def test_target_car_matches_and_is_admitted_only_with_closed_gates():
  cp = make_cp()
  assert ti1_configuration(cp, marked_only=True)
  assert ti1_qualified(cp, CLOSED_GATES, marked_only=True)
  # The recorded gates are open until the CX-8 capture is checked, so nothing is admitted yet.
  assert not all(TI1_EVIDENCE_GATES.values())
  assert not ti1_qualified(cp, marked_only=True)
  assert not admitted(cp, marked_only=True)


@pytest.mark.parametrize("gates", [{}, {"cx8_signals": True, "extra": True}, {"other": True}])
def test_gate_table_must_be_exactly_the_known_set(gates):
  assert not ti1_qualified(make_cp(), gates)


@pytest.mark.parametrize("change", [
  {"flags": TI1 | int(MazdaSafetyFlags.NO_FSC), "param": TI1 | int(MazdaSafetyFlags.NO_FSC)},
  {"flags": TI1 | int(MazdaSafetyFlags.RADAR_INTERCEPTOR), "param": TI1 | int(MazdaSafetyFlags.RADAR_INTERCEPTOR)},
  {"flags": TI1 | int(MazdaSafetyFlags.NO_MRCC), "param": TI1 | int(MazdaSafetyFlags.NO_MRCC)},
  {"flags": int(MazdaSafetyFlags.GEN1), "param": 0},
  {"param": 0},
  {"fingerprint": CAR.MAZDA_CX9_2021},
  {"alternative_experience": 0, "marked": True},
])
def test_excluded_configurations(change):
  change = dict(change)
  change.pop("marked", None)
  cp = make_cp(**change)
  assert not ti1_qualified(cp, CLOSED_GATES, marked_only=True)


@pytest.mark.parametrize("field, value", [
  ("openpilotLongitudinalControl", True), ("pcmCruise", False), ("passive", True), ("dashcamOnly", True),
  ("transmissionType", structs.CarParams.TransmissionType.manual),
  ("fingerprintSource", structs.CarParams.FingerprintSource.fixed),
])
def test_excluded_car_params(field, value):
  cp = make_cp()
  setattr(cp, field, value)
  assert not ti1_qualified(cp, CLOSED_GATES)


def test_ti1_is_not_claimed_by_the_other_profiles():
  cp = make_cp()
  assert not qualified(cp) and not gen2_qualified(cp)
  assert set(TI1_CARS) == {CAR.MAZDA_CX8}
