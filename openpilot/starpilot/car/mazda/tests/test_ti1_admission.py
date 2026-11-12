"""TI1 AOL host policy and native profile for the Mazda adapter."""
from unittest import mock

from opendbc.car import structs
from opendbc.car.mazda import stock_aol
from opendbc.car.mazda.values import CAR, MazdaSafetyFlags
from openpilot.starpilot.car.mazda import aol

MAZDA_MODEL = int(structs.CarParams.SafetyModel.mazda)
TI1 = int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR)


def ti1_cp(alternative_experience=0):
  cp = structs.CarParams(carFingerprint=CAR.MAZDA_CX8, brand="mazda", flags=TI1, pcmCruise=True,
                         openpilotLongitudinalControl=False, passive=False, notCar=False, dashcamOnly=False,
                         minSteerSpeed=0.0, transmissionType=structs.CarParams.TransmissionType.automatic,
                         fingerprintSource=structs.CarParams.FingerprintSource.can)
  cp.alternativeExperience = alternative_experience
  cp.init("safetyConfigs", 1)
  cp.safetyConfigs[0].safetyModel = MAZDA_MODEL
  cp.safetyConfigs[0].safetyParam = TI1
  return cp


def test_native_profile_includes_ti1():
  assert aol.native_profile_supported(MAZDA_MODEL, TI1)


def test_policy_stays_off_while_a_gate_is_open():
  with mock.patch.dict(stock_aol.TI1_EVIDENCE_GATES, dict.fromkeys(stock_aol.TI1_EVIDENCE_GATES, False)):
    policy = aol.policy_for(ti1_cp())
    assert not policy.intent_supported and policy.alternative_experience_addition == 0
    assert not aol.native_accepts_cp(ti1_cp(32), MAZDA_MODEL, TI1)


def test_recorded_gates_admit_the_cx8():
  policy = aol.policy_for(ti1_cp())
  assert policy.intent_supported and policy.alternative_experience_addition == 32
  assert aol.native_accepts_cp(ti1_cp(32), MAZDA_MODEL, TI1)


def test_policy_and_native_profile_once_the_gate_closes():
  with mock.patch.dict(stock_aol.TI1_EVIDENCE_GATES, dict.fromkeys(stock_aol.TI1_EVIDENCE_GATES, True)):
    policy = aol.policy_for(ti1_cp())
    assert policy.intent_supported and policy.runtime_supported and policy.explicit_latch
    assert policy.alternative_experience_addition == 32
    assert aol.native_accepts_cp(ti1_cp(32), MAZDA_MODEL, TI1)
    # The TI1 car must not claim the stock GEN1 or GEN2 native profile.
    assert not aol.native_accepts_cp(ti1_cp(32), MAZDA_MODEL, 0)
    assert not aol.native_accepts_cp(ti1_cp(32), MAZDA_MODEL, int(MazdaSafetyFlags.GEN2))
