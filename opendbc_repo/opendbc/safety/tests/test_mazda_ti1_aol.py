"""GEN1 Torque Interceptor (TI1) AOL profile in panda: exactly GEN1 | TORQUE_INTERCEPTOR (param 9).

Same protocol as stock GEN1 AOL (test_mazda_stock_aol.py) with TI_FEEDBACK liveness in place of the EPS
0x241 lockout bit, and lateral withheld (token kept) while the TI1 reports a violation or a state other
than RUN or DRIVER_OVER.
"""
import unittest

from opendbc.car import structs
from opendbc.can.dbc import DBC
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_mazda import TI1_KEY

TI1 = 1 | 8
TI_DISCOVER, TI_OFF, TI_DRIVER_OVER, TI_RUN = 0, 1, 2, 3


class TestMazdaTi1Aol(unittest.TestCase):
  def tearDown(self):
    self.safety.init_tests()
    self.safety.set_alternative_experience(0)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)

  def configure(self, experience=32, word=TI1):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.safety.set_alternative_experience(experience)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.mazda, word)
    self.time = 1_000_000
    self.safety.set_timer(self.time)
    self.safety.set_aol_test_heartbeat(True)
    self.packer = CANPackerSafety('mazda_2017')
    self.dbc = DBC('mazda_2017')

  def packet(self, name, fields, bus=0):
    self.assertFalse(fields.keys() - self.dbc.name_to_msg[name].sigs.keys())
    return self.packer.make_can_msg_safety(name, bus, fields)

  def feedback(self, state=TI_RUN, viol=0):
    return self.packet('TI_FEEDBACK', {'STATE': state, 'VIOL': viol, 'VERSION_NUMBER': 1, 'TI_TORQUE_SENSOR': 0}, 1)

  def feed(self, main=True, reverse=False, lkas_block=False, state=TI_RUN, viol=0, feedback=True, advance_us=0, engaged=False):
    self.time += advance_us
    self.safety.set_timer(self.time)
    sources = [
      ('CRZ_CTRL', 0, {'CRZ_AVAILABLE': int(main), 'CRZ_ACTIVE': int(engaged)}),
      ('CRZ_BTNS', 0, {'RES': 0, 'SET_M': 0, 'CAN_OFF': 0}),
      ('STEER_TORQUE', 0, {'STEER_TORQUE_SENSOR': 0}),
      ('ENGINE_DATA', 0, {'SPEED': 80, 'PEDAL_GAS': 0}),
      ('PEDALS', 0, {'BRAKE_ON': 0}),
      ('GEAR', 0, {'GEAR': 2 if reverse else 4}),
      ('STEER_RATE', 0, {'LKAS_BLOCK': int(lkas_block)}),
      ('CAM_LKAS', 2, {'ERR_BIT_1': 0, 'LKAS_REQUEST': 0}),
    ]
    for name, bus, fields in sources:
      self.safety.safety_rx_hook(self.packet(name, fields, bus))
    if feedback:
      self.safety.safety_rx_hook(self.feedback(state, viol))
    self.safety.safety_tick()
    self.safety.aol_set_host_request(1)

  def arm(self):
    for _ in range(10):
      self.feed(main=True)
    self.feed(main=False)
    self.feed(main=True)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def steer(self, torque):
    lkas = self.safety.safety_tx_hook(self.packet('CAM_LKAS', {'LKAS_REQUEST': torque}))
    ti = self.safety.safety_tx_hook(self.packet('CAM_LKAS2', {'LKAS_REQUEST': torque, 'CHKSUM': torque, 'KEY': TI1_KEY}, 1))
    return lkas, ti

  def test_ti1_aol_grants_lateral_on_both_channels(self):
    self.configure()
    self.feed()
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.arm()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.steer(10), (True, True))
    # Accepted LKAS takes over the camera's 0x243/0x440 like stock GEN1 AOL.
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x243), -1)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x440), -1)
    # Panda limits still apply.
    self.assertEqual(self.steer(801)[0], False)

  def test_eps_lockout_bit_is_not_a_ti1_restriction(self):
    self.configure()
    self.arm()
    self.feed(lkas_block=True)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    # Also when the lockout frame is the latest EPS-side frame, after the TI1 feedback.
    self.safety.safety_rx_hook(self.packet('STEER_RATE', {'LKAS_BLOCK': 1}))
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_relay_checks_follow_the_profile(self):
    # As in stock GEN1 AOL, the camera's own 0x243/0x440 reach the car while openpilot does not own lateral,
    # so seeing them on bus 0 is not a relay malfunction with the TI1 profile, and still is without it.
    for experience, malfunction in ((32, False), (0, True)):
      for name in ('CAM_LKAS', 'CAM_LANEINFO'):
        with self.subTest(experience=experience, name=name):
          self.configure(experience=experience)
          # init_tests() enables relay checks (set_safety_hooks() starts the transition timeout). The tx
          # table's check_relay flags were already set by the mode init above.
          self.safety.init_tests()
          self.safety.safety_rx_hook(self.packet(name, {}))
          self.assertEqual(self.safety.get_relay_malfunction(), malfunction)

  def test_ti1_states_withhold_lateral_and_keep_the_token(self):
    for state, viol, allowed in ((TI_RUN, 0x11, False), (TI_OFF, 0x11, False), (TI_OFF, 0, False),
                                 (TI_DISCOVER, 0, False), (TI_DRIVER_OVER, 0, True)):
      with self.subTest(state=state, viol=viol):
        self.configure()
        self.arm()
        self.feed(state=state, viol=viol)
        self.assertEqual(self.safety.aol_get_permission_mask(), int(allowed))
        if not allowed:
          self.assertEqual(self.steer(10), (False, False))
        self.feed()
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)  # recovers without a new driver action

  def test_reverse_still_restricts(self):
    self.configure()
    self.arm()
    self.feed(reverse=True)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.feed()
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_silent_ti1_expires_the_session(self):
    self.configure()
    self.arm()
    for _ in range(40):  # 400 ms without TI_FEEDBACK: past the 300 ms source freshness
      self.feed(feedback=False, advance_us=10_000)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.feed()
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)  # needs a new main edge
    self.arm()

  def test_only_the_exact_ti1_parameter_gets_the_profile(self):
    for word in (TI1 | 16, TI1 | 32, TI1 | 64):  # RI, NoFSC, NoMRCC variants stay excluded and fail closed
      with self.subTest(word=word):
        self.configure(word=word)
        for _ in range(12):
          self.feed()
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.assertFalse(self.safety.safety_tx_hook(self.packet('CAM_LKAS2', {'LKAS_REQUEST': 0, 'CHKSUM': 0, 'KEY': TI1_KEY}, 1)))
    # Without the AOL experience bit param 9 is the ordinary TI1 mode, with no AOL token.
    self.configure(experience=0)
    for _ in range(12):
      self.feed()
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.assertEqual(self.steer(10), (False, False))

  def test_permanent_violation_revokes_until_a_fresh_driver_action(self):
    # Independent review P1: a class 1 (or unknown) code must not come back on clean feedback alone.
    for viol in (0x19, 0x15, 0x41, 0x5A):
      with self.subTest(viol=hex(viol)):
        self.configure()
        self.arm()
        self.feed(state=TI_OFF, viol=viol)
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.assertFalse(self.safety.get_controls_allowed())
        for _ in range(5):
          self.feed()
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.assertEqual(self.steer(10), (False, False))
        self.arm()  # a new main edge re-arms once the TI1 is healthy

  def test_driver_action_during_a_permanent_violation_does_not_survive_it(self):
    self.configure()
    self.arm()
    for _ in range(3):  # the driver re-arms while the fault is still reported
      self.feed(state=TI_OFF, viol=0x19)
    self.feed(main=False, state=TI_OFF, viol=0x19)
    self.feed(main=True, state=TI_OFF, viol=0x19)
    self.feed()
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)

  def test_permanent_violation_also_drops_ordinary_engagement(self):
    self.configure(experience=0)
    for _ in range(3):
      self.feed()
    self.feed(engaged=True)  # the driver engages cruise: a rising edge
    self.assertTrue(self.safety.get_controls_allowed())
    self.feed(engaged=True, state=TI_OFF, viol=0x19)
    self.assertFalse(self.safety.get_controls_allowed())
    self.feed(engaged=True)  # clean feedback with cruise still on does not re-engage
    self.assertFalse(self.safety.get_controls_allowed())

  def test_recoverable_violations_keep_the_token(self):
    for viol in (0x11, 0x12, 0x13, 0x14, 0x17, 0x18, 0x22, 0x25, 0x31, 0x32, 0x33, 0x34):
      with self.subTest(viol=hex(viol)):
        self.configure()
        self.arm()
        self.feed(state=TI_OFF, viol=viol)
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.feed()
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_driver_override_refuses_nonzero_ti_torque_under_aol(self):
    # Review test gap: lateral permission stays during DRIVER_OVER, but the TI1 tx check must still refuse.
    self.configure()
    self.arm()
    self.feed(state=TI_DRIVER_OVER)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    self.assertEqual(self.steer(10), (True, False))


def test_panda_recoverable_codes_match_the_host_table():
  import re
  from pathlib import Path
  from opendbc.car.mazda.torque_interceptor import PERMANENT_CLASS, VIOLATIONS
  source = (Path(__file__).parents[1] / "modes" / "mazda.h").read_text()
  body = source[source.index("static bool mazda_ti1_viol_recoverable"):]
  body = body[:body.index("default:")]
  panda = {int(code, 16) for code in re.findall(r"case (0x[0-9A-Fa-f]+)U:", body)}
  host = {code for code, (_, cls, _) in VIOLATIONS.items() if cls != PERMANENT_CLASS}
  assert panda == host


if __name__ == '__main__':
  unittest.main()
