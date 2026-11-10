"""The car side of lateral-only: the opt-in, and the buttons the host is told about (panda's side: safety/tests/test_ford.py)."""
import unittest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.ford import lateral_only
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, FordFlags, FordSafetyFlags

CANCEL, LKAS, GAP = 'cancel', 'lkas', 'gapAdjustCruise'      # ButtonEvent.Type, as a reader prints it


def interface(opted_in: bool, car=CAR.FORD_MUSTANG_MACH_E_MK1):
  CP = CarInterface.get_params(car, gen_empty_fingerprint(), [], True, False, False)
  if opted_in:
    lateral_only.enable(CP)
  CI = CarInterface(CP)
  CI.update([])      # the parser takes a message from its second update on: the first names what carstate reads
  return CI


class TestLateralOnlyCar(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("ford_lincoln_base_pt")
    self.t = 0

  def buttons(self, CI, **signals):
    """One Steering_Data_FD1 from the car; the button events carState reports for it."""
    self.t += int(1e8)
    addr, dat, bus = self.packer.make_can_msg("Steering_Data_FD1", 0, signals)
    return [(str(b.type), b.pressed) for b in CI.update([(self.t, [CanData(addr, dat, bus)])]).buttonEvents]

  def test_enable_sets_both_sides_and_only_on_can_fd(self):
    CI = interface(False)
    self.assertFalse(lateral_only.enabled(CI.CP))
    before = CI.CP.safetyConfigs[-1].safetyParam
    lateral_only.enable(CI.CP)
    self.assertTrue(lateral_only.enabled(CI.CP))
    self.assertEqual(CI.CP.safetyConfigs[-1].safetyParam, before | FordSafetyFlags.LAT_ONLY)
    self.assertTrue(CI.CP.flags & FordFlags.LAT_ONLY)
    CP = CarInterface.get_params(CAR.FORD_BRONCO_SPORT_MK1, gen_empty_fingerprint(), [], False, False, False)
    self.assertFalse(CP.flags & FordFlags.CANFD)
    with self.assertRaises(ValueError):
      lateral_only.enable(CP)

  def test_no_fingerprint_opts_in(self):
    for car in CAR:
      CP = CarInterface.get_params(car, gen_empty_fingerprint(), [], True, False, False)
      self.assertFalse(CP.flags & FordFlags.LAT_ONLY, car)
      self.assertFalse(CP.safetyConfigs[-1].safetyParam & FordSafetyFlags.LAT_ONLY, car)

  def test_the_cancel_button_is_reported_only_when_opted_in(self):
    for opted_in in (True, False):
      with self.subTest(opted_in=opted_in):
        CI = interface(opted_in)
        self.assertEqual(self.buttons(CI), [])
        cancel = [(CANCEL, True)] if opted_in else []
        self.assertEqual(self.buttons(CI, CcAslButtnCnclPress=1), cancel)
        self.assertEqual(self.buttons(CI, CcAslButtnCnclPress=1), [], "held: one event")
        self.assertEqual(self.buttons(CI), [(CANCEL, False)] if opted_in else [])

  def test_the_other_buttons_are_reported_either_way(self):
    for opted_in in (True, False):
      with self.subTest(opted_in=opted_in):
        CI = interface(opted_in)
        self.assertEqual(self.buttons(CI, TjaButtnOnOffPress=1), [(LKAS, True)])
        self.assertEqual(self.buttons(CI, TjaButtnOnOffPress=1, AccButtnGapTogglePress=1), [(GAP, True)])
        self.assertEqual(sorted(self.buttons(CI)), sorted([(GAP, False), (LKAS, False)]))

  def test_cancel_and_lane_centering_in_one_message(self):
    CI = interface(True)
    self.assertEqual(sorted(self.buttons(CI, TjaButtnOnOffPress=1, CcAslButtnCnclPress=1)),
                     sorted([(LKAS, True), (CANCEL, True)]))


if __name__ == "__main__":
  unittest.main()
