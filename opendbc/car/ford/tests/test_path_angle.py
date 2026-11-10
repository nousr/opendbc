"""The low-speed path angle command on the bench: the real Ford CarController into the real panda safety.

Four stages are kept apart for every steer frame, because each can differ from the one before:
  requested   what the profile asks for (PathAngleTest.update)
  encoded     the path angle in the LateralMotionControl2 frame the controller built, read back from its bytes
  accepted    whether panda's safety (both copies, through safety_tx_hook) lets that frame out
  response    what the steering did. There is none on a bench; on the car it comes from the log.

    python -m opendbc.car.ford.tests.test_path_angle              the tests
    python -m opendbc.car.ford.tests.test_path_angle bench 0.3    print one run, stage by stage
"""
import random
import re
import sys
import unittest
from pathlib import Path

from opendbc.can import CANPacker
from opendbc.car import DT_CTRL, gen_empty_fingerprint, structs
from opendbc.car.ford import path_angle
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.path_angle import PathAngleTest, Profile, Stage
from opendbc.car.ford.values import CAR, CarControllerParams as P, FordFlags, FordSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
LMC2 = 0x3D6
FORD_H = Path(__file__).parents[3] / "safety/modes/ford.h"


def field(dat: bytes, start: int, length: int) -> int:
  """A big-endian DBC signal, by its start bit and length."""
  val, bit = 0, start
  for _ in range(length):
    byte, b = divmod(bit, 8)
    val = (val << 1) | ((dat[byte] >> b) & 1)
    bit = (byte + 1) * 8 + 7 if b == 0 else bit - 1
  return val


def decode(dat: bytes) -> dict:
  return {"mode": field(dat, 6, 3), "ramp": field(dat, 1, 2), "path_angle": field(dat, 28, 11) - 1000, "curvature": field(dat, 23, 11) - 1000,
          "path_offset": field(dat, 33, 10) - 512, "curvature_rate": field(dat, 55, 11) - 1024}


class Bench:
  """A Mach-E's CarInterface and panda's Ford safety, stepped together at 100 Hz."""

  def __init__(self, opt_in: bool = True, alpha_long: bool = True):
    CP = CarInterface.get_params(CAR.FORD_MUSTANG_MACH_E_MK1, gen_empty_fingerprint(), [], alpha_long, False, False)
    self.default_flags, self.default_param = CP.flags, CP.safetyConfigs[-1].safetyParam
    if opt_in:
      path_angle.enable(CP)
    self.CP = CP
    self.CI = CarInterface(CP)
    self.CI.update([])                       # stock-message defaults the controller passes through
    self.test: PathAngleTest = self.CI.CC.path_angle_test
    self.safety = libsafety_py.libsafety
    assert self.safety.set_safety_hooks(structs.CarParams.SafetyModel.ford, CP.safetyConfigs[-1].safetyParam) == 0
    self.safety.init_tests()
    self.safety.safety_redundancy_fault = False
    self.packer = CANPacker("ford_lincoln_base_pt")
    self.frame = 0
    self.rows: list[dict] = []
    self.refused: list[tuple[int, int]] = []     # (frame, address) of anything panda refused

  def _rx(self, name, values):
    addr, dat, bus = self.packer.make_can_msg(name, 0, values)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))

  def step(self, v=2.0, lat_active=True, model_curvature=0.0, torque=0.0, pressed=False, button=False,
           gear=GearShifter.drive, fault=False, panda_v=None) -> dict | None:
    """One 100 Hz frame. Returns the steer row if this frame sent LateralMotionControl2."""
    cs = structs.CarState(vEgo=v, vEgoRaw=v, steeringTorque=torque, steeringPressed=pressed, gearShifter=gear, steerFaultTemporary=fault)
    cs.cruiseState.available = True
    if button:
      cs.buttonEvents = [structs.CarState.ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=True)]
    self.CI.CS.out = cs
    CC = structs.CarControl(enabled=lat_active, latActive=lat_active)
    CC.actuators.curvature = model_curvature

    # panda hears the car as the controller does, and follows the engagement
    self.safety.set_timer(int(self.frame * DT_CTRL * 1e6))
    pv = v if panda_v is None else panda_v
    self._rx("BrakeSysFeatures", {"Veh_V_ActlBrk": pv * 3.6, "VehVActlBrk_D_Qf": 3})
    self._rx("EngVehicleSpThrottle2", {"Veh_V_ActlEng": pv * 3.6, "VehVActlEng_D_Qf": 3})
    self._rx("Yaw_Data_FD1", {"VehYaw_W_Actl": 0.0, "VehYawWActl_D_Qf": 3})
    self.safety.set_controls_allowed(lat_active)

    _, can_sends = self.CI.apply(CC.as_reader(), int(self.frame * DT_CTRL * 1e9))
    row = None
    for addr, dat, bus in can_sends:
      ok = bool(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)))
      if not ok:
        self.refused.append((self.frame, addr))
      if addr == LMC2:
        cc = self.CI.CC
        row = {"t": self.frame * DT_CTRL, "stage": str(self.test.stage), "requested": cc.path_angle_request, "accepted": ok,
               "bytes": bytes(dat), **decode(dat)}
        self.rows.append(row)
    self.frame += 1
    return row

  def steer_frame(self, **kw) -> dict:
    """Step to the next frame that sends the steering message; a button in kw is pressed on the first step only."""
    while True:
      row = self.step(**kw)
      kw.pop("button", None)
      if row is not None:
        return row

  def run(self, n: int, **kw) -> list[dict]:
    return [self.steer_frame(**kw) for _ in range(n)]

  def play(self, **kw) -> list[dict]:
    """Press the button and step until the run is over. The rows from the press on."""
    rows = [self.steer_frame(button=True, **kw)]
    while self.test.running and len(rows) < 2000:
      rows.append(self.steer_frame(**kw))
    return rows


class TestPathAngleBench(unittest.TestCase):
  def tearDown(self):
    self.assertFalse(libsafety_py.libsafety.safety_redundancy_fault, "the two safety copies disagreed")

  def bench(self, *levels, **kw) -> Bench:
    b = Bench(**kw)
    if levels:
      b.test.arm([Profile(level) for level in levels])
    return b

  # ---- the opt-in

  def test_constants_are_pandas(self):
    defines = dict(re.findall(r"^#define (FORD_P\w+) +([\d.]+)", FORD_H.read_text(), re.M))
    self.assertEqual(int(defines["FORD_PARAM_PATH_ANGLE"]), FordSafetyFlags.PATH_ANGLE)
    self.assertEqual(int(defines["FORD_PATH_ANGLE_MAX"]), P.PATH_ANGLE_MAX_CAN)
    self.assertEqual(int(defines["FORD_PATH_ANGLE_MAX_DELTA"]), P.PATH_ANGLE_MAX_DELTA_CAN)
    self.assertLess(P.PATH_ANGLE_MAX_SPEED, float(defines["FORD_PATH_ANGLE_MAX_SPEED"]), "the controller lets go before panda refuses")
    self.assertEqual(P.PATH_ANGLE_TO_CAN, round(1 / 0.0005))

  def test_off_unless_the_host_opts_in(self):
    for alpha_long in (False, True):
      b = Bench(opt_in=False, alpha_long=alpha_long)
      self.assertFalse(b.CP.flags & FordFlags.PATH_ANGLE)
      self.assertFalse(b.CP.safetyConfigs[-1].safetyParam & FordSafetyFlags.PATH_ANGLE)
      b.test.arm([Profile(0.3)])               # even armed by mistake, a controller without the flag sends nothing
      rows = b.run(20, model_curvature=0.01) + b.play(model_curvature=0.01) + b.run(60, model_curvature=0.01)
      self.assertTrue(all(r["path_angle"] == 0 and r["ramp"] == 0 for r in rows))
      self.assertTrue(any(r["curvature"] != 0 for r in rows), "and the model still steers")
      self.assertEqual(b.refused, [])

  def test_enable_sets_both_sides_or_neither(self):
    b = Bench(opt_in=True)
    self.assertEqual(b.CP.flags, b.default_flags | FordFlags.PATH_ANGLE)
    self.assertEqual(b.CP.safetyConfigs[-1].safetyParam, b.default_param | FordSafetyFlags.PATH_ANGLE)
    CP = CarInterface.get_non_essential_params(CAR.FORD_FOCUS_MK4)      # LateralMotionControl, not CAN FD
    with self.assertRaises(ValueError):
      path_angle.enable(CP)
    self.assertFalse(CP.flags & FordFlags.PATH_ANGLE)

  # ---- one run, stage by stage

  def test_a_run_reaches_the_wire_and_panda_as_prescribed(self):
    for level in (0.3, -0.3, 0.49, -0.49, 0.05):
      with self.subTest(level=level):
        b = self.bench(level)
        b.run(12)
        rows = b.play()
        want = round(level * P.PATH_ANGLE_TO_CAN)
        step = 50                                              # 0.5 rad/s at 20 Hz
        peak = [r for r in rows if r["path_angle"] == want]
        self.assertEqual(len(peak), 1 + 40, "the frame that reaches the level, then 2.0 s")
        up = [r["path_angle"] for r in rows[:rows.index(peak[0]) + 1]]
        self.assertEqual(up, [(1 if want > 0 else -1) * min(abs(want), step * (i + 1)) for i in range(len(up))])
        down = [r["path_angle"] for r in rows[rows.index(peak[-1]):]]
        self.assertEqual(down[-1], 0)
        self.assertTrue(all(abs(a) - abs(c) in (0, step, abs(a)) for a, c in zip(down, down[1:], strict=False)))
        self.assertTrue(all(abs(a) >= abs(c) for a, c in zip(down, down[1:], strict=False)), "never back up while letting go")
        for r in rows:
          self.assertEqual(r["requested"], r["path_angle"], "nothing between the profile and the frame changed it")
          self.assertTrue(r["accepted"])
          self.assertEqual((r["mode"], r["curvature"], r["path_offset"], r["curvature_rate"]), (1, 0, 0, 0))
          self.assertEqual(r["ramp"], P.PATH_ANGLE_RAMP_TYPE if r["path_angle"] else 0)
        self.assertEqual(b.refused, [])
        self.assertEqual((b.test.ended, b.test.stage), ("completed", Stage.done))

  def test_the_command_does_not_depend_on_the_model(self):
    def model(kind, i):
      return {"zero": 0.0, "left": 0.02, "right": -0.02, "huge": 5.0, "noise": random.Random(i).uniform(-0.03, 0.03),
              "square": 0.02 if (i // 7) % 2 else -0.02}[kind]

    runs = {}
    for kind in ("zero", "left", "right", "huge", "noise", "square"):
      b = self.bench(0.4, -0.25)
      i = 0
      rows = []
      for _ in range(60):                       # armed: the curvature command is on its way to zero or there
        rows.append(b.steer_frame(model_curvature=model(kind, i)))
        i += 1
      for _ in range(2):                        # both profiles
        rows.append(b.steer_frame(button=True, model_curvature=model(kind, i)))
        while b.test.running:
          i += 1
          rows.append(b.steer_frame(model_curvature=model(kind, i)))
        rows += [b.steer_frame(model_curvature=model(kind, i + k)) for k in range(12)]
      self.assertEqual(b.refused, [])
      runs[kind] = [r["bytes"] for r in rows]
    for kind, frames in runs.items():
      self.assertEqual(frames, runs["zero"], f"model output '{kind}' changed a frame")
    self.assertTrue(any(decode(f)["path_angle"] > 700 for f in runs["zero"]) and any(decode(f)["path_angle"] < -400 for f in runs["zero"]))

  def test_armed_means_the_model_steers_nothing(self):
    b = self.bench()
    rows = b.run(40, model_curvature=0.02)
    self.assertNotEqual(rows[-1]["curvature"], 0, "not armed: the stock curvature command")
    b.test.arm([Profile(0.3)])
    rows = b.run(40, model_curvature=0.02)
    self.assertTrue(all(r["curvature"] == 0 for r in rows[1:]), "armed: zero, reached under the curvature limits")
    rows = b.play(model_curvature=0.02) + b.run(40, model_curvature=0.02)
    self.assertTrue(all(r["curvature"] == 0 for r in rows), "through the run and after the last profile")
    self.assertEqual(b.test.stage, Stage.done)
    self.assertEqual(b.refused, [])
    b.test.disarm()
    self.assertNotEqual(b.run(40, model_curvature=0.02)[-1]["curvature"], 0, "disarmed: the model steers again")

  def test_a_run_does_not_start_over_a_curvature_command(self):
    # at these speeds the curvature limits let the command reach zero in a frame, so the wait is tested where it is decided
    test = PathAngleTest()
    test.arm([Profile(0.3)])
    cs = structs.CarState(vEgoRaw=2.0, gearShifter=GearShifter.drive)
    press = [structs.CarState.ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=True)]
    for _ in range(30):
      test.update(cs, True, curvature_zero=False)
    test.buttons(press)
    self.assertEqual((test.update(cs, True, curvature_zero=False), test.stage), (0, Stage.armed))
    self.assertEqual((test.update(cs, True, curvature_zero=True), test.stage), (0, Stage.armed), "that press is not kept: the driver presses again")
    test.buttons(press)
    self.assertEqual((test.update(cs, True, curvature_zero=True), test.stage), (0, Stage.armed), "nor does it start the moment curvature is zero")
    for _ in range(path_angle.READY_FRAMES):
      test.update(cs, True, curvature_zero=True)
    test.buttons(press)
    self.assertGreater(test.update(cs, True, curvature_zero=True), 0)

  def test_a_run_starts_only_after_half_a_second_with_nothing_against_it(self):
    for kw in ({"torque": 0.8}, {"v": P.PATH_ANGLE_MAX_SPEED + 1.0}, {"lat_active": False}, {"v": 0.1}):
      with self.subTest(kw=kw):
        b = self.bench(0.3)
        b.run(20)
        b.run(3, **kw)
        for i in range(path_angle.READY_FRAMES):
          self.assertEqual(b.steer_frame(button=True)["path_angle"], 0, f"{i} clean frames after it")
        self.assertGreater(b.steer_frame(button=True)["path_angle"], 0)
        self.assertEqual(b.refused, [])

  # ---- ending a run

  ABORTS = {
    "driver steering": [{"torque": 0.6}, {"torque": -0.6}, {"pressed": True}],
    "lateral control not active": [{"lat_active": False}],
    "speed outside the band": [{"v": P.PATH_ANGLE_MAX_SPEED + 0.05}, {"v": 0.3}],
    "steering fault": [{"fault": True}],
    "not in drive": [{"gear": GearShifter.neutral}],
    "stopped by the driver's button": [{"button": True}],
  }

  def test_an_abort_zeroes_the_command_and_withdraws_the_request_in_the_same_frame(self):
    for why, cases in self.ABORTS.items():
      for kw in cases:
        for after in (3, 12, 30, 55):                           # winding up, at the level, into the hold, letting go
          with self.subTest(why=why, kw=kw, after=after):
            b = self.bench(0.3, -0.2)
            b.run(12)
            b.steer_frame(button=True)
            b.run(after)
            self.assertTrue(b.test.running)
            self.assertEqual(b.rows[-1]["mode"], 1)
            self.assertNotEqual(b.rows[-1]["path_angle"], 0)
            row = b.steer_frame(model_curvature=0.02, **kw)
            self.assertEqual((row["mode"], row["path_angle"], row["curvature"], row["ramp"]), (0, 0, 0, 0),
                             "no steering request, and nothing in the signals, in the very next frame")
            self.assertEqual((b.test.ended, b.test.stage), (why, Stage.aborted))
            self.assertTrue(row["accepted"])
            self.assertEqual(b.refused, [])

  def test_after_an_abort_nothing_brings_the_request_back_but_disengaging(self):
    for why, cases in self.ABORTS.items():
      kw = cases[0]
      with self.subTest(why=why):
        b = self.bench(0.3, -0.2)
        b.run(12)
        b.steer_frame(button=True)
        b.run(12)
        b.steer_frame(**kw)
        # the reason gone, hands off, in the band, engaged the whole time: for five seconds and a dozen presses
        rows = []
        for i in range(100):
          rows.append(b.steer_frame(button=i % 8 == 0, model_curvature=0.02))
        # (also when the abort was lateral dropping out for that one frame: a blip is not the driver disengaging)
        self.assertTrue(all((r["mode"], r["path_angle"], r["curvature"]) == (0, 0, 0) for r in rows), "still withdrawn")
        self.assertEqual(b.test.stage, Stage.aborted)
        b.steer_frame(lat_active=False)                          # the driver disengages...
        self.assertEqual(b.test.stage, Stage.armed)
        row = b.steer_frame(button=True)                         # ...and engages again: steering is requested, at zero
        self.assertEqual((row["mode"], row["path_angle"], row["curvature"]), (1, 0, 0), "and a press this soon starts nothing")
        b.run(12)
        rows = b.play()
        self.assertEqual(max(r["path_angle"] for r in rows), 600, "the profile that was cut short, again")
        self.assertEqual((b.test.ended, b.test.index), ("completed", 1))
        self.assertEqual(b.refused, [])

  def test_a_completed_run_keeps_the_request_through_its_own_unwind(self):
    b = self.bench(0.3, -0.2)
    b.run(12)
    rows = b.play() + b.run(20)
    self.assertTrue(all(r["mode"] == 1 for r in rows), "controlled unwind: the PSCM stays in control down to zero and after")
    self.assertEqual((b.test.ended, b.test.stage), ("completed", Stage.armed))
    self.assertFalse(b.test.withdrawn)

  def test_disarming_does_not_hand_back_a_withdrawn_request(self):
    b = self.bench(0.3)
    b.run(12)
    b.steer_frame(button=True)
    b.run(12)
    b.steer_frame(torque=0.8)
    b.test.disarm()
    self.assertTrue(all(r["mode"] == 0 and r["curvature"] == 0 for r in b.run(20, model_curvature=0.02)))
    b.steer_frame(lat_active=False)
    self.assertEqual(b.test.stage, Stage.idle)
    self.assertNotEqual(b.run(20, model_curvature=0.02)[-1]["curvature"], 0, "disengaged and re-engaged with nothing armed: the stock controller")

  def test_the_dash_is_told_when_the_request_is_withdrawn(self):
    # the lane-centering UI message follows what is actually asked of the PSCM, not openpilot's own state
    def ui_states(b, n, **kw):
      seen = []
      for _ in range(n):
        b.step(**kw)
        seen.append(b.CI.CC.lkas_enabled_last)
      return seen
    b = self.bench(0.3)
    b.run(12)
    b.steer_frame(button=True)
    b.run(12)
    self.assertTrue(b.CI.CC.lkas_enabled_last)
    b.steer_frame(torque=0.8)
    self.assertEqual(set(ui_states(b, 20)), {False})

  def test_a_press_between_steer_frames_counts(self):
    for offset in range(5):
      b = self.bench(0.3)
      b.run(12)
      for _ in range(offset):
        b.step()
      b.step(button=True)
      self.assertTrue(b.steer_frame()["path_angle"] > 0 or b.rows[-1]["path_angle"] > 0, offset)

  def test_profiles_play_in_order_one_per_press(self):
    b = self.bench(0.1, -0.1, 0.2)
    b.run(12)
    peaks = []
    for _ in range(3):
      rows = b.play()
      peaks.append(max((r["path_angle"] for r in rows), key=abs))
      b.run(12)
    self.assertEqual(peaks, [200, -200, 400])
    self.assertEqual((b.test.stage, b.test.runs), (Stage.done, 3))
    self.assertTrue(all(r["path_angle"] == 0 for r in b.play()), "nothing left to play")
    self.assertEqual(b.refused, [])

  def test_a_running_profile_cannot_be_replaced(self):
    b = self.bench(0.3)
    b.run(12)
    b.steer_frame(button=True)
    with self.assertRaises(RuntimeError):
      b.test.arm([Profile(-0.4)])

  # ---- bounds

  def test_profiles_outside_the_bounds_are_not_accepted(self):
    for bad in ({"level": 0.0}, {"level": 0.5}, {"level": -0.6}, {"level": 0.3, "rise": 1.2}, {"level": 0.3, "fall": 5.0},
                {"level": 0.3, "rise": 0.0}, {"level": 0.3, "hold": 0.0}, {"level": 0.3, "hold": 11.0}):
      with self.subTest(bad=bad), self.assertRaises(ValueError):
        Profile(**bad)
    Profile(0.49, rise=1.0, fall=1.0, hold=10.0)
    Profile(-0.49, rise=0.01, fall=0.01)

  def test_whatever_happens_every_frame_is_inside_pandas_bounds(self):
    # random profiles, speeds, interruptions and model outputs: the controller never builds a frame panda refuses,
    # the encoded value never leaves the range or lands on the wrong side, and it never winds up faster than the step
    rng = random.Random(7)
    for trial in range(60):
      levels = [rng.choice((-1, 1)) * rng.uniform(0.02, 0.49) for _ in range(3)]
      b = Bench(alpha_long=rng.random() < 0.5)
      b.test.arm([Profile(level, hold=rng.uniform(0.2, 1.5), rise=rng.uniform(0.05, 1.0), fall=rng.uniform(0.05, 1.0)) for level in levels])
      last = 0
      for _ in range(400):
        kw = {"v": rng.choice((0.2, 1.0, 2.0, 3.9, 4.0, 4.2, 6.0)) if rng.random() < 0.05 else 2.0,
              "torque": rng.uniform(-1.5, 1.5) if rng.random() < 0.02 else 0.0, "lat_active": rng.random() > 0.01,
              "button": rng.random() < 0.08, "model_curvature": rng.uniform(-0.03, 0.03)}
        row = b.steer_frame(**kw)
        pa = row["path_angle"]
        self.assertLessEqual(abs(pa), P.PATH_ANGLE_MAX_CAN)
        if pa and b.test.profile is not None:
          self.assertEqual(pa > 0, b.test.profile.level > 0, "on the commanded side")
        if abs(pa) > abs(last) or (pa * last < 0):
          self.assertLessEqual(abs(pa - last), P.PATH_ANGLE_MAX_DELTA_CAN)
        if pa:
          self.assertEqual(row["curvature"], 0)
        last = pa
      self.assertEqual(b.refused, [], f"trial {trial}")

  def test_limit_is_pandas_rule(self):
    # limit() on arbitrary targets against panda itself: its output is always accepted, and one unit more is not
    rng = random.Random(11)
    b = self.bench()
    safety = b.safety
    packer = b.packer

    def tx(can):
      b.frame += 5                                           # a steer frame every 50 ms, as panda's message-rate check expects
      safety.set_timer(int(b.frame * DT_CTRL * 1e6))
      values = {"LatCtl_D2_Rq": 1, "LatCtlPath_An_Actl": can / P.PATH_ANGLE_TO_CAN, "LatCtlPathOffst_L_Actl": 0, "LatCtlCurv_No_Actl": 0,
                "LatCtlCrv_NoRate2_Actl": 0}
      addr, dat, bus = packer.make_can_msg("LateralMotionControl2", 0, values)
      return bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)))

    b.run(3)
    last = 0
    for _ in range(3000):
      target = rng.randint(-1100, 1100) if rng.random() < 0.7 else last + rng.randint(-160, 160)
      out = path_angle.limit(target, last, 2.0, True)
      self.assertEqual(safety.get_ford_path_angle_last(), last)
      self.assertTrue(tx(out), (last, target, out))
      last = out
    for last, over in ((0, 101), (0, -101), (300, 401), (-300, -401), (30, -71), (980, 981)):
      safety.set_safety_hooks(structs.CarParams.SafetyModel.ford, b.CP.safetyConfigs[-1].safetyParam)
      safety.init_tests()
      b.run(3)
      at = 0
      while at != last:
        at = path_angle.limit(last, at, 2.0, True)
        self.assertTrue(tx(at))
      self.assertFalse(tx(over), (last, over))

  def test_above_the_speed_the_controller_has_let_go_before_panda_refuses(self):
    b = self.bench(0.4)
    b.run(12)
    b.steer_frame(button=True)
    b.run(20)
    self.assertGreater(b.rows[-1]["path_angle"], 0)
    # the car reads 4.05 m/s, panda's own reading may differ: anywhere up to its limit the release is accepted
    row = b.steer_frame(v=P.PATH_ANGLE_MAX_SPEED + 0.05, panda_v=4.4)
    self.assertEqual((row["path_angle"], row["accepted"]), (0, True))
    self.assertEqual(b.refused, [])


def print_run(level: float) -> None:
  b = Bench()
  b.test.arm([Profile(level)])
  b.run(12)
  rows = b.play() + b.run(3)
  print(f"profile: level {level:+.3f} rad, hold 2.0 s, rise 0.5 rad/s, fall 0.5 rad/s; safety param {b.CP.safetyConfigs[-1].safetyParam}")
  print("    t  stage   requested   encoded (rad)   curv mode ramp  accepted   frame")
  for r in rows:
    rad = r['path_angle'] / P.PATH_ANGLE_TO_CAN
    print(f"{r['t']:5.2f}  {r['stage']:6s} {r['requested']:+9d}   {r['path_angle']:+5d} ({rad:+.4f})  {r['curvature']:+4d}  {r['mode']}    {r['ramp']}     " +
          f"{'yes' if r['accepted'] else 'NO '}      {r['bytes'].hex()}")
  print(f"ended: {b.test.ended}; frames panda refused: {b.refused}; safety copies disagreed: {bool(b.safety.safety_redundancy_fault)}")
  print("response: none on a bench. On the car it is read from the log (steering angle, PSCM status).")


if __name__ == "__main__":
  if len(sys.argv) > 1 and sys.argv[1] == "bench":
    print_run(float(sys.argv[2]) if len(sys.argv) > 2 else 0.3)
  else:
    unittest.main()
