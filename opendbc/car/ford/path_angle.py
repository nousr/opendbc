"""Low-speed path angle command for CAN FD Fords: opt-in, bounded, and independent of the driving model.

The stock port steers with LatCtlCurv_No_Actl alone, which the wire stops at 0.02 1/m. The PSCM also follows
LatCtlPath_An_Actl. How much steering it gives for a path angle at walking speed, and whether it lets go again, is
not known for any car until it is measured there; this module exists to make that measurement and nothing else yet.

What it does: on the driver's own button press it plays one prescribed profile (wind up at a fixed rate, hold, let
go at a fixed rate) into the path angle signal, with the curvature signal at zero. Nothing from the model, the
planner or controlsd enters the command: CarControl is read for latActive only, and from the moment profiles are
armed until they are disarmed the curvature command is held at zero as well.

Everything is kept in wire units (0.0005 rad, positive left, as on the bus), so the limits here are the limits panda
checks and no sign or scale conversion sits between a clip and the frame.

Bounds on what is sent, each also enforced by panda (opendbc/safety/modes/ford.h):
  range     +-PATH_ANGLE_MAX_CAN
  wind-up   PATH_ANGLE_MAX_DELTA_CAN per frame; toward zero on the same side is not limited
  speed     zero above PATH_ANGLE_MAX_SPEED
  alone     the curvature command is zero in any frame that carries a path angle
These are bounds on the command. They are not a bound on the steering response, which is what is being measured.

A run ends at once, and the command goes to zero in the same frame, when the driver steers, presses the button
again, when lateral control is no longer active, the speed leaves the band, or the PSCM reports a fault. Ending a run
early costs nothing: it is simply a shorter measurement.
"""
from dataclasses import dataclass
from enum import StrEnum

from opendbc.car import structs
from opendbc.car.ford.values import CarControllerParams as P, FordFlags, FordSafetyFlags

ButtonType = structs.CarState.ButtonEvent.Type
START_STOP_BUTTON = ButtonType.gapAdjustCruise   # the ACC distance button on the wheel
DRIVER_TORQUE_MAX = 0.5        # Nm on the column, below the 1.0 Nm that sets steeringPressed
MIN_SPEED = 0.5                # m/s: do not wind the wheel of a stopped car
STEER_HZ = 20                  # LateralMotionControl2 frames per second (100 Hz / STEER_STEP)
# A run starts only after this many steer frames in a row with nothing against it (0.5 s): hands off, in the speed
# band, engaged. It also keeps the start clear of panda's view of the speed, which is the fastest of its last six
# samples: a run may not begin on a reading panda has not caught up with, since a frame panda refuses is followed by
# refusals until the command is back near zero.
READY_FRAMES = 10


def enable(CP) -> None:
  """Opt a CarParams in, on both sides at once. For the host to call, on an explicit setting, before the safety
  configuration is sent to panda. A fingerprint never sets this."""
  if not CP.flags & FordFlags.CANFD:
    raise ValueError("the path angle command is for LateralMotionControl2 (CAN FD) only")
  CP.flags |= FordFlags.PATH_ANGLE.value
  CP.safetyConfigs[-1].safetyParam |= FordSafetyFlags.PATH_ANGLE.value


def to_can(rad: float) -> int:
  return round(rad * P.PATH_ANGLE_TO_CAN)


def limit(target: int, last: int, v_ego: float, active: bool) -> int:
  """The next path angle command in wire units: the same rules panda applies, in the same integers."""
  if not active or v_ego > P.PATH_ANGLE_MAX_SPEED:
    return 0
  target = max(-P.PATH_ANGLE_MAX_CAN, min(P.PATH_ANGLE_MAX_CAN, int(target)))
  if target == 0 or (last > 0 and 0 < target <= last) or (last < 0 and last <= target < 0):
    return target                                  # zero, or letting go on the same side
  return max(last - P.PATH_ANGLE_MAX_DELTA_CAN, min(last + P.PATH_ANGLE_MAX_DELTA_CAN, target))


@dataclass(frozen=True)
class Profile:
  """One run. level is signed in wire convention: positive left. Rates are per second."""
  level: float            # rad
  hold: float = 2.0       # s at the level
  rise: float = 0.5       # rad/s winding up
  fall: float = 0.5       # rad/s letting go

  def __post_init__(self):
    rate_max = P.PATH_ANGLE_MAX_DELTA_CAN * STEER_HZ / P.PATH_ANGLE_TO_CAN
    if not 0 < abs(to_can(self.level)) <= P.PATH_ANGLE_MAX_CAN:
      raise ValueError(f"level must be within +-{P.PATH_ANGLE_MAX_CAN / P.PATH_ANGLE_TO_CAN} rad and not zero")
    if not 0 < self.hold <= 10.0:
      raise ValueError("hold must be within (0, 10] s")
    for name, rate in (("rise", self.rise), ("fall", self.fall)):
      if not 0.01 <= rate <= rate_max:
        raise ValueError(f"{name} must be within [0.01, {rate_max}] rad/s")


class Stage(StrEnum):
  idle = "idle"          # nothing armed
  armed = "armed"        # a profile is loaded; the driver's button starts it
  rise = "rise"
  hold = "hold"
  fall = "fall"
  done = "done"          # every profile has been played; still armed, so the model still steers nothing


class PathAngleTest:
  """Plays armed Profiles, one per press of the driver's button. buttons() is called on every car state, update()
  once per steer frame (20 Hz). A profile that completes makes way for the next; one that was cut short is kept, so
  the next press repeats it. After the last one the button does nothing more, until the host disarms."""

  def __init__(self):
    self.hz = STEER_HZ
    self.profiles: list[Profile] = []
    self.index = 0
    self.stage = Stage.idle
    self.frames = 0          # frames spent in the current stage
    self.target = 0          # wire units the profile asks for, before limit()
    self.runs = 0
    self.ended = ""          # why the last run ended
    self.ready = 0           # steer frames in a row with nothing against a run
    self._pressed = False

  def arm(self, profiles: list[Profile]) -> None:
    """Load the profiles to play. Only between runs: a run in progress is not replaced under the driver."""
    if self.running:
      raise RuntimeError("a run is in progress")
    self.profiles, self.index, self.target, self.frames, self._pressed = list(profiles), 0, 0, 0, False
    self.stage = Stage.armed if self.profiles else Stage.idle

  def disarm(self) -> None:
    self._end("disarmed")
    self.profiles, self.stage = [], Stage.idle

  @property
  def running(self) -> bool:
    return self.stage in (Stage.rise, Stage.hold, Stage.fall)

  @property
  def armed(self) -> bool:
    """A profile is loaded, running or not. From here to disarm the curvature command is held at zero, so the
    model steers nothing before, during or between runs."""
    return self.stage != Stage.idle

  @property
  def profile(self) -> Profile | None:
    return self.profiles[self.index] if self.index < len(self.profiles) else None

  def _end(self, why: str) -> None:
    if self.running:
      self.ended = why
      if why == "completed":
        self.index += 1
      self.stage = Stage.armed if self.profile is not None else Stage.done
    self.target, self.frames = 0, 0

  def buttons(self, button_events) -> None:
    """Remember a press of the start/stop button until the next steer frame: car states come five to a frame."""
    self._pressed |= any(b.type == START_STOP_BUTTON and b.pressed for b in button_events)

  def stop_reason(self, CS, lat_active: bool) -> str:
    """Why a run may not start or go on, or ''."""
    if not lat_active:
      return "lateral control not active"
    if CS.steeringPressed or abs(CS.steeringTorque) > DRIVER_TORQUE_MAX:
      return "driver steering"
    if CS.steerFaultTemporary or CS.steerFaultPermanent:
      return "steering fault"
    if not MIN_SPEED <= CS.vEgoRaw <= P.PATH_ANGLE_MAX_SPEED:
      return "speed outside the band"
    if CS.gearShifter != structs.CarState.GearShifter.drive:
      return "not in drive"
    return ""

  def update(self, CS, lat_active: bool, curvature_zero: bool = True) -> int:
    """The profile's target for this frame in wire units; zero unless a run is in progress. curvature_zero: the
    curvature command has reached zero, which a run waits for before it starts."""
    pressed, self._pressed = self._pressed, False
    why = self.stop_reason(CS, lat_active)
    was_ready = self.ready >= READY_FRAMES
    self.ready = 0 if (why or not curvature_zero) else self.ready + 1
    if self.running:
      if pressed or why:
        self._end(why or "stopped by the driver's button")
        return 0
    elif self.stage == Stage.armed and pressed and was_ready and not why and curvature_zero:
      self.stage, self.frames, self.target, self.ended = Stage.rise, 0, 0, ""
      self.runs += 1
    if not self.running:
      return 0

    p = self.profile
    level = to_can(p.level)
    sign = 1 if level > 0 else -1
    self.frames += 1
    if self.stage == Stage.rise:
      self.target = sign * min(abs(level), abs(self.target) + max(1, to_can(p.rise) // self.hz))
      if self.target == level:
        self.stage, self.frames = Stage.hold, 0
    elif self.stage == Stage.hold:
      if self.frames >= round(p.hold * self.hz):
        self.stage, self.frames = Stage.fall, 0
    else:
      self.target = sign * max(0, abs(self.target) - max(1, to_can(p.fall) // self.hz))
      if self.target == 0:
        self._end("completed")
    return self.target
