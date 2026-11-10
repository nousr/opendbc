"""Lateral-only control on a CAN FD Ford: steering without ACC, the pedals the driver's own.

The driver asks for it, and takes it back, with the lane centering button on the wheel (ButtonType.lkas in
carState.buttonEvents), while cruise is switched on (cruiseState.available). The cancel button (ButtonType.cancel,
reported only when opted in) and switching cruise off also end it. The brake does not.

Panda reads the same buttons off the bus and keeps its own permission (opendbc/safety/modes/ford.h,
FORD_PARAM_LAT_ONLY): every lane centering press grants it, and it lasts only while the host's heartbeat says the
host is engaged. So the host alone decides whether a press means on or off, and panda follows: a press the host
refuses, and an engagement the host ends for any reason, are gone from panda within about half a second. Cancel
and cruise off remove it in panda directly. The steering message is allowed while that permission or
controls_allowed holds, and nothing longitudinal is.

This module only opts a CarParams in. The engagement itself is the host's state machine. No fingerprint sets it, and
only a debug panda build honours the safety param bit.
"""
from opendbc.car.ford.values import FordFlags, FordSafetyFlags


def enable(CP) -> None:
  """Opt a CarParams in, on both sides at once, before the safety configuration is sent to panda."""
  if not CP.flags & FordFlags.CANFD:
    raise ValueError("lateral-only control is for CAN FD Fords only")
  CP.flags |= FordFlags.LAT_ONLY.value
  CP.safetyConfigs[-1].safetyParam |= FordSafetyFlags.LAT_ONLY.value


def enabled(CP) -> bool:
  return CP.brand == "ford" and bool(CP.flags & FordFlags.LAT_ONLY)
