#!/usr/bin/env python3
import numpy as np
import random
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.lateral import MAX_LATERAL_ACCEL, MAX_LATERAL_JERK
from opendbc.car.ford.values import FordSafetyFlags
from opendbc.car.structs import CarParams

MSG_BrakeSysFeatures = 0x415       # RX from ABS, for vehicle speed
MSG_EngVehicleSpThrottle2 = 0x202  # RX from PCM, for second vehicle speed
MSG_Yaw_Data_FD1 = 0x91            # RX from RCM, for yaw rate
MSG_Steering_Data_FD1 = 0x083      # TX by OP, various driver switches and LKAS/CC buttons
MSG_ACCDATA = 0x186                # TX by OP, ACC controls
MSG_ACCDATA_3 = 0x18A              # TX by OP, ACC/TJA user interface
MSG_Lane_Assist_Data1 = 0x3CA      # TX by OP, Lane Keep Assist
MSG_LateralMotionControl = 0x3D3   # TX by OP, Lateral Control message
MSG_LateralMotionControl2 = 0x3D6  # TX by OP, alternate Lateral Control message
MSG_IPMA_Data = 0x3D8              # TX by OP, IPMA and LKAS user interface


class Buttons:
  CANCEL = 0
  RESUME = 1
  TJA_TOGGLE = 2


# Ford safety has four different configurations tested here:
#  * CAN with openpilot longitudinal
#  * CAN FD with stock longitudinal
#  * CAN FD with openpilot longitudinal

class TestFordSafetyBase(common.CarSafetyTest):
  DBC = "ford_lincoln_base_pt"
  SAFETY_MODEL = CarParams.SafetyModel.ford

  STANDSTILL_THRESHOLD = 1
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                                 MSG_LateralMotionControl2, MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                               MSG_LateralMotionControl2, MSG_IPMA_Data]}

  STEER_MESSAGE = 0

  # Curvature control limits
  DEG_TO_CAN = 50000   # CAN units per rad/m
  MAX_CURVATURE = 0.02 # rad/m, 1000 CAN units
  MAX_CURVATURE_ERROR = 0.002         # rad/m, 100 CAN units
  CURVATURE_ERROR_MIN_SPEED = 10.0    # m/s
  LATERAL_FREQUENCY = 20              # Hz, for per-frame jerk limit

  lat_ctl_frame = 0

  def _get_max_curvature_can(self, speed):
    fudged_speed = max(speed - 1.0, 1.0)
    return int(MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed) * self.DEG_TO_CAN) + 1

  def _get_max_curvature_delta_can(self, speed):
    fudged_speed = max(speed - 1.0, 1.0)
    return int(MAX_LATERAL_JERK / (fudged_speed * fudged_speed) / self.LATERAL_FREQUENCY * self.DEG_TO_CAN) + 1

  def _get_max_curvature_delta_relaxed_can(self, speed):
    # flipped fudge, this is the least movement toward the error bounds safety requires
    fudged_speed = speed + 1.0
    return int(MAX_LATERAL_JERK / (fudged_speed * fudged_speed) / self.LATERAL_FREQUENCY * self.DEG_TO_CAN) - 1

  def _get_max_curvature_relaxed_can(self, speed):
    # flipped fudge, safety never requires commanding more curvature than openpilot can send
    fudged_speed = speed + 1.0
    max_curvature_accel_can = int(MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed) * self.DEG_TO_CAN) - 1
    return min(max_curvature_accel_can, round(self.MAX_CURVATURE * self.DEG_TO_CAN))

  def _set_prev_desired_angle(self, t):
    t = round(t * self.DEG_TO_CAN)
    self.safety.set_desired_curvature_last(t)

  def _reset_curvature_measurement(self, curvature, speed):
    for _ in range(6):
      self._rx(self._speed_msg(speed))
      self._rx(self._speed_msg_2(speed))
      self._rx(self._yaw_rate_msg(curvature, speed))

  # Driver brake pedal
  def _user_brake_msg(self, brake: bool):
    # brake pedal and cruise state share same message, so we have to send
    # the other signal too
    enable = self.safety.get_controls_allowed()
    values = {
      "BpedDrvAppl_D_Actl": 2 if brake else 1,
      "CcStat_D_Actl": 5 if enable else 0,
    }
    return self.packer.make_can_msg_safety("EngBrakeData", 0, values)

  # ABS vehicle speed
  def _speed_msg(self, speed: float, quality_flag=True):
    values = {"Veh_V_ActlBrk": speed * 3.6, "VehVActlBrk_D_Qf": 3 if quality_flag else 0}
    return self.packer.make_can_msg_safety("BrakeSysFeatures", 0, values)

  # PCM vehicle speed
  def _speed_msg_2(self, speed: float, quality_flag=True):
    # Ford relies on speed for driver curvature limiting, so it checks two sources
    values = {"Veh_V_ActlEng": speed * 3.6, "VehVActlEng_D_Qf": 3 if quality_flag else 0}
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle2", 0, values)

  # Standstill state
  def _vehicle_moving_msg(self, speed: float):
    values = {"VehStop_D_Stat": 1 if speed <= self.STANDSTILL_THRESHOLD else random.choice((0, 2, 3))}
    return self.packer.make_can_msg_safety("DesiredTorqBrk", 0, values)

  # Current curvature
  def _yaw_rate_msg(self, curvature: float, speed: float, quality_flag=True):
    values = {"VehYaw_W_Actl": curvature * speed, "VehYawWActl_D_Qf": 3 if quality_flag else 0}
    return self.packer.make_can_msg_safety("Yaw_Data_FD1", 0, values)

  # Drive throttle input
  def _user_gas_msg(self, gas: float):
    values = {"ApedPos_Pc_ActlArb": gas}
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle", 0, values)

  # Cruise status
  def _pcm_status_msg(self, enable: bool):
    # brake pedal and cruise state share same message, so we have to send
    # the other signal too
    brake = self.safety.get_brake_pressed_prev()
    values = {
      "BpedDrvAppl_D_Actl": 2 if brake else 1,
      "CcStat_D_Actl": 5 if enable else 0,
    }
    return self.packer.make_can_msg_safety("EngBrakeData", 0, values)

  def test_enable_control_allowed_from_queue_assist(self):
    msg = self.packer.make_can_msg_safety("EngBrakeData", 0, {"BpedDrvAppl_D_Actl": 1, "CcStat_D_Actl": 4})
    self.assertTrue(self._rx(msg))
    self.assertTrue(self.safety.get_controls_allowed())

  # LKAS command
  def _lkas_command_msg(self, action: int):
    values = {
      "LkaActvStats_D2_Req": action,
    }
    return self.packer.make_can_msg_safety("Lane_Assist_Data1", 0, values)

  # LCA command
  def _lat_ctl_msg(self, enabled: bool, path_offset: float, path_angle: float, curvature: float, curvature_rate: float,
                   increment_timer: bool = True):
    if increment_timer:
      self.safety.set_timer(self.lat_ctl_frame * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.lat_ctl_frame += 1
    if self.STEER_MESSAGE == MSG_LateralMotionControl:
      values = {
        "LatCtl_D_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,     # Path offset [-5.12|5.11] meter
        "LatCtlPath_An_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
        "LatCtlCurv_NoRate_Actl": curvature_rate,  # Curvature rate [-0.001024|0.00102375] 1/meter^2
        "LatCtlCurv_No_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
      }
      return self.packer.make_can_msg_safety("LateralMotionControl", 0, values)
    elif self.STEER_MESSAGE == MSG_LateralMotionControl2:
      values = {
        "LatCtl_D2_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,     # Path offset [-5.12|5.11] meter
        "LatCtlPath_An_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
        "LatCtlCrv_NoRate2_Actl": curvature_rate,  # Curvature rate [-0.001024|0.001023] 1/meter^2
        "LatCtlCurv_No_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
      }
      return self.packer.make_can_msg_safety("LateralMotionControl2", 0, values)

  # Cruise control buttons
  def _acc_button_msg(self, button: int, bus: int):
    values = {
      "CcAslButtnCnclPress": 1 if button == Buttons.CANCEL else 0,
      "CcAsllButtnResPress": 1 if button == Buttons.RESUME else 0,
      "TjaButtnOnOffPress": 1 if button == Buttons.TJA_TOGGLE else 0,
    }
    return self.packer.make_can_msg_safety("Steering_Data_FD1", bus, values)

  def test_rx_hook_speed_mismatch(self):
    for speed in np.arange(0, 40, 0.5):
      for speed_delta in np.arange(-5, 5, 0.1):
        speed_2 = round(max(speed + speed_delta, 0), 1)
        self._rx(self._speed_msg(speed))
        self._rx(self._speed_msg_2(speed_2))
        self.safety.set_controls_allowed(True)
        self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0))

        within_delta = common.speeds_within_delta(speed, speed_2)
        self.assertEqual(self.safety.get_controls_allowed(), within_delta)

  def test_rx_hook(self):
    # checksum, counter, and quality flag checks
    for quality_flag in [True, False]:
      for msg_type in ["speed", "speed_2", "yaw"]:
        self.safety.set_controls_allowed(True)
        # send multiple times to verify counter checks
        for _ in range(10):
          if msg_type == "speed":
            msg = self._speed_msg(0, quality_flag=quality_flag)
          elif msg_type == "speed_2":
            msg = self._speed_msg_2(0, quality_flag=quality_flag)
          elif msg_type == "yaw":
            msg = self._yaw_rate_msg(0, 0, quality_flag=quality_flag)

          self.assertEqual(quality_flag, self._rx(msg))
          self.assertEqual(quality_flag, self.safety.get_controls_allowed())

        # Mess with checksum to make it fail, checksum is not checked for 2nd speed
        msg[0].data[3] = 0  # Speed checksum & half of yaw signal
        should_rx = msg_type == "speed_2" and quality_flag
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

  def test_angle_measurements(self):
    """Tests rx hook correctly parses the curvature measurement from the vehicle speed and yaw rate"""
    for speed in np.arange(0.5, 40, 0.5):
      for curvature in np.arange(0, self.MAX_CURVATURE * 2, 2e-3):
        self._rx(self._speed_msg(speed))
        for c in (curvature, -curvature, 0, 0, 0, 0):
          self._rx(self._yaw_rate_msg(c, speed))

        self.assertEqual(self.safety.get_curvature_meas_min(), round(-curvature * self.DEG_TO_CAN))
        self.assertEqual(self.safety.get_curvature_meas_max(), round(curvature * self.DEG_TO_CAN))

        self._rx(self._yaw_rate_msg(0, speed))
        self.assertEqual(self.safety.get_curvature_meas_min(), round(-curvature * self.DEG_TO_CAN))
        self.assertEqual(self.safety.get_curvature_meas_max(), 0)

        self._rx(self._yaw_rate_msg(0, speed))
        self.assertEqual(self.safety.get_curvature_meas_min(), 0)
        self.assertEqual(self.safety.get_curvature_meas_max(), 0)

  def test_max_lateral_acceleration(self):
    # Ford CAN FD can achieve a higher max lateral acceleration than CAN so we limit curvature based on speed
    max_curvature_can = round(self.MAX_CURVATURE * self.DEG_TO_CAN)
    for speed in np.arange(0, 40, 0.5):
      max_can = min(self._get_max_curvature_can(speed), max_curvature_can)
      for offset in (-5, -1, 0, 1, 5):
        curvature_can = max_can + offset
        curvature = curvature_can / self.DEG_TO_CAN

        for sign in (-1, 1):
          signed_curvature = sign * curvature
          self.safety.set_controls_allowed(True)
          self._set_prev_desired_angle(signed_curvature)
          self._reset_curvature_measurement(signed_curvature, speed)

          should_tx = abs(curvature_can) <= max_can
          self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, signed_curvature, 0)))

  def test_steer_allowed(self):
    path_offsets = np.arange(-5.12, 5.11, 2.5).round()
    path_angles = np.arange(-0.5, 0.5235, 0.25).round(1)
    curvature_rates = np.arange(-0.001024, 0.00102375, 0.001).round(3)
    curvatures = np.arange(-0.02, 0.02094, 0.01).round(2)

    for speed in (self.CURVATURE_ERROR_MIN_SPEED - 1,
                  self.CURVATURE_ERROR_MIN_SPEED + 1):
      max_curvature_can = self._get_max_curvature_can(speed)
      for controls_allowed in (True, False):
        for steer_control_enabled in (True, False):
          for path_offset in path_offsets:
            for path_angle in path_angles:
              for curvature_rate in curvature_rates:
                for curvature in curvatures:
                  self.safety.set_controls_allowed(controls_allowed)
                  self._set_prev_desired_angle(curvature)
                  self._reset_curvature_measurement(curvature, speed)

                  should_tx = path_offset == 0 and path_angle == 0 and curvature_rate == 0
                  # when request bit is 0, only allow curvature of 0 since the signal range
                  # is not large enough to enforce it tracking measured
                  should_tx = should_tx and (controls_allowed if steer_control_enabled else curvature == 0)
                  should_tx = should_tx and abs(round(curvature * self.DEG_TO_CAN)) <= max_curvature_can

                  with self.subTest(controls_allowed=controls_allowed, steer_control_enabled=steer_control_enabled,
                                    path_offset=float(path_offset), path_angle=float(path_angle), curvature_rate=float(curvature_rate),
                                    curvature=float(curvature)):
                    self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(steer_control_enabled, path_offset, path_angle, curvature, curvature_rate)))

  def test_curvature_rate_limits(self):
    """
    When the curvature error is exceeded, commanded curvature must start moving towards meas respecting rate limits.
    Since safety allows higher rate limits to avoid false positives, we need to allow a lower rate to move towards meas.
    """
    self.safety.set_controls_allowed(True)
    # safety fudges the speed (1 m/s) and rate limits (1 CAN unit) to avoid false positives
    small_curvature = 1 / self.DEG_TO_CAN  # significant small amount of curvature to cross boundary

    for speed in np.arange(0, 40, 0.5):
      curvature_accel_limit = self._get_max_curvature_can(speed) / self.DEG_TO_CAN
      limit_command = speed > self.CURVATURE_ERROR_MIN_SPEED
      # ensure our limits match the safety's rounded limits
      # lateral jerk is symmetric, so the wind up and wind down limits are the same
      max_delta = self._get_max_curvature_delta_can(speed) / self.DEG_TO_CAN
      max_delta_relaxed = self._get_max_curvature_delta_relaxed_can(speed) / self.DEG_TO_CAN

      up_cases = (self.MAX_CURVATURE_ERROR * 2, [
        (not limit_command, 0, 0),
        (not limit_command, 0, max_delta_relaxed - small_curvature),
        (True, 0, max_delta_relaxed),
        (True, 0, max_delta),
        (False, 0, max_delta + small_curvature),
        # stay at boundary limit
        (True, self.MAX_CURVATURE_ERROR - small_curvature, self.MAX_CURVATURE_ERROR - small_curvature),
        # 1 unit below boundary limit
        (not limit_command, self.MAX_CURVATURE_ERROR - small_curvature * 2, self.MAX_CURVATURE_ERROR - small_curvature * 2),
        # shouldn't allow command to move outside the boundary limit if last was inside
        (not limit_command, self.MAX_CURVATURE_ERROR - small_curvature, self.MAX_CURVATURE_ERROR - small_curvature * 2),
      ])

      down_cases = (self.MAX_CURVATURE - self.MAX_CURVATURE_ERROR * 2, [
        (not limit_command, self.MAX_CURVATURE, self.MAX_CURVATURE),
        (not limit_command, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta_relaxed + small_curvature),
        (True, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta_relaxed),
        (True, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta),
        (False, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta - small_curvature),
      ])

      # the driver can hold a curvature openpilot may not command, safety must never require moving past
      # the most it can send: the lower of the lateral acceleration limit and what the EPS accepts
      max_curvature_relaxed = self._get_max_curvature_relaxed_can(speed) / self.DEG_TO_CAN
      # safety fudges the speed down for the accel check and up for the cap, so the last command can sit above the cap
      max_curvature_allowed_can = min(self._get_max_curvature_can(speed), round(self.MAX_CURVATURE * self.DEG_TO_CAN))
      winds_down_within_jerk = (max_curvature_allowed_can - self._get_max_curvature_relaxed_can(speed) <=
                                self._get_max_curvature_delta_can(speed))
      relaxed_cases = (self.MAX_CURVATURE * 2, [
        (True, max_curvature_relaxed, max_curvature_relaxed),
        (not limit_command, max_curvature_relaxed, max_curvature_relaxed - small_curvature),
        # no longer requiring the command to wind towards meas doesn't stop rate limiting it winding away
        (winds_down_within_jerk, max_curvature_allowed_can / self.DEG_TO_CAN, max_curvature_relaxed),
      ])

      for sign in (-1, 1):
        for angle_meas, cases in (up_cases, down_cases, relaxed_cases):
          self._reset_curvature_measurement(sign * angle_meas, speed)
          for should_tx, initial_curvature, desired_curvature in cases:

            # at low speeds one frame of jerk exceeds the curvature signal, so the should_tx=False cases will rightly not fail.
            # assert we never drop a case at a speed where the curvature error is enforced
            if abs(desired_curvature) > self.MAX_CURVATURE:
              self.assertLess(speed, self.CURVATURE_ERROR_MIN_SPEED)
              continue

            # can not send if the curvature is above the max lateral acceleration
            should_tx = should_tx and abs(desired_curvature) <= curvature_accel_limit

            self._set_prev_desired_angle(sign * initial_curvature)
            self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, sign * desired_curvature, 0)))

    # the newest speed sample gates the check, not the whole sample window
    max_error = self.MAX_CURVATURE_ERROR + small_curvature * 2
    for sign in (-1, 1):
      self._reset_curvature_measurement(0, self.CURVATURE_ERROR_MIN_SPEED - 1)
      self._rx(self._speed_msg(self.CURVATURE_ERROR_MIN_SPEED + 1))
      self._set_prev_desired_angle(sign * self.MAX_CURVATURE_ERROR)
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, sign * max_error, 0)))

  def test_curvature_violation(self):
    # If violation occurs, curvature cmd is blocked until reset to 0
    self.safety.set_controls_allowed(True)
    speed = 25.
    max_delta_can = self._get_max_curvature_delta_can(speed)
    self._reset_curvature_measurement(0, speed)

    self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0))
    over_curvature = (max_delta_can + 5) / self.DEG_TO_CAN
    for _ in range(20):
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, over_curvature, 0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0)))

    # prev tracks the commanded curvature on a passing tx (not reset to 0 every frame)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, max_delta_can / self.DEG_TO_CAN, 0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 2 * max_delta_can / self.DEG_TO_CAN, 0)))

  def test_rt_limits(self):
    # send rate is limited over a rolling 250ms window split into two half-interval buckets
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, 0)
    max_rt_msgs = int(self.LATERAL_FREQUENCY * common.RT_INTERVAL / 1e6 * 1.2 + 1)
    half = common.RT_INTERVAL // 2

    # too many messages within one window is blocked
    self.safety.set_timer(0)
    for i in range(max_rt_msgs * 2):
      self.assertEqual(i <= max_rt_msgs, self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))

    # shift the overflow into the previous bucket
    self.safety.set_timer(half)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))

    # previous bucket still counts within the half interval
    self.safety.set_timer(half + 2 * common.RT_INTERVAL // 5)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))

    # both buckets clear after a full interval
    self.safety.set_timer(half + common.RT_INTERVAL)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))
    self.safety.set_timer(half + 2 * common.RT_INTERVAL)
    for _ in range(max_rt_msgs):
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))

  def test_prevent_lkas_action(self):
    self.safety.set_controls_allowed(1)
    self.assertFalse(self._tx(self._lkas_command_msg(1)))

    self.safety.set_controls_allowed(0)
    self.assertFalse(self._tx(self._lkas_command_msg(1)))

  def test_acc_buttons(self):
    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for enabled in (True, False):
        self._rx(self._pcm_status_msg(enabled))
        self.assertTrue(self._tx(self._acc_button_msg(Buttons.TJA_TOGGLE, 2)))

    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for bus in (0, 2):
        self.assertEqual(allowed, self._tx(self._acc_button_msg(Buttons.RESUME, bus)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      for bus in (0, 2):
        self.assertEqual(enabled, self._tx(self._acc_button_msg(Buttons.CANCEL, bus)))


class TestFordCANFDStockSafety(TestFordSafetyBase):
  SAFETY_PARAM = FordSafetyFlags.CANFD

  STEER_MESSAGE = MSG_LateralMotionControl2

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl2, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                               MSG_IPMA_Data]}


class TestFordLongitudinalSafetyBase(TestFordSafetyBase):
  MAX_ACCEL = 2.0  # accel is used for brakes, but openpilot can set positive values
  MIN_ACCEL = -3.5
  INACTIVE_ACCEL = 0.0

  MAX_GAS = 2.0
  MIN_GAS = -0.5
  INACTIVE_GAS = -5.0

  # ACC command
  def _acc_command_msg(self, gas: float, brake: float, brake_actuation: bool, cmbb_deny: bool = False):
    values = {
      "AccPrpl_A_Rq": gas,                              # [-5|5.23] m/s^2
      "AccPrpl_A_Pred": gas,                            # [-5|5.23] m/s^2
      "AccBrkTot_A_Rq": brake,                          # [-20|11.9449] m/s^2
      "AccBrkPrchg_B_Rq": 1 if brake_actuation else 0,  # Pre-charge brake request: 0=No, 1=Yes
      "AccBrkDecel_B_Rq": 1 if brake_actuation else 0,  # Deceleration request: 0=Inactive, 1=Active
      "CmbbDeny_B_Actl": 1 if cmbb_deny else 0,         # [0|1] deny AEB actuation
    }
    return self.packer.make_can_msg_safety("ACCDATA", 0, values)

  def test_stock_aeb(self):
    # Test that CmbbDeny_B_Actl is never 1, it prevents the ABS module from actuating AEB requests from ACCDATA_2
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for cmbb_deny in (True, False):
        should_tx = not cmbb_deny
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.INACTIVE_GAS, self.INACTIVE_ACCEL, controls_allowed, cmbb_deny)))
        should_tx = controls_allowed and not cmbb_deny
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.MAX_GAS, self.MAX_ACCEL, controls_allowed, cmbb_deny)))

  def test_gas_safety_check(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for gas in np.concatenate((np.arange(self.MIN_GAS - 2, self.MAX_GAS + 2, 0.05), [self.INACTIVE_GAS])):
        gas = round(gas, 2)  # floats might not hit exact boundary conditions without rounding
        should_tx = (controls_allowed and self.MIN_GAS <= gas <= self.MAX_GAS) or gas == self.INACTIVE_GAS
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(gas, self.INACTIVE_ACCEL, controls_allowed)))

  def test_brake_safety_check(self):
    brake_values = self._boundary_values([self.MIN_ACCEL, self.MAX_ACCEL, self.INACTIVE_ACCEL],
                                         self.MIN_ACCEL - 2, self.MAX_ACCEL + 2, 0.05)
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for brake_actuation in (True, False):
        for brake in brake_values:
          should_tx = (controls_allowed and self.MIN_ACCEL <= brake <= self.MAX_ACCEL) or brake == self.INACTIVE_ACCEL
          should_tx = should_tx and (controls_allowed or not brake_actuation)
          self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.INACTIVE_GAS, brake, brake_actuation)))


class TestFordLongitudinalSafety(TestFordLongitudinalSafetyBase):
  # Make sure we enforce long safety even without long flag for CAN
  SAFETY_PARAM = 0
  STEER_MESSAGE = MSG_LateralMotionControl

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA, 0], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                               MSG_IPMA_Data]}


class TestFordCANFDLongitudinalSafety(TestFordLongitudinalSafetyBase):
  SAFETY_PARAM = FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD

  STEER_MESSAGE = MSG_LateralMotionControl2

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA, 0], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl2, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                               MSG_IPMA_Data]}


class TestFordCANFDPathAngleSafety(TestFordCANFDLongitudinalSafety):
  """The low-speed path angle command, opted in. Everything the stock configuration checks still holds; a path
  angle is accepted only inside these bounds. Each frame goes through safety_tx_hook, so both safety copies check
  it, and tearDown fails the test if they ever disagreed."""

  SAFETY_PARAM = FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD | FordSafetyFlags.PATH_ANGLE

  TO_CAN = 2000        # wire units per rad
  MAX = 980            # 0.49 rad
  MAX_DELTA = 100      # per frame winding up
  MAX_SPEED = 4.5      # m/s
  SPEED = 2.0

  def _ready(self, speed=None, param=None):
    """Fresh safety state, controls allowed, driving straight at `speed`."""
    self.safety.set_safety_hooks(self.SAFETY_MODEL, self.SAFETY_PARAM if param is None else param)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, self.SPEED if speed is None else speed)

  def _pa(self, can: int, enabled: bool = True, curvature: float = 0., path_offset: float = 0., curvature_rate: float = 0.):
    """One frame with the path angle at `can` wire units about zero."""
    return self._tx(self._lat_ctl_msg(enabled, path_offset, can / self.TO_CAN, curvature, curvature_rate))

  def _raw_pa(self, raw: int):
    """A frame with the path angle field set to a raw value the DBC range would not pack."""
    msg = self._lat_ctl_msg(True, 0, 0, 0, 0)
    dat = bytearray(bytes(msg[0].data)[:8])
    dat[3] = (dat[3] & 0xE0) | ((raw >> 6) & 0x1F)
    dat[4] = (dat[4] & 0x03) | ((raw & 0x3F) << 2)
    msg[0].data = bytes(dat)
    return self._tx(msg)

  def _wind_to(self, can: int):
    step = self.MAX_DELTA if can > 0 else -self.MAX_DELTA
    at = 0
    while at != can:
      at = can if abs(can - at) <= self.MAX_DELTA else at + step
      self.assertTrue(self._pa(at), at)
    self.assertEqual(self.safety.get_ford_path_angle_last(), can)

  def test_steer_allowed(self):
    # the stock sweep expects any path angle to be refused; with the opt-in, zero path angle behaves as stock
    for speed in (self.CURVATURE_ERROR_MIN_SPEED - 1, self.CURVATURE_ERROR_MIN_SPEED + 1):
      max_curvature_can = self._get_max_curvature_can(speed)
      for controls_allowed in (True, False):
        for steer_control_enabled in (True, False):
          for path_offset in (-2.0, 0.0, 2.0):
            for curvature_rate in (-0.001, 0.0, 0.001):
              for curvature in (-0.02, -0.01, 0.0, 0.01, 0.02):
                self.safety.set_controls_allowed(controls_allowed)
                self._set_prev_desired_angle(curvature)
                self._reset_curvature_measurement(curvature, speed)
                should_tx = path_offset == 0 and curvature_rate == 0
                should_tx = should_tx and (controls_allowed if steer_control_enabled else curvature == 0)
                should_tx = should_tx and abs(round(curvature * self.DEG_TO_CAN)) <= max_curvature_can
                with self.subTest(controls_allowed=controls_allowed, steer_control_enabled=steer_control_enabled, speed=speed,
                                  path_offset=path_offset, curvature_rate=curvature_rate, curvature=curvature):
                  self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(steer_control_enabled, path_offset, 0, curvature, curvature_rate)))

  def test_path_angle_wind_up_is_rate_limited(self):
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self._ready()
        self.assertTrue(self._pa(sign * self.MAX_DELTA))
        self.assertTrue(self._pa(sign * 2 * self.MAX_DELTA))
        self.assertFalse(self._pa(sign * (3 * self.MAX_DELTA + 1)), "one unit over the step")
        self.assertEqual(self.safety.get_ford_path_angle_last(), 0, "a refused frame is not on the bus")
        self.assertFalse(self._pa(sign * 2 * self.MAX_DELTA), "so the next is checked from zero")
        self.assertTrue(self._pa(sign * self.MAX_DELTA))

  def test_path_angle_step_from_zero(self):
    # the failure of a rate check kept outside the per-copy state: copy B updates the shared last value, copy A then
    # compares the frame with itself and allows any step. Here both copies refuse, and they agree.
    self._ready()
    for can in (self.MAX_DELTA + 1, 200, 900, self.MAX, -(self.MAX_DELTA + 1), -900, -self.MAX):
      with self.subTest(can=can):
        self.assertFalse(self._pa(can))
        self.assertFalse(self.safety.safety_redundancy_fault)
        self.assertTrue(self._pa(0))

  def test_path_angle_range(self):
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self._ready()
        self._wind_to(sign * self.MAX)
        self.assertTrue(self._pa(sign * self.MAX), "holding the limit")
        self.assertFalse(self._pa(sign * (self.MAX + 1)))
    # the ends of the 11-bit field, which the DBC range leaves unused on the left
    self._ready()
    self._wind_to(self.MAX)
    for raw in (1000 + self.MAX + 1, 2047):
      self.assertFalse(self._raw_pa(raw), raw)

  def test_path_angle_letting_go_is_not_limited(self):
    for sign in (1, -1):
      for to in (900, 500, 1, 0):
        with self.subTest(sign=sign, to=to):
          self._ready()
          self._wind_to(sign * self.MAX)
          self.assertTrue(self._pa(sign * to))
          self.assertEqual(self.safety.get_ford_path_angle_last(), sign * to)

  def test_path_angle_may_not_jump_across_zero(self):
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self._ready()
        self._wind_to(sign * 600)
        self.assertFalse(self._pa(-sign * 10), "smaller, but on the other side: 610 units in a frame")
        self._wind_to(sign * 30)
        self.assertTrue(self._pa(-sign * 20), "across zero within one step")

  def test_path_angle_wire_wrap_is_refused(self):
    # a hard right that leaves the signal's range re-enters it as a hard left: raw 80 (-0.46 rad) to raw 2018 (+0.509)
    self._ready()
    self._wind_to(-920)
    self.assertFalse(self._raw_pa(2018))
    self.assertEqual(self.safety.get_ford_path_angle_last(), 0)
    self.assertFalse(self.safety.safety_redundancy_fault)

  def test_path_angle_speed_limit(self):
    for speed, allowed in ((0.0, True), (3.0, True), (self.MAX_SPEED - 0.1, True), (self.MAX_SPEED + 0.1, False), (9.0, False), (15.0, False)):
      with self.subTest(speed=speed):
        self._ready(speed)
        self.assertEqual(self._pa(self.MAX_DELTA), allowed)
        self.assertTrue(self._pa(0), "zero is acceptable at any speed")
    # accelerating through the limit with a command held: the fastest recent sample decides
    self._ready()
    self._wind_to(400)
    self._rx(self._speed_msg(self.MAX_SPEED + 0.2))     # one sample of six; both speed sources, or they mismatch
    self._rx(self._speed_msg_2(self.MAX_SPEED + 0.2))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self._pa(400))
    self.assertFalse(self._pa(350), "letting go slowly is not an exception: above the speed the signal is zero")
    self.assertTrue(self._pa(0))

  def test_path_angle_alone(self):
    # one signal at a time: no path angle in a frame that carries curvature
    self._ready()
    self.assertFalse(self._pa(self.MAX_DELTA, curvature=0.001))
    self.assertFalse(self._pa(self.MAX_DELTA, path_offset=0.5))
    self.assertFalse(self._pa(self.MAX_DELTA, curvature_rate=0.0005))
    self.assertTrue(self._pa(self.MAX_DELTA))

  def test_path_angle_needs_controls_allowed_and_the_request(self):
    self._ready()
    self._wind_to(300)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._pa(300))
    self.assertFalse(self._pa(10))
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._pa(300), "and it does not resume where it was")
    self.assertTrue(self._pa(self.MAX_DELTA))
    self.assertFalse(self._pa(self.MAX_DELTA, enabled=False), "not with the request bit clear")
    self.assertTrue(self._pa(0, enabled=False))

  def test_a_refused_curvature_frame_resets_the_path_angle(self):
    self._ready()
    self._wind_to(300)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0.0005)))
    self.assertEqual(self.safety.get_ford_path_angle_last(), 0)

  def test_path_angle_is_refused_without_the_opt_in(self):
    for param in (FordSafetyFlags.CANFD, FordSafetyFlags.CANFD | FordSafetyFlags.LONG_CONTROL):
      with self.subTest(param=int(param)):
        self._ready(param=param)
        self.assertFalse(self._pa(1))
        self.assertFalse(self._pa(-1))
        self.assertTrue(self._pa(0))


class TestFordCANFDLateralOnlySafety(TestFordCANFDLongitudinalSafety):
  """Lateral-only control, opted in. Every stock test runs with the opt-in set and must still hold: the bit by
  itself changes nothing. The driver's lane centering button then grants a permission of its own, which lets the
  steering message through and nothing else, and which the brake does not end."""

  SAFETY_PARAM = FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD | FordSafetyFlags.LAT_ONLY
  SPEED = 3.0

  def tearDown(self):
    self.assertFalse(self.safety.safety_redundancy_fault, "the two safety copies disagreed")
    super().tearDown()

  # ---- messages from the car

  def _cruise(self, state: int, brake: bool = False):
    """EngBrakeData with CcStat_D_Actl: 0 off, 1-2 faults, 3 standby, 4-5 engaged."""
    return self._rx(self.packer.make_can_msg_safety("EngBrakeData", 0, {"BpedDrvAppl_D_Actl": 2 if brake else 1, "CcStat_D_Actl": state}))

  def _button(self, pressed: bool, bus: int = 0, cancel: bool = False):
    values = {"TjaButtnOnOffPress": 1 if pressed else 0, "CcAslButtnCnclPress": 1 if cancel else 0}
    return self._rx(self.packer.make_can_msg_safety("Steering_Data_FD1", bus, values))

  def _cancel(self, bus: int = 0):
    self._button(False, bus, cancel=True)
    self._button(False, bus)

  def _heartbeats(self, engaged: bool, n: int = 1):
    for _ in range(n):
      self.safety.safety_heartbeat(engaged)

  def _press(self, bus: int = 0):
    self._button(True, bus)
    self._button(True, bus)      # held for a second frame: still one press
    self._button(False, bus)

  def _ready(self, param=None, cruise: int = 3, speed=None):
    """Fresh state, cruise switched on and waiting, driving straight, openpilot not engaged with ACC."""
    self.safety.set_safety_hooks(self.SAFETY_MODEL, self.SAFETY_PARAM if param is None else param)
    self.safety.init_tests()
    self._reset_curvature_measurement(0, self.SPEED if speed is None else speed)
    self._cruise(cruise)
    self._button(False)
    self.assertEqual(self.safety.get_controls_allowed(), cruise in (4, 5))
    self.assertFalse(self.safety.get_lateral_only_allowed())

  def _granted(self, **kw):
    self._ready(**kw)
    self._press()
    self.assertTrue(self.safety.get_lateral_only_allowed())

  def _steer(self, curvature: float = 0.001, enabled: bool = True):
    self._set_prev_desired_angle(curvature)
    return self._tx(self._lat_ctl_msg(enabled, 0, 0, curvature, 0))

  # ---- granting and removing

  def test_the_bit_alone_grants_nothing(self):
    self._ready()
    self.assertFalse(self._steer())
    self.assertTrue(self._steer(0, enabled=False))

  def test_without_the_opt_in_the_button_does_nothing(self):
    for param in (FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD, FordSafetyFlags.CANFD, FordSafetyFlags.LAT_ONLY,
                  FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD | FordSafetyFlags.PATH_ANGLE):
      with self.subTest(param=int(param)):
        self._ready(param=param)
        self._press()
        self.assertFalse(self.safety.get_lateral_only_allowed())
        if param & FordSafetyFlags.CANFD:
          self.assertFalse(self._steer())

  def test_a_press_grants_steering_and_nothing_else(self):
    for cruise in (3, ):
      self._granted(cruise=cruise)
      self.assertFalse(self.safety.get_controls_allowed())
      self.assertTrue(self._steer())
      self.assertTrue(self._steer(-0.001))
      # nothing longitudinal: no gas, no brake, no brake actuation, no resume press
      self.assertFalse(self._tx(self._acc_command_msg(self.MAX_GAS, self.INACTIVE_ACCEL, False)))
      self.assertFalse(self._tx(self._acc_command_msg(self.INACTIVE_GAS, self.MIN_ACCEL, True)))
      self.assertFalse(self._tx(self._acc_command_msg(self.INACTIVE_GAS, self.INACTIVE_ACCEL, True)))
      self.assertTrue(self._tx(self._acc_command_msg(self.INACTIVE_GAS, self.INACTIVE_ACCEL, False)), "the inactive frame is always allowed")
      for bus in (0, 2):
        self.assertFalse(self._tx(self._acc_button_msg(Buttons.RESUME, bus)))
        self.assertFalse(self._tx(self._acc_button_msg(Buttons.CANCEL, bus)), "cancel only while cruise is engaged")

  def test_the_steering_limits_are_the_engaged_ones(self):
    # the same frames, once fully engaged and once lateral-only, get the same answers
    def answers(lateral_only):
      out = []
      for speed in (3.0, 9.0, self.CURVATURE_ERROR_MIN_SPEED + 5):
        for curvature in (0.0, 0.0005, 0.004, 0.012, 0.02, 0.021, -0.0005, -0.02, -0.021):
          for measured in (0.0, curvature):
            if lateral_only:
              self._granted(speed=speed)
            else:
              self._ready(speed=speed)
              self.safety.set_controls_allowed(True)
            self._reset_curvature_measurement(measured, speed)
            self._set_prev_desired_angle(curvature)
            out.append((speed, curvature, measured, self._tx(self._lat_ctl_msg(True, 0, 0, curvature, 0)),
                        self._tx(self._lat_ctl_msg(True, 0.5, 0, curvature, 0)), self._tx(self._lat_ctl_msg(True, 0, 0.1, curvature, 0)),
                        self._tx(self._lat_ctl_msg(True, 0, 0, curvature, 0.0005))))
      return out
    engaged, lateral_only = answers(False), answers(True)
    self.assertEqual(engaged, lateral_only)
    self.assertTrue(any(row[3] for row in engaged) and not all(row[3] for row in engaged), "the sweep crosses the limits")
    self.assertFalse(any(any(row[4:]) for row in engaged), "path offset, path angle and curvature rate stay refused")

  def test_the_brake_and_the_accelerator_do_not_end_it(self):
    self._granted()
    for moving in (True, False):
      self._rx(self._vehicle_moving_msg(self.SPEED if moving else 0))
      self._cruise(3, brake=True)
      self.assertTrue(self.safety.get_lateral_only_allowed())
      self.assertTrue(self._steer())
      self._cruise(3, brake=False)
      self._rx(self._user_gas_msg(50))
      self.assertTrue(self.safety.get_lateral_only_allowed())
      self.assertTrue(self._steer())
      self._rx(self._user_gas_msg(0))

  def test_the_brake_still_ends_full_engagement_and_leaves_steering(self):
    self._granted()
    self._cruise(5)
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._vehicle_moving_msg(self.SPEED))
    self._cruise(5, brake=True)
    self.assertFalse(self.safety.get_controls_allowed(), "the brake ends controls_allowed as it always did")
    self.assertFalse(self._tx(self._acc_command_msg(self.MAX_GAS, self.INACTIVE_ACCEL, False)))
    self.assertTrue(self.safety.get_lateral_only_allowed())
    self.assertTrue(self._steer())

  # ---- a press only ever asks. What takes it back: openpilot not using it, the cancel button, cruise off, faults

  IDLE_MAX = 5       # heartbeats in a row with openpilot not engaged

  def test_a_second_press_does_not_remove_it(self):
    # whether a press means on or off depends on openpilot's state at that instant, which panda does not have.
    # Toggling here put the two out of step whenever openpilot refused or ended an engagement by itself.
    self._granted()
    self._heartbeats(True, 3)
    for _ in range(3):
      self._press()
      self.assertTrue(self.safety.get_lateral_only_allowed())
      self.assertTrue(self._steer())

  def test_openpilot_not_taking_it_up_ends_it(self):
    self._granted()
    for expected in [True] * (self.IDLE_MAX - 1) + [False]:
      self._heartbeats(False)
      self.assertEqual(self.safety.get_lateral_only_allowed(), expected)
    self.assertFalse(self._steer())
    self._heartbeats(True, 3)
    self.assertFalse(self.safety.get_lateral_only_allowed(), "openpilot engaging afterwards is not a request")

  def test_openpilot_letting_go_ends_it(self):
    self._granted()
    self._heartbeats(True, 50)
    self.assertTrue(self.safety.get_lateral_only_allowed(), "engaged: it stays as long as that lasts")
    self._heartbeats(False, self.IDLE_MAX - 1)
    self.assertTrue(self._steer())
    self._heartbeats(False)
    self.assertFalse(self.safety.get_lateral_only_allowed())
    self.assertFalse(self._steer())

  def test_the_count_is_of_heartbeats_in_a_row(self):
    self._granted()
    for _ in range(10):
      self._heartbeats(False, self.IDLE_MAX - 1)
      self._heartbeats(True)
    self.assertTrue(self.safety.get_lateral_only_allowed())

  def test_a_press_is_a_fresh_request_whatever_came_before(self):
    # refused or let go, then pressed again before the count ran out: the new request gets the whole count
    self._granted()
    self._heartbeats(False, self.IDLE_MAX - 1)
    self._press()
    self._heartbeats(False, self.IDLE_MAX - 1)
    self.assertTrue(self.safety.get_lateral_only_allowed())
    self._heartbeats(True)
    self.assertTrue(self._steer())
    # and after it ran out, a press grants again
    self._heartbeats(False, self.IDLE_MAX)
    self.assertFalse(self.safety.get_lateral_only_allowed())
    self._press()
    self.assertTrue(self.safety.get_lateral_only_allowed())

  def test_the_count_does_not_run_while_there_is_nothing_to_end(self):
    self._ready()
    self._heartbeats(False, 50)
    self._press()
    self._heartbeats(False, self.IDLE_MAX - 1)
    self.assertTrue(self.safety.get_lateral_only_allowed(), "heartbeats before the request are not held against it")

  def test_the_cancel_button_removes_it_whatever_openpilot_says(self):
    self._granted()
    self._heartbeats(True, 5)
    self._cancel()
    self.assertFalse(self.safety.get_lateral_only_allowed())
    self.assertFalse(self._steer())
    self.assertTrue(self._steer(0, enabled=False))
    self._heartbeats(True, 5)
    self.assertFalse(self.safety.get_lateral_only_allowed())
    self._press()
    self.assertTrue(self.safety.get_lateral_only_allowed(), "and the next press asks again")

  def test_cancel_wins_over_a_lane_centering_press_in_the_same_message(self):
    self._ready()
    self._button(True, cancel=True)
    self.assertFalse(self.safety.get_lateral_only_allowed())
    self._button(True, cancel=True)
    self._button(True)                 # cancel released with lane centering still held: no new edge, no grant
    self.assertFalse(self.safety.get_lateral_only_allowed())
    self._granted()
    self._button(True, cancel=True)
    self.assertFalse(self.safety.get_lateral_only_allowed())

  def test_only_the_cars_own_cancel_on_the_main_bus_counts(self):
    self._granted()
    self._cancel(bus=2)
    self.assertTrue(self.safety.get_lateral_only_allowed(), "the camera side")
    self._cruise(5)                    # cruise engaged, so openpilot's own cancel message is one it may send
    for bus in (0, 2):
      self.assertTrue(self._tx(self._acc_button_msg(Buttons.CANCEL, bus)))
      self.assertTrue(self.safety.get_lateral_only_allowed(), "a cancel openpilot sends is not the driver's")

  def test_a_held_button_is_one_press(self):
    self._ready()
    for _ in range(50):
      self._button(True)
      self.assertTrue(self.safety.get_lateral_only_allowed())
    self._button(False)
    self.assertTrue(self.safety.get_lateral_only_allowed())
    # and it is one request: held through the whole count, it is not asked again
    self._ready()
    self._button(True)
    for _ in range(self.IDLE_MAX):
      self._button(True)
      self._heartbeats(False)
    self.assertFalse(self.safety.get_lateral_only_allowed())

  def test_a_button_already_held_when_the_mode_is_set_is_not_a_press(self):
    self.safety.set_safety_hooks(self.SAFETY_MODEL, self.SAFETY_PARAM)
    self.safety.init_tests()
    self._cruise(3)
    self._button(True)
    self.assertTrue(self.safety.get_lateral_only_allowed(), "the first frame seen pressed is an edge: there is no earlier sample")
    # so the host must not rely on it: selfdrived needs its own edge too. What matters here: it is one grant, not a toggle storm
    self._button(True)
    self.assertTrue(self.safety.get_lateral_only_allowed())

  def test_cruise_must_be_switched_on(self):
    for state in (0, 1, 2):
      with self.subTest(state=state):
        self._ready(cruise=state)
        self._press()
        self.assertFalse(self.safety.get_lateral_only_allowed(), "no grant with cruise off or faulted")
        self._granted()
        self._cruise(state)
        self.assertFalse(self.safety.get_lateral_only_allowed(), "and it ends when cruise is switched off or faults")
        self.assertFalse(self._steer())
        self._cruise(3)
        self.assertFalse(self.safety.get_lateral_only_allowed(), "switching cruise back on is not a request")
    for state in (3, 4, 5):
      with self.subTest(state=state):
        self._ready(cruise=state)
        self._press()
        self.assertTrue(self.safety.get_lateral_only_allowed())

  def test_only_the_cars_own_button_on_the_main_bus_counts(self):
    self._ready()
    self._press(bus=2)
    self.assertFalse(self.safety.get_lateral_only_allowed(), "the camera side")
    for bus in (0, 2):
      self._tx(self._acc_button_msg(Buttons.TJA_TOGGLE, bus))
      self._tx(self._acc_button_msg(-1, bus))
      self.assertFalse(self.safety.get_lateral_only_allowed(), "a button message openpilot sends")
    self._granted()
    for bus in (0, 2):
      self._tx(self._acc_button_msg(Buttons.TJA_TOGGLE, bus))
      self._tx(self._acc_button_msg(-1, bus))
      self.assertTrue(self.safety.get_lateral_only_allowed(), "nor does anything openpilot sends take it away")

  # ---- every fault that ends controls_allowed ends it too

  def test_openpilot_not_engaged_for_three_heartbeats_ends_it(self):
    self._granted()
    self.safety.safety_heartbeat(True)
    for _ in range(10):
      self.safety.safety_heartbeat_check()
    self.assertTrue(self.safety.get_lateral_only_allowed())
    self.safety.safety_heartbeat(False)
    for expected in (True, True, False):
      self.safety.safety_heartbeat_check()
      self.assertEqual(self.safety.get_lateral_only_allowed(), expected)
    self.assertFalse(self._steer())
    # a new grant gets the full three again, not what was left of the last count
    self._press()
    for expected in (True, True, False):
      self.safety.safety_heartbeat_check()
      self.assertEqual(self.safety.get_lateral_only_allowed(), expected)
    # (the 1 Hz check is the stock one, kept; with pandad's 10 Hz heartbeat the count above ends it first)

  def test_a_checked_message_going_missing_ends_it(self):
    lag_us = 1000000
    n = self.safety.get_rx_checks_len()
    self.assertEqual(n, 7, "the six stock checks and the button message")
    for stale in range(n):
      for age, allowed in ((lag_us, True), (lag_us + 1, False)):
        with self.subTest(stale=stale, age=age):
          self._granted()
          self.safety.set_timer(2 * lag_us)
          for i in range(n):
            self.safety.set_rx_checks_received(i, 0)
          self.safety.set_rx_checks_received(stale, age)
          self.safety.safety_tick()
          self.assertEqual(self.safety.get_lateral_only_allowed(), allowed)
          self.assertEqual(self._steer(), allowed)
          if not allowed:
            self._press()
            self.assertFalse(self.safety.get_lateral_only_allowed(), "and no new grant while a check is failing")

  def test_the_stock_configuration_does_not_check_the_button_message(self):
    self.safety.set_safety_hooks(self.SAFETY_MODEL, FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD)
    self.safety.init_tests()
    self.assertEqual(self.safety.get_rx_checks_len(), 6)

  def test_a_bad_checked_message_ends_it(self):
    self._granted()
    self._rx(self._speed_msg(self.SPEED, quality_flag=False))
    self.assertFalse(self.safety.get_lateral_only_allowed())

  def test_a_speed_mismatch_ends_it(self):
    self._granted()
    self._rx(self._speed_msg(self.SPEED))
    self._rx(self._speed_msg_2(self.SPEED + 5))
    self.assertFalse(self._steer(), "the curvature check compares the two speeds and refuses")
    self.assertFalse(self.safety.get_lateral_only_allowed())

  def test_a_relay_malfunction_ends_it(self):
    self._granted()
    self._rx(self._lat_ctl_msg(True, 0, 0, 0.001, 0))            # the stock camera's steering message on our bus
    self.assertTrue(self.safety.get_relay_malfunction())
    self.assertFalse(self.safety.get_lateral_only_allowed())
    self._press()
    self.assertFalse(self.safety.get_lateral_only_allowed())

  def test_setting_the_mode_again_ends_it(self):
    self._granted()
    self.safety.set_safety_hooks(self.SAFETY_MODEL, self.SAFETY_PARAM)
    self.assertFalse(self.safety.get_lateral_only_allowed())
    self._granted()
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
    self.assertFalse(self.safety.get_lateral_only_allowed())

  def test_the_path_angle_command_under_lateral_only(self):
    # Stage A as it will run: steering only, pedals the driver's own, path angle opted in for the drive
    param = self.SAFETY_PARAM | FordSafetyFlags.PATH_ANGLE
    self._granted(param=param, speed=2.0)
    self._cruise(3, brake=True)
    for can in (100, 200, 300, 200, 0):
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, can / 2000, 0, 0)), can)
    self._cancel()
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.05, 0, 0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(False, 0, 0.05, 0, 0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(False, 0, 0, 0, 0)))


if __name__ == "__main__":
  unittest.main()
