#pragma once

#include "opendbc/safety/declarations.h"

// Safety-relevant CAN messages for Ford vehicles.
#define FORD_EngBrakeData          0x165U   // RX from PCM, for driver brake pedal and cruise state
#define FORD_EngVehicleSpThrottle  0x204U   // RX from PCM, for driver throttle input
#define FORD_DesiredTorqBrk        0x213U   // RX from ABS, for standstill state
#define FORD_BrakeSysFeatures      0x415U   // RX from ABS, for vehicle speed
#define FORD_EngVehicleSpThrottle2 0x202U   // RX from PCM, for second vehicle speed
#define FORD_Yaw_Data_FD1          0x91U    // RX from RCM, for yaw rate
#define FORD_Steering_Data_FD1     0x083U   // TX by OP, various driver switches and LKAS/CC buttons
#define FORD_ACCDATA               0x186U   // TX by OP, ACC controls
#define FORD_ACCDATA_3             0x18AU   // TX by OP, ACC/TJA user interface
#define FORD_Lane_Assist_Data1     0x3CAU   // TX by OP, Lane Keep Assist
#define FORD_LateralMotionControl  0x3D3U   // TX by OP, Lateral Control message
#define FORD_LateralMotionControl2 0x3D6U   // TX by OP, alternate Lateral Control message
#define FORD_IPMA_Data             0x3D8U   // TX by OP, IPMA and LKAS user interface

// CAN bus numbers.
#define FORD_MAIN_BUS 0U
#define FORD_CAM_BUS  2U

static uint8_t ford_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cnt
    cnt = (msg->data[2] >> 2) & 0xFU;
  }
  if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYaw_No_Cnt
    cnt = msg->data[5];
  }
  return cnt;
}

static uint32_t ford_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cs
    chksum = msg->data[3];
  }
  if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYawW_No_Cs
    chksum = msg->data[4];
  }
  return chksum;
}

static uint32_t ford_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    chksum += msg->data[0] + msg->data[1];  // Veh_V_ActlBrk
    chksum += msg->data[2] >> 6;                    // VehVActlBrk_D_Qf
    chksum += (msg->data[2] >> 2) & 0xFU;           // VehVActlBrk_No_Cnt
    chksum = 0xFFU - chksum;
  }
  if (msg->addr == FORD_Yaw_Data_FD1) {
    chksum += msg->data[0] + msg->data[1];  // VehRol_W_Actl
    chksum += msg->data[2] + msg->data[3];  // VehYaw_W_Actl
    chksum += msg->data[5];                         // VehRollYaw_No_Cnt
    chksum += msg->data[6] >> 6;                    // VehRolWActl_D_Qf
    chksum += (msg->data[6] >> 4) & 0x3U;           // VehYawWActl_D_Qf
    chksum = 0xFFU - chksum;
  }
  return chksum;
}

static bool ford_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == FORD_BrakeSysFeatures) {
    valid = (msg->data[2] >> 6) == 0x3U;           // VehVActlBrk_D_Qf
  }
  if (msg->addr == FORD_EngVehicleSpThrottle2) {
    valid = ((msg->data[4] >> 5) & 0x3U) == 0x3U;  // VehVActlEng_D_Qf
  }
  if (msg->addr == FORD_Yaw_Data_FD1) {
    valid = ((msg->data[6] >> 4) & 0x3U) == 0x3U;  // VehYawWActl_D_Qf
  }
  return valid;
}

#define FORD_INACTIVE_CURVATURE 1000U
#define FORD_INACTIVE_CURVATURE_RATE 4096U
#define FORD_INACTIVE_PATH_OFFSET 512U
#define FORD_INACTIVE_PATH_ANGLE 1000U

#define FORD_CANFD_INACTIVE_CURVATURE_RATE 1024U

static const CurvatureSteeringLimits FORD_STEERING_LIMITS = {
  .max_curvature = 1000,              // 0.02 rad/m * curvature_to_can
  .curvature_to_can = 50000,          // CAN units per rad/m
  .frequency = 20,                    // Hz
  .max_curvature_error = 100,         // 0.002 rad/m * curvature_to_can
  .curvature_error_min_speed = 10.0,  // m/s
  .max_steer_power = 0,               // disabled, Ford has no steed power signal
};

// Low-speed path angle command (opt-in, debug builds only). Wire units of LatCtlPath_An_Actl: 0.0005 rad, raw 1000 is zero,
// positive is left. These bound what may be sent. They say nothing about how far the steering then moves: that is
// measured on the car, and nothing here stands in for it.
#define FORD_PARAM_PATH_ANGLE 4U
#define FORD_PATH_ANGLE_MAX 980            // 0.49 rad either way: inside the signal's range [-1000, 1047] on both sides
#define FORD_PATH_ANGLE_MAX_DELTA 100      // per 20 Hz frame while winding up: 0.05 rad, 1.0 rad/s
#define FORD_PATH_ANGLE_MAX_SPEED 4.5      // m/s: no path angle command above this

// Lateral-only control, opt-in (safety param bit 8, debug builds only, CAN FD only).
// The driver's lane centering button on the wheel asks for it and takes it back; the brake does not end it.
//   granted  on a press, if cruise is switched on (CcStat_D_Actl standby or active) and nothing is at fault
//   removed  on the next press, when cruise is switched off or faults, and by every fault that also clears
//            controls_allowed: a bad or late checked message, a speed mismatch, a relay malfunction, and openpilot
//            not reporting itself engaged for three heartbeats
// It allows the steering message only. Nothing longitudinal and no button message asks get_lateral_allowed().
// The button is read from the car's own Steering_Data_FD1 on the main bus; what openpilot sends is never received.
#define FORD_PARAM_LAT_ONLY 8U

static bool ford_lat_only_enabled(void) {
#ifdef ALLOW_DEBUG
  const uint16_t FORD_PARAM_CANFD_ = 2;
  return GET_FLAG(safety_setup.current_safety_param, FORD_PARAM_LAT_ONLY) && GET_FLAG(safety_setup.current_safety_param, FORD_PARAM_CANFD_);
#else
  return false;
#endif
}

static bool ford_path_angle_enabled(void) {
#ifdef ALLOW_DEBUG
  return GET_FLAG(safety_setup.current_safety_param, FORD_PARAM_PATH_ANGLE);
#else
  return false;
#endif
}

// desired_path_angle: wire units about zero. curvature_requested: the same frame carries a curvature command.
static bool ford_path_angle_cmd_checks(int desired_path_angle, bool curvature_requested, bool steer_control_enabled) {
  bool violation = false;

  if (!ford_path_angle_enabled() || !get_lateral_allowed() || !steer_control_enabled) {
    // not opted in, or not steering: the signal stays at zero
    violation |= desired_path_angle != 0;
  } else if (desired_path_angle != 0) {
    const int last = safety.ford_path_angle_last;

    // range, checked as encoded: a value outside it cannot wrap to the other side
    violation |= safety_max_limit_check(desired_path_angle, FORD_PATH_ANGLE_MAX, -FORD_PATH_ANGLE_MAX);

    // one signal at a time: the response to path angle with curvature is not measured
    violation |= curvature_requested;

    // low speed only, by the fastest recent speed sample
    violation |= (safety.vehicle_speed.max / VEHICLE_SPEED_FACTOR) > FORD_PATH_ANGLE_MAX_SPEED;

    // Letting go toward zero on the same side is always allowed, as dropping the request is.
    // Winding up, or crossing zero, is rate limited from the last accepted command.
    const bool unwinding = ((last > 0) && (desired_path_angle > 0) && (desired_path_angle <= last)) ||
                           ((last < 0) && (desired_path_angle < 0) && (desired_path_angle >= last));
    if (!unwinding) {
      violation |= safety_max_limit_check(desired_path_angle, last + FORD_PATH_ANGLE_MAX_DELTA, last - FORD_PATH_ANGLE_MAX_DELTA);
    }
  } else {
    // zero is always acceptable
  }

  // a refused frame is not on the bus: the next one is checked from zero
  safety.ford_path_angle_last = violation ? 0 : desired_path_angle;
  return violation;
}

static void ford_rx_hook(const CANPacket_t *msg) {
  // Update in motion state from standstill signal
  if (msg_matches(msg, FORD_DesiredTorqBrk, FORD_MAIN_BUS)) {
    // Signal: VehStop_D_Stat
    safety.vehicle_moving = ((msg->data[3] >> 3) & 0x3U) != 1U;
  }

  // Update vehicle speed
  if (msg_matches(msg, FORD_BrakeSysFeatures, FORD_MAIN_BUS)) {
    // Signal: Veh_V_ActlBrk
    UPDATE_VEHICLE_SPEED(((msg->data[0] << 8) | msg->data[1]) * 0.01 * KPH_TO_MS);
  }

  // Check vehicle speed against a second source
  if (msg_matches(msg, FORD_EngVehicleSpThrottle2, FORD_MAIN_BUS)) {
    // Disable controls if speeds from ABS and PCM ECUs are too far apart.
    // Signal: Veh_V_ActlEng
    float filtered_pcm_speed = ((msg->data[6] << 8) | msg->data[7]) * 0.01 * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED_2(filtered_pcm_speed);
  }

  // Update vehicle yaw rate
  if (msg_matches(msg, FORD_Yaw_Data_FD1, FORD_MAIN_BUS)) {
    // FIXME: safety can receive yaw before new vehicle speed, it should recompute meas on either received
    // Signal: VehYaw_W_Actl
    // TODO: we should use the speed which results in the closest angle measurement to the desired angle
    float ford_yaw_rate = (((msg->data[2] << 8U) | msg->data[3]) * 0.0002) - 6.5;
    float current_curvature = ford_yaw_rate / SAFETY_MAX(safety.vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
    // convert current curvature into units on CAN for comparison with desired curvature
    update_sample(&safety.curvature_state.meas, ROUND(current_curvature * FORD_STEERING_LIMITS.curvature_to_can));
  }

  // Update gas pedal
  if (msg_matches(msg, FORD_EngVehicleSpThrottle, FORD_MAIN_BUS)) {
    // Pedal position: (0.1 * val) in percent
    // Signal: ApedPos_Pc_ActlArb
    safety.gas_pressed = (((msg->data[0] & 0x03U) << 8) | msg->data[1]) > 0U;
  }

  // Update brake pedal and cruise state
  if (msg_matches(msg, FORD_EngBrakeData, FORD_MAIN_BUS)) {
    // Signal: BpedDrvAppl_D_Actl
    safety.brake_pressed = ((msg->data[0] >> 4) & 0x3U) == 2U;

    // Signal: CcStat_D_Actl
    unsigned int cruise_state = msg->data[1] & 0x07U;
    bool cruise_engaged = (cruise_state == 4U) || (cruise_state == 5U);
    pcm_cruise_check(cruise_engaged);

    // lateral-only needs cruise switched on: standby (3) or engaged. Off (0) and the fault states (1, 2) end it.
    safety.ford_cruise_available = (cruise_state == 3U) || cruise_engaged;
    if (!safety.ford_cruise_available) {
      safety.lateral_only_allowed = false;
    }
  }

  // The driver's lane centering button. Only in the rx checks, so only delivered here, when lateral-only is opted in
  if (msg_matches(msg, FORD_Steering_Data_FD1, FORD_MAIN_BUS)) {
    // Signal: TjaButtnOnOffPress
    bool pressed = (msg->data[5] & 0x1U) != 0U;
    if (pressed && !safety.ford_lat_button_prev && ford_lat_only_enabled()) {
      if (safety.lateral_only_allowed) {
        safety.lateral_only_allowed = false;
      } else {
        safety.lateral_only_allowed = safety.ford_cruise_available && !safety.relay_malfunction && !safety.safety_rx_checks_invalid;
      }
    }
    safety.ford_lat_button_prev = pressed;
  }
}

static bool ford_tx_hook(const CANPacket_t *msg) {
  const LongitudinalLimits FORD_LONG_LIMITS = {
    // acceleration cmd limits (used for brakes)
    // Signal: AccBrkTot_A_Rq
    .max_accel = 5641,       //  1.9999 m/s^s
    .min_accel = 4231,       // -3.4991 m/s^2
    .inactive_accel = 5128,  // -0.0008 m/s^2

    // gas cmd limits
    // Signal: AccPrpl_A_Rq & AccPrpl_A_Pred
    .max_gas = 700,          //  2.0 m/s^2
    .min_gas = 450,          // -0.5 m/s^2
    .inactive_gas = 0,       // -5.0 m/s^2
  };

  bool tx = true;

  // Safety check for ACCDATA accel and brake requests
  if (msg->addr == FORD_ACCDATA) {
    // Signal: AccPrpl_A_Rq
    int gas = ((msg->data[6] & 0x3U) << 8) | msg->data[7];
    // Signal: AccPrpl_A_Pred
    int gas_pred = ((msg->data[2] & 0x3U) << 8) | msg->data[3];
    // Signal: AccBrkTot_A_Rq
    int accel = ((msg->data[0] & 0x1FU) << 8) | msg->data[1];
    // Signal: CmbbDeny_B_Actl
    bool cmbb_deny = (msg->data[4] >> 5) & 1U;

    // Signal: AccBrkPrchg_B_Rq & AccBrkDecel_B_Rq
    bool brake_actuation = (msg->data[6] & 0xC0U) != 0U;

    bool violation = false;
    violation |= longitudinal_accel_checks(accel, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas_pred, FORD_LONG_LIMITS);

    // Safety check for stock AEB
    violation |= cmbb_deny;  // do not prevent stock AEB actuation

    violation |= !get_longitudinal_allowed() && brake_actuation;

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Steering_Data_FD1 button signals
  // Note: Many other signals in this message are not relevant to safety (e.g. blinkers, wiper switches, high beam)
  // which we passthru in OP.
  if (msg->addr == FORD_Steering_Data_FD1) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    bool violation = false;
    violation |= ((msg->data[1] >> 0) & 1U) && !safety.cruise_engaged_prev;   // Signal: CcAslButtnCnclPress (cancel)
    violation |= ((msg->data[3] >> 1) & 1U) && !safety.controls_allowed;     // Signal: CcAsllButtnResPress (resume)

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Lane_Assist_Data1 action
  if (msg->addr == FORD_Lane_Assist_Data1) {
    // Do not allow steering using Lane_Assist_Data1 (Lane-Departure Aid).
    // This message must be sent for Lane Centering to work, and can include
    // values such as the steering angle or lane curvature for debugging,
    // but the action (LkaActvStats_D2_Req) must be set to zero.
    unsigned int action = msg->data[0] >> 5;
    if (action != 0U) {
      tx = false;
    }
  }

  // Safety check for LateralMotionControl action
  if (msg->addr == FORD_LateralMotionControl) {
    // Signal: LatCtl_D_Rq
    bool steer_control_enabled = ((msg->data[4] >> 2) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[0] << 3) | (msg->data[1] >> 5);
    unsigned int raw_curvature_rate = ((msg->data[1] & 0x1FU) << 8) | msg->data[2];
    unsigned int raw_path_angle = (msg->data[3] << 3) | (msg->data[4] >> 5);
    unsigned int raw_path_offset = (msg->data[5] << 2) | (msg->data[6] >> 6);

    // These signals are not yet tested with the current safety limits
    bool violation = (raw_curvature_rate != FORD_INACTIVE_CURVATURE_RATE) || (raw_path_angle != FORD_INACTIVE_PATH_ANGLE) || (raw_path_offset != FORD_INACTIVE_PATH_OFFSET);

    // Check angle error and steer_control_enabled
    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;  // /FORD_STEERING_LIMITS.curvature_to_can to get real curvature
    violation |= steer_curvature_cmd_checks(desired_curvature, 0, steer_control_enabled, FORD_STEERING_LIMITS);

    if (violation) {
      tx = false;
    }
  }

  // Safety check for LateralMotionControl2 action
  if (msg->addr == FORD_LateralMotionControl2) {
    // Signal: LatCtl_D2_Rq
    bool steer_control_enabled = ((msg->data[0] >> 4) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[2] << 3) | (msg->data[3] >> 5);
    unsigned int raw_curvature_rate = (msg->data[6] << 3) | (msg->data[7] >> 5);
    unsigned int raw_path_angle = ((msg->data[3] & 0x1FU) << 6) | (msg->data[4] >> 2);
    unsigned int raw_path_offset = ((msg->data[4] & 0x3U) << 8) | msg->data[5];

    // These signals are not yet tested with the current safety limits
    bool violation = (raw_curvature_rate != FORD_CANFD_INACTIVE_CURVATURE_RATE) || (raw_path_offset != FORD_INACTIVE_PATH_OFFSET);

    // Path angle: zero unless the low-speed path angle command is opted in, then bounded
    int desired_path_angle = (int)raw_path_angle - (int)FORD_INACTIVE_PATH_ANGLE;
    violation |= ford_path_angle_cmd_checks(desired_path_angle, raw_curvature != FORD_INACTIVE_CURVATURE, steer_control_enabled);

    // Check angle error and steer_control_enabled
    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;  // /FORD_STEERING_LIMITS.curvature_to_can to get real curvature
    violation |= steer_curvature_cmd_checks(desired_curvature, 0, steer_control_enabled, FORD_STEERING_LIMITS);

    if (violation) {
      // nothing of a refused frame carries over
      safety.ford_path_angle_last = 0;
    }

    if (violation) {
      tx = false;
    }
  }

  return tx;
}

static safety_config ford_init(uint16_t param) {
  // warning: quality flags are not yet checked in openpilot's CAN parser,
  // this may be the cause of blocked messages
  // FORD_EngVehicleSpThrottle2 has a counter that either randomly skips or by 2, likely ECU bug
  // Some hybrid models also experience a bug where this checksum mismatches for one or two frames under heavy acceleration with ACC
  // It has been confirmed that the Bronco Sport's camera only disallows ACC for bad quality flags, not counters or checksums, so we match that
  // The last three have no counter or checksum
  #define FORD_COMMON_RX_CHECKS \
    {.msg = {{FORD_BrakeSysFeatures, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_EngVehicleSpThrottle2, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_Yaw_Data_FD1, 0, 8, 100U, .max_counter = 255U}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_EngBrakeData, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_EngVehicleSpThrottle, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_DesiredTorqBrk, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

  static const RxCheck ford_rx_checks[] = {
    FORD_COMMON_RX_CHECKS
  };

#ifdef ALLOW_DEBUG
  // Lateral-only: the car's own button message is received and has to stay alive (10 Hz, no counter or checksum).
  static const RxCheck ford_lat_only_rx_checks[] = {
    FORD_COMMON_RX_CHECKS
    {.msg = {{FORD_Steering_Data_FD1, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
#endif

  #define FORD_COMMON_TX_MSGS \
    {FORD_Steering_Data_FD1, 0, 8, .check_relay = false}, \
    {FORD_Steering_Data_FD1, 2, 8, .check_relay = false}, \
    {FORD_ACCDATA_3, 0, 8, .check_relay = true},          \
    {FORD_Lane_Assist_Data1, 0, 8, .check_relay = true},  \
    {FORD_IPMA_Data, 0, 8, .check_relay = true},          \

#ifdef ALLOW_DEBUG
  static const CanMsg FORD_CANFD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };
#endif

  static const CanMsg FORD_CANFD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  const uint16_t FORD_PARAM_CANFD = 2;
  const bool ford_canfd = GET_FLAG(param, FORD_PARAM_CANFD);

  safety_config ret;
  if (ford_canfd) {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
#ifdef ALLOW_DEBUG
    const uint16_t FORD_PARAM_LONGITUDINAL = 1;
    const bool ford_long = GET_FLAG(param, FORD_PARAM_LONGITUDINAL);
    if (GET_FLAG(param, FORD_PARAM_LAT_ONLY)) {
      if (ford_long) {
        ret = BUILD_SAFETY_CFG(ford_lat_only_rx_checks, FORD_CANFD_LONG_TX_MSGS);
      } else {
        ret = BUILD_SAFETY_CFG(ford_lat_only_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
      }
    } else if (ford_long) {
      ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_LONG_TX_MSGS);
    } else {
      // stock longitudinal, set above
    }
#endif
  } else {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_LONG_TX_MSGS);
  }
  return ret;
}

const safety_hooks ford_hooks = {
  .init = ford_init,
  .rx = ford_rx_hook,
  .tx = ford_tx_hook,
  .get_counter = ford_get_counter,
  .get_checksum = ford_get_checksum,
  .compute_checksum = ford_compute_checksum,
  .get_quality_flag_valid = ford_get_quality_flag_valid,
};
