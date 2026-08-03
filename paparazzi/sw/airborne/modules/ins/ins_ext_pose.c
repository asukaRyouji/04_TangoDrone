/*
 * Copyright (C) 2023 MAVLab
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/ins/ins_ext_pose.c
 * Integrated Navigation System interface.
 */


#include <math.h>
#include <stdint.h>

#include "ins_ext_pose.h"
#include "state.h"
#include "math/pprz_algebra_float.h"
#include "modules/imu/imu.h"
#include "modules/ins/ins.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/core/abi.h"

#if 0
#include <stdio.h>
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) {}
#endif

#ifndef INS_EXT_POSE_Q_ACCEL
#define INS_EXT_POSE_Q_ACCEL 0.50f
#endif

#ifndef INS_EXT_POSE_Q_GYRO
#define INS_EXT_POSE_Q_GYRO 0.01f
#endif

#ifndef INS_EXT_POSE_R_POS
#define INS_EXT_POSE_R_POS 0.00001f
#endif

#ifndef INS_EXT_POSE_R_VEL
#define INS_EXT_POSE_R_VEL 0.01f
#endif

#ifndef INS_EXT_POSE_R_ATT
#define INS_EXT_POSE_R_ATT 0.1f
#endif

#ifndef INS_EXT_POSE_MAX_DT
#define INS_EXT_POSE_MAX_DT 0.01f
#endif

#ifndef INS_EXT_POSE_TIMEOUT
#define INS_EXT_POSE_TIMEOUT 0.10f
#endif

#ifndef INS_EXT_POSE_STAMP_RESET_TIMEOUT
#define INS_EXT_POSE_STAMP_RESET_TIMEOUT 1.0f
#endif

/*
 * External-pose delay handling.
 */
#ifndef INS_EXT_POSE_FIXED_DELAY
#define INS_EXT_POSE_FIXED_DELAY 0.0f
#endif

#ifndef INS_EXT_POSE_MAX_COMP_DELAY
#define INS_EXT_POSE_MAX_COMP_DELAY 0.0f
#endif

#ifndef INS_EXT_POSE_MAX_QUEUE_DELAY
#define INS_EXT_POSE_MAX_QUEUE_DELAY 0.08f
#endif

#ifndef INS_EXT_POSE_MAX_TRACKED_DELAY
#define INS_EXT_POSE_MAX_TRACKED_DELAY 0.25f
#endif

#ifndef INS_EXT_POSE_MIN_RX_INTERVAL
#define INS_EXT_POSE_MIN_RX_INTERVAL 0.012f
#endif

#ifndef INS_EXT_POSE_RECOVERY_QUEUE_DELAY
#define INS_EXT_POSE_RECOVERY_QUEUE_DELAY 0.005f
#endif

/*
 * Covariance numerical limits.
 */
#ifndef INS_EXT_POSE_P_MIN
#define INS_EXT_POSE_P_MIN 1e-9f
#endif

#ifndef INS_EXT_POSE_P_MAX
#define INS_EXT_POSE_P_MAX 1000.0f
#endif

#ifndef INS_EXT_POSE_P_RESET
#define INS_EXT_POSE_P_RESET 1.0f
#endif

/*
 * Fixed yaw rotation between the OptiTrack rigid-body local
 * x/y axes and the Bebop body p/q axes.
 *
 * Estimated from the 2026-08-03 motors-off sign test:
 * approximately 19.8 degrees = 0.3456 radians.
 *
 * This is a body-axis alignment correction. It is not the
 * field/world heading setpoint.
 */
#ifndef INS_EXT_POSE_BODY_AXIS_YAW_OFFSET
#define INS_EXT_POSE_BODY_AXIS_YAW_OFFSET 0.3456f
#endif


/** Data for telemetry and LTP origin.
 */


struct InsExtPose {
  /* Inputs */
  struct FloatRates gyros_f;
  struct FloatVect3 accels_f;
  bool has_new_gyro;
  bool has_new_acc;

  /* External pose measurement */
  struct FloatVect3 ev_pos;
  struct FloatVect3 ev_speed;
  struct FloatEulers ev_att;

  uint32_t ext_pose_stamp_ms;
  float ext_pose_rx_time_s;

  /*
  * Source and onboard timing diagnostics.
  */
  float ext_pose_source_dt_s;
  float ext_pose_rx_dt_s;

  /*
  * Time of the most recent external measurement actually
  * accepted by the EKF.
  */
  float ext_pose_fused_time_s;
  bool has_fused_ext_pose;

  /*
  * Estimated additional queue delay relative to the normal stream.
  * This does not include unknown fixed capture/network delay.
  */
  float ext_pose_queue_delay_s;

  /*
  * Total delay considered by the EKF:
  * fixed delay + estimated additional queue delay.
  */
  float ext_pose_measurement_delay_s;

  /*
  * Delay actually used to forward-project position and attitude
  * after applying the MAX_COMP_DELAY cap.
  */
  float ext_pose_compensation_delay_s;

  bool has_ext_pose_stamp;
  bool has_new_ext_pose;

  /* Origin */
  struct LtpDef_i  ltp_def;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;
};

struct InsExtPose ins_ext_pos;

static bool ins_ext_pose_ready = false;
static bool ins_ext_pose_fresh = false;
static uint32_t ins_ext_pose_dt_clamp_count = 0;

/*
 * Latency and covariance diagnostics.
 */
static uint32_t ins_ext_pose_latency_reject_count = 0;

static uint32_t ekf_covariance_repair_count = 0;
static uint32_t ekf_covariance_reset_count = 0;
static bool ekf_covariance_healthy = true;

/*
 * EKF prediction and measurement-update diagnostics.
 *
 * These remain private to ins_ext_pose.c. They are appended to
 * the CSV by ins_ext_pos_log_data(), so no extern declarations
 * are required in ins_ext_pose.h.
 */
static float ekf_last_raw_dt_s = 0.0f;
static float ekf_last_used_dt_s = 0.0f;

static uint32_t ekf_fusion_count = 0;
static uint32_t ekf_measurement_failure_count = 0;

static uint8_t ekf_last_measurement_success = 0;

static float ekf_last_innovation[EKF_NUM_OUTPUTS] = {0};
static float ekf_last_correction[EKF_NUM_OUTPUTS] = {0};


static void ins_ext_pose_init_from_flightplan(void)
{

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_ext_pos.ltp_def, &ecef_nav0);
  ins_ext_pos.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_ext_pos.ltp_def);
}


/** Provide telemetry.
 */

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_ext_pos.ltp_pos.x, &ins_ext_pos.ltp_pos.y, &ins_ext_pos.ltp_pos.z,
                    &ins_ext_pos.ltp_speed.x, &ins_ext_pos.ltp_speed.y, &ins_ext_pos.ltp_speed.z,
                    &ins_ext_pos.ltp_accel.x, &ins_ext_pos.ltp_accel.y, &ins_ext_pos.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_baro_z = 0.0;
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      (float *)&fake_baro_z, &ins_ext_pos.ltp_pos.z,
                      &ins_ext_pos.ltp_speed.z, &ins_ext_pos.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_qfe = 0.0;
  pprz_msg_send_INS_REF(trans, dev, AC_ID,
                        &ins_ext_pos.ltp_def.ecef.x, &ins_ext_pos.ltp_def.ecef.y, &ins_ext_pos.ltp_def.ecef.z,
                        &ins_ext_pos.ltp_def.lla.lat, &ins_ext_pos.ltp_def.lla.lon, &ins_ext_pos.ltp_def.lla.alt,
                        &ins_ext_pos.ltp_def.hmsl, (float *)&fake_qfe);
}
#endif


/**
 * Import Gyro and Acc from ABI.
 */

#ifndef INS_EXT_POSE_IMU_ID
#define INS_EXT_POSE_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EXT_POSE_IMU_ID)

static abi_event accel_ev;
static abi_event gyro_ev;

static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);



static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro)
{
  RATES_FLOAT_OF_BFP(ins_ext_pos.gyros_f, *gyro);
  ins_ext_pos.has_new_gyro = true;
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  ACCELS_FLOAT_OF_BFP(ins_ext_pos.accels_f, *accel);
  ins_ext_pos.has_new_acc = true;
}

/*
 * Latest raw EXTERNAL_POSE data and onboard reception timing.
 */
float ins_ext_pose_raw_enu_x = 0.0f;
float ins_ext_pose_raw_enu_y = 0.0f;
float ins_ext_pose_raw_enu_z = 0.0f;

float ins_ext_pose_raw_qi = 1.0f;
float ins_ext_pose_raw_qx = 0.0f;
float ins_ext_pose_raw_qy = 0.0f;
float ins_ext_pose_raw_qz = 0.0f;

/*
 * External-attitude transformation diagnostics.
 *
 * These are private to ins_ext_pose.c and are written through
 * ins_ext_pos_log_data(), so no declarations are needed in
 * ins_ext_pose.h.
 */

/* Norm of the quaternion received from EXTERNAL_POSE. */
static float ext_diag_raw_q_norm = 1.0f;

/* Normalized quaternion exactly as received from NatNet/IVY. */
static struct FloatQuat ext_diag_raw_q_unit = {
  .qi = 1.0f,
  .qx = 0.0f,
  .qy = 0.0f,
  .qz = 0.0f
};

/*
 * Euler conversion of the normalized raw quaternion.
 * This is diagnostic only and is not assumed to use the
 * Paparazzi body/world convention.
 */
static struct FloatEulers ext_diag_raw_eulers = {
  .phi = 0.0f,
  .theta = 0.0f,
  .psi = 0.0f
};

/*
 * Normalized quaternion after the CURRENT component mapping:
 *
 * qi = raw qi
 * qx = raw qy
 * qy = -raw qx
 * qz = -raw qz
 */
static struct FloatQuat ext_diag_mapped_q_unit = {
  .qi = 1.0f,
  .qx = 0.0f,
  .qy = 0.0f,
  .qz = 0.0f
};

/*
 * Euler angles produced from the current mapped quaternion
 * BEFORE the separate theta sign reversal.
 */
static struct FloatEulers ext_diag_mapped_eulers_preflip = {
  .phi = 0.0f,
  .theta = 0.0f,
  .psi = 0.0f
};

uint32_t ins_ext_pose_rx_count = 0;
uint32_t ins_ext_pose_last_rx_usec = 0;
uint32_t ins_ext_pose_dt_usec = 0;

/**
 * Import External Pose Message
 */

void ins_ext_pose_msg_update(uint8_t *buf)
{
  if (DL_EXTERNAL_POSE_ac_id(buf) != AC_ID) {
    return;
  }

  const float now = get_sys_time_float();
  const uint32_t stamp_ms = DL_EXTERNAL_POSE_timestamp(buf);

  /*
   * Reject duplicate and out-of-order measurements.
   *
   * When the NatNet bridge has been silent for more than one second,
   * accept a timestamp reset so that restarting natnet2ivy does not
   * permanently block new measurements.
   */
  int32_t stamp_delta = 0;
  bool timestamp_reset = false;

  if (ins_ext_pos.has_ext_pose_stamp) {
    stamp_delta =
      (int32_t)(stamp_ms -
                ins_ext_pos.ext_pose_stamp_ms);

    const float time_since_last_rx =
      now - ins_ext_pos.ext_pose_rx_time_s;

    /*
    * Any sufficiently long interruption starts a new
    * queue-delay baseline, regardless of whether the source
    * timestamp itself restarted.
    */
    if (time_since_last_rx >
        INS_EXT_POSE_STAMP_RESET_TIMEOUT) {

      timestamp_reset = true;

    } else if (stamp_delta <= 0) {

      /*
      * Duplicate or out-of-order packet in the current,
      * continuously active timestamp sequence.
      */
      return;
    }
  }

  const float enu_x = DL_EXTERNAL_POSE_enu_x(buf);
  const float enu_y = DL_EXTERNAL_POSE_enu_y(buf);
  const float enu_z = DL_EXTERNAL_POSE_enu_z(buf);

  const float enu_xd = DL_EXTERNAL_POSE_enu_xd(buf);
  const float enu_yd = DL_EXTERNAL_POSE_enu_yd(buf);
  const float enu_zd = DL_EXTERNAL_POSE_enu_zd(buf);

  const float quat_i = DL_EXTERNAL_POSE_body_qi(buf);
  const float quat_x = DL_EXTERNAL_POSE_body_qx(buf);
  const float quat_y = DL_EXTERNAL_POSE_body_qy(buf);
  const float quat_z = DL_EXTERNAL_POSE_body_qz(buf);

  if (!isfinite(enu_x)  || !isfinite(enu_y)  || !isfinite(enu_z) ||
      !isfinite(enu_xd) || !isfinite(enu_yd) || !isfinite(enu_zd) ||
      !isfinite(quat_i) || !isfinite(quat_x) || !isfinite(quat_y) ||
      !isfinite(quat_z)) {
    return;
  }

  /*
  * The raw quaternion already carries roll primarily in quat_x
  * and pitch primarily in quat_y. Do not swap x and y.
  *
  * First convert the raw body-axis convention:
  *
  *   basic_q = [qi, qx, -qy, -qz]
  *
  * Then rotate the quaternion-vector x/y components by the
  * fixed rigid-body yaw-axis alignment angle.
  *
  * This is equivalent to:
  *
  *   q_corrected =
  *     qz(offset) * basic_q * qz(-offset)
  *
  * The similarity transformation rotates the roll/pitch axes
  * without adding a constant yaw offset.
  */
  const float body_axis_c =
    cosf(INS_EXT_POSE_BODY_AXIS_YAW_OFFSET);

  const float body_axis_s =
    sinf(INS_EXT_POSE_BODY_AXIS_YAW_OFFSET);

  struct FloatQuat orient = {
    .qi = quat_i,

    .qx =
      body_axis_c * quat_x +
      body_axis_s * quat_y,

    .qy =
      body_axis_s * quat_x -
      body_axis_c * quat_y,

    .qz = -quat_z
  };
  /*
   * Normalize the quaternion and reject invalid measurements.
   */
  const float q_norm_sq =
      orient.qi * orient.qi +
      orient.qx * orient.qx +
      orient.qy * orient.qy +
      orient.qz * orient.qz;

  if (!isfinite(q_norm_sq) || q_norm_sq < 0.25f) {
    return;
  }

  const float inv_q_norm = 1.0f / sqrtf(q_norm_sq);

  /*
  * Save the norm before normalization.
  *
  * A healthy rigid-body quaternion should remain very close
  * to unit length. Large deviations would indicate malformed
  * bridge data rather than a frame-conversion problem.
  */
  ext_diag_raw_q_norm = sqrtf(q_norm_sq);

  /*
  * Normalize the raw quaternion exactly as received.
  */
  ext_diag_raw_q_unit.qi = quat_i * inv_q_norm;
  ext_diag_raw_q_unit.qx = quat_x * inv_q_norm;
  ext_diag_raw_q_unit.qy = quat_y * inv_q_norm;
  ext_diag_raw_q_unit.qz = quat_z * inv_q_norm;

  /*
  * Convert the raw quaternion directly to Euler angles for
  * diagnostics only.
  *
  * These angles may use the wrong frame or rotation direction.
  * They are not fused.
  */
  float_eulers_of_quat(
    &ext_diag_raw_eulers,
    &ext_diag_raw_q_unit
  );

  /*
  * Store raw valid EXTERNAL_POSE values for diagnostics.
  */
  ins_ext_pose_raw_enu_x = enu_x;
  ins_ext_pose_raw_enu_y = enu_y;
  ins_ext_pose_raw_enu_z = enu_z;

  ins_ext_pose_raw_qi = quat_i;
  ins_ext_pose_raw_qx = quat_x;
  ins_ext_pose_raw_qy = quat_y;
  ins_ext_pose_raw_qz = quat_z;

  /*
  * Measure onboard message-reception timing.
  */
  const uint32_t rx_usec = get_sys_time_usec();

  if (ins_ext_pose_last_rx_usec != 0) {
    ins_ext_pose_dt_usec =
      rx_usec - ins_ext_pose_last_rx_usec;
  } else {
    ins_ext_pose_dt_usec = 0;
  }

  ins_ext_pose_last_rx_usec = rx_usec;
  ins_ext_pose_rx_count++;

  orient.qi *= inv_q_norm;
  orient.qx *= inv_q_norm;
  orient.qy *= inv_q_norm;
  orient.qz *= inv_q_norm;

  /*
  * Save the normalized corrected quaternion.
  *
  * Although the diagnostic variable still contains "mapped"
  * in its name, it now stores the complete corrected candidate:
  *
  *   qi = raw qi
  *
  *   qx =
  *     cos(offset) * raw qx +
  *     sin(offset) * raw qy
  *
  *   qy =
  *     sin(offset) * raw qx -
  *     cos(offset) * raw qy
  *
  *   qz = -raw qz
  */
  ext_diag_mapped_q_unit = orient;

  /*
  * Convert the corrected quaternion to Euler angles exactly once.
  *
  * There is no separate pitch-sign reversal anymore.
  */
  float_eulers_of_quat(
    &ext_diag_mapped_eulers_preflip,
    &orient
  );

  /*
  * Use the exact same corrected Euler angles for:
  *
  * 1. CSV diagnostics;
  * 2. external-pose storage;
  * 3. EKF initialization and measurement fusion.
  */
  ins_ext_pos.ev_att =
    ext_diag_mapped_eulers_preflip;

  /*
   * Corrected ENU -> NED transformation:
   *
   * NED x = ENU y
   * NED y = ENU x
   * NED z = -ENU z
   */
  ins_ext_pos.ev_pos.x = enu_y;
  ins_ext_pos.ev_pos.y = enu_x;
  ins_ext_pos.ev_pos.z = -enu_z;

  ins_ext_pos.ev_speed.x = enu_yd;
  ins_ext_pos.ev_speed.y = enu_xd;
  ins_ext_pos.ev_speed.z = -enu_zd;

  /*
  * Estimate additional variable queue delay.
  *
  * This assumes EXTERNAL_POSE timestamp is expressed in milliseconds,
  * as the current code's stamp_ms notation expects.
  */
  float source_dt_s = 0.0f;
  float rx_dt_s = 0.0f;
  float queue_delay_s =
    ins_ext_pos.ext_pose_queue_delay_s;

  if (timestamp_reset) {

    /*
    * A restarted source timestamp creates a new timing
    * reference. Do not carry an old backlog estimate into
    * the new sequence.
    */
    source_dt_s = 0.0f;
    rx_dt_s = 0.0f;
    queue_delay_s = 0.0f;

  } else if (ins_ext_pos.has_ext_pose_stamp &&
            stamp_delta > 0) {

    source_dt_s = 0.001f * (float)stamp_delta;
    rx_dt_s = now - ins_ext_pos.ext_pose_rx_time_s;

    if (isfinite(source_dt_s) &&
        isfinite(rx_dt_s) &&
        source_dt_s > 0.0f &&
        rx_dt_s >= 0.0f) {

      /*
      * When reception takes longer than the source interval,
      * additional queue delay has accumulated.
      *
      * During a burst, rx_dt becomes smaller than source_dt,
      * allowing the estimate to decrease again.
      */
      queue_delay_s += rx_dt_s - source_dt_s;

      if (queue_delay_s < 0.0f) {
        queue_delay_s = 0.0f;
      }

      if (queue_delay_s >
          INS_EXT_POSE_MAX_TRACKED_DELAY) {
        queue_delay_s =
          INS_EXT_POSE_MAX_TRACKED_DELAY;
      }
    }
  }

  ins_ext_pos.ext_pose_source_dt_s = source_dt_s;
  ins_ext_pos.ext_pose_rx_dt_s = rx_dt_s;
  ins_ext_pos.ext_pose_queue_delay_s = queue_delay_s;

  /*
   * Set the flag last, after the complete measurement has been stored.
   */
  ins_ext_pos.ext_pose_stamp_ms = stamp_ms;
  ins_ext_pos.ext_pose_rx_time_s = now;
  ins_ext_pos.has_ext_pose_stamp = true;
  ins_ext_pos.has_new_ext_pose = true;
}

void ins_reset_local_origin(void)
{
  // Ext pos does not allow geoinit: FP origin only
}

void ins_reset_altitude_ref(void)
{
  // Ext pos does not allow geoinit: FP origin only
}


/** EKF protos
 */

static inline void ekf_init(void);
static inline void ekf_run(void);

/** Module
 */


void ins_ext_pose_init(void)
{
  ins_ext_pose_rx_count = 0;
  ins_ext_pose_last_rx_usec = 0;
  ins_ext_pose_dt_usec = 0;

  ext_diag_raw_q_norm = 1.0f;

  ext_diag_raw_q_unit.qi = 1.0f;
  ext_diag_raw_q_unit.qx = 0.0f;
  ext_diag_raw_q_unit.qy = 0.0f;
  ext_diag_raw_q_unit.qz = 0.0f;

  ext_diag_raw_eulers.phi = 0.0f;
  ext_diag_raw_eulers.theta = 0.0f;
  ext_diag_raw_eulers.psi = 0.0f;

  ext_diag_mapped_q_unit.qi = 1.0f;
  ext_diag_mapped_q_unit.qx = 0.0f;
  ext_diag_mapped_q_unit.qy = 0.0f;
  ext_diag_mapped_q_unit.qz = 0.0f;

  ext_diag_mapped_eulers_preflip.phi = 0.0f;
  ext_diag_mapped_eulers_preflip.theta = 0.0f;
  ext_diag_mapped_eulers_preflip.psi = 0.0f;

  // Initialize inputs
  ins_ext_pos.has_new_acc = false;
  ins_ext_pos.has_new_gyro = false;
  ins_ext_pos.has_new_ext_pose = false;

  ins_ext_pos.ev_speed.x = 0.0f;
  ins_ext_pos.ev_speed.y = 0.0f;
  ins_ext_pos.ev_speed.z = 0.0f;

  ins_ext_pos.ext_pose_stamp_ms = 0;
  ins_ext_pos.ext_pose_rx_time_s = 0.0f;

  ins_ext_pos.has_ext_pose_stamp = false;

  ins_ext_pose_ready = false;
  ins_ext_pose_fresh = false;
  ins_ext_pose_dt_clamp_count = 0;

  ins_ext_pos.ext_pose_fused_time_s = 0.0f;
  ins_ext_pos.has_fused_ext_pose = false;

  ins_ext_pose_latency_reject_count = 0;

  ekf_covariance_repair_count = 0;
  ekf_covariance_reset_count = 0;
  ekf_covariance_healthy = true;

  ekf_last_raw_dt_s = 0.0f;
  ekf_last_used_dt_s = 0.0f;

  ekf_fusion_count = 0;
  ekf_measurement_failure_count = 0;
  ekf_last_measurement_success = 0;

  for (int i = 0; i < EKF_NUM_OUTPUTS; i++) {
    ekf_last_innovation[i] = 0.0f;
    ekf_last_correction[i] = 0.0f;
  }

  ins_ext_pos.ext_pose_source_dt_s = 0.0f;
  ins_ext_pos.ext_pose_rx_dt_s = 0.0f;
  ins_ext_pos.ext_pose_queue_delay_s = 0.0f;

  ins_ext_pos.ext_pose_measurement_delay_s = 0.0f;
  ins_ext_pos.ext_pose_compensation_delay_s = 0.0f;

  // Get External Pose Origin From Flightplan
  ins_ext_pose_init_from_flightplan();

  // Provide telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
#endif

  // Get IMU through ABI
  AbiBindMsgIMU_ACCEL(INS_EXT_POSE_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_GYRO(INS_EXT_POSE_IMU_ID, &gyro_ev, gyro_cb);

  // Get External Pose through datalink message: setup in xml

  // Initialize EKF
  ekf_init();
}

void ins_ext_pose_run(void)
{
  ekf_run();
}




/***************************************************
 * Kalman Filter.
 */



static inline void ekf_f(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES]);
static inline void ekf_F(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS],
                         float out[EKF_NUM_STATES][EKF_NUM_STATES]);
static inline void ekf_L(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS],
                         float out[EKF_NUM_STATES][EKF_NUM_INPUTS]);

static inline void ekf_prediction_step(const float U[EKF_NUM_INPUTS], const float dt);
static inline bool ekf_measurement_step(const float Z[EKF_NUM_OUTPUTS]);

static inline void ekf_reset_covariance(void);
static inline bool ekf_condition_covariance(void);



float ekf_X[EKF_NUM_STATES];
float ekf_U[EKF_NUM_INPUTS];
float ekf_Z[EKF_NUM_OUTPUTS];
float ekf_P[EKF_NUM_STATES][EKF_NUM_STATES];
float ekf_Q[EKF_NUM_INPUTS][EKF_NUM_INPUTS];
float ekf_R[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];

float ekf_H[EKF_NUM_OUTPUTS][EKF_NUM_STATES] = {{0}};


float t0;
float t1;

void ekf_set_diag(float **a, float *b, int n);
void ekf_set_diag(float **a, float *b, int n)
{
  int i, j;
  for (i = 0 ; i < n; i++) {
    for (j = 0 ; j < n; j++) {
      if (i == j) {
        a[i][j] = b[i];
      } else {
        a[i][j] = 0.0;
      }
    }
  }
}



static inline void ekf_init(void)
{

  DEBUG_PRINT("ekf init");
  float X0[EKF_NUM_STATES] = {0};
  float U0[EKF_NUM_INPUTS] = {0};
  float Z0[EKF_NUM_OUTPUTS] = {0};

  /*
  * Z[0:8] directly measures X[0:8]:
  * position, velocity and attitude.
  */
  for (int i = 0; i < EKF_NUM_OUTPUTS; i++) {
    ekf_H[i][i] = 1.0f;
  }

  float Pdiag[EKF_NUM_STATES] = {1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.};
  float Qdiag[EKF_NUM_INPUTS] = {
    INS_EXT_POSE_Q_ACCEL,
    INS_EXT_POSE_Q_ACCEL,
    INS_EXT_POSE_Q_ACCEL,

    INS_EXT_POSE_Q_GYRO,
    INS_EXT_POSE_Q_GYRO,
    INS_EXT_POSE_Q_GYRO};

  float Rdiag[EKF_NUM_OUTPUTS] = {
    INS_EXT_POSE_R_POS,
    INS_EXT_POSE_R_POS,
    INS_EXT_POSE_R_POS,

    INS_EXT_POSE_R_VEL,
    INS_EXT_POSE_R_VEL,
    INS_EXT_POSE_R_VEL,

    INS_EXT_POSE_R_ATT,
    INS_EXT_POSE_R_ATT,
    INS_EXT_POSE_R_ATT};

  MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ekf_Q_, ekf_Q, EKF_NUM_INPUTS);
  MAKE_MATRIX_PTR(ekf_R_, ekf_R, EKF_NUM_OUTPUTS);

  ekf_set_diag(ekf_P_, Pdiag, EKF_NUM_STATES);
  ekf_set_diag(ekf_Q_, Qdiag, EKF_NUM_INPUTS);
  ekf_set_diag(ekf_R_, Rdiag, EKF_NUM_OUTPUTS);
  float_vect_copy(ekf_X, X0, EKF_NUM_STATES);
  float_vect_copy(ekf_U, U0, EKF_NUM_INPUTS);
  float_vect_copy(ekf_Z, Z0, EKF_NUM_OUTPUTS);
}

static inline void ekf_f(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES])
{
  float x0 = cos(X[8]);
  float x1 = U[0] - X[9];
  float x2 = cos(X[7]);
  float x3 = x1 * x2;
  float x4 = U[2] - X[11];
  float x5 = sin(X[6]);
  float x6 = sin(X[8]);
  float x7 = x5 * x6;
  float x8 = sin(X[7]);
  float x9 = cos(X[6]);
  float x10 = x0 * x9;
  float x11 = U[1] - X[10];
  float x12 = x6 * x9;
  float x13 = x0 * x5;
  float x14 = tan(X[7]);
  float x15 = U[4] - X[13];
  float x16 = x15 * x5;
  float x17 = U[5] - X[14];
  float x18 = x17 * x9;
  float x19 = 1.0 / x2;
  out[0] = X[3];
  out[1] = X[4];
  out[2] = X[5];
  out[3] = x0 * x3 + x11 * (-x12 + x13 * x8) + x4 * (x10 * x8 + x7);
  out[4] = x11 * (x10 + x7 * x8) + x3 * x6 + x4 * (x12 * x8 - x13);
  out[5] = -x1 * x8 + x11 * x2 * x5 + x2 * x4 * x9 + 9.8100000000000005;
  out[6] = U[3] - X[12] + x14 * x16 + x14 * x18;
  out[7] = x15 * x9 - x17 * x5;
  out[8] = x16 * x19 + x18 * x19;
  out[9] = 0;
  out[10] = 0;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
}

static inline void ekf_F(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS],
                         float out[EKF_NUM_STATES][EKF_NUM_STATES])
{
  float x0 = U[1] - X[10];
  float x1 = sin(X[6]);
  float x2 = sin(X[8]);
  float x3 = x1 * x2;
  float x4 = sin(X[7]);
  float x5 = cos(X[6]);
  float x6 = cos(X[8]);
  float x7 = x5 * x6;
  float x8 = x4 * x7;
  float x9 = x3 + x8;
  float x10 = U[2] - X[11];
  float x11 = x2 * x5;
  float x12 = x1 * x6;
  float x13 = x12 * x4;
  float x14 = x11 - x13;
  float x15 = U[0] - X[9];
  float x16 = x15 * x4;
  float x17 = cos(X[7]);
  float x18 = x0 * x17;
  float x19 = x10 * x17;
  float x20 = x17 * x2;
  float x21 = x11 * x4;
  float x22 = x12 - x21;
  float x23 = -x3 * x4 - x7;
  float x24 = x17 * x6;
  float x25 = x17 * x5;
  float x26 = x1 * x17;
  float x27 = x4 * x5;
  float x28 = U[4] - X[13];
  float x29 = tan(X[7]);
  float x30 = x29 * x5;
  float x31 = U[5] - X[14];
  float x32 = x1 * x29;
  float x33 = pow(x29, 2) + 1;
  float x34 = x1 * x28;
  float x35 = 1.0 / x17;
  float x36 = x35 * x5;
  float x37 = x1 * x35;
  float x38 = pow(x17, -2);
  out[0][0] = 0;
  out[0][1] = 0;
  out[0][2] = 0;
  out[0][3] = 1;
  out[0][4] = 0;
  out[0][5] = 0;
  out[0][6] = 0;
  out[0][7] = 0;
  out[0][8] = 0;
  out[0][9] = 0;
  out[0][10] = 0;
  out[0][11] = 0;
  out[0][12] = 0;
  out[0][13] = 0;
  out[0][14] = 0;
  out[1][0] = 0;
  out[1][1] = 0;
  out[1][2] = 0;
  out[1][3] = 0;
  out[1][4] = 1;
  out[1][5] = 0;
  out[1][6] = 0;
  out[1][7] = 0;
  out[1][8] = 0;
  out[1][9] = 0;
  out[1][10] = 0;
  out[1][11] = 0;
  out[1][12] = 0;
  out[1][13] = 0;
  out[1][14] = 0;
  out[2][0] = 0;
  out[2][1] = 0;
  out[2][2] = 0;
  out[2][3] = 0;
  out[2][4] = 0;
  out[2][5] = 1;
  out[2][6] = 0;
  out[2][7] = 0;
  out[2][8] = 0;
  out[2][9] = 0;
  out[2][10] = 0;
  out[2][11] = 0;
  out[2][12] = 0;
  out[2][13] = 0;
  out[2][14] = 0;
  out[3][0] = 0;
  out[3][1] = 0;
  out[3][2] = 0;
  out[3][3] = 0;
  out[3][4] = 0;
  out[3][5] = 0;
  out[3][6] = x0 * x9 + x10 * x14;
  out[3][7] = x12 * x18 - x16 * x6 + x19 * x7;
  out[3][8] = x0 * x23 + x10 * x22 - x15 * x20;
  out[3][9] = -x24;
  out[3][10] = x14;
  out[3][11] = -x3 - x8;
  out[3][12] = 0;
  out[3][13] = 0;
  out[3][14] = 0;
  out[4][0] = 0;
  out[4][1] = 0;
  out[4][2] = 0;
  out[4][3] = 0;
  out[4][4] = 0;
  out[4][5] = 0;
  out[4][6] = x0 * (-x12 + x21) + x10 * x23;
  out[4][7] = x11 * x19 - x16 * x2 + x18 * x3;
  out[4][8] = x0 * (-x11 + x13) + x10 * x9 + x15 * x24;
  out[4][9] = -x20;
  out[4][10] = x23;
  out[4][11] = x22;
  out[4][12] = 0;
  out[4][13] = 0;
  out[4][14] = 0;
  out[5][0] = 0;
  out[5][1] = 0;
  out[5][2] = 0;
  out[5][3] = 0;
  out[5][4] = 0;
  out[5][5] = 0;
  out[5][6] = x0 * x25 - x10 * x26;
  out[5][7] = -x0 * x1 * x4 - x10 * x27 + x17 * (-U[0] + X[9]);
  out[5][8] = 0;
  out[5][9] = x4;
  out[5][10] = -x26;
  out[5][11] = -x25;
  out[5][12] = 0;
  out[5][13] = 0;
  out[5][14] = 0;
  out[6][0] = 0;
  out[6][1] = 0;
  out[6][2] = 0;
  out[6][3] = 0;
  out[6][4] = 0;
  out[6][5] = 0;
  out[6][6] = x28 * x30 - x31 * x32;
  out[6][7] = x31 * x33 * x5 + x33 * x34;
  out[6][8] = 0;
  out[6][9] = 0;
  out[6][10] = 0;
  out[6][11] = 0;
  out[6][12] = -1;
  out[6][13] = -x32;
  out[6][14] = -x30;
  out[7][0] = 0;
  out[7][1] = 0;
  out[7][2] = 0;
  out[7][3] = 0;
  out[7][4] = 0;
  out[7][5] = 0;
  out[7][6] = -x34 + x5 * (-U[5] + X[14]);
  out[7][7] = 0;
  out[7][8] = 0;
  out[7][9] = 0;
  out[7][10] = 0;
  out[7][11] = 0;
  out[7][12] = 0;
  out[7][13] = -x5;
  out[7][14] = x1;
  out[8][0] = 0;
  out[8][1] = 0;
  out[8][2] = 0;
  out[8][3] = 0;
  out[8][4] = 0;
  out[8][5] = 0;
  out[8][6] = x28 * x36 - x31 * x37;
  out[8][7] = x27 * x31 * x38 + x34 * x38 * x4;
  out[8][8] = 0;
  out[8][9] = 0;
  out[8][10] = 0;
  out[8][11] = 0;
  out[8][12] = 0;
  out[8][13] = -x37;
  out[8][14] = -x36;
  out[9][0] = 0;
  out[9][1] = 0;
  out[9][2] = 0;
  out[9][3] = 0;
  out[9][4] = 0;
  out[9][5] = 0;
  out[9][6] = 0;
  out[9][7] = 0;
  out[9][8] = 0;
  out[9][9] = 0;
  out[9][10] = 0;
  out[9][11] = 0;
  out[9][12] = 0;
  out[9][13] = 0;
  out[9][14] = 0;
  out[10][0] = 0;
  out[10][1] = 0;
  out[10][2] = 0;
  out[10][3] = 0;
  out[10][4] = 0;
  out[10][5] = 0;
  out[10][6] = 0;
  out[10][7] = 0;
  out[10][8] = 0;
  out[10][9] = 0;
  out[10][10] = 0;
  out[10][11] = 0;
  out[10][12] = 0;
  out[10][13] = 0;
  out[10][14] = 0;
  out[11][0] = 0;
  out[11][1] = 0;
  out[11][2] = 0;
  out[11][3] = 0;
  out[11][4] = 0;
  out[11][5] = 0;
  out[11][6] = 0;
  out[11][7] = 0;
  out[11][8] = 0;
  out[11][9] = 0;
  out[11][10] = 0;
  out[11][11] = 0;
  out[11][12] = 0;
  out[11][13] = 0;
  out[11][14] = 0;
  out[12][0] = 0;
  out[12][1] = 0;
  out[12][2] = 0;
  out[12][3] = 0;
  out[12][4] = 0;
  out[12][5] = 0;
  out[12][6] = 0;
  out[12][7] = 0;
  out[12][8] = 0;
  out[12][9] = 0;
  out[12][10] = 0;
  out[12][11] = 0;
  out[12][12] = 0;
  out[12][13] = 0;
  out[12][14] = 0;
  out[13][0] = 0;
  out[13][1] = 0;
  out[13][2] = 0;
  out[13][3] = 0;
  out[13][4] = 0;
  out[13][5] = 0;
  out[13][6] = 0;
  out[13][7] = 0;
  out[13][8] = 0;
  out[13][9] = 0;
  out[13][10] = 0;
  out[13][11] = 0;
  out[13][12] = 0;
  out[13][13] = 0;
  out[13][14] = 0;
  out[14][0] = 0;
  out[14][1] = 0;
  out[14][2] = 0;
  out[14][3] = 0;
  out[14][4] = 0;
  out[14][5] = 0;
  out[14][6] = 0;
  out[14][7] = 0;
  out[14][8] = 0;
  out[14][9] = 0;
  out[14][10] = 0;
  out[14][11] = 0;
  out[14][12] = 0;
  out[14][13] = 0;
  out[14][14] = 0;
}

static inline void ekf_L(const float X[EKF_NUM_STATES], __attribute__((unused))  const float U[EKF_NUM_INPUTS],
                         float out[EKF_NUM_STATES][EKF_NUM_INPUTS])
{
  float x0 = cos(X[7]);
  float x1 = cos(X[8]);
  float x2 = sin(X[8]);
  float x3 = cos(X[6]);
  float x4 = x2 * x3;
  float x5 = sin(X[7]);
  float x6 = sin(X[6]);
  float x7 = x1 * x6;
  float x8 = x2 * x6;
  float x9 = x1 * x3;
  float x10 = tan(X[7]);
  float x11 = 1.0 / x0;
  out[0][0] = 0;
  out[0][1] = 0;
  out[0][2] = 0;
  out[0][3] = 0;
  out[0][4] = 0;
  out[0][5] = 0;
  out[1][0] = 0;
  out[1][1] = 0;
  out[1][2] = 0;
  out[1][3] = 0;
  out[1][4] = 0;
  out[1][5] = 0;
  out[2][0] = 0;
  out[2][1] = 0;
  out[2][2] = 0;
  out[2][3] = 0;
  out[2][4] = 0;
  out[2][5] = 0;
  out[3][0] = -x0 * x1;
  out[3][1] = x4 - x5 * x7;
  out[3][2] = -x5 * x9 - x8;
  out[3][3] = 0;
  out[3][4] = 0;
  out[3][5] = 0;
  out[4][0] = -x0 * x2;
  out[4][1] = -x5 * x8 - x9;
  out[4][2] = -x4 * x5 + x7;
  out[4][3] = 0;
  out[4][4] = 0;
  out[4][5] = 0;
  out[5][0] = x5;
  out[5][1] = -x0 * x6;
  out[5][2] = -x0 * x3;
  out[5][3] = 0;
  out[5][4] = 0;
  out[5][5] = 0;
  out[6][0] = 0;
  out[6][1] = 0;
  out[6][2] = 0;
  out[6][3] = -1;
  out[6][4] = -x10 * x6;
  out[6][5] = -x10 * x3;
  out[7][0] = 0;
  out[7][1] = 0;
  out[7][2] = 0;
  out[7][3] = 0;
  out[7][4] = -x3;
  out[7][5] = x6;
  out[8][0] = 0;
  out[8][1] = 0;
  out[8][2] = 0;
  out[8][3] = 0;
  out[8][4] = -x11 * x6;
  out[8][5] = -x11 * x3;
  out[9][0] = 0;
  out[9][1] = 0;
  out[9][2] = 0;
  out[9][3] = 0;
  out[9][4] = 0;
  out[9][5] = 0;
  out[10][0] = 0;
  out[10][1] = 0;
  out[10][2] = 0;
  out[10][3] = 0;
  out[10][4] = 0;
  out[10][5] = 0;
  out[11][0] = 0;
  out[11][1] = 0;
  out[11][2] = 0;
  out[11][3] = 0;
  out[11][4] = 0;
  out[11][5] = 0;
  out[12][0] = 0;
  out[12][1] = 0;
  out[12][2] = 0;
  out[12][3] = 0;
  out[12][4] = 0;
  out[12][5] = 0;
  out[13][0] = 0;
  out[13][1] = 0;
  out[13][2] = 0;
  out[13][3] = 0;
  out[13][4] = 0;
  out[13][5] = 0;
  out[14][0] = 0;
  out[14][1] = 0;
  out[14][2] = 0;
  out[14][3] = 0;
  out[14][4] = 0;
  out[14][5] = 0;
}

static inline float ins_ext_pose_wrap_pi(float angle)
{
  while (angle > (float)M_PI) {
    angle -= 2.0f * (float)M_PI;
  }

  while (angle < -(float)M_PI) {
    angle += 2.0f * (float)M_PI;
  }

  return angle;
}

static inline void ins_ext_pose_compensate_attitude(
  const struct FloatEulers *measurement,
  float delay_s,
  struct FloatEulers *compensated)
{
  *compensated = *measurement;

  if (!isfinite(delay_s) || delay_s <= 0.0f) {
    return;
  }

  const float phi = measurement->phi;
  const float theta = measurement->theta;

  const float sin_phi = sinf(phi);
  const float cos_phi = cosf(phi);
  const float cos_theta = cosf(theta);

  /*
   * Do not apply Euler-angle forward propagation close to
   * the pitch singularity.
   */
  if (fabsf(cos_theta) < 0.2f) {
    return;
  }

  const float tan_theta = tanf(theta);

  /*
   * Bias-corrected body rates from the Bebop gyro.
   */
  const float p = ekf_U[3] - ekf_X[12];
  const float q = ekf_U[4] - ekf_X[13];
  const float r = ekf_U[5] - ekf_X[14];

  const float phi_dot =
    p + tan_theta * (q * sin_phi + r * cos_phi);

  const float theta_dot =
    q * cos_phi - r * sin_phi;

  const float psi_dot =
    (q * sin_phi + r * cos_phi) / cos_theta;

  compensated->phi =
    ins_ext_pose_wrap_pi(
      measurement->phi + phi_dot * delay_s);

  compensated->theta =
    ins_ext_pose_wrap_pi(
      measurement->theta + theta_dot * delay_s);

  compensated->psi =
    ins_ext_pose_wrap_pi(
      measurement->psi + psi_dot * delay_s);
}

static inline void ekf_reset_covariance(void)
{
  for (int i = 0; i < EKF_NUM_STATES; i++) {
    for (int j = 0; j < EKF_NUM_STATES; j++) {
      ekf_P[i][j] =
        (i == j)
          ? INS_EXT_POSE_P_RESET
          : 0.0f;
    }
  }

  ekf_covariance_reset_count++;
  ekf_covariance_healthy = false;
}


static inline bool ekf_condition_covariance(void)
{
  /*
   * A NaN or infinity cannot be repaired safely by
   * symmetrization, so reset the covariance.
   */
  for (int i = 0; i < EKF_NUM_STATES; i++) {
    for (int j = 0; j < EKF_NUM_STATES; j++) {
      if (!isfinite(ekf_P[i][j])) {
        ekf_reset_covariance();
        return false;
      }
    }
  }

  bool repaired = false;

  /*
   * Force symmetry:
   *
   * P = 0.5 * (P + P')
   */
  for (int i = 0; i < EKF_NUM_STATES; i++) {
    for (int j = i + 1;
         j < EKF_NUM_STATES;
         j++) {

      const float asymmetry =
      fabsf(ekf_P[i][j] - ekf_P[j][i]);

    const float symmetric =
      0.5f *
      (ekf_P[i][j] + ekf_P[j][i]);

    /*
    * Always enforce symmetry, but count it as a repair only
    * when the asymmetry is numerically meaningful.
    */
    if (asymmetry > 1e-6f) {
      repaired = true;
    }

    ekf_P[i][j] = symmetric;
    ekf_P[j][i] = symmetric;
    }
  }

  /*
   * Keep diagonal variances within finite numerical bounds.
   */
  for (int i = 0; i < EKF_NUM_STATES; i++) {
    if (ekf_P[i][i] < INS_EXT_POSE_P_MIN) {
      ekf_P[i][i] = INS_EXT_POSE_P_MIN;
      repaired = true;
    }

    if (ekf_P[i][i] > INS_EXT_POSE_P_MAX) {
      ekf_P[i][i] = INS_EXT_POSE_P_MAX;
      repaired = true;
    }
  }

  if (repaired) {
    ekf_covariance_repair_count++;
  }

  ekf_covariance_healthy = true;
  return true;
}

static inline void ekf_prediction_step(const float U[EKF_NUM_INPUTS], const float dt)
{
  // [1] Predicted (a priori) state estimate:
  float Xkk_1[EKF_NUM_STATES];
  // Xkk_1 = f(X,U)
  ekf_f(ekf_X, U, Xkk_1);
  // Xkk_1 *= dt
  float_vect_scale(Xkk_1, dt, EKF_NUM_STATES);
  // Xkk_1 += X
  float_vect_add(Xkk_1, ekf_X, EKF_NUM_STATES);


  // [2] Get matrices
  float F[EKF_NUM_STATES][EKF_NUM_STATES];
  float Ld[EKF_NUM_STATES][EKF_NUM_INPUTS];
  ekf_F(ekf_X, U, F);
  ekf_L(ekf_X, U, Ld);


  // [3] Continuous to discrete
  // Fd = eye(N) + F*dt
  // Ld = L*dt
  float Fd[EKF_NUM_STATES][EKF_NUM_STATES];

  MAKE_MATRIX_PTR(F_, F, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(Fd_, Fd, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(Ld_, Ld, EKF_NUM_STATES);

  // Fd = I+F*dt/2
  float_mat_diagonal_scal(Fd_, 1, EKF_NUM_STATES);
  float_mat_sum_scaled(Fd_, F_, dt, EKF_NUM_STATES, EKF_NUM_STATES);

  // Ld = Ld*dt
  float_mat_scale(Ld_, dt, EKF_NUM_STATES, EKF_NUM_INPUTS);


  // [4] Predicted covariance estimate:
  // Pkk_1 = Fd*P*Fd.T + Ld*Q*Ld.T
  float Pkk_1[EKF_NUM_STATES][EKF_NUM_STATES];
  float LdT[EKF_NUM_INPUTS][EKF_NUM_STATES];
  float QLdT[EKF_NUM_INPUTS][EKF_NUM_STATES];
  float tmp[EKF_NUM_STATES][EKF_NUM_STATES];

  MAKE_MATRIX_PTR(Pkk_1_, Pkk_1, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ekf_Q_, ekf_Q, EKF_NUM_INPUTS);
  MAKE_MATRIX_PTR(LdT_, LdT, EKF_NUM_INPUTS);
  MAKE_MATRIX_PTR(QLdT_, QLdT, EKF_NUM_INPUTS);
  MAKE_MATRIX_PTR(tmp_, tmp, EKF_NUM_STATES);

  // Fd = Fd.T
  float_mat_transpose_square(Fd_, EKF_NUM_STATES);

  // tmp = P*Fd
  float_mat_mul(tmp_, ekf_P_, Fd_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

  // Fd = Fd.T
  float_mat_transpose_square(Fd_, EKF_NUM_STATES);

  // Pkk_1 = Fd*tmp
  float_mat_mul(Pkk_1_, Fd_, tmp_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

  // LdT = Ld.T
  float_mat_transpose(LdT_, Ld_, EKF_NUM_STATES, EKF_NUM_INPUTS);

  // QLdT = Q*LdT
  float_mat_mul(QLdT_, ekf_Q_, LdT_, EKF_NUM_INPUTS, EKF_NUM_INPUTS, EKF_NUM_STATES);

  // tmp = Ld*QLdT
  float_mat_mul(tmp_, Ld_, QLdT_, EKF_NUM_STATES, EKF_NUM_INPUTS, EKF_NUM_STATES);

  // Pkk_1 += tmp
  float_mat_sum_scaled(Pkk_1_, tmp_, 1, EKF_NUM_STATES, EKF_NUM_STATES);

  // X = Xkk_1
  float_vect_copy(ekf_X, Xkk_1, EKF_NUM_STATES);

  // P = Pkk_1
  float_mat_copy(ekf_P_, Pkk_1_, EKF_NUM_STATES, EKF_NUM_STATES);

  ekf_condition_covariance();
}

static inline bool ekf_measurement_step(const float Z[EKF_NUM_OUTPUTS])
{
  // Xkk_1 = X
  float Xkk_1[EKF_NUM_STATES];
  float_vect_copy(Xkk_1, ekf_X, EKF_NUM_STATES);

  // Pkk_1 = P
  float Pkk_1[EKF_NUM_STATES][EKF_NUM_STATES];
  MAKE_MATRIX_PTR(Pkk_1_, Pkk_1, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
  float_mat_copy(Pkk_1_, ekf_P_, EKF_NUM_STATES, EKF_NUM_STATES);

  // [5] Measurement residual:
  // yk = Z - H*Xkk_1
  float yk[EKF_NUM_OUTPUTS];

  MAKE_MATRIX_PTR(ekf_H_, ekf_H, EKF_NUM_OUTPUTS);

  float_mat_vect_mul(yk, ekf_H_, Xkk_1, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
  float_vect_scale(yk, -1, EKF_NUM_OUTPUTS);
  float_vect_add(yk, Z, EKF_NUM_OUTPUTS);

  /*
  * Z[6:8] contains roll, pitch and yaw.
  * Wrap the angular innovations to [-pi, pi].
  */
  yk[6] = ins_ext_pose_wrap_pi(yk[6]);
  yk[7] = ins_ext_pose_wrap_pi(yk[7]);
  yk[8] = ins_ext_pose_wrap_pi(yk[8]);

  /*
  * Store the exact innovation used by this measurement update.
  *
  * y = Z - H * X_pred
  *
  * H directly observes the first nine states, so this is:
  * position, velocity and attitude measurement minus prediction.
  */
  for (int i = 0; i < EKF_NUM_OUTPUTS; i++) {
    ekf_last_innovation[i] = yk[i];
  }

  // [6] Residual covariance:
  // Sk = H*Pkk_1*H.T + R
  float Sk[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
  float PHT[EKF_NUM_STATES][EKF_NUM_OUTPUTS];

  MAKE_MATRIX_PTR(Sk_, Sk, EKF_NUM_OUTPUTS);
  MAKE_MATRIX_PTR(PHT_, PHT, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ekf_R_, ekf_R, EKF_NUM_OUTPUTS);

  // PHT = Pkk_1*H.T
  float_mat_transpose(PHT_, ekf_H_, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
  float_mat_mul_copy(PHT_, Pkk_1_, PHT_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_OUTPUTS);

  // Sk = H*PHT
  float_mat_mul(Sk_, ekf_H_, PHT_, EKF_NUM_OUTPUTS, EKF_NUM_STATES, EKF_NUM_OUTPUTS);

  // Sk += R
  float_mat_sum_scaled(Sk_, ekf_R_, 1, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


  // [7] Near-optimal Kalman gain:
  // K = Pkk_1*H.T*inv(Sk)
  float Sk_inv[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
  float K[EKF_NUM_STATES][EKF_NUM_OUTPUTS];

  MAKE_MATRIX_PTR(Sk_inv_, Sk_inv, EKF_NUM_OUTPUTS);
  MAKE_MATRIX_PTR(K_, K, EKF_NUM_STATES);

  // Sk_inv = inv(Sk)
  float_mat_invert(Sk_inv_, Sk_, EKF_NUM_OUTPUTS);

  /*
  * Reject a numerically invalid residual-covariance inverse
  * before it can contaminate the EKF state.
  */
  for (int i = 0; i < EKF_NUM_OUTPUTS; i++) {
    for (int j = 0; j < EKF_NUM_OUTPUTS; j++) {
      if (!isfinite(Sk_inv[i][j])) {
        /*
        * The measurement was not fused.
        * Keep the predicted state and reset only the covariance.
        */
        ekf_reset_covariance();
        return false;
      }
    }
  }

  // K = PHT*Sk_inv
  float_mat_mul(K_, PHT_, Sk_inv_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


  // [8] Updated state estimate
  // Xkk = Xkk_1 + K*yk
  float_mat_vect_mul(ekf_X, K_, yk, EKF_NUM_STATES, EKF_NUM_OUTPUTS);
  float_vect_add(ekf_X, Xkk_1, EKF_NUM_STATES);

  /*
  * Make sure the correction did not create an invalid state.
  */
  for (int i = 0; i < EKF_NUM_STATES; i++) {
    if (!isfinite(ekf_X[i])) {

      /*
      * Restore the valid predicted state.
      */
      float_vect_copy(
        ekf_X,
        Xkk_1,
        EKF_NUM_STATES
      );

      ekf_reset_covariance();
      return false;
    }
  }


  /*
  * [9] Joseph-form covariance update:
  *
  * P = A * P_pred * A' + K * R * K'
  * where A = I - K * H
  */
  float A[EKF_NUM_STATES][EKF_NUM_STATES];
  float ap_mat[EKF_NUM_STATES][EKF_NUM_STATES];
  float AT[EKF_NUM_STATES][EKF_NUM_STATES];
  float APAT[EKF_NUM_STATES][EKF_NUM_STATES];

  float KT[EKF_NUM_OUTPUTS][EKF_NUM_STATES];
  float RKT[EKF_NUM_OUTPUTS][EKF_NUM_STATES];
  float KRKT[EKF_NUM_STATES][EKF_NUM_STATES];

  MAKE_MATRIX_PTR(A_, A, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ap_mat_, ap_mat, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(AT_, AT, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(APAT_, APAT, EKF_NUM_STATES);

  MAKE_MATRIX_PTR(KT_, KT, EKF_NUM_OUTPUTS);
  MAKE_MATRIX_PTR(RKT_, RKT, EKF_NUM_OUTPUTS);
  MAKE_MATRIX_PTR(KRKT_, KRKT, EKF_NUM_STATES);

  /*
  * A = I - K*H
  */
  float_mat_mul(
    A_,
    K_,
    ekf_H_,
    EKF_NUM_STATES,
    EKF_NUM_OUTPUTS,
    EKF_NUM_STATES
  );

  float_mat_scale(
    A_,
    -1.0f,
    EKF_NUM_STATES,
    EKF_NUM_STATES
  );

  for (int i = 0; i < EKF_NUM_STATES; i++) {
    A[i][i] += 1.0f;
  }

  /*
  * APAT = A * P_pred * A'
  */
  float_mat_mul(
    ap_mat_,
    A_,
    Pkk_1_,
    EKF_NUM_STATES,
    EKF_NUM_STATES,
    EKF_NUM_STATES
  );

  float_mat_transpose(
    AT_,
    A_,
    EKF_NUM_STATES,
    EKF_NUM_STATES
  );

  float_mat_mul(
    APAT_,
    ap_mat_,
    AT_,
    EKF_NUM_STATES,
    EKF_NUM_STATES,
    EKF_NUM_STATES
  );

  /*
  * KRKT = K * R * K'
  */
  float_mat_transpose(
    KT_,
    K_,
    EKF_NUM_STATES,
    EKF_NUM_OUTPUTS
  );

  float_mat_mul(
    RKT_,
    ekf_R_,
    KT_,
    EKF_NUM_OUTPUTS,
    EKF_NUM_OUTPUTS,
    EKF_NUM_STATES
  );

  float_mat_mul(
    KRKT_,
    K_,
    RKT_,
    EKF_NUM_STATES,
    EKF_NUM_OUTPUTS,
    EKF_NUM_STATES
  );

  /*
  * P = APAT + KRKT
  */
  float_mat_copy(
    ekf_P_,
    APAT_,
    EKF_NUM_STATES,
    EKF_NUM_STATES
  );

  float_mat_sum_scaled(
    ekf_P_,
    KRKT_,
    1.0f,
    EKF_NUM_STATES,
    EKF_NUM_STATES
  );

  /*
  * Reject the correction if its resulting covariance was
  * numerically invalid and had to be reset.
  */
  if (!ekf_condition_covariance()) {

    /*
    * Restore the predicted state because the complete
    * correction cannot be considered valid.
    */
    float_vect_copy(
      ekf_X,
      Xkk_1,
      EKF_NUM_STATES
    );

    return false;
  }

  /*
  * Store the successful correction applied to the directly
  * measured states.
  *
  * Xkk_1 contains the predicted state copied at the beginning
  * of this function. ekf_X now contains the corrected state.
  */
  for (int i = 0; i < EKF_NUM_OUTPUTS; i++) {
    ekf_last_correction[i] = ekf_X[i] - Xkk_1[i];
  }

  /*
  * Wrap attitude corrections to [-pi, pi].
  */
  ekf_last_correction[6] = ins_ext_pose_wrap_pi(ekf_last_correction[6]);

  ekf_last_correction[7] = ins_ext_pose_wrap_pi(ekf_last_correction[7]);

  ekf_last_correction[8] = ins_ext_pose_wrap_pi(ekf_last_correction[8]);
  
  return true;

}


static inline void ekf_export_state(void)
{
  struct NedCoor_f ned_pos = {
    .x = ekf_X[0],
    .y = ekf_X[1],
    .z = ekf_X[2]
  };

  struct NedCoor_f ned_speed = {
    .x = ekf_X[3],
    .y = ekf_X[4],
    .z = ekf_X[5]
  };

  struct FloatEulers ned_to_body_eulers = {
    .phi = ekf_X[6],
    .theta = ekf_X[7],
    .psi = ekf_X[8]
  };

  /*
   * Body rates remain based on the high-rate Bebop gyro.
   * OptiTrack attitude is only approximately 45-50 Hz and should not
   * replace the gyro as the stabilization rate measurement.
   */
  struct FloatRates body_rates = {
    .p = ekf_U[3] - ekf_X[12],
    .q = ekf_U[4] - ekf_X[13],
    .r = ekf_U[5] - ekf_X[14]
  };

  /*
   * Bias-corrected specific force in body coordinates.
   */
  struct FloatVect3 accel_body_f = {
    .x = ekf_U[0] - ekf_X[9],
    .y = ekf_U[1] - ekf_X[10],
    .z = ekf_U[2] - ekf_X[11]
  };

  /*
   * Use the current EKF attitude directly.
   * Do not retrieve a possibly stale global-state rotation matrix.
   */
  struct FloatRMat ned_to_body_rmat;
  float_rmat_of_eulers(&ned_to_body_rmat, &ned_to_body_eulers);

  struct FloatVect3 accel_ned_f;
  float_rmat_transp_vmult(
    &accel_ned_f,
    &ned_to_body_rmat,
    &accel_body_f
  );

  /*
   * NED z points downward.
   * The accelerometer measures specific force, so add gravity.
   */
  accel_ned_f.z += 9.81f;

  struct NedCoor_f ned_accel = {
    .x = accel_ned_f.x,
    .y = accel_ned_f.y,
    .z = accel_ned_f.z
  };

  struct Int32Vect3 accel_body_i;
  ACCELS_BFP_OF_REAL(accel_body_i, accel_body_f);

  /*
  * Maintain the integer LTP values used by INS telemetry.
  */
  ins_ext_pos.ltp_pos.x = POS_BFP_OF_REAL(ned_pos.x);
  ins_ext_pos.ltp_pos.y = POS_BFP_OF_REAL(ned_pos.y);
  ins_ext_pos.ltp_pos.z = POS_BFP_OF_REAL(ned_pos.z);

  ins_ext_pos.ltp_speed.x = SPEED_BFP_OF_REAL(ned_speed.x);
  ins_ext_pos.ltp_speed.y = SPEED_BFP_OF_REAL(ned_speed.y);
  ins_ext_pos.ltp_speed.z = SPEED_BFP_OF_REAL(ned_speed.z);

  ins_ext_pos.ltp_accel.x = ACCEL_BFP_OF_REAL(ned_accel.x);
  ins_ext_pos.ltp_accel.y = ACCEL_BFP_OF_REAL(ned_accel.y);
  ins_ext_pos.ltp_accel.z = ACCEL_BFP_OF_REAL(ned_accel.z);

  /*
   * ins_ext_pose is now the only state provider for these quantities.
   */
  stateSetNedToBodyEulers_f(&ned_to_body_eulers);
  stateSetBodyRates_f(&body_rates);

  stateSetPositionNed_f(&ned_pos);
  stateSetSpeedNed_f(&ned_speed);

  stateSetAccelBody_i(&accel_body_i);
  stateSetAccelNed_f(&ned_accel);
}


static inline void ekf_run(void)
{
  static bool started = false;
  static bool have_accel = false;
  static bool have_gyro = false;

  const float now = get_sys_time_float();

  /*
   * Read the newest IMU samples.
   */
  if (ins_ext_pos.has_new_acc) {
    ekf_U[0] = ins_ext_pos.accels_f.x;
    ekf_U[1] = ins_ext_pos.accels_f.y;
    ekf_U[2] = ins_ext_pos.accels_f.z;

    ins_ext_pos.has_new_acc = false;
    have_accel = true;
  }

  if (ins_ext_pos.has_new_gyro) {
    ekf_U[3] = ins_ext_pos.gyros_f.p;
    ekf_U[4] = ins_ext_pos.gyros_f.q;
    ekf_U[5] = ins_ext_pos.gyros_f.r;

    ins_ext_pos.has_new_gyro = false;
    have_gyro = true;
  }

  /*
   * Do not publish an all-zero navigation state before receiving:
   * 1. one accelerometer sample;
   * 2. one gyroscope sample;
   * 3. one complete OptiTrack measurement.
   */
  if (!started) {
    ins_ext_pose_ready = false;
    ins_ext_pose_fresh = false;

    if (!have_accel || !have_gyro || !ins_ext_pos.has_new_ext_pose) {
      return;
    }

    /*
     * Initialize position from OptiTrack.
     */
    ekf_X[0] = ins_ext_pos.ev_pos.x;
    ekf_X[1] = ins_ext_pos.ev_pos.y;
    ekf_X[2] = ins_ext_pos.ev_pos.z;

    /*
     * Initialize velocity from OptiTrack.
     */
    ekf_X[3] = ins_ext_pos.ev_speed.x;
    ekf_X[4] = ins_ext_pos.ev_speed.y;
    ekf_X[5] = ins_ext_pos.ev_speed.z;

    /*
     * Initialize attitude from OptiTrack.
     */
    ekf_X[6] = ins_ext_pos.ev_att.phi;
    ekf_X[7] = ins_ext_pos.ev_att.theta;
    ekf_X[8] = ins_ext_pos.ev_att.psi;

    /*
     * Initialize accelerometer and gyroscope biases to zero.
     */
    for (int i = 9; i < EKF_NUM_STATES; i++) {
      ekf_X[i] = 0.0f;
    }

    /*
    * Keep the logged measurement vector consistent from startup.
    */
    ekf_Z[0] = ins_ext_pos.ev_pos.x;
    ekf_Z[1] = ins_ext_pos.ev_pos.y;
    ekf_Z[2] = ins_ext_pos.ev_pos.z;

    ekf_Z[3] = ins_ext_pos.ev_speed.x;
    ekf_Z[4] = ins_ext_pos.ev_speed.y;
    ekf_Z[5] = ins_ext_pos.ev_speed.z;

    ekf_Z[6] = ins_ext_pos.ev_att.phi;
    ekf_Z[7] = ins_ext_pos.ev_att.theta;
    ekf_Z[8] = ins_ext_pos.ev_att.psi;

    ins_ext_pos.has_new_ext_pose = false;

    ins_ext_pos.ext_pose_fused_time_s = now;
    ins_ext_pos.has_fused_ext_pose = true;

    t0 = now;
    started = true;

    ins_ext_pose_ready = true;
    ins_ext_pose_fresh = true;

    ekf_export_state();
    return;
  }

  /*
   * Calculate the local EKF prediction interval.
   */
  float dt = now - t0;
  t0 = now;

  /*
  * Preserve the interval before clamping.
  */
  ekf_last_raw_dt_s = dt;

  /*
  * Set this to zero until a valid interval has been accepted.
  */
  ekf_last_used_dt_s = 0.0f;

  if (!isfinite(dt) || dt <= 0.0f) {
    return;
  }

  /*
   * Protect the filter against large scheduler or logging pauses.
   *
   * A normal 512 Hz interval is approximately 0.00195 s.
   */
  if (dt > INS_EXT_POSE_MAX_DT) {
    dt = INS_EXT_POSE_MAX_DT;
    ins_ext_pose_dt_clamp_count++;
  }

  /*
  * This is the interval actually passed into the prediction model.
  */
  ekf_last_used_dt_s = dt;

  /*
   * High-rate prediction from Bebop IMU.
   */
  ekf_prediction_step(ekf_U, dt);

  /*
   * Keep propagated Euler states bounded.
   */
  ekf_X[6] = ins_ext_pose_wrap_pi(ekf_X[6]);
  ekf_X[7] = ins_ext_pose_wrap_pi(ekf_X[7]);
  ekf_X[8] = ins_ext_pose_wrap_pi(ekf_X[8]);

  /*
   * External-pose health.
   */
  ins_ext_pose_ready = true;


  /*
   * Apply the newest OptiTrack correction once.
   */
  if (ins_ext_pos.has_new_ext_pose) {
    /*
    * Consume the packet once, regardless of acceptance or rejection.
    */
    ins_ext_pos.has_new_ext_pose = false;

    /*
    * Estimate total measurement delay.
    *
    * FIXED_DELAY remains zero until independently measured.
    */
    float measurement_delay_s = INS_EXT_POSE_FIXED_DELAY + ins_ext_pos.ext_pose_queue_delay_s;

    /*
    * Store the total delay for diagnostics.
    */
    ins_ext_pos.ext_pose_measurement_delay_s = measurement_delay_s;

    /*
    * A rejected packet or a disabled compensation path uses
    * no forward projection.
    */
    ins_ext_pos.ext_pose_compensation_delay_s = 0.0f;

    /*
    * Detect queued EXTERNAL_POSE packets being replayed much
    * faster than their original source interval.
    *
    * Conditions:
    * 1. Some queue delay is still present.
    * 2. The source timestamp indicates a normal OptiTrack frame interval.
    * 3. The onboard receive interval is valid.
    * 4. Packets are arriving much faster than normal.
    */
    const bool burst_packet =
      ins_ext_pos.ext_pose_queue_delay_s > INS_EXT_POSE_RECOVERY_QUEUE_DELAY &&
      ins_ext_pos.ext_pose_source_dt_s > 0.018f &&
      ins_ext_pos.ext_pose_rx_dt_s > 0.0f &&
      ins_ext_pos.ext_pose_rx_dt_s < INS_EXT_POSE_MIN_RX_INTERVAL;

    /*
    * Reject a severely delayed/backlogged measurement.
    */
    if (!isfinite(measurement_delay_s) || ins_ext_pos.ext_pose_queue_delay_s > INS_EXT_POSE_MAX_QUEUE_DELAY || burst_packet) {

      /*
      * Discard this delayed measurement but retain the
      * already-completed IMU prediction.
      */
      ins_ext_pose_latency_reject_count++;
    } else {

      /*
      * Limit forward compensation to a short, safe interval.
      */
      float compensation_delay_s = measurement_delay_s;

      if (compensation_delay_s < 0.0f) {
        compensation_delay_s = 0.0f;
      }

      if (compensation_delay_s > INS_EXT_POSE_MAX_COMP_DELAY) {
        compensation_delay_s = INS_EXT_POSE_MAX_COMP_DELAY;
      }

      ins_ext_pos.ext_pose_compensation_delay_s = compensation_delay_s;

      /*
      * Constant-velocity position forward projection:
      *
      * p_now ≈ p_capture + v_capture * delay
      */
      ekf_Z[0] = ins_ext_pos.ev_pos.x + ins_ext_pos.ev_speed.x * compensation_delay_s;

      ekf_Z[1] = ins_ext_pos.ev_pos.y + ins_ext_pos.ev_speed.y * compensation_delay_s;

      ekf_Z[2] = ins_ext_pos.ev_pos.z + ins_ext_pos.ev_speed.z * compensation_delay_s;

      /*
      * Velocity is left unchanged because EXTERNAL_POSE does not
      * provide an external acceleration measurement.
      */
      ekf_Z[3] = ins_ext_pos.ev_speed.x;
      ekf_Z[4] = ins_ext_pos.ev_speed.y;
      ekf_Z[5] = ins_ext_pos.ev_speed.z;

      /*
      * First-order attitude forward projection.
      */
      struct FloatEulers compensated_attitude;

      ins_ext_pose_compensate_attitude(&ins_ext_pos.ev_att, compensation_delay_s, &compensated_attitude);

      ekf_Z[6] = compensated_attitude.phi;
      ekf_Z[7] = compensated_attitude.theta;
      ekf_Z[8] = compensated_attitude.psi;

      /*
      * Only declare the measurement fused when the complete
      * state and covariance correction succeeded.
      */
      if (ekf_measurement_step(ekf_Z)) {

        ekf_fusion_count++;
        ekf_last_measurement_success = 1;

        ins_ext_pos.ext_pose_fused_time_s = now;
        ins_ext_pos.has_fused_ext_pose = true;

        ekf_X[6] =
          ins_ext_pose_wrap_pi(ekf_X[6]);

        ekf_X[7] =
          ins_ext_pose_wrap_pi(ekf_X[7]);

        ekf_X[8] =
          ins_ext_pose_wrap_pi(ekf_X[8]);

      } else {

        ekf_measurement_failure_count++;
        ekf_last_measurement_success = 0;

        /*
        * Avoid displaying a stale correction from the previous
        * successful fusion after a failed update.
        */
        for (int i = 0; i < EKF_NUM_OUTPUTS; i++) {
          ekf_last_correction[i] = 0.0f;
        }
      }
    }
  }
  ins_ext_pose_fresh = ins_ext_pos.has_fused_ext_pose && ((now - ins_ext_pos.ext_pose_fused_time_s) <= INS_EXT_POSE_TIMEOUT);

  ekf_export_state();
}



/**
 * Logging
 */

void ins_ext_pos_log_header(FILE *file)
{
  /*
  * Current Paparazzi NED-to-body attitude quaternion.
  *
  * This is the same state representation used by the attitude
  * controller.
  */
  
  fprintf(file,
          "ekf_X1,ekf_X2,ekf_X3,ekf_X4,ekf_X5,ekf_X6,ekf_X7,ekf_X8,ekf_X9,ekf_X10,ekf_X11,ekf_X12,ekf_X13,ekf_X14,ekf_X15,");
  fprintf(file, "ekf_U1,ekf_U2,ekf_U3,ekf_U4,ekf_U5,ekf_U6,");

  fprintf(
    file,
    "ekf_Z1,ekf_Z2,ekf_Z3,"
    "ekf_Z4,ekf_Z5,ekf_Z6,"
    "ekf_Z7,ekf_Z8,ekf_Z9,");

    fprintf(
      file,
      "ext_pos_x,ext_pos_y,ext_pos_z,"
      "ext_vel_x,ext_vel_y,ext_vel_z,"
      "ext_phi,ext_theta,ext_psi,"
      "ext_pose_age,"
      "ext_pose_ready,"
      "ext_pose_fresh,"
      "ekf_dt_clamp_count,");

    fprintf(
      file,
      "ext_fused_age,"
      "ext_source_dt,"
      "ext_rx_dt,"
      "ext_queue_delay,"
      "ext_measurement_delay,"
      "ext_compensation_delay,"
      "latency_reject_count,"
      "covariance_healthy,"
      "covariance_repair_count,"
      "covariance_reset_count,"
    );

    fprintf(
      file,
      "ekf_fusion_count,"
      "ekf_measurement_failure_count,"
      "ekf_last_measurement_success,"
      "ekf_raw_dt,"
      "ekf_used_dt,"
    );

    fprintf(
      file,
      "innov_x,innov_y,innov_z,"
      "innov_vx,innov_vy,innov_vz,"
      "innov_phi,innov_theta,innov_psi,"
    );

    fprintf(
      file,
      "corr_x,corr_y,corr_z,"
      "corr_vx,corr_vy,corr_vz,"
      "corr_phi,corr_theta,corr_psi,"
    );

    /*
    * External-attitude transformation diagnostics.
    */
    fprintf(
      file,
      "raw_qi,raw_qx,raw_qy,raw_qz,"
      "raw_q_norm,"
    );

    fprintf(
      file,
      "raw_euler_phi,"
      "raw_euler_theta,"
      "raw_euler_psi,"
    );

    fprintf(
      file,
      "mapped_qi,"
      "mapped_qx,"
      "mapped_qy,"
      "mapped_qz,"
    );

    fprintf(
      file,
      "mapped_pre_phi,"
      "mapped_pre_theta,"
      "mapped_pre_psi,"
    );

    fprintf(
      file,
      "state_qi,"
      "state_qx,"
      "state_qy,"
      "state_qz,"
    );
}

void ins_ext_pos_log_data(FILE *file)
{
  struct FloatQuat *state_q = stateGetNedToBodyQuat_f();
  fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,", ekf_X[0], ekf_X[1], ekf_X[2], ekf_X[3], ekf_X[4],
          ekf_X[5], ekf_X[6], ekf_X[7], ekf_X[8], ekf_X[9], ekf_X[10], ekf_X[11], ekf_X[12], ekf_X[13], ekf_X[14]);
  fprintf(file, "%f,%f,%f,%f,%f,%f,", ekf_U[0], ekf_U[1], ekf_U[2], ekf_U[3], ekf_U[4], ekf_U[5]);
  fprintf(file,"%f,%f,%f,%f,%f,%f,%f,%f,%f,",
    ekf_Z[0],
    ekf_Z[1],
    ekf_Z[2],
    ekf_Z[3],
    ekf_Z[4],
    ekf_Z[5],
    ekf_Z[6],
    ekf_Z[7],
    ekf_Z[8]);

  const float ext_pose_age =
    ins_ext_pos.has_ext_pose_stamp
      ? get_sys_time_float() - ins_ext_pos.ext_pose_rx_time_s
      : -1.0f;

  const float ext_fused_age =
    ins_ext_pos.has_fused_ext_pose
      ? get_sys_time_float() -
        ins_ext_pos.ext_pose_fused_time_s
      : -1.0f;

fprintf(
  file,
  "%f,%f,%f,"
  "%f,%f,%f,"
  "%f,%f,%f,"
  "%f,%d,%d,%u,",

  ins_ext_pos.ev_pos.x,
  ins_ext_pos.ev_pos.y,
  ins_ext_pos.ev_pos.z,

  ins_ext_pos.ev_speed.x,
  ins_ext_pos.ev_speed.y,
  ins_ext_pos.ev_speed.z,

  ins_ext_pos.ev_att.phi,
  ins_ext_pos.ev_att.theta,
  ins_ext_pos.ev_att.psi,

  ext_pose_age,
  ins_ext_pose_ready ? 1 : 0,
  ins_ext_pose_fresh ? 1 : 0,
  (unsigned int)ins_ext_pose_dt_clamp_count);

  fprintf(
    file,
    "%f,%f,%f,%f,%f,%f,%u,%d,%u,%u,",

    ext_fused_age,

    ins_ext_pos.ext_pose_source_dt_s,
    ins_ext_pos.ext_pose_rx_dt_s,
    ins_ext_pos.ext_pose_queue_delay_s,

    ins_ext_pos.ext_pose_measurement_delay_s,
    ins_ext_pos.ext_pose_compensation_delay_s,

    (unsigned int)ins_ext_pose_latency_reject_count,

    ekf_covariance_healthy ? 1 : 0,

    (unsigned int)ekf_covariance_repair_count,
    (unsigned int)ekf_covariance_reset_count
  );

  /*
  * Measurement-update status and EKF timing.
  */
  fprintf(
    file,
    "%u,%u,%u,%f,%f,",

    (unsigned int)ekf_fusion_count,
    (unsigned int)ekf_measurement_failure_count,
    (unsigned int)ekf_last_measurement_success,

    ekf_last_raw_dt_s,
    ekf_last_used_dt_s
  );

  /*
  * Latest measurement innovation.
  */
  fprintf(
    file,
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,",

    ekf_last_innovation[0],
    ekf_last_innovation[1],
    ekf_last_innovation[2],

    ekf_last_innovation[3],
    ekf_last_innovation[4],
    ekf_last_innovation[5],

    ekf_last_innovation[6],
    ekf_last_innovation[7],
    ekf_last_innovation[8]
  );

  /*
  * Latest successful state correction.
  */
  fprintf(
    file,
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,",

    ekf_last_correction[0],
    ekf_last_correction[1],
    ekf_last_correction[2],

    ekf_last_correction[3],
    ekf_last_correction[4],
    ekf_last_correction[5],

    ekf_last_correction[6],
    ekf_last_correction[7],
    ekf_last_correction[8]
  );
  /*
 * Raw normalized NatNet quaternion and its original norm.
 */
fprintf(
  file,
  "%f,%f,%f,%f,%f,",

  ext_diag_raw_q_unit.qi,
  ext_diag_raw_q_unit.qx,
  ext_diag_raw_q_unit.qy,
  ext_diag_raw_q_unit.qz,

  ext_diag_raw_q_norm
);

/*
 * Direct Euler conversion of the raw quaternion.
 */
fprintf(
  file,
  "%f,%f,%f,",

  ext_diag_raw_eulers.phi,
  ext_diag_raw_eulers.theta,
  ext_diag_raw_eulers.psi
);

/*
 * Quaternion after the current axis/sign component mapping.
 */
fprintf(
  file,
  "%f,%f,%f,%f,",

  ext_diag_mapped_q_unit.qi,
  ext_diag_mapped_q_unit.qx,
  ext_diag_mapped_q_unit.qy,
  ext_diag_mapped_q_unit.qz
);

/*
 * Euler conversion before the separate theta negation.
 */
fprintf(
  file,
  "%f,%f,%f,",

  ext_diag_mapped_eulers_preflip.phi,
  ext_diag_mapped_eulers_preflip.theta,
  ext_diag_mapped_eulers_preflip.psi
);

/*
 * Current Paparazzi NED-to-body state quaternion.
 */
fprintf(
  file,
  "%f,%f,%f,%f,",

  state_q->qi,
  state_q->qx,
  state_q->qy,
  state_q->qz
);
}

bool ins_ext_pose_is_ready(void)
{
  return ins_ext_pose_ready;
}

bool ins_ext_pose_is_fresh(void)
{
  return ins_ext_pose_fresh;
}
