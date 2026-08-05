/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/visual_servoing/visual_servoing.c"
 * @author Chenyao Wang
 * This module is used for the thesis project: 04_TangoDrone
 */

#include <math.h>
#include "math/pprz_stat.h"
#include "math/pprz_algebra_float.h"
#include "modules/visual_servoing/visual_servoing.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/ins/ins_ext_pose.h"
#include "modules/core/abi.h"
#include "modules/datalink/telemetry.h"
#include <stdio.h>
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef VS_NOM_THROTTLE
#define VS_NOM_THROTTLE 0.666
#endif

#ifndef VS_SET_POINT
#define VS_SET_POINT 0.0
#endif

#ifndef VS_OL_X_PGAIN
#define VS_OL_X_PGAIN 3
#endif

#ifndef VS_OL_X_IGAIN
#define VS_OL_X_IGAIN 0.015
#endif

#ifndef VS_OL_Y_OF_GAIN
#define VS_OL_Y_OF_GAIN 4.8
#endif

#ifndef VS_OL_Y_YAW_GAIN
#define VS_OL_Y_YAW_GAIN 4.8
#endif

#ifndef VS_OL_Y_OA_GAIN
#define VS_OL_Y_OA_GAIN 0.0
#endif

#ifndef VS_OL_Z_PGAIN
#define VS_OL_Z_PGAIN 10.0
#endif

#ifndef VS_OL_Z_DGAIN
#define VS_OL_Z_DGAIN 0.0100
#endif

#ifndef VS_OL_Z_PFF
#define VS_OL_Z_PFF 10.0
#endif

#ifndef VS_OL_Z_SP
#define VS_OL_Z_SP 1.0
#endif

#ifndef VS_OL_X_VEL_PGAIN
#define VS_OL_X_VEL_PGAIN 10.00
#endif

// Should be slightly higher than the desired Vx (e.g. 0.0225 voor 0.02, 0.0440 voor 0.04)
#ifndef VS_OL_X_VEL_SP
#define VS_OL_X_VEL_SP 0.0
#endif

#ifndef VS_LP_CONST
#define VS_LP_CONST 12
#endif

#ifndef VS_THETA_OFFSET
#define VS_THETA_OFFSET 0.0
#endif

#ifndef VS_SWITCH_MAGNITUDE
#define VS_SWITCH_MAGNITUDE 1.2
#endif

#ifndef VS_SWITCH_DECAY
#define VS_SWITCH_DECAY 1.0
#endif

#ifndef VS_MANUAL_SWITCHING
#define VS_MANUAL_SWITCHING 0
#endif

#ifndef VS_APPROACH_MODE
#define VS_APPROACH_MODE 3
#endif

#ifndef VS_ABLATION_MODE
#define VS_ABLATION_MODE 0
#endif

#ifndef VS_NEW_SET_POINT
#define VS_NEW_SET_POINT 0.3
#endif

#ifndef VS_CC_THRESHOLD
#define VS_CC_THRESHOLD 5000
#endif

/*
 * ==============================================================
 * Forward-position hold
 * ==============================================================
 */

/*
 * Physical-unit gains:
 *
 *   acceleration =
 *       FWD_KP * position_error
 *     - FWD_KD * forward_velocity
 *
 * Position is in metres, velocity in m/s and output in m/s^2.
 */
#ifndef VS_FWD_KP
#define VS_FWD_KP 0.50f
#endif

#ifndef VS_FWD_KD
#define VS_FWD_KD 1.10f
#endif

/*
 * Conservative first-flight acceleration limit.
 *
 * 0.40 m/s^2 corresponds to approximately 2.3 degrees of
 * pitch when divided by gravity.
 */
#ifndef VS_FWD_MAX_ACCEL
#define VS_FWD_MAX_ACCEL 0.40f
#endif


/*
 * ==============================================================
 * Lateral optic-flow control
 * ==============================================================
 */

/*
 * Preserve the scaling used in the earlier experiments.
 */
#ifndef VS_OF_SCALE
#define VS_OF_SCALE 0.005f
#endif

/*
 * First-flight lateral acceleration limit.
 *
 * This is intentionally much smaller than the approximately
 * 1 m/s^2 peak acceleration needed for final 1 Hz tracking.
 */
#ifndef VS_LAT_MAX_ACCEL
#define VS_LAT_MAX_ACCEL 0.30f
#endif

/*
 * Lateral velocity damping used only when optic flow is not
 * ready. This is not a lateral position controller.
 */
#ifndef VS_LAT_FALLBACK_KD
#define VS_LAT_FALLBACK_KD 0.80f
#endif

/*
 * Slew limit for both horizontal acceleration-vector commands.
 */
#ifndef VS_ACCEL_SLEW
#define VS_ACCEL_SLEW 1.00f
#endif


/*
 * ==============================================================
 * Vision validity
 * ==============================================================
 */

#ifndef VS_VISION_MIN_DT
#define VS_VISION_MIN_DT 0.005f
#endif

#ifndef VS_VISION_MAX_DT
#define VS_VISION_MAX_DT 0.25f
#endif

#ifndef VS_VISION_TIMEOUT
#define VS_VISION_TIMEOUT 0.15f
#endif

#ifndef VS_VISION_MIN_STREAK
#define VS_VISION_MIN_STREAK 5
#endif


/*
 * ==============================================================
 * First-flight attitude limits
 * ==============================================================
 */

#ifndef VS_MAX_PITCH_DEG
#define VS_MAX_PITCH_DEG 4.0f
#endif

#ifndef VS_MAX_ROLL_DEG
#define VS_MAX_ROLL_DEG 4.0f
#endif

// define and initialise global variables
float fps = 0;
float end_time = 0;
float m_dt;
float pitch_sp = 0;
float roll_sp = 0;
float thrust_set = 0;
float true_distance = 0;
int8_t set_point_count = 0;
bool landing = FALSE;
bool switching = FALSE;
float switch_time_start = 0.0;
float switch_time_end = 0.0;
float start_color_count = 0;
float switch_distance = 10;
// To begin the first PID phase for Ablation experiments
bool reset_switch_time_end_bool = FALSE;
bool first_interation = FALSE;

float vs_enable_time = 0.0f;

int switch_count = 0;
float divsp_list[4] = {0.037963f, 0.105616f, 0.174661f, 0.523801f};

// Setup the message for the logger
static void send_vs_attitude(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers *attitude = stateGetNedToBodyEulers_f();
  pprz_msg_send_VISUAL_SERVOING(trans, dev, AC_ID,
                                &(attitude->theta),
                                &(attitude->phi),
                                &visual_servoing.divergence,
                                &visual_servoing.true_divergence,
                                &fps,
                                &visual_servoing.box_centroid_x,
                                &visual_servoing.box_centroid_y,
                                &visual_servoing.distance_est,
                                &true_distance,
                                &visual_servoing.color_count,
                                &(stateGetAccelNed_f()->x),
                                &visual_servoing.divergence_sp,
                                &visual_servoing.div_err_sum,
                                &visual_servoing.mu_x,
                                &visual_servoing.p_output,
                                &visual_servoing.i_output,
                                &visual_servoing.pid_on,
                                &visual_servoing.of_y,
                                &visual_servoing.of_y_d);
}

// This call back will be used to receive the color count and centroid from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp,
    int16_t pixel_x, int16_t pixel_y, int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
    int32_t quality, int16_t __attribute__((unused)) extra)
{
  /*
   * Store all fields belonging to this image first.
   */
  visual_servoing.vision_stamp_us = stamp;

  visual_servoing.color_count = (float)quality;

  visual_servoing.box_centroid_x = (float)pixel_x;

  visual_servoing.box_centroid_y = (float)pixel_y;

  /*
   * Increment the sequence counter last.
   *
   * The guidance loop treats the changed sequence number as
   * evidence that the complete sample above is ready.
   */
  visual_servoing.vision_sequence++;
}

// struct containing most relevant parameters
struct VisualServoing visual_servoing;

void visual_servoing_module_init(void);

void visual_servoing_module_enter(void);

void visual_servoing_module_run(bool in_flight);

static void update_errors(float box_x_err, float box_y_err, float div_err, float dt);

#if 0
static void final_land_in_box(float start_time);
#endif

static float divergence_step(float switch_time, float magnitude);

static float reset_switch_time_end(float switch_time_end);

static void visual_servoing_kf_init(float of_meas, float ofd_meas);

static void visual_servoing_kf_update(float of_meas, float ofd_meas, float dt);

/*
 * Capture forward-position, heading and hover-trim references
 * when visual-servoing mode is entered.
 */
static void visual_servoing_capture_reference(void);

/*
 * Clear optic-flow states after startup, target loss,
 * invalid frame timing or external-pose loss.
 */
static void visual_servoing_reset_of_state(void);

/*
 * Process each VISUAL_DETECTION callback exactly once.
 */
static void visual_servoing_process_new_vision_sample(
    uint32_t now_us);

/*
 * Rate-limit changes in horizontal acceleration commands.
 */
static float visual_servoing_slew(float current, float target, float rate_limit, float dt);

/*
 * Initialisation function
 */
void visual_servoing_module_init(void)
{
  visual_servoing.dt = 0.065;
  visual_servoing.mu_x = 0;
  visual_servoing.mu_y = 0;
  visual_servoing.mu_z = 0;
  visual_servoing.nominal_throttle = VS_NOM_THROTTLE;
  visual_servoing.divergence_sp = VS_SET_POINT;
  visual_servoing.set_point = VS_SET_POINT;
  visual_servoing.divergence = 0;
  visual_servoing.true_divergence = 0;
  visual_servoing.raw_divergence = 0;
  visual_servoing.ol_x_pgain = VS_OL_X_PGAIN;
  visual_servoing.ol_x_igain = VS_OL_X_IGAIN;                     
  visual_servoing.ol_y_OF_gain = VS_OL_Y_OF_GAIN;
  visual_servoing.ol_y_YAW_gain = VS_OL_Y_YAW_GAIN;
  visual_servoing.ol_y_OA_gain = VS_OL_Y_OA_GAIN;    
  visual_servoing.ol_z_pgain = VS_OL_Z_PGAIN;                            
  visual_servoing.ol_z_dgain = VS_OL_Z_DGAIN;             
  visual_servoing.ol_z_pff = VS_OL_Z_PFF;
  visual_servoing.height_setpoint = VS_OL_Z_SP;
  visual_servoing.height_error = 0;
  visual_servoing.prev_height_error = 0;       
  visual_servoing.previous_box_x_err = 0;
  visual_servoing.box_x_err_sum = 0;
  visual_servoing.box_x_err_d = 0;
  visual_servoing.previous_box_y_err = 0;
  visual_servoing.box_y_err_sum = 0;
  visual_servoing.box_y_err_d = 0;
  visual_servoing.div_err = 0;
  visual_servoing.previous_div_err = 0;
  visual_servoing.div_err_sum = 0;
  visual_servoing.lp_const = VS_LP_CONST;
  visual_servoing.switch_magnitude = VS_SWITCH_MAGNITUDE;
  visual_servoing.switch_decay = VS_SWITCH_DECAY;
  visual_servoing.distance_est = 10;
  visual_servoing.theta_offset = VS_THETA_OFFSET;
  visual_servoing.manual_switching = VS_MANUAL_SWITCHING;
  visual_servoing.approach_mode = VS_APPROACH_MODE;
  visual_servoing.ablation_mode = VS_ABLATION_MODE;
  visual_servoing.new_set_point = VS_NEW_SET_POINT;
  visual_servoing.color_count_threshold = VS_CC_THRESHOLD;
  visual_servoing.pitch_sum = 0;
  visual_servoing.delta_pixels = 0;
  visual_servoing.color_count = 0;
  visual_servoing.p_output = 0;
  visual_servoing.i_output = 0;
  visual_servoing.pid_on = 2;
  visual_servoing.time_since_last = 0.0;
  visual_servoing.vel_x = 0;
  visual_servoing.vel_x_sp = VS_OL_X_VEL_SP;
  visual_servoing.Kp_vx = VS_OL_X_VEL_PGAIN;
  visual_servoing.mu_vx_ff = 0.0;
  visual_servoing.err_vx = 0.0;
  visual_servoing.raw_of_y = 0;
  visual_servoing.of_y = 0;
  visual_servoing.raw_of_y_d = 0;
  visual_servoing.of_y_d = 0;
  visual_servoing.prev_box_centroid_y = 0;
  visual_servoing.prev_of_y = 0;
  visual_servoing.prev_raw_of_y = 0;
  //kalman filter
  visual_servoing.kf_x1 = 0.0f;
  visual_servoing.kf_x2 = 0.0f;
  visual_servoing.kf_x1_pred = 0.0f;
  visual_servoing.kf_x2_pred = 0.0f;

  visual_servoing.kf_p11 = 1.0f;
  visual_servoing.kf_p12 = 0.0f;
  visual_servoing.kf_p21 = 0.0f;
  visual_servoing.kf_p22 = 1.0f;

  visual_servoing.kf_q11 = 0.0010f;
  visual_servoing.kf_q22 = 0.05f;

  visual_servoing.kf_r11 = 0.0050f;
  visual_servoing.kf_r22 = 0.5000f;

  visual_servoing.kf_initialized = FALSE;
  visual_servoing.lp_of_b0 = 0.46515307f;
  visual_servoing.lp_of_b1 = 0.93030615f;
  visual_servoing.lp_of_b2 = 0.46515307f;
  visual_servoing.lp_of_a1 = 0.6202041028f;
  visual_servoing.lp_of_a2 = 0.240408205f;

  visual_servoing.yaw_vel = 0.0f;

  /*
   * Forward-position controller parameters.
   */
  visual_servoing.fwd_kp = VS_FWD_KP;

  visual_servoing.fwd_kd = VS_FWD_KD;

  visual_servoing.fwd_max_accel = VS_FWD_MAX_ACCEL;

  /*
   * Lateral controller parameters.
   */
  visual_servoing.of_scale = VS_OF_SCALE;

  visual_servoing.lat_max_accel = VS_LAT_MAX_ACCEL;

  visual_servoing.lat_fallback_kd = VS_LAT_FALLBACK_KD;

  visual_servoing.accel_slew = VS_ACCEL_SLEW;

  /*
   * Vision-validity parameters.
   */
  visual_servoing.vision_min_dt = VS_VISION_MIN_DT;

  visual_servoing.vision_max_dt = VS_VISION_MAX_DT;

  visual_servoing.vision_timeout = VS_VISION_TIMEOUT;

  visual_servoing.vision_min_streak = VS_VISION_MIN_STREAK;

  /*
   * Initial attitude limits.
   */
  visual_servoing.max_pitch_deg = VS_MAX_PITCH_DEG;

  visual_servoing.max_roll_deg = VS_MAX_ROLL_DEG;

  /*
   * Vision timestamp and sequence initialization.
   */
  visual_servoing.vision_stamp_us = 0;
  visual_servoing.vision_sequence = 0;
  visual_servoing.processed_vision_sequence = 0;
  visual_servoing.previous_vision_stamp_us = 0;
  visual_servoing.last_valid_vision_rx_us = 0;
  visual_servoing.last_control_time_us = 0;

  visual_servoing.vision_new_frame = false;
  visual_servoing.vision_valid = false;
  visual_servoing.have_previous_valid_target = false;
  visual_servoing.of_ready = false;
  visual_servoing.pose_ok = false;
  visual_servoing.using_of_control = false;
  visual_servoing.using_lateral_fallback = false;

  visual_servoing.vision_valid_streak = 0;
  visual_servoing.vision_dt = 0.0f;
  visual_servoing.vision_age = 1000.0f;
  visual_servoing.control_dt = 0.0f;

  /*
   * Reference-frame and controller diagnostics.
   */
  visual_servoing.heading_ref = 0.0f;

  visual_servoing.forward_axis_n = 1.0f;
  visual_servoing.forward_axis_e = 0.0f;

  visual_servoing.forward_position_ref = 0.0f;
  visual_servoing.forward_position = 0.0f;
  visual_servoing.forward_velocity = 0.0f;
  visual_servoing.forward_error = 0.0f;
  visual_servoing.right_velocity = 0.0f;

  visual_servoing.pitch_trim = 0.0f;
  visual_servoing.roll_trim = 0.0f;
  visual_servoing.mu_x_trim = 0.0f;
  visual_servoing.mu_y_trim = 0.0f;

  visual_servoing.forward_accel_cmd = 0.0f;
  visual_servoing.of_scaled = 0.0f;

  visual_servoing.mu_x_control = 0.0f;
  visual_servoing.mu_y_of = 0.0f;
  visual_servoing.mu_y_yaw = 0.0f;
  visual_servoing.mu_y_fallback = 0.0f;
  visual_servoing.mu_y_control = 0.0f;

  visual_servoing.mu_x_target = 0.0f;
  visual_servoing.mu_y_target = 0.0f;

  visual_servoing.pitch_sp_cmd = 0.0f;
  visual_servoing.roll_sp_cmd = 0.0f;
  visual_servoing.yaw_sp_cmd = 0.0f;

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISUAL_SERVOING, send_vs_attitude);
} 

static void reset_all_vars(void)
{
  visual_servoing.set_point = VS_SET_POINT;
  visual_servoing.dt = 0.065;
  visual_servoing.mu_x = 0;
  visual_servoing.mu_y = 0;
  visual_servoing.mu_z = 0;
  visual_servoing.divergence = 0;
  visual_servoing.true_divergence = 0;
  visual_servoing.raw_divergence = 0;
  visual_servoing.previous_box_x_err = 0;
  visual_servoing.box_x_err_sum = 0;
  visual_servoing.box_x_err_d = 0;
  visual_servoing.previous_box_y_err = 0;
  visual_servoing.box_y_err_sum = 0;
  visual_servoing.box_y_err_d = 0;
  visual_servoing.height_setpoint = VS_OL_Z_SP;
  visual_servoing.height_error = 0;
  visual_servoing.prev_height_error = 0;      
  visual_servoing.div_err = 0;
  visual_servoing.previous_div_err = 0;
  visual_servoing.div_err_sum = 0;
  visual_servoing.distance_est = 10;
  pitch_sp = 0;
  roll_sp = 0;

  set_point_count = 0;
  visual_servoing.divergence_sp = VS_SET_POINT;
  visual_servoing.p_output = 0;
  visual_servoing.i_output = 0;
  visual_servoing.pid_on = 0;
  visual_servoing.vel_x = 0;
  visual_servoing.vel_x_sp = VS_OL_X_VEL_SP;
  visual_servoing.Kp_vx = VS_OL_X_VEL_PGAIN;
  visual_servoing.mu_vx_ff = 0.0;
  visual_servoing.err_vx = 0.0;
  visual_servoing.raw_of_y = 0;
  visual_servoing.of_y = 0;
  visual_servoing.raw_of_y_d = 0;
  visual_servoing.of_y_d = 0;
  visual_servoing.prev_box_centroid_y = 0;
  visual_servoing.prev_of_y = 0;
  visual_servoing.prev_raw_of_y = 0;

  //kalman filter
  visual_servoing.kf_x1 = 0.0f;
  visual_servoing.kf_x2 = 0.0f;
  visual_servoing.kf_x1_pred = 0.0f;
  visual_servoing.kf_x2_pred = 0.0f;

  visual_servoing.kf_p11 = 1.0f;
  visual_servoing.kf_p12 = 0.0f;
  visual_servoing.kf_p21 = 0.0f;
  visual_servoing.kf_p22 = 1.0f;

  visual_servoing.kf_q11 = 0.001f;
  visual_servoing.kf_q22 = 0.05f;

  visual_servoing.kf_r11 = 0.0050f;
  visual_servoing.kf_r22 = 0.5000f;

  visual_servoing.kf_initialized = FALSE;
  visual_servoing.lp_of_b0 = 0.46515307f;
  visual_servoing.lp_of_b1 = 0.93030615f;
  visual_servoing.lp_of_b2 = 0.46515307f;
  visual_servoing.lp_of_a1 = 0.6202041028f;
  visual_servoing.lp_of_a2 = 0.240408205f;

  visual_servoing.yaw_vel = 0.0f;

  vs_enable_time = (float)get_sys_time_usec() / 1e6;
  landing = FALSE;
  switch_time_start = 0.0;
  switch_time_end = 0.0;
  visual_servoing.pitch_sum = 0;
  switch_distance = 10;
  switching = FALSE;
  first_interation = FALSE;
  reset_switch_time_end_bool = FALSE;
  switch_count = 0;

  /*
   * Capture the current hover reference, heading and trim only
   * after all legacy controller states have been reset.
   */
  visual_servoing_capture_reference();
}

static float visual_servoing_slew(float current, float target, float rate_limit, float dt)
{
  /*
   * Return the current command if timing or input is invalid.
   */
  if (!isfinite(current) || !isfinite(target) || !isfinite(rate_limit) || !isfinite(dt) || rate_limit <= 0.0f || dt <= 0.0f) {
    return current;
  }

  float delta = target - current;

  const float max_delta = rate_limit * dt;

  Bound(
    delta,
    -max_delta,
    max_delta
  );

  return current + delta;
}

static void visual_servoing_reset_of_state(void)
{
  visual_servoing.raw_of_y = 0.0f;
  visual_servoing.of_y = 0.0f;

  visual_servoing.raw_of_y_d = 0.0f;
  visual_servoing.of_y_d = 0.0f;

  visual_servoing.prev_raw_of_y = 0.0f;
  visual_servoing.prev_of_y = 0.0f;

  visual_servoing.kf_x1 = 0.0f;
  visual_servoing.kf_x2 = 0.0f;

  visual_servoing.kf_x1_pred = 0.0f;
  visual_servoing.kf_x2_pred = 0.0f;

  visual_servoing.kf_initialized = false;

  visual_servoing.have_previous_valid_target = false;
  visual_servoing.vision_valid_streak = 0;
  visual_servoing.of_ready = false;

  visual_servoing.using_of_control = false;
}

static void visual_servoing_capture_reference(void)
{
  const struct NedCoor_f *position = stateGetPositionNed_f();

  /*
   * Capture the previous standard-hover attitude setpoint.
   *
   * stab_att_sp_euler still contains the setpoint generated by
   * the ordinary horizontal hover controller immediately before
   * GUIDANCE_H_MODE_MODULE is entered.
   */
  visual_servoing.heading_ref = ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.psi);

  visual_servoing.pitch_trim = ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.theta);

  visual_servoing.roll_trim = ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.phi);

  /*
   * Do not capture a large transient as permanent hover trim.
   *
   * Previous hover data showed trim of roughly one degree, so
   * a three-degree bound is conservative.
   */
  BoundAbs(
    visual_servoing.pitch_trim,
    RadOfDeg(3.0f)
  );

  BoundAbs(
    visual_servoing.roll_trim,
    RadOfDeg(3.0f)
  );

  /*
   * Define the experiment's fixed forward axis in NED.
   */
  visual_servoing.forward_axis_n = cosf(visual_servoing.heading_ref);

  visual_servoing.forward_axis_e = sinf(visual_servoing.heading_ref);

  /*
   * Project the current NED position onto the fixed entry-heading
   * forward direction.
   *
   * No corresponding lateral-position reference is captured.
   */
  visual_servoing.forward_position_ref =
      visual_servoing.forward_axis_n * position->x
    + visual_servoing.forward_axis_e * position->y;

  visual_servoing.forward_position = visual_servoing.forward_position_ref;

  visual_servoing.forward_velocity = 0.0f;
  visual_servoing.forward_error = 0.0f;
  visual_servoing.right_velocity = 0.0f;

  /*
   * Convert the captured attitude trim to horizontal acceleration
   * vector components.
   *
   * Later:
   *
   *   pitch = atan2(mu_x, g)
   *   roll  = atan2(mu_y, g)
   *
   * so g*tan(trim angle) reconstructs the old trim setpoint.
   */
  visual_servoing.mu_x_trim = 9.81f * tanf(visual_servoing.pitch_trim);

  visual_servoing.mu_y_trim = 9.81f * tanf(visual_servoing.roll_trim);

  visual_servoing.mu_x = visual_servoing.mu_x_trim;

  visual_servoing.mu_y = visual_servoing.mu_y_trim;

  /*
   * mu_z is diagnostic only after this modification.
   */
  visual_servoing.mu_z = 9.81f;

  pitch_sp = visual_servoing.pitch_trim;

  roll_sp = visual_servoing.roll_trim;

  /*
   * Reset control timing.
   */
  visual_servoing.last_control_time_us = get_sys_time_usec();

  visual_servoing.control_dt = 0.0f;

  /*
   * Ignore any image that arrived before module entry.
   *
   * The first image received after entry establishes a new visual
   * timing baseline.
   */
  visual_servoing.processed_vision_sequence = visual_servoing.vision_sequence;

  visual_servoing.previous_vision_stamp_us = 0;
  visual_servoing.last_valid_vision_rx_us = 0;

  visual_servoing.vision_new_frame = false;
  visual_servoing.vision_valid = false;
  visual_servoing.vision_dt = 0.0f;
  visual_servoing.vision_age = 1000.0f;

  visual_servoing.pose_ok = ins_ext_pose_is_ready() && ins_ext_pose_is_fresh();

  visual_servoing.forward_accel_cmd = 0.0f;

  visual_servoing.mu_x_control = 0.0f;
  visual_servoing.mu_y_of = 0.0f;
  visual_servoing.mu_y_yaw = 0.0f;
  visual_servoing.mu_y_fallback = 0.0f;
  visual_servoing.mu_y_control = 0.0f;

  visual_servoing.mu_x_target = visual_servoing.mu_x_trim;

  visual_servoing.mu_y_target = visual_servoing.mu_y_trim;

  visual_servoing_reset_of_state();
}

static void visual_servoing_process_new_vision_sample(uint32_t now_us)
{
  visual_servoing.vision_new_frame = false;

  /*
   * No new callback has arrived.
   */
  if (visual_servoing.vision_sequence == visual_servoing.processed_vision_sequence) {
    return;
  }

  /*
   * Claim this sample exactly once.
   */
  visual_servoing.processed_vision_sequence = visual_servoing.vision_sequence;

  visual_servoing.vision_new_frame = true;

  const uint32_t current_stamp_us = visual_servoing.vision_stamp_us;

  /*
   * The first frame after mode entry only establishes a timestamp
   * baseline. It cannot provide optic flow because no previous
   * post-entry target sample exists.
   */
  if (visual_servoing.previous_vision_stamp_us == 0) {
    visual_servoing.previous_vision_stamp_us = current_stamp_us;

    visual_servoing.vision_valid = false;

    visual_servoing_reset_of_state();

    return;
  }

  /*
   * Unsigned subtraction also behaves correctly through a normal
   * uint32 timestamp wrap.
   */
  const uint32_t frame_dt_us = current_stamp_us - visual_servoing.previous_vision_stamp_us;

  visual_servoing.previous_vision_stamp_us = current_stamp_us;

  visual_servoing.vision_dt = 1.0e-6f * (float)frame_dt_us;

  /*
   * Keep the legacy dt field synchronized for telemetry and
   * existing logger compatibility.
   */
  visual_servoing.dt = visual_servoing.vision_dt;

  const bool valid_dt =
      isfinite(visual_servoing.vision_dt)
    && visual_servoing.vision_dt >= visual_servoing.vision_min_dt
    && visual_servoing.vision_dt <= visual_servoing.vision_max_dt;

  if (valid_dt) {
    fps = 1.0f / visual_servoing.vision_dt;
  } else {
    fps = 0.0f;
  }

  /*
   * Reuse the existing configurable CC threshold.
   *
   * Do not assume that the apparent 0-7 values in the old log
   * represent the true detector quality; that CSV had a formatting
   * mismatch. Calibrate CC_THRESHOLD using corrected shadow-mode
   * logging.
   */
  const bool valid_target = valid_dt
    && isfinite(visual_servoing.box_centroid_y)
    && visual_servoing.color_count >= visual_servoing.color_count_threshold;

  if (!valid_target) {
    visual_servoing.vision_valid = false;

    visual_servoing_reset_of_state();

    return;
  }

  visual_servoing.vision_valid = true;

  /*
   * Use onboard time to monitor how long control has gone without
   * a valid accepted target sample.
   */
  visual_servoing.last_valid_vision_rx_us = now_us;

  /*
   * First valid target after startup or target loss:
   * establish centroid history without generating a false OF spike.
   */
  if (!visual_servoing.have_previous_valid_target) {
    visual_servoing.prev_box_centroid_y = visual_servoing.box_centroid_y;

    visual_servoing.prev_raw_of_y = 0.0f;
    visual_servoing.prev_of_y = 0.0f;

    visual_servoing_kf_init(
      0.0f,
      0.0f
    );

    visual_servoing.have_previous_valid_target = true;

    visual_servoing.vision_valid_streak = 1;
    visual_servoing.of_ready = false;

    return;
  }

  /*
   * Centroid optic flow in pixels per second.
   */
  visual_servoing.raw_of_y =
    (visual_servoing.box_centroid_y - visual_servoing.prev_box_centroid_y) / visual_servoing.vision_dt;

  /*
   * OF derivative is retained for logging and the existing
   * two-state Kalman filter, but it is not used directly by the
   * lateral controller.
   */
  visual_servoing.raw_of_y_d =
    (visual_servoing.raw_of_y - visual_servoing.prev_raw_of_y) / visual_servoing.vision_dt;

  /*
   * Reject non-finite measurements before they reach the filter.
   */
  if (!isfinite(visual_servoing.raw_of_y) || !isfinite(visual_servoing.raw_of_y_d)) {
    visual_servoing.vision_valid = false;

    visual_servoing_reset_of_state();

    return;
  }

  visual_servoing_kf_update(
    visual_servoing.raw_of_y,
    visual_servoing.raw_of_y_d,
    visual_servoing.vision_dt
  );

  visual_servoing.of_y = visual_servoing.kf_x1;

  visual_servoing.of_y_d = visual_servoing.kf_x2;

  visual_servoing.prev_box_centroid_y = visual_servoing.box_centroid_y;

  visual_servoing.prev_raw_of_y = visual_servoing.raw_of_y;

  visual_servoing.prev_of_y = visual_servoing.of_y;

  if (visual_servoing.vision_valid_streak < 255) {
    visual_servoing.vision_valid_streak++;
  }

  /*
   * Require several consecutive healthy frames before roll is
   * actuated from optic flow.
   */
  visual_servoing.of_ready = visual_servoing.vision_valid_streak >= visual_servoing.vision_min_streak;
}


void visual_servoing_module_run(bool in_flight)
{
  const uint32_t now_us = get_sys_time_usec();

  const struct NedCoor_f *position = stateGetPositionNed_f();

  const struct NedCoor_f *speed = stateGetSpeedNed_f();

  const struct FloatRates *rates = stateGetBodyRates_f();

  /*
   * ============================================================
   * 1. Calculate guidance-loop timing
   * ============================================================
   */

  const uint32_t control_dt_us = now_us - visual_servoing.last_control_time_us;

  visual_servoing.last_control_time_us = now_us;

  float control_dt = 1.0e-6f * (float)control_dt_us;

  /*
   * Reject startup, stall or otherwise unreasonable intervals.
   *
   * 0.002 s is approximately the expected 512 Hz control interval.
   */
  if (!isfinite(control_dt) || control_dt <= 0.0005f || control_dt > 0.05f) {
    control_dt = 0.002f;
  }

  visual_servoing.control_dt = control_dt;

  /*
   * ============================================================
   * 2. Process a genuinely new vision sample
   * ============================================================
   */

  visual_servoing_process_new_vision_sample(now_us);

  /*
   * Calculate age of the latest accepted target sample.
   */
  if (visual_servoing.last_valid_vision_rx_us == 0) {
    visual_servoing.vision_age = 1000.0f;
  } else {
    visual_servoing.vision_age = 1.0e-6f * (float)(now_us - visual_servoing.last_valid_vision_rx_us);
  }

  /*
   * Clear stale optic flow.
   */
  if (visual_servoing.vision_age > visual_servoing.vision_timeout) {
    visual_servoing.vision_valid = false;

    visual_servoing_reset_of_state();
  }

  /*
   * ============================================================
   * 3. Check external-pose health
   * ============================================================
   */

  visual_servoing.pose_ok = ins_ext_pose_is_ready() && ins_ext_pose_is_fresh();

  /*
   * Optic-flow control is not allowed to remain active when the
   * position/attitude estimator is stale.
   */
  if (!visual_servoing.pose_ok) {
    visual_servoing_reset_of_state();
  }

  /*
   * ============================================================
   * 4. Express motion in the fixed entry-heading frame
   * ============================================================
   */

  const float heading_cos = visual_servoing.forward_axis_n;

  const float heading_sin = visual_servoing.forward_axis_e;

  /*
   * Forward position along the heading captured at mode entry:
   *
   *   s_f = cos(psi_ref) * north
   *       + sin(psi_ref) * east
   */
  visual_servoing.forward_position = heading_cos * position->x + heading_sin * position->y;

  /*
   * Forward velocity along the same fixed axis.
   */
  visual_servoing.forward_velocity = heading_cos * speed->x + heading_sin * speed->y;

  /*
   * Rightward velocity:
   *
   *   v_right = -sin(psi_ref) * v_north
   *             +cos(psi_ref) * v_east
   */
  visual_servoing.right_velocity = -heading_sin * speed->x + heading_cos * speed->y;

  /*
   * ============================================================
   * 5. Forward position + velocity hold
   * ============================================================
   */

  visual_servoing.forward_error = visual_servoing.forward_position_ref - visual_servoing.forward_position;

  if (visual_servoing.pose_ok) {
    visual_servoing.forward_accel_cmd =
        visual_servoing.fwd_kp * visual_servoing.forward_error
        - visual_servoing.fwd_kd * visual_servoing.forward_velocity;
  } else {
    /*
     * Do not continue position correction from a stale estimate.
     */
    visual_servoing.forward_accel_cmd = 0.0f;
  }

  Bound(
    visual_servoing.forward_accel_cmd,
    -visual_servoing.fwd_max_accel,
     visual_servoing.fwd_max_accel
  );

  /*
   * Paparazzi/Bebop sign convention:
   *
   * negative pitch commands forward acceleration.
   *
   * Therefore a positive desired physical forward acceleration
   * becomes a negative mu_x correction.
   */
  visual_servoing.mu_x_control = -visual_servoing.forward_accel_cmd;

  visual_servoing.mu_x_target = visual_servoing.mu_x_trim + visual_servoing.mu_x_control;

  /*
   * Smoothly transition from standard-hover trim to the new
   * forward-controller output.
   */
  visual_servoing.mu_x =
    visual_servoing_slew(
      visual_servoing.mu_x,
      visual_servoing.mu_x_target,
      visual_servoing.accel_slew,
      control_dt
    );

  /*
   * Preserve legacy forward diagnostics where useful.
   */
  visual_servoing.vel_x = -visual_servoing.forward_velocity;

  visual_servoing.err_vx = -visual_servoing.forward_velocity;

  visual_servoing.mu_vx_ff = 0.0f;

  /*
   * ============================================================
   * 6. Lateral optic-flow + yaw-rate controller
   * ============================================================
   */

  visual_servoing.yaw_vel = rates->r;

  visual_servoing.of_scaled = visual_servoing.of_scale * visual_servoing.of_y;

  visual_servoing.mu_y_of = 0.0f;
  visual_servoing.mu_y_yaw = 0.0f;
  visual_servoing.mu_y_fallback = 0.0f;

  visual_servoing.using_of_control = false;
  visual_servoing.using_lateral_fallback = false;

  if (visual_servoing.pose_ok && visual_servoing.of_ready) {
    /*
     * Preserve the experimentally derived controller structure:
     *
     *   mu_y =
     *       -K_OF  * scaled_OF
     *       +K_YAW * yaw_rate
     *
     * There is deliberately no world-frame lateral-position term.
     */
    visual_servoing.mu_y_of = -visual_servoing.ol_y_OF_gain * visual_servoing.of_scaled;

    visual_servoing.mu_y_yaw = visual_servoing.ol_y_YAW_gain * visual_servoing.yaw_vel;

    visual_servoing.mu_y_control = visual_servoing.mu_y_of + visual_servoing.mu_y_yaw;

    visual_servoing.using_of_control = true;
  }
  else if (visual_servoing.pose_ok) {
    /*
     * Target-loss/startup fallback:
     *
     * only damp lateral velocity. Do not hold an absolute lateral
     * position, because that would conflict with flower following.
     *
     * Any interval using this fallback must be marked invalid for
     * the optic-flow experiment.
     */
    visual_servoing.mu_y_fallback = -visual_servoing.lat_fallback_kd * visual_servoing.right_velocity;

    visual_servoing.mu_y_control = visual_servoing.mu_y_fallback;

    visual_servoing.using_lateral_fallback = true;
  }
  else {
    /*
     * Pose is stale: return toward captured trim without trusting
     * position or velocity feedback.
     */
    visual_servoing.mu_y_control = 0.0f;
  }

  Bound(
    visual_servoing.mu_y_control,
    -visual_servoing.lat_max_accel,
     visual_servoing.lat_max_accel
  );

  visual_servoing.mu_y_target = visual_servoing.mu_y_trim + visual_servoing.mu_y_control;

  visual_servoing.mu_y =
    visual_servoing_slew(
      visual_servoing.mu_y,
      visual_servoing.mu_y_target,
      visual_servoing.accel_slew,
      control_dt
    );

  /*
   * ============================================================
   * 7. Vertical diagnostics only
   * ============================================================
   *
   * The visual module no longer controls thrust.
   */

  const float height = -position->z;

  /*
   * Standard guidance_v stores z in NED, where positive z is down.
   */
  visual_servoing.height_setpoint = -POS_FLOAT_OF_BFP(guidance_v_z_sp);

  visual_servoing.height_error = visual_servoing.height_setpoint - height;

  /*
   * Retain mu_z only as a diagnostic nominal-gravity value.
   */
  visual_servoing.mu_z = 9.81f;

  /*
   * Record the command already produced by guidance_v.
   *
   * Do not write it here.
   */
  thrust_set = (float)stabilization_cmd[COMMAND_THRUST];

  /*
   * ============================================================
   * 8. Convert horizontal acceleration vectors to attitude
   * ============================================================
   */

  pitch_sp = atan2f(visual_servoing.mu_x, 9.81f);

  roll_sp = atan2f(visual_servoing.mu_y, 9.81f);

  /*
   * The original code allowed 80 degrees of roll.
   * That is inappropriate for the first indoor hover test.
   */
  BoundAbs(pitch_sp, RadOfDeg(visual_servoing.max_pitch_deg));

  BoundAbs(roll_sp, RadOfDeg(visual_servoing.max_roll_deg));

  visual_servoing.pitch_sp_cmd = pitch_sp;

  visual_servoing.roll_sp_cmd = roll_sp;

  visual_servoing.yaw_sp_cmd = visual_servoing.heading_ref;

  /*
   * ============================================================
   * 9. Send roll, pitch and fixed heading to INDI
   * ============================================================
   */

  struct Int32Eulers rpy = {
    .phi = 
    ANGLE_BFP_OF_REAL(roll_sp),

    .theta = 
    ANGLE_BFP_OF_REAL(pitch_sp),

    .psi =
      ANGLE_BFP_OF_REAL(visual_servoing.heading_ref)
  };

  stabilization_indi_set_rpy_setpoint_i(&rpy);

  stabilization_attitude_run(in_flight);

  /*
   * Deliberately absent:
   *
   *   stabilization_cmd[COMMAND_THRUST] = ...
   *
   * Standard guidance_v is the only thrust-command owner.
   */
}

/**
 * Updates the integral and differential errors for PID control and sets the previous error
 * @param[in] err: the error of the divergence and divergence setpoint
 * @param[in] dt:  time difference since last update
 */
void update_errors(float box_x_err, float box_y_err, float div_err, float dt)
{
  float lp_factor = dt / 0.02;
  Bound(lp_factor, 0.f, 1.f);

  // maintain the controller errors:
  // Error of box x coordinate
  visual_servoing.box_x_err_sum += box_x_err;
  visual_servoing.box_x_err_d = ((box_x_err - visual_servoing.previous_box_x_err) / dt);
  visual_servoing.previous_box_x_err = box_x_err;

  // Error of box y coordinate
  visual_servoing.box_y_err_sum += box_y_err;
  visual_servoing.box_y_err_d = ((box_y_err - visual_servoing.previous_box_y_err) / dt);
  visual_servoing.previous_box_y_err = box_y_err;

  // Error of divergence
  visual_servoing.div_err_sum += div_err;
}

#if 0
/**
 * Execute the final landing procedure to land in the box when a certain distance from the target is reached
 */
void final_land_in_box(float start_time)
{
  float c_time = (float)get_sys_time_usec() / 1e6;
  float d_time = c_time - start_time;
  // first 2 seconds accelerate forward
  if (d_time <= 3.0f){
    pitch_sp = -0.08;
    roll_sp = 0;
    thrust_set = visual_servoing.nominal_throttle * MAX_PPRZ * 0.99;
  }
  // then, 1 second descending
  if (3.0f < d_time && d_time <= 5.0f){
    pitch_sp = -0.03;
    roll_sp = 0;
    thrust_set = visual_servoing.nominal_throttle * MAX_PPRZ * 0.91;
  }
  // kill throttle
  if (d_time > 7.00f){
    autopilot_set_kill_throttle(true);
  }
}
  #endif

/**
 * Kalman filter for optic flow PD reconstruction 
 */

static void visual_servoing_kf_init(float of_meas, float ofd_meas)
{
  visual_servoing.kf_x1 = of_meas;
  visual_servoing.kf_x2 = ofd_meas;

  visual_servoing.kf_x1_pred = of_meas;
  visual_servoing.kf_x2_pred = ofd_meas;

  visual_servoing.kf_p11 = 1.0f;
  visual_servoing.kf_p12 = 0.0f;
  visual_servoing.kf_p21 = 0.0f;
  visual_servoing.kf_p22 = 1.0f;

  visual_servoing.kf_initialized = TRUE;
}

static void visual_servoing_kf_update(float of_meas, float ofd_meas, float dt)
{
  if (dt < 1e-5f) {
    return;
  }

  // State transition A = [1 dt; 0 1]
  const float A11 = 1.0f;
  const float A12 = dt;
  const float A21 = 0.0f;
  const float A22 = 1.0f;

  // Current state
  const float x1 = visual_servoing.kf_x1;
  const float x2 = visual_servoing.kf_x2;

  // Predict state
  const float x1_pred = A11 * x1 + A12 * x2;
  const float x2_pred = A21 * x1 + A22 * x2;

  visual_servoing.kf_x1_pred = x1_pred;
  visual_servoing.kf_x2_pred = x2_pred;

  // Predict covariance: P_pred = A P A' + Q
  const float p11 = visual_servoing.kf_p11;
  const float p12 = visual_servoing.kf_p12;
  const float p21 = visual_servoing.kf_p21;
  const float p22 = visual_servoing.kf_p22;

  const float ap11 = A11*p11 + A12*p21;
  const float ap12 = A11*p12 + A12*p22;
  const float ap21 = A21*p11 + A22*p21;
  const float ap22 = A21*p12 + A22*p22;

  float p11_pred = ap11*A11 + ap12*A12 + visual_servoing.kf_q11;
  float p12_pred = ap11*A21 + ap12*A22;
  float p21_pred = ap21*A11 + ap22*A12;
  float p22_pred = ap21*A21 + ap22*A22 + visual_servoing.kf_q22;

  // Measurement z = [of_meas; ofd_meas], H = I
  const float y1 = of_meas  - x1_pred;
  const float y2 = ofd_meas - x2_pred;

  // S = P_pred + R
  const float s11 = p11_pred + visual_servoing.kf_r11;
  const float s12 = p12_pred;
  const float s21 = p21_pred;
  const float s22 = p22_pred + visual_servoing.kf_r22;

  const float detS = s11*s22 - s12*s21;
  if (fabsf(detS) < 1e-12f) {
    // Fallback: keep prediction
    visual_servoing.kf_x1 = x1_pred;
    visual_servoing.kf_x2 = x2_pred;
    visual_servoing.kf_p11 = p11_pred;
    visual_servoing.kf_p12 = p12_pred;
    visual_servoing.kf_p21 = p21_pred;
    visual_servoing.kf_p22 = p22_pred;
    return;
  }

  // inv(S)
  const float invS11 =  s22 / detS;
  const float invS12 = -s12 / detS;
  const float invS21 = -s21 / detS;
  const float invS22 =  s11 / detS;

  // K = P_pred * inv(S)
  const float K11 = p11_pred*invS11 + p12_pred*invS21;
  const float K12 = p11_pred*invS12 + p12_pred*invS22;
  const float K21 = p21_pred*invS11 + p22_pred*invS21;
  const float K22 = p21_pred*invS12 + p22_pred*invS22;

  // Update state
  visual_servoing.kf_x1 = x1_pred + K11*y1 + K12*y2;
  visual_servoing.kf_x2 = x2_pred + K21*y1 + K22*y2;

  // Update covariance: P = (I - K) P_pred   since H = I
  const float IK11 = 1.0f - K11;
  const float IK12 =      - K12;
  const float IK21 =      - K21;
  const float IK22 = 1.0f - K22;

  visual_servoing.kf_p11 = IK11*p11_pred + IK12*p21_pred;
  visual_servoing.kf_p12 = IK11*p12_pred + IK12*p22_pred;
  visual_servoing.kf_p21 = IK21*p11_pred + IK22*p21_pred;
  visual_servoing.kf_p22 = IK21*p12_pred + IK22*p22_pred;
}

/**
 * Make a standardized step in divergence
 */
float divergence_step(float switch_time, float mag)
{
  float current_time = (float)get_sys_time_usec() / 1e6;
  float delta_time = current_time - switch_time;
  float accel_x;

// Ablation mode 0: normal visual servo approach, 1: increase wrt time / current measurement
  if (visual_servoing.ablation_mode == 0){
    // Initial acceleration is equal to the given magnitude
    if (delta_time < 0.5f){
      accel_x = -mag;
    }
    // Exponential decay from initial magnitude
    else {accel_x = -mag * expf(-visual_servoing.switch_decay * delta_time);}

    // Switch ends after 2.5 seconds or when divergence > 0.3
    float end = 1.7;
    if (delta_time >= end || visual_servoing.divergence > 0.5){
      float new_sp = visual_servoing.divergence;
      visual_servoing.delta_pixels = (sqrtf(visual_servoing.color_count) - sqrtf(start_color_count)) / delta_time;
      visual_servoing.divergence_sp = new_sp;
      visual_servoing.set_point = new_sp;
      visual_servoing.div_err = 0;
    // Non-adaptive integral control  
//      visual_servoing.div_err_sum = 0;
    // Adaptive integral feedforward term
      if (visual_servoing.divergence_sp < 0.1){
        visual_servoing.div_err_sum = 50;
      }
      else if (visual_servoing.divergence_sp < 0.25){
        visual_servoing.div_err_sum = 40;
      }
      else if (visual_servoing.divergence_sp < 0.4){
        visual_servoing.div_err_sum = 30;
      }
      else {visual_servoing.div_err_sum = 20;
      }

  //    visual_servoing.ol_x_pgain = 0.16 / (new_sp * new_sp);
      visual_servoing.ol_x_pgain = 0.80 / new_sp;
  //    visual_servoing.ol_x_pgain = 20.0;
      switch_distance = 0.09f* powf(-visual_servoing.pitch_sum, 0.483f) * powf(new_sp, -1.02f);
      switch_time_end = current_time;
      switching = FALSE;
    }
  }

  if(visual_servoing.ablation_mode == 1){
    accel_x = -0.2;
    if (switching || visual_servoing.divergence > 0.5){
//      float new_sp = visual_servoing.divergence_sp + 0.05;
//      float new_sp = visual_servoing.divergence;
      float new_sp = divsp_list[switch_count];
      switch_count += 1;

      visual_servoing.delta_pixels = (sqrtf(visual_servoing.color_count) - sqrtf(start_color_count)) / delta_time;
      visual_servoing.divergence_sp = new_sp;
      visual_servoing.set_point = new_sp;
      visual_servoing.div_err = 0;
    // Non-adaptive integral control  
//      visual_servoing.div_err_sum = 0;
    // Adaptive integral feedforward term
      if (visual_servoing.divergence_sp < 0.1){
        visual_servoing.div_err_sum = 50;
      }
      else if (visual_servoing.divergence_sp < 0.25){
        visual_servoing.div_err_sum = 40;
      }
      else if (visual_servoing.divergence_sp < 0.4){
        visual_servoing.div_err_sum = 30;
      }
      else {visual_servoing.div_err_sum = 20;
      }

  //    visual_servoing.ol_x_pgain = 0.16 / (new_sp * new_sp);
      visual_servoing.ol_x_pgain = 0.80 / new_sp;
  //    visual_servoing.ol_x_pgain = 20.0;
      switch_distance = 0.09f* powf(-visual_servoing.pitch_sum, 0.483f) * powf(new_sp, -1.02f);
      switch_time_end = current_time;
      switching = FALSE;
    }
  }

  return accel_x;
}

/** Function to reset switch_time_end as 0 at the first interation to enter the PID phase at the beginning
 * 
 */
float reset_switch_time_end(float switchTimeEnd){
  if (reset_switch_time_end_bool){
    if (first_interation){

      switchTimeEnd = (float)get_sys_time_usec() / 1e6;

      if (visual_servoing.divergence_sp < 0.1){
        visual_servoing.div_err_sum = 50;
      }
      else if (visual_servoing.divergence_sp < 0.25){
        visual_servoing.div_err_sum = 40;
      }
      else if (visual_servoing.divergence_sp < 0.4){
        visual_servoing.div_err_sum = 30;
      }
      else {visual_servoing.div_err_sum = 20;
      }

      visual_servoing.ol_x_pgain = 0.80 / VS_SET_POINT;

      first_interation = FALSE;
    }
  }

  return switchTimeEnd;
}

////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  visual_servoing_module_init();
}

void guidance_h_module_enter(void)
{
  reset_all_vars();
}

void guidance_h_module_read_rc(void)
{
  
}

void guidance_h_module_run(bool in_flight)
{
  // Call full inner-/outerloop / horizontal-/vertical controller:
  visual_servoing_module_run(in_flight);
}
