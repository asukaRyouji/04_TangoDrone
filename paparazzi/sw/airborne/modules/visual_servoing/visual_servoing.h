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

#ifndef VISUAL_SERVOING_H
#define VISUAL_SERVOING_H

#include <std.h>
#include "filters/low_pass_filter.h"

/*
 * ==============================================================
 * Visual-servo activation state machine
 * ==============================================================
 *
 * IDLE:
 *   No activation request is pending.
 *
 * SETTLE:
 *   Standard horizontal NAV control remains active while the
 *   aircraft velocities are checked.
 *
 * READY:
 *   All velocity conditions have remained valid for the required
 *   dwell time. A switch to AP_MODE_MODULE is being issued.
 *
 * ACTIVE:
 *   GUIDANCE_H_MODE_MODULE owns the horizontal controller.
 */
enum VisualServoingActivationState {
  VS_ACTIVATION_IDLE = 0,
  VS_ACTIVATION_SETTLE = 1,
  VS_ACTIVATION_READY = 2,
  VS_ACTIVATION_ACTIVE = 3
};

struct VisualServoing {
  float dt;
  float nominal_throttle;
  float mu_x;
  float mu_y;
  float mu_z;
  float divergence_sp;
  float set_point;
  float divergence;
  float raw_divergence;
  float true_divergence;
  float ol_x_pgain;  
  float ol_x_igain;                  
  float ol_y_OF_gain;
  float ol_y_YAW_gain;
  float ol_y_OA_gain;   
  float ol_z_pgain;                               
  float ol_z_dgain;
  float ol_z_pff;                             
  float box_centroid_x;                             
  float box_centroid_y;                             
  float previous_box_x_err;
  float box_x_err_sum;
  float box_x_err_d;
  float previous_box_y_err;
  float box_y_err_sum;
  float box_y_err_d;
  // z-axis position based controller
  float height_setpoint;
  float height_error;
  float prev_height_error;
  float div_err;
  float previous_div_err;
  float div_err_sum;
  float lp_const;
  float switch_magnitude;
  float distance_est;
  float switch_decay;
  float theta_offset;
  float div_cutoff_freq;
  float color_count;
  float delta_pixels;
  float pitch_sum;
  float manual_switching;
  float approach_mode;
  float ablation_mode;
  float new_set_point;
  float color_count_threshold;
  float p_output;
  float i_output;
  float pid_on;
  float time_since_last;
  // TangoDrone approach
  float vel_x;
  float vel_x_sp;
  float Kp_vx;
  float mu_vx_ff;
  float err_vx;
  // Optic flow PD calculation
  float raw_of_y;
  float of_y;
  float raw_of_y_d;
  float of_y_d;
  float prev_box_centroid_y;
  float prev_of_y;
  float prev_raw_of_y;
  // Kalman filter
  float kf_x1;      // estimated OF
  float kf_x2;      // estimated OFD
  // One-step prediction
  float kf_x1_pred;
  float kf_x2_pred;
  // Covariance matrix P
  float kf_p11;
  float kf_p12;
  float kf_p21;
  float kf_p22;
  // Process noise Q
  float kf_q11;
  float kf_q22;
  // Measurement noise R
  float kf_r11;     // OF measurement variance
  float kf_r22;     // OFD measurement variance
  // Init flag
  bool kf_initialized;
  // Low pass 15 Hz and 5 hz
  float lp_of_b0;
  float lp_of_b1;
  float lp_of_b2;
  float lp_of_a1;
  float lp_of_a2;
  // angular rate 
  float yaw_vel;

    /*
   * ============================================================
   * Vision sample timing and validity
   * ============================================================
   */

  /*
   * Original integer timestamp received from VISUAL_DETECTION.
   *
   * Do not convert this timestamp to float inside the callback.
   * Keeping it as uint32_t avoids loss of timestamp precision.
   */
  uint32_t vision_stamp_us;

  /*
   * Incremented exactly once for every visual-detection callback.
   *
   * The guidance loop compares vision_sequence with
   * processed_vision_sequence to determine whether a genuinely
   * new camera sample has arrived.
   */
  uint32_t vision_sequence;
  uint32_t processed_vision_sequence;

  /*
   * Timestamp of the previously processed visual frame.
   */
  uint32_t previous_vision_stamp_us;

  /*
   * Onboard time when the latest valid target measurement was
   * accepted.
   */
  uint32_t last_valid_vision_rx_us;

  /*
   * Previous execution time of the horizontal controller.
   *
   * This is independent of the camera interval.
   */
  uint32_t last_control_time_us;

  bool vision_new_frame;
  bool vision_valid;
  bool have_previous_valid_target;
  bool of_ready;

  /*
   * True only when the external-pose EKF has been initialized
   * and its latest successfully fused measurement is fresh.
   */
  bool pose_ok;
  
    /*
   * ============================================================
   * Automatic settle-before-activation state
   * ============================================================
   */

  /*
   * Current value from enum VisualServoingActivationState.
   */
  uint8_t activation_state;

  /*
   * True after the operator requests visual-servo activation and
   * before the automatic switch to MODULE is completed.
   */
  bool activation_requested;

  /*
   * True during the current observer cycle only when all entry
   * conditions are satisfied.
   */
  bool settle_condition;

  /*
   * Latched true when settle_condition has remained continuously
   * true for settle_dwell_time.
   */
  bool settle_ready;

  /*
   * Prevent repeated AP_MODE_MODULE requests after readiness has
   * already triggered the transition.
   */
  bool mode_switch_issued;

  /*
   * Onboard timestamp at which the current uninterrupted settled
   * interval began.
   */
  uint32_t settle_condition_start_us;

  /*
   * Duration for which all conditions have remained continuously
   * satisfied.
   */
  float settle_elapsed;

  /*
   * Raw body-aligned velocities used for diagnostics.
   */
  float settle_forward_velocity;
  float settle_right_velocity;
  float settle_vertical_velocity;

  /*
   * Low-pass-filtered velocities used by the activation gate.
   */
  float settle_forward_velocity_filtered;
  float settle_right_velocity_filtered;
  float settle_vertical_velocity_filtered;

  /*
   * Internal state of the settle-velocity low-pass filter.
   *
   * settle_filter_last_time_us:
   *   timestamp of the previous filter update.
   *
   * settle_filter_initialized:
   *   false until the first valid velocity sample initializes the
   *   filter directly, avoiding a startup transient from zero.
   */
  uint32_t settle_filter_last_time_us;
  bool settle_filter_initialized;

    /*
   * ============================================================
   * Settle position gate
   * ============================================================
   */

  /*
   * Fixed NAV reference captured when VS_SETTLE is requested.
   *
   * Horizontal coordinates are NED, in metres.
   *
   * IMPORTANT:
   * These are copied from guidance_h.sp.pos, i.e. the waypoint
   * that INIT2 is already holding. They are NOT copied from the
   * instantaneous aircraft position.
   */
  float settle_ref_n;
  float settle_ref_e;

  /*
   * Vertical reference expressed as positive height above the
   * NED origin.
   *
   * guidance_v_z_sp itself is NED-z, therefore negative above
   * the origin. We convert it to positive height when captured.
   */
  float settle_ref_height;

  /*
   * Measured position errors used by the activation gate.
   *
   * horizontal_position_error:
   *   radial horizontal distance from the captured INIT2 target.
   *
   * vertical_position_error:
   *   current height - requested height.
   *   Positive means the aircraft is too high.
   */
  float settle_horizontal_position_error;
  float settle_vertical_position_error;

  /*
   * Maximum permitted position errors before MODULE activation.
   */
  float settle_horizontal_pos_max;
  float settle_vertical_pos_max;

  /*
   * Diagnostics.
   */
  bool settle_velocity_ok;
  bool settle_position_ok;
  bool settle_horizontal_position_ok;
  bool settle_vertical_position_ok;

  /*
   * Configurable readiness thresholds.
   */
  float settle_fwd_speed_max;
  float settle_right_speed_max;
  float settle_raw_right_speed_max;
  float settle_vertical_speed_max;
  float settle_dwell_time;

  float settle_right_position_error;
  float settle_right_pos_max;
  bool settle_right_position_ok;

  /*
  * External-pose loss duration.
  *
  * Used to distinguish a brief timing dropout from a sustained
  * localization failure.
  */
  uint32_t pose_loss_start_us;
  float pose_loss_elapsed;
  bool pose_loss_too_long;

  /*
   * Indicates which lateral branch is currently active.
   */
  bool using_of_control;
  bool using_lateral_fallback;

  uint8_t vision_valid_streak;
  uint8_t vision_min_streak;

  /*
   * Actual camera interval and age of the latest valid target.
   */
  float vision_dt;
  float vision_age;

  /*
   * Validity thresholds for camera samples.
   */
  float vision_min_dt;
  float vision_max_dt;
  float vision_timeout;

  /*
   * Actual guidance-loop interval.
   */
  float control_dt;

  /*
   * Scaling applied to optic flow before multiplying by gain.
   *
   * This preserves the 0.005 scaling used in the earlier
   * sideways-following experiments.
   */
  float of_scale;

  /*
   * ============================================================
   * Reference frame captured when visual mode is entered
   * ============================================================
   */

  /*
   * Fixed heading reference at visual-servo activation.
   */
  float heading_ref;

  /*
   * Unit vector of the entry-heading forward direction in NED:
   *
   *   forward_n = cos(heading_ref)
   *   forward_e = sin(heading_ref)
   */
  float forward_axis_n;
  float forward_axis_e;

  /*
   * Position along the entry-heading forward direction.
   *
   * Only this axis is position-controlled. No lateral world
   * position reference is stored.
   */
  float forward_position_ref;
  float forward_position;
  float forward_velocity;
  float forward_error;

  /*
   * Velocity along the entry-heading rightward direction.
   * Used only for a target-loss safety fallback.
   */
  float right_velocity;

  /*
   * ============================================================
   * Hover trim captured when visual mode is entered
   * ============================================================
   */


  /*
   * ============================================================
   * NAV-derived roll-trim estimator
   * ============================================================
   *
   * While VS_SETTLE is active and all position/velocity entry
   * conditions are continuously satisfied, sample the roll bias
   * learned by the normal NAV horizontal integrator.
   *
   * At MODULE entry the average is frozen and becomes roll_trim.
   */

  /*
   * Latest instantaneous NAV-derived body-roll trim sample [rad].
   */
  float roll_trim_sample;

  /*
   * Sum and mean of all valid samples from the current
   * uninterrupted settled interval.
   */
  float roll_trim_sum;
  float roll_trim_average;

  /*
   * Number of valid samples accumulated.
   */
  uint32_t roll_trim_sample_count;

  /*
   * Time at which the current trim averaging interval started.
   */
  uint32_t roll_trim_avg_start_us;

  /*
   * Duration of current uninterrupted trim averaging interval [s].
   */
  float roll_trim_avg_elapsed;

  /*
   * True only after sufficient averaging time and sufficient
   * valid samples have been collected.
   */
  bool roll_trim_average_valid;

  /*
   * The standard horizontal hover controller normally requires
   * small nonzero roll/pitch setpoints to reject vehicle,
   * payload and aerodynamic asymmetries.
   */
  float pitch_trim;
  float roll_trim;

  /*
   * Equivalent horizontal acceleration-vector trim terms.
   */
  float mu_x_trim;
  float mu_y_trim;

  /*
  * ============================================================
  * Forward-position PID controller parameters
  * ============================================================
  */

  /*
  * Physical-unit controller:
  *
  *   a_forward =
  *       fwd_kp * forward_error
  *     - fwd_kd * forward_velocity
  *     + fwd_ki * integral(forward_error)
  *
  * Units:
  *
  *   fwd_kp:  1/s^2
  *   fwd_kd:  1/s
  *   fwd_ki:  1/s^3
  *
  * The output is physical forward acceleration in m/s^2.
  */
  float fwd_kp;
  float fwd_kd;
  float fwd_ki;

  /*
  * Maximum absolute forward acceleration command generated by the
  * complete P + D + I controller.
  */
  float fwd_max_accel;

  /*
  * Maximum absolute acceleration contribution from the I term.
  *
  * Limiting the I contribution separately prevents a large stored
  * integral from dominating the controller after a disturbance.
  */
  float fwd_i_max_accel;
  /*
   * ============================================================
   * Lateral controller and safety parameters
   * ============================================================
   */

  float lat_max_accel;
  float lat_fallback_kd;

  /*
   * Maximum rate of change of mu_x and mu_y in m/s^3.
   */
  float accel_slew;

  /*
   * Initial attitude-command limits.
   */
  float max_pitch_deg;
  float max_roll_deg;

  /*
  * ============================================================
  * Controller diagnostics
  * ============================================================
  */

  /*
  * Time integral of forward position error.
  *
  * Units:
  *
  *   forward_error:          m
  *   forward_error_integral: m*s
  */
  float forward_error_integral;

  /*
  * Individual physical forward-acceleration contributions.
  *
  * All are expressed in m/s^2.
  */
  float forward_p_cmd;
  float forward_d_cmd;
  float forward_i_cmd;

  /*
  * PID output before and after the total acceleration bound.
  */
  float forward_accel_unbounded;
  float forward_accel_cmd;

  /*
  * Diagnostic flags:
  *
  * forward_i_limited:
  *   the I contribution has reached fwd_i_max_accel.
  *
  * forward_accel_saturated:
  *   the complete PID output has reached fwd_max_accel.
  */
  bool forward_i_limited;
  bool forward_accel_saturated;

  float of_scaled;
  float mu_x_control;

  float mu_y_of;
  float mu_y_yaw;
  float mu_y_fallback;
  float mu_y_control;

  float mu_x_target;
  float mu_y_target;

  float pitch_sp_cmd;
  float roll_sp_cmd;
  float yaw_sp_cmd;
};

extern struct VisualServoing visual_servoing;

#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_NAV

/*
 * ==============================================================
 * Settle-before-activation public interface
 * ==============================================================
 */

/*
 * Start the automatic settle procedure.
 *
 * This function only arms the request. It does not immediately
 * switch from NAV to MODULE.
 */
extern void visual_servoing_request_start(void);

/*
 * Cancel a pending activation request and reset the settle timer.
 *
 * The caller is still responsible for selecting AP_MODE_NAV.
 */
extern void visual_servoing_cancel_request(void);

/*
 * Return true after the settle conditions have remained valid for
 * the configured dwell time.
 */
extern bool visual_servoing_is_ready(void);

/*
 * Return true when GUIDANCE_H_MODE_MODULE currently owns horizontal
 * guidance.
 */
extern bool visual_servoing_is_active(void);

/*
 * Always-running vision observer used for shadow-mode logging.
 * It never writes roll, pitch, yaw or thrust commands.
 */
extern void visual_servoing_observer_periodic(void);

extern bool visual_servoing_pose_loss_too_long(void);

// Implement own horizontal loop:
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_run(bool in_flight);
extern void guidance_h_module_read_rc(void);

#endif
