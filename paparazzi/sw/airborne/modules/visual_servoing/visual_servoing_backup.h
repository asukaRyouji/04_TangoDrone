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
   * Forward-position controller parameters
   * ============================================================
   */

  float fwd_kp;
  float fwd_kd;
  float fwd_max_accel;

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

  float forward_accel_cmd;

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
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER

// Implement own horizontal loop:
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_run(bool in_flight);
extern void guidance_h_module_read_rc(void);

#endif
