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
};

extern struct VisualServoing visual_servoing;

#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Implement own horizontal loop:
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_run(bool in_flight);
extern void guidance_h_module_read_rc(void);

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

#endif
