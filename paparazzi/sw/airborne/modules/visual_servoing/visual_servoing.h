/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/visual_servoing/visual_servoing.c"
 * @author Sander Hazelaar
 * This module is used for the thesis project: "Adaptive Visual Servoing Control for Quadrotors: A Bio-inspired Strategy Using Active Vision"
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
  float ol_y_pgain;   
  float ol_z_pgain;                 
  float ol_y_dgain;               
  float ol_z_dgain;
  float ol_x_igain;                             
  float box_centroid_x;                             
  float box_centroid_y;                             
  float previous_box_x_err;
  float box_x_err_sum;
  float box_x_err_d;
  float previous_box_y_err;
  float box_y_err_sum;
  float box_y_err_d;
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
