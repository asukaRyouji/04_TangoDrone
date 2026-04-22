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
#define VS_OL_Y_OF_GAIN 2.4
#endif

#ifndef VS_OL_Y_YAW_GAIN
#define VS_OL_Y_YAW_GAIN 2.4
#endif

#ifndef VS_OL_Z_PGAIN
#define VS_OL_Z_PGAIN 7.5
#endif

#ifndef VS_OL_Z_DGAIN
#define VS_OL_Z_DGAIN 0.0075
#endif

#ifndef VS_OL_Z_PFF
#define VS_OL_Z_PFF 10.0
#endif

#ifndef VS_OL_Z_SP
#define VS_OL_Z_SP 1.1
#endif

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

// define and initialise global variables
float fps = 0;
float end_time = 0;
float m_dt;
float pitch_sp = 0;
float roll_sp = 0;
float thrust_set = 0;
float vision_time, prev_vision_time;
float last_color_count = 1;
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
                               int16_t pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  vision_time = (float)stamp / 1e6;
  visual_servoing.color_count = (float)quality;
  visual_servoing.box_centroid_x = (float)pixel_x;
  visual_servoing.box_centroid_y = (float)pixel_y;
}

// struct containing most relevant parameters
struct VisualServoing visual_servoing;

void visual_servoing_module_init(void);

void visual_servoing_module_enter(void);

void visual_servoing_module_run(bool in_flight);

static void update_errors(float box_x_err, float box_y_err, float div_err, float dt);

static void final_land_in_box(float start_time);

static float divergence_step(float switch_time, float magnitude);

static float reset_switch_time_end(float switch_time_end);

static void visual_servoing_kf_init(float of_meas, float ofd_meas);

static void visual_servoing_kf_update(float of_meas, float ofd_meas, float dt);

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
  visual_servoing.Kp_vx = 0.52;
  visual_servoing.mu_vx_ff = 0.0;
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
  visual_servoing.color_count = 0;                
  last_color_count = 1;
  visual_servoing.box_centroid_x = 0;
  visual_servoing.box_centroid_y = 0;
  set_point_count = 0;
  visual_servoing.divergence_sp = VS_SET_POINT;
  visual_servoing.p_output = 0;
  visual_servoing.i_output = 0;
  visual_servoing.pid_on = 0;
  visual_servoing.vel_x = 0;
  visual_servoing.vel_x_sp = VS_OL_X_VEL_SP;
  visual_servoing.Kp_vx = 0.52;
  visual_servoing.mu_vx_ff = 0.0;
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
}


void visual_servoing_module_run(bool in_flight)
{ 
  // Get the current state of the quadrotor
  struct FloatEulers *attitude = stateGetNedToBodyEulers_f();
  // attitude->theta += visual_servoing.theta_offset;
  struct NedCoor_f *position = stateGetPositionNed_f();
  struct NedCoor_f *speed = stateGetSpeedNed_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  // Integral of pitch angle
  visual_servoing.pitch_sum += attitude->theta;

  // Compute time step between previous image and current image
  visual_servoing.dt = vision_time - prev_vision_time;
  prev_vision_time = vision_time;

  // Initiate final landing maneuver
  // if (visual_servoing.color_count > 55000 && !landing){
  //   landing = TRUE;
  //   end_time = (float)get_sys_time_usec() / 1e6;
  // }

  // Calculate time after the last set-point switch
  float vs_time = (float)get_sys_time_usec() / 1e6;

  // Reset the switch_time_end at the beginning to start PID phase for the ablation tests
  // switch_time_end = reset_switch_time_end(switch_time_end);

  float time_since_last = vs_time - switch_time_end;
  
  visual_servoing.time_since_last = time_since_last;

  // Initiate divergence step 0.25 before 3 secs before
  if (visual_servoing.color_count != 0 && !switching && time_since_last > 3.0 && visual_servoing.divergence_sp < 0.25){
    switching = TRUE;
    switch_time_start = (float)get_sys_time_usec() / 1e6;
    visual_servoing.pitch_sum = 0;
    start_color_count = visual_servoing.color_count;
  }

  // When manual switching
  // if (visual_servoing.manual_switching == 1 && set_point_count == 0 && visual_servoing.color_count > visual_servoing.color_count_threshold){
  //  visual_servoing.set_point = visual_servoing.new_set_point;
  //  set_point_count += 1;
  // }
  
  // check if new measurement received
  if (visual_servoing.dt > 1e-5 && !landing){
    fps = 1/visual_servoing.dt;

    // Compute divergence
    if (last_color_count && visual_servoing.color_count != 0){
      float a1 = sqrt(last_color_count);
      float a2 = sqrt(visual_servoing.color_count);
      visual_servoing.raw_divergence = ((a2 - a1) / visual_servoing.dt) / a2;
    }
    else {visual_servoing.raw_divergence = visual_servoing.divergence;}

    // deal with (unlikely) fast changes in divergence:
    // static const float max_div_dt = 0.40f;
    // if (fabsf(visual_servoing.raw_divergence - visual_servoing.divergence) > max_div_dt) {
    //   if (visual_servoing.raw_divergence < visual_servoing.divergence) { visual_servoing.raw_divergence = visual_servoing.divergence - max_div_dt; }
    //   else { visual_servoing.raw_divergence = visual_servoing.divergence + max_div_dt; }
    // }

    // low-pass filter the divergence:
    Bound(visual_servoing.lp_const, 0.001f, 100.f);
    float lp_factor = visual_servoing.dt / (visual_servoing.lp_const / sqrt(visual_servoing.color_count));
    Bound(lp_factor, 0.f, 1.f);

    visual_servoing.divergence += (visual_servoing.raw_divergence - visual_servoing.divergence) * lp_factor;

    // Ground truth divergence from Optitrack
    true_distance = sqrtf(pow(4 - position->x, 2) + pow(0 - position->y, 2) + pow(0.7 + position->z, 2));
    visual_servoing.true_divergence = speed->x / true_distance;

    // 2 [1/s] ramp to setpoint
    // one error spoted, Sander updated the div_sp with setpoint instead of the error between div_sp and set_point
    float divergence_sp_err = visual_servoing.set_point - visual_servoing.divergence_sp;

    if (fabsf(divergence_sp_err) > 0.1*visual_servoing.dt){
      visual_servoing.divergence_sp += 0.1*visual_servoing.dt * divergence_sp_err / fabsf(divergence_sp_err);
    } else {
      visual_servoing.divergence_sp = visual_servoing.set_point;
    }

    visual_servoing.div_err = visual_servoing.divergence_sp - visual_servoing.divergence;

    // update control errors
    update_errors(visual_servoing.box_centroid_x, visual_servoing.box_centroid_y, visual_servoing.div_err, visual_servoing.dt);


    // Compute optic flow
    if (last_color_count && visual_servoing.color_count != 0){
      visual_servoing.raw_of_y = (visual_servoing.box_centroid_y - visual_servoing.prev_box_centroid_y) / visual_servoing.dt;
    }
    else {visual_servoing.raw_of_y = visual_servoing.of_y;}

    // Raw OF derivative from raw OF
    visual_servoing.raw_of_y_d =
      (visual_servoing.raw_of_y - visual_servoing.prev_raw_of_y) / visual_servoing.dt;

    // Initialize or update Kalman filter
    if (!visual_servoing.kf_initialized) {
      visual_servoing_kf_init(visual_servoing.raw_of_y, visual_servoing.raw_of_y_d);
    } else {
      visual_servoing_kf_update(visual_servoing.raw_of_y,
                                visual_servoing.raw_of_y_d,
                                visual_servoing.dt);
    }

    // Filtered estimates
    visual_servoing.of_y   = visual_servoing.kf_x1;
    visual_servoing.of_y_d = visual_servoing.kf_x2;
    
    // update for the next iteration
    visual_servoing.prev_box_centroid_y = visual_servoing.box_centroid_y;
    // Shift input history
    visual_servoing.prev_raw_of_y = visual_servoing.raw_of_y;

    // Shift output history
    visual_servoing.prev_of_y = visual_servoing.of_y;
  }


  // Extrapolate distance estimate
  visual_servoing.distance_est = switch_distance * expf(-visual_servoing.divergence_sp * time_since_last);

  // Compute desired inertial accelerations with PID

  // When setting approach mode to 1 this gives sin input to the forward acceleration to make Figure 10 of the paper
  if (visual_servoing.approach_mode == 1){
    visual_servoing.mu_x = 1 * (speed->x - (0.7 * sinf(2 * M_PI * vs_time * 0.9f) + 0.7));
  }

  else if (visual_servoing.approach_mode == 2){
   if(speed->x >= -0.5){
     visual_servoing.mu_x = -0.5;
   }
   else{
     visual_servoing.mu_x = 0.0;
   }
  }

  // approach mode 3 for optic flow based sideway controller
  else if (visual_servoing.approach_mode == 3){
    float ff_time = vs_time - vs_enable_time;
    visual_servoing.vel_x = speed->x;
    // 0.50 Hz dur 0.30 mag -0.08 Kp 0.52
    if (ff_time <= 0.30f){
      visual_servoing.mu_vx_ff = -0.08f;
    }
    else {
      visual_servoing.mu_vx_ff = 0.0f;
    }

    visual_servoing.mu_x = visual_servoing.mu_vx_ff + visual_servoing.Kp_vx * (visual_servoing.vel_x_sp - visual_servoing.vel_x);
    // visual_servoing.mu_x = 0.0;
    Bound(visual_servoing.mu_x, -0.4f, 0.4f);
  }

  else{
    if (!switching || visual_servoing.manual_switching){   
      visual_servoing.mu_x = - visual_servoing.ol_x_pgain * visual_servoing.div_err 
            - visual_servoing.ol_x_igain * visual_servoing.div_err_sum;
      visual_servoing.p_output = - visual_servoing.ol_x_pgain * visual_servoing.div_err;
      visual_servoing.i_output = - visual_servoing.ol_x_igain * visual_servoing.div_err_sum;
      visual_servoing.pid_on = 1;
    }
    else{
      visual_servoing.mu_x = divergence_step(switch_time_start, visual_servoing.switch_magnitude);
      visual_servoing.p_output = 0.0;
      visual_servoing.i_output = 0.0;
      visual_servoing.pid_on = 0;
    }
  }

  // Always control y and z with vision
  // visual_servoing.mu_y = - visual_servoing.ol_y_pgain * (visual_servoing.box_centroid_y - 2) - visual_servoing.ol_y_dgain * visual_servoing.box_y_err_d;
  float of_y_p_input = 0.005 * visual_servoing.of_y;
  float of_y_d_input = 0.005 * visual_servoing.of_y_d;
  visual_servoing.yaw_vel = rates->r;
  visual_servoing.mu_y = - visual_servoing.ol_y_OF_gain * of_y_p_input
                         + visual_servoing.ol_y_YAW_gain * visual_servoing.yaw_vel;
  Bound(visual_servoing.mu_y, -25.0f, 25.0f);
  // float freq = 1.0f;
  // float t_y = vs_time - vs_enable_time;
  // visual_servoing.mu_y = 0.3f * 4.0f * M_PI * M_PI * freq * freq * cosf(2* M_PI * t_y * freq);
  float height = -position->z;              // positive upward
  float height_error = visual_servoing.height_setpoint - height;
  float height_rate = -speed->z;            // positive upward climb rate

  visual_servoing.height_error = height_error;
  visual_servoing.mu_z = 9.81f + visual_servoing.ol_z_pgain * height_error - visual_servoing.ol_z_dgain * height_rate;


  if (!landing){
    // set the desired thrust
    float mass = (visual_servoing.nominal_throttle * MAX_PPRZ) / 9.81;
    thrust_set = sqrtf(pow(visual_servoing.mu_x, 2) + pow(visual_servoing.mu_y, 2) + pow(visual_servoing.mu_z, 2)) * mass;

    // set desired attitude angles
    pitch_sp = atan2f(visual_servoing.mu_x, visual_servoing.mu_z);
    roll_sp = asinf(mass * visual_servoing.mu_y/thrust_set);
    BoundAbs(pitch_sp, RadOfDeg(10.0));
    BoundAbs(roll_sp, RadOfDeg(60.0));
  }
  else{
    final_land_in_box(end_time);
  }
  
  // Give thrust command to autopilot
  if (in_flight) {
    Bound(thrust_set, 0.25 * guidance_v_nominal_throttle, MAX_PPRZ);
    stabilization_cmd[COMMAND_THRUST] = thrust_set;
  }

  float psi_cmd = attitude->psi;

  struct Int32Eulers rpy = { .phi = (int32_t)ANGLE_BFP_OF_REAL(roll_sp),
        .theta = (int32_t)ANGLE_BFP_OF_REAL(pitch_sp), .psi = (int32_t)ANGLE_BFP_OF_REAL(psi_cmd)
  };

  // set the desired roll pitch and yaw:
  stabilization_indi_set_rpy_setpoint_i(&rpy);
  // execute attitude stabilization:
  stabilization_attitude_run(in_flight);

  last_color_count = visual_servoing.color_count;
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

void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
}

// Implement own Vertical loops
void guidance_v_module_enter(void)
{
  // your code that should be executed when entering this vertical mode goes here
}

void guidance_v_module_run(UNUSED bool in_flight)
{
  // your vertical controller goes here
}
