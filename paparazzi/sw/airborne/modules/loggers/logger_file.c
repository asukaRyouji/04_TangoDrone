/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2019 Tom van Dijk <tomvand@users.noreply.github.com>
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
 *
 */

/** @file modules/loggers/logger_file.c
 *  @brief File logger for Linux based autopilots
 */

#include "logger_file.h"

#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "std.h"

#include "mcu_periph/sys_time.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/visual_servoing/visual_servoing.h"
#include "modules/computer_vision/cv_detect_color_object.h"

// For optitrack multi-object position logging by Chenyao
#include "modules/mission/moving_setup_logger_optitrack.h"

#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif

#include "generated/modules.h"

/** Set the default File logger path to the USB drive */
#ifndef LOGGER_FILE_PATH
#define LOGGER_FILE_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *logger_file = NULL;

/*
 * Logger timing and buffered-write diagnostics.
 */
static uint32_t logger_last_write_usec = 0;
static uint32_t logger_max_write_usec = 0;

static char logger_file_buffer[64 * 1024];

static void logger_file_write_camera_fps_header(FILE *file)
{
  fprintf(
    file,
    "cod_callback_count,"
    "cod_exec_time_us,"
    "cod_exec_time_max_us,"
  );
}

static void logger_file_write_camera_fps_row(FILE *file)
{
  fprintf(
    file,
    "%u,%u,%u,",
    (unsigned int)cod_callback_count,
    (unsigned int)cod_exec_time_us,
    (unsigned int)cod_exec_time_max_us
  );
}

static void logger_file_write_vs_activation_header(FILE *file)
{
  fprintf(
    file,
    "vs_activation_state,"
    "vs_activation_requested,"
    "vs_settle_condition,"
    "vs_settle_ready,"
    "vs_mode_switch_issued,"
    "vs_settle_elapsed,"
    "vs_settle_forward_vel,"
    "vs_settle_right_vel,"
    "vs_settle_vertical_vel,"
    "vs_settle_fwd_speed_max,"
    "vs_settle_right_speed_max,"
    "vs_settle_vertical_speed_max,"
    "vs_settle_dwell_time,"
    "vs_settle_ref_n,"
    "vs_settle_ref_e,"
    "vs_settle_ref_height,"
    "vs_settle_horizontal_pos_error,"
    "vs_settle_vertical_pos_error,"
    "vs_settle_horizontal_pos_max,"
    "vs_settle_vertical_pos_max,"
    "vs_settle_velocity_ok,"
    "vs_settle_position_ok,"
    "vs_settle_horizontal_pos_ok,"
    "vs_settle_vertical_pos_ok,"
  );
}

static void logger_file_write_vs_activation_row(FILE *file)
{
  fprintf(
    file,

    /*
     * Activation state.
     */
    "%u,%u,%u,%u,%u,"

    /*
     * Settle timing.
     */
    "%f,"

    /*
     * Raw settle velocities.
     */
    "%f,%f,%f,"

    /*
     * Velocity thresholds and dwell time.
     */
    "%f,%f,%f,%f,"

    /*
     * Captured NAV / vertical references.
     */
    "%f,%f,%f,"

    /*
     * Position errors.
     */
    "%f,%f,"

    /*
     * Position thresholds.
     */
    "%f,%f,"

    /*
     * Gate diagnostics.
     */
    "%u,%u,%u,%u,",

    /*
     * ----------------------------------------------------------
     * Activation state
     * ----------------------------------------------------------
     */

    (unsigned int)
      visual_servoing.activation_state,

    visual_servoing.activation_requested
      ? 1U : 0U,

    visual_servoing.settle_condition
      ? 1U : 0U,

    visual_servoing.settle_ready
      ? 1U : 0U,

    visual_servoing.mode_switch_issued
      ? 1U : 0U,

    /*
     * ----------------------------------------------------------
     * Settle timing
     * ----------------------------------------------------------
     */

    visual_servoing.settle_elapsed,

    /*
     * ----------------------------------------------------------
     * Raw body-aligned velocities
     * ----------------------------------------------------------
     */

    visual_servoing.settle_forward_velocity,
    visual_servoing.settle_right_velocity,
    visual_servoing.settle_vertical_velocity,

    /*
     * ----------------------------------------------------------
     * Velocity limits + dwell
     * ----------------------------------------------------------
     */

    visual_servoing.settle_fwd_speed_max,
    visual_servoing.settle_right_speed_max,
    visual_servoing.settle_vertical_speed_max,
    visual_servoing.settle_dwell_time,

    /*
     * ----------------------------------------------------------
     * Captured entry references
     * ----------------------------------------------------------
     */

    visual_servoing.settle_ref_n,
    visual_servoing.settle_ref_e,
    visual_servoing.settle_ref_height,

    /*
     * ----------------------------------------------------------
     * Position errors
     * ----------------------------------------------------------
     */

    visual_servoing.settle_horizontal_position_error,
    visual_servoing.settle_vertical_position_error,

    /*
     * ----------------------------------------------------------
     * Position thresholds
     * ----------------------------------------------------------
     */

    visual_servoing.settle_horizontal_pos_max,
    visual_servoing.settle_vertical_pos_max,

    /*
     * ----------------------------------------------------------
     * Individual gate status
     * ----------------------------------------------------------
     */

    visual_servoing.settle_velocity_ok
      ? 1U : 0U,

    visual_servoing.settle_position_ok
      ? 1U : 0U,

    visual_servoing.settle_horizontal_position_ok
      ? 1U : 0U,

    visual_servoing.settle_vertical_position_ok
      ? 1U : 0U
  );
}
/*
 * Visual-servo diagnostic fields.
 *
 * Keep the header and row in dedicated helper functions so that the
 * large visual-servo block cannot silently become misaligned.
 */
static void logger_file_write_visual_header(FILE *file)
{
  fprintf(
    file,
    "vs_vision_stamp_us,"
    "vs_vision_sequence,"
    "vs_processed_sequence,"
    "vs_new_frame,"
    "vs_vision_valid,"
    "vs_valid_streak,"
    "vs_of_ready,"
    "vs_vision_dt,"
    "vs_vision_age,"
    "vs_control_dt,"
    "vs_color_count,"
    "vs_centroid_x,"
    "vs_centroid_y,"
    "vs_raw_of_y,"
    "vs_of_y,"
    "vs_raw_of_y_d,"
    "vs_of_y_d,"
    "vs_yaw_vel,"
    "vs_pose_ok,"
    "vs_using_of,"
    "vs_using_fallback,"
    "vs_heading_ref,"
    "vs_forward_axis_n,"
    "vs_forward_axis_e,"
    "vs_forward_ref,"
    "vs_forward_pos,"
    "vs_forward_vel,"
    "vs_forward_error,"
    "vs_right_vel,"
    "vs_pitch_trim,"
    "vs_roll_trim,"
    "vs_mu_x_trim,"
    "vs_mu_y_trim,"
    "vs_fwd_kp,"
    "vs_fwd_kd,"
    "vs_of_gain,"
    "vs_yaw_gain,"
    "vs_fwd_max_accel,"
    "vs_lat_max_accel,"
    "vs_forward_accel_cmd,"
    "vs_of_scaled,"
    "vs_mu_x_control,"
    "vs_mu_y_of,"
    "vs_mu_y_yaw,"
    "vs_mu_y_fallback,"
    "vs_mu_y_control,"
    "vs_mu_x_target,"
    "vs_mu_y_target,"
    "vs_mu_x,"
    "vs_mu_y,"
    "vs_mu_z,"
    "vs_height_sp,"
    "vs_height_error,"
    "vs_pitch_sp,"
    "vs_roll_sp,"
    "vs_yaw_sp,"
  );
}

static void logger_file_write_visual_row(FILE *file)
{
  fprintf(
    file,
    "%u,%u,%u,"
    "%u,%u,%u,%u,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,%f,%f,"
    "%u,%u,%u,"
    "%f,%f,%f,"
    "%f,%f,%f,%f,%f,"
    "%f,%f,%f,%f,"
    "%f,%f,%f,%f,%f,%f,"
    "%f,%f,%f,%f,%f,%f,%f,"
    "%f,%f,"
    "%f,%f,%f,"
    "%f,%f,"
    "%f,%f,%f,",

    (unsigned int)visual_servoing.vision_stamp_us,
    (unsigned int)visual_servoing.vision_sequence,
    (unsigned int)visual_servoing.processed_vision_sequence,

    visual_servoing.vision_new_frame ? 1U : 0U,
    visual_servoing.vision_valid ? 1U : 0U,
    (unsigned int)visual_servoing.vision_valid_streak,
    visual_servoing.of_ready ? 1U : 0U,

    visual_servoing.vision_dt,
    visual_servoing.vision_age,
    visual_servoing.control_dt,

    visual_servoing.color_count,
    visual_servoing.box_centroid_x,
    visual_servoing.box_centroid_y,

    visual_servoing.raw_of_y,
    visual_servoing.of_y,
    visual_servoing.raw_of_y_d,
    visual_servoing.of_y_d,
    visual_servoing.yaw_vel,

    visual_servoing.pose_ok ? 1U : 0U,
    visual_servoing.using_of_control ? 1U : 0U,
    visual_servoing.using_lateral_fallback ? 1U : 0U,

    visual_servoing.heading_ref,
    visual_servoing.forward_axis_n,
    visual_servoing.forward_axis_e,

    visual_servoing.forward_position_ref,
    visual_servoing.forward_position,
    visual_servoing.forward_velocity,
    visual_servoing.forward_error,
    visual_servoing.right_velocity,

    visual_servoing.pitch_trim,
    visual_servoing.roll_trim,
    visual_servoing.mu_x_trim,
    visual_servoing.mu_y_trim,

    visual_servoing.fwd_kp,
    visual_servoing.fwd_kd,
    visual_servoing.ol_y_OF_gain,
    visual_servoing.ol_y_YAW_gain,
    visual_servoing.fwd_max_accel,
    visual_servoing.lat_max_accel,

    visual_servoing.forward_accel_cmd,
    visual_servoing.of_scaled,
    visual_servoing.mu_x_control,
    visual_servoing.mu_y_of,
    visual_servoing.mu_y_yaw,
    visual_servoing.mu_y_fallback,
    visual_servoing.mu_y_control,

    visual_servoing.mu_x_target,
    visual_servoing.mu_y_target,

    visual_servoing.mu_x,
    visual_servoing.mu_y,
    visual_servoing.mu_z,

    visual_servoing.height_setpoint,
    visual_servoing.height_error,

    visual_servoing.pitch_sp_cmd,
    visual_servoing.roll_sp_cmd,
    visual_servoing.yaw_sp_cmd
  );
}

/*
 * Forward PID integral diagnostics.
 *
 * Keep these fields in their own paired header/row functions so
 * the existing large visual-servo CSV block remains untouched.
 */
static void logger_file_write_forward_pid_header(FILE *file)
{
  fprintf(
    file,
    "vs_fwd_ki,"
    "vs_fwd_i_max_accel,"
    "vs_forward_error_integral,"
    "vs_forward_p_cmd,"
    "vs_forward_d_cmd,"
    "vs_forward_i_cmd,"
    "vs_forward_accel_unbounded,"
    "vs_forward_i_limited,"
    "vs_forward_accel_saturated,"
  );
}

static void logger_file_write_forward_pid_row(FILE *file)
{
  fprintf(
    file,
    "%f,%f,"
    "%f,"
    "%f,%f,%f,"
    "%f,"
    "%u,%u,",

    visual_servoing.fwd_ki,
    visual_servoing.fwd_i_max_accel,

    visual_servoing.forward_error_integral,

    visual_servoing.forward_p_cmd,
    visual_servoing.forward_d_cmd,
    visual_servoing.forward_i_cmd,

    visual_servoing.forward_accel_unbounded,

    visual_servoing.forward_i_limited
      ? 1U
      : 0U,

    visual_servoing.forward_accel_saturated
      ? 1U
      : 0U
  );
}

/*
 * Moving flower/setup fields.
 */
static void logger_file_write_setup_header(FILE *file)
{
  fprintf(
    file,
    "setup_valid,"
    "setup_target_id,"
    "setup_target_timestamp,"
    "setup_age_ms,"
    "setup_enu_x,"
    "setup_enu_y,"
    "setup_enu_z,"
    "setup_enu_xd,"
    "setup_enu_yd,"
    "setup_enu_zd,"
  );
}

static void logger_file_write_setup_row(FILE *file)
{
  const uint32_t now_ms = get_sys_time_msec();

  const uint32_t setup_age_ms =
    moving_setup.valid
      ? (now_ms - moving_setup.last_rx_time)
      : 0U;

  fprintf(
    file,
    "%u,%u,%u,%u,"
    "%f,%f,%f,"
    "%f,%f,%f,",

    moving_setup.valid ? 1U : 0U,
    (unsigned int)moving_setup.target_id,
    (unsigned int)moving_setup.target_timestamp,
    (unsigned int)setup_age_ms,

    moving_setup.enu_x,
    moving_setup.enu_y,
    moving_setup.enu_z,

    moving_setup.enu_xd,
    moving_setup.enu_yd,
    moving_setup.enu_zd
  );
}


/** Logging functions */

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in logger_file_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void logger_file_write_header(FILE *file) {
  fprintf(file, "time,");
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "acc_x,acc_y,acc_z,");
  fprintf(file, "att_phi,att_theta,att_psi,");

  logger_file_write_camera_fps_header(file);
  logger_file_write_vs_activation_header(file);
  logger_file_write_visual_header(file);
  logger_file_write_forward_pid_header(file);

  // fprintf(file, "distance_est,centroid_x,centroid_y,");
  // fprintf(file, "dt, color_count,");
  // fprintf(file, "raw_of_y, of_y, raw_of_y_d, of_y_d, yaw_vel,");
  // fprintf(file, "mu_x, mu_y, mu_z,");

  #ifdef INS_EXT_POSE_H
    fprintf(
      file,
      "ext_rx_count,"
      "ext_last_rx_usec,"
      "ext_dt_usec,"
      "ext_age_usec,"
      "ext_new_sample,"
    );
  #endif

  #ifdef COMMAND_THRUST
    /*
    * Vertical-guidance diagnostics.
    */
    fprintf(file, "guidance_v_mode,");
    fprintf(file, "z_sp,z_ref,z_error,");
    fprintf(file, "vz_sp,vz_ref,vz_error,");
    fprintf(file, "az_ref,");
    fprintf(file, "z_sum_error,");
    fprintf(file, "guidance_kp,guidance_kd,guidance_ki,");
    fprintf(file, "guidance_ff_cmd,guidance_fb_cmd,guidance_delta_t,");
    fprintf(file,
      "gv_hover_count,"
      "gv_err_z_used,"
      "gv_err_zd_used,"
      "gv_p_cmd,"
      "gv_d_cmd,"
      "gv_i_cmd,"
      "gv_ff_before_tilt,"
      "gv_ff_unbounded,"
      "gv_ff_saturated,"
      "gv_delta_unbounded,"
      "gv_output_saturated,"
      "gv_thrust_coeff,"
      "gv_nominal_throttle,"
      "gv_adapt_enabled,"
    );

    fprintf(file,
      "gh_traj_count,"
      "gh_mode,"
      "gh_use_ref,"
      "gh_approx_force,"
      "gh_sp_mask,"
    );

    fprintf(
      file,
      "gh_sp_x,gh_sp_y,"
      "gh_sp_vx,gh_sp_vy,"
      "gh_heading_sp,"
    );

    fprintf(
      file,
      "gh_ref_x,gh_ref_y,"
      "gh_ref_vx,gh_ref_vy,"
      "gh_ref_ax,gh_ref_ay,"
    );

    fprintf(
      file,
      "gh_pos_err_x,gh_pos_err_y,"
      "gh_speed_err_x,gh_speed_err_y,"
    );

    fprintf(
      file,
      "gh_p_n,gh_p_e,"
      "gh_d_n,gh_d_e,"
      "gh_vff_n,gh_vff_e,"
      "gh_aff_n,gh_aff_e,"
    );

    fprintf(
      file,
      "gh_cmd_pre_sat_n,"
      "gh_cmd_pre_sat_e,"
      "gh_i_n,gh_i_e,"
      "gh_i_raw_n,gh_i_raw_e,"
      "gh_cmd_after_i_n,"
      "gh_cmd_after_i_e,"
      "gh_cmd_final_n,"
      "gh_cmd_final_e,"
    );

    fprintf(
      file,
      "gh_traj_sat_n,"
      "gh_traj_sat_e,"
      "gh_final_sat_n,"
      "gh_final_sat_e,"
    );

    fprintf(
      file,
      "att_sp_phi,"
      "att_sp_theta,"
      "att_sp_psi,"
    );
  #endif

  fprintf(file, "body_p,body_q,body_r,");

  /*
   * Moving setup / flower setup received from OptiTrack.
   */
  // logger_file_write_setup_header(file);

#ifdef BOARD_BEBOP
  fprintf(file, "rpm_obs_1,rpm_obs_2,rpm_obs_3,rpm_obs_4,");
  fprintf(file, "rpm_ref_1,rpm_ref_2,rpm_ref_3,rpm_ref_4,");
#endif
#ifdef INS_EXT_POSE_H
  ins_ext_pos_log_header(file);
#endif

  fprintf(
    file,
    "logger_write_usec,"
    "logger_max_write_usec,"
  );

#ifdef COMMAND_THRUST
  fprintf(
    file,
    "indi_rate_run_count,"
    "indi_att_fb_x,"
    "indi_att_fb_y,"
    "indi_att_fb_z,"
  );

  fprintf(
    file,
    "indi_rate_sp_p,"
    "indi_rate_sp_q,"
    "indi_rate_sp_r,"
  );

  fprintf(
    file,
    "indi_rate_fb_p,"
    "indi_rate_fb_q,"
    "indi_rate_fb_r,"
  );

  fprintf(
    file,
    "indi_rate_filt_p,"
    "indi_rate_filt_q,"
    "indi_rate_filt_r,"
  );

  fprintf(
    file,
    "indi_rate_d_p,"
    "indi_rate_d_q,"
    "indi_rate_d_r,"
  );

  fprintf(
    file,
    "indi_acc_ref_p,"
    "indi_acc_ref_q,"
    "indi_acc_ref_r,"
  );

  fprintf(
    file,
    "indi_u_act_p,"
    "indi_u_act_q,"
    "indi_u_act_r,"
  );

  fprintf(
    file,
    "indi_u_filt_p,"
    "indi_u_filt_q,"
    "indi_u_filt_r,"
  );

  fprintf(
    file,
    "indi_du_p,"
    "indi_du_q,"
    "indi_du_r,"
  );

  fprintf(
    file,
    "indi_u_unbounded_p,"
    "indi_u_unbounded_q,"
    "indi_u_unbounded_r,"
  );

  fprintf(
    file,
    "indi_u_in_p,"
    "indi_u_in_q,"
    "indi_u_in_r,"
  );

  fprintf(
    file,
    "indi_sat_p,"
    "indi_sat_q,"
    "indi_sat_r,"
  );

  fprintf(
    file,
    "indi_g1_p,"
    "indi_g1_q,"
    "indi_g1_r,"
    "indi_g2,"
  );

  fprintf(file, "cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw\n");
#else
  fprintf(file, "h_ctl_aileron_setpoint,h_ctl_elevator_setpoint\n");
#endif
}

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of logger_file_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void logger_file_write_row(FILE *file) {
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct NedCoor_f *acc = stateGetAccelNed_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();

  struct FloatRates *body_rates = stateGetBodyRates_f();

  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", acc->x, acc->y, acc->z);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);

  logger_file_write_camera_fps_row(file);
  logger_file_write_vs_activation_row(file);
  logger_file_write_visual_row(file);
  logger_file_write_forward_pid_row(file);
  // fprintf(file, "%f,%f,%f,", visual_servoing.distance_est, visual_servoing.box_centroid_x, visual_servoing.box_centroid_y);
  // fprintf(file, "%f,%f,", visual_servoing.dt, visual_servoing.color_count);
  // fprintf(file, "%f,%f,%f,%f,%f,", visual_servoing.raw_of_y, visual_servoing.of_y, visual_servoing.raw_of_y_d, visual_servoing.of_y_d, visual_servoing.yaw_vel);
  // fprintf(file, "%f,%f,%f,", visual_servoing.mu_x, visual_servoing.mu_y, visual_servoing.mu_z);
  #ifdef INS_EXT_POSE_H
    /*
    * Determine the age of the most recently received EXTERNAL_POSE message.
    */
    const uint32_t logger_now_usec = get_sys_time_usec();

    const uint32_t ext_age_usec =
        (ins_ext_pose_rx_count > 0)
        ? (logger_now_usec - ins_ext_pose_last_rx_usec)
        : 0;

    /*
    * The logger may run faster than EXTERNAL_POSE.
    * ext_new_sample becomes 1 only when the receive counter changes.
    */
    static uint32_t previous_ext_rx_count = 0;

    const uint8_t ext_new_sample =
        (ins_ext_pose_rx_count != previous_ext_rx_count) ? 1 : 0;

    previous_ext_rx_count = ins_ext_pose_rx_count;

      fprintf(
        file,
        "%u,%u,%u,%u,%u,",

        (unsigned int)ins_ext_pose_rx_count,
        (unsigned int)ins_ext_pose_last_rx_usec,
        (unsigned int)ins_ext_pose_dt_usec,
        (unsigned int)ext_age_usec,
        (unsigned int)ext_new_sample
      );
  #endif

  #ifdef COMMAND_THRUST
    /*
    * Integer-state values are converted to physical SI units.
    *
    * z:       Q23.8
    * vz:      Q12.19
    * az:      Q21.10
    */
    const float z_sp_m =
        POS_FLOAT_OF_BFP(guidance_v_z_sp);

    const float z_ref_m =
        POS_FLOAT_OF_BFP(guidance_v_z_ref);

    const float vz_sp_mps =
        SPEED_FLOAT_OF_BFP(guidance_v_zd_sp);

    const float vz_ref_mps =
        SPEED_FLOAT_OF_BFP(guidance_v_zd_ref);

    const float az_ref_mps2 =
        ACCEL_FLOAT_OF_BFP(guidance_v_zdd_ref);

    /*
    * pos->z and vel->z were obtained at the top of this function.
    */
    const float z_error_m = z_ref_m - pos->z;
    const float vz_error_mps = vz_ref_mps - vel->z;

    fprintf(file, "%u,", guidance_v_mode);

    fprintf(file, "%f,%f,%f,",
            z_sp_m,
            z_ref_m,
            z_error_m);

    fprintf(file, "%f,%f,%f,",
            vz_sp_mps,
            vz_ref_mps,
            vz_error_mps);

    fprintf(file, "%f,", az_ref_mps2);

    /*
    * Keep the integral accumulator in its native representation.
    * It is mainly useful for checking wind-up and slow oscillations.
    */
    fprintf(file, "%d,", guidance_v_z_sum_err);

    fprintf(file, "%d,%d,%d,",
            guidance_v_kp,
            guidance_v_kd,
            guidance_v_ki);

    fprintf(file, "%d,%d,%d,",
            guidance_v_ff_cmd,
            guidance_v_fb_cmd,
            guidance_v_delta_t);

    /*
    * Convert thrust coefficient from INT32_TRIG_FRAC to a
    * dimensionless floating-point value.
    */
    const float gv_thrust_coeff_f =
      (float)guidance_v_thrust_coeff /
      (float)(1 << INT32_TRIG_FRAC);

    fprintf(file,
      "%u,"
      "%f,%f,"
      "%d,%d,%d,"
      "%d,%d,"
      "%u,"
      "%d,"
      "%u,"
      "%f,%f,%u,",

      (unsigned int)guidance_v_hover_count_diag,

      POS_FLOAT_OF_BFP(
        guidance_v_err_z_diag
      ),

      SPEED_FLOAT_OF_BFP(
        guidance_v_err_zd_diag
      ),

      guidance_v_p_cmd_diag,
      guidance_v_d_cmd_diag,
      guidance_v_i_cmd_diag,

      guidance_v_ff_before_tilt_diag,
      guidance_v_ff_unbounded_diag,

      (unsigned int)
        guidance_v_ff_saturated_diag,

      guidance_v_delta_t_unbounded_diag,

      (unsigned int)
        guidance_v_output_saturated_diag,

      gv_thrust_coeff_f,
      guidance_v_nominal_throttle,

      guidance_v_adapt_throttle_enabled
        ? 1U
        : 0U
    );

    fprintf(
      file,
      "%u,%u,%u,%u,%u,",

      (unsigned int)
        guidance_h_traj_count_diag,

      (unsigned int)
        guidance_h.mode,

      guidance_h.use_ref
        ? 1U
        : 0U,

      guidance_h.approx_force_by_thrust
        ? 1U
        : 0U,

      (unsigned int)
        guidance_h.sp.mask
    );

    /*
    * Setpoint.
    */
    fprintf(
      file,
      "%f,%f,%f,%f,%f,",

      POS_FLOAT_OF_BFP(
        guidance_h.sp.pos.x
      ),

      POS_FLOAT_OF_BFP(
        guidance_h.sp.pos.y
      ),

      SPEED_FLOAT_OF_BFP(
        guidance_h.sp.speed.x
      ),

      SPEED_FLOAT_OF_BFP(
        guidance_h.sp.speed.y
      ),

      guidance_h.sp.heading
    );

    /*
    * Reference generated from the setpoint.
    */
    fprintf(
      file,
      "%f,%f,%f,%f,%f,%f,",

      POS_FLOAT_OF_BFP(
        guidance_h.ref.pos.x
      ),

      POS_FLOAT_OF_BFP(
        guidance_h.ref.pos.y
      ),

      SPEED_FLOAT_OF_BFP(
        guidance_h.ref.speed.x
      ),

      SPEED_FLOAT_OF_BFP(
        guidance_h.ref.speed.y
      ),

      ACCEL_FLOAT_OF_BFP(
        guidance_h.ref.accel.x
      ),

      ACCEL_FLOAT_OF_BFP(
        guidance_h.ref.accel.y
      )
    );

    /*
    * Actual bounded errors used by horizontal guidance.
    */
    fprintf(
      file,
      "%f,%f,%f,%f,",

      POS_FLOAT_OF_BFP(
        guidance_h_pos_err.x
      ),

      POS_FLOAT_OF_BFP(
        guidance_h_pos_err.y
      ),

      SPEED_FLOAT_OF_BFP(
        guidance_h_speed_err.x
      ),

      SPEED_FLOAT_OF_BFP(
        guidance_h_speed_err.y
      )
    );

    /*
    * Individual P, D, velocity-feedforward and
    * acceleration-feedforward angle contributions.
    */
    fprintf(
      file,
      "%f,%f,"
      "%f,%f,"
      "%f,%f,"
      "%f,%f,",

      ANGLE_FLOAT_OF_BFP(
        guidance_h_p_cmd_diag.x
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_p_cmd_diag.y
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_d_cmd_diag.x
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_d_cmd_diag.y
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_v_ff_cmd_diag.x
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_v_ff_cmd_diag.y
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_a_ff_cmd_diag.x
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_a_ff_cmd_diag.y
      )
    );

    /*
    * Commands at each stage of horizontal guidance.
    */
    fprintf(
      file,
      "%f,%f,"
      "%f,%f,"
      "%d,%d,"
      "%f,%f,"
      "%f,%f,",

      ANGLE_FLOAT_OF_BFP(
        guidance_h_cmd_pre_sat_diag.x
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_cmd_pre_sat_diag.y
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_i_cmd_diag.x
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_i_cmd_diag.y
      ),

      guidance_h_trim_att_integrator.x,
      guidance_h_trim_att_integrator.y,

      ANGLE_FLOAT_OF_BFP(
        guidance_h_cmd_after_i_diag.x
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_cmd_after_i_diag.y
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_cmd_earth.x
      ),

      ANGLE_FLOAT_OF_BFP(
        guidance_h_cmd_earth.y
      )
    );

    fprintf(
      file,
      "%u,%u,%u,%u,",

      (unsigned int)
        guidance_h_traj_sat_x_diag,

      (unsigned int)
        guidance_h_traj_sat_y_diag,

      (unsigned int)
        guidance_h_final_sat_x_diag,

      (unsigned int)
        guidance_h_final_sat_y_diag
    );

    /*
    * Body-frame roll, pitch and yaw setpoint produced from the
    * final NED horizontal command and heading.
    */
    fprintf(
      file,
      "%f,%f,%f,",

      ANGLE_FLOAT_OF_BFP(
        stab_att_sp_euler.phi
      ),

      ANGLE_FLOAT_OF_BFP(
        stab_att_sp_euler.theta
      ),

      ANGLE_FLOAT_OF_BFP(
        stab_att_sp_euler.psi
      )
    );
  #endif

  fprintf(file, "%f,%f,%f,", body_rates->p, body_rates->q, body_rates->r);

  /*
   * Moving setup / flower setup data.
   */
  // logger_file_write_setup_row(file);

#ifdef BOARD_BEBOP
  fprintf(file, "%d,%d,%d,%d,",actuators_bebop.rpm_obs[0],actuators_bebop.rpm_obs[1],actuators_bebop.rpm_obs[2],actuators_bebop.rpm_obs[3]);
  fprintf(file, "%d,%d,%d,%d,",actuators_bebop.rpm_ref[0],actuators_bebop.rpm_ref[1],actuators_bebop.rpm_ref[2],actuators_bebop.rpm_ref[3]);
#endif
#ifdef INS_EXT_POSE_H
  ins_ext_pos_log_data(file);
#endif

 /*
  * These values describe the previous completed logger call,
  * because the current call cannot know its own duration until
  * after the row has been written.
  */
  fprintf(
    file,
    "%u,%u,",
    (unsigned int)logger_last_write_usec,
    (unsigned int)logger_max_write_usec
  );

#ifdef COMMAND_THRUST
  fprintf(
    file,
    "%u,%f,%f,%f,",

    (unsigned int)
      indi.rate_run_count,

    indi.att_error_fb.x,
    indi.att_error_fb.y,
    indi.att_error_fb.z
  );

  fprintf(
    file,
    "%f,%f,%f,",

    indi.rate_sp.p,
    indi.rate_sp.q,
    indi.rate_sp.r
  );

  fprintf(
    file,
    "%f,%f,%f,",

    indi.rate_feedback.p,
    indi.rate_feedback.q,
    indi.rate_feedback.r
  );

  /*
  * Butterworth-filtered body rates used for finite-difference
  * angular-acceleration estimation.
  */
  fprintf(
    file,
    "%f,%f,%f,",

    indi.rate[0].o[0],
    indi.rate[1].o[0],
    indi.rate[2].o[0]
  );

  fprintf(
    file,
    "%f,%f,%f,",

    indi.rate_d[0],
    indi.rate_d[1],
    indi.rate_d[2]
  );

  fprintf(
    file,
    "%f,%f,%f,",

    indi.angular_accel_ref.p,
    indi.angular_accel_ref.q,
    indi.angular_accel_ref.r
  );

  fprintf(
    file,
    "%f,%f,%f,",

    indi.u_act_dyn.p,
    indi.u_act_dyn.q,
    indi.u_act_dyn.r
  );

  /*
  * Filtered actuator-model command used as the baseline for
  * the next incremental command.
  */
  fprintf(
    file,
    "%f,%f,%f,",

    indi.u[0].o[0],
    indi.u[1].o[0],
    indi.u[2].o[0]
  );

  fprintf(
    file,
    "%f,%f,%f,",

    indi.du.p,
    indi.du.q,
    indi.du.r
  );

  fprintf(
    file,
    "%f,%f,%f,",

    indi.u_in_unbounded.p,
    indi.u_in_unbounded.q,
    indi.u_in_unbounded.r
  );

  fprintf(
    file,
    "%f,%f,%f,",

    indi.u_in.p,
    indi.u_in.q,
    indi.u_in.r
  );

  fprintf(
    file,
    "%u,%u,%u,",

    (unsigned int)
      indi.saturated_p,

    (unsigned int)
      indi.saturated_q,

    (unsigned int)
      indi.saturated_r
  );

  fprintf(
    file,
    "%f,%f,%f,%f,",

    indi.g1.p,
    indi.g1.q,
    indi.g1.r,
    indi.g2
  );

  fprintf(file, "%d,%d,%d,%d\n",
      stabilization_cmd[COMMAND_THRUST], stabilization_cmd[COMMAND_ROLL],
      stabilization_cmd[COMMAND_PITCH], stabilization_cmd[COMMAND_YAW]);
#else
  fprintf(file, "%d,%d\n", h_ctl_aileron_setpoint, h_ctl_elevator_setpoint);
#endif

}


/** Start the file logger and open a new file */
void logger_file_start(void)
{
  // Ensure that the module is running when started with this function
  logger_file_logger_file_periodic_status = MODULES_RUN;
  
  // Create output folder if necessary
  if (access(STRINGIFY(LOGGER_FILE_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(LOGGER_FILE_PATH));
    if (system(save_dir_cmd) != 0) {
      printf("[logger_file] Could not create log file directory %s.\n", STRINGIFY(LOGGER_FILE_PATH));
      return;
    }
  }
  /*
  * Buffer CSV output in userspace to avoid forcing a storage
  * operation after every 128 Hz logger row.
  */

  // Get current date/time for filename
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y%m%d-%H%M%S", &tstruct);

  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(LOGGER_FILE_PATH), date_time);
  while ((logger_file = fopen(filename, "r"))) {
    fclose(logger_file);

    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(LOGGER_FILE_PATH), date_time, counter);
    counter++;
  }

  logger_file = fopen(filename, "w");

  if (logger_file == NULL) {
    printf(
      "[logger_file] ERROR opening log file %s!\n",
      filename
    );
    return;
  }

  /*
  * setvbuf must be called after fopen and before the first
  * write to this stream.
  */
  if (setvbuf(
        logger_file,
        logger_file_buffer,
        _IOFBF,
        sizeof(logger_file_buffer)
      ) != 0) {

    printf(
      "[logger_file] WARNING: could not configure file buffer.\n"
    );
  }

  logger_last_write_usec = 0;
  logger_max_write_usec = 0;

  printf(
    "[logger_file] Start logging to %s...\n",
    filename
  );

  /*
  * Write the CSV header exactly once.
  */
  logger_file_write_header(logger_file);

  /*
  * One startup flush is acceptable. It ensures the header has
  * left the userspace buffer before the flight begins.
  */
  fflush(logger_file);
}

/** Stop the logger an nicely close the file */
void logger_file_stop(void)
{
  if (logger_file != NULL) {
    fclose(logger_file);
    logger_file = NULL;
  }
}

void logger_file_periodic(void)
{
  if (logger_file == NULL) {
    return;
  }

  const uint32_t write_start_usec =
    get_sys_time_usec();

  logger_file_write_row(logger_file);

  /*
   * Do not call fflush() from the periodic flight-control path.
   * fclose() in logger_file_stop() will flush the buffer.
   */

  logger_last_write_usec =
    get_sys_time_usec() - write_start_usec;

  if (logger_last_write_usec >
      logger_max_write_usec) {
    logger_max_write_usec =
      logger_last_write_usec;
  }
}