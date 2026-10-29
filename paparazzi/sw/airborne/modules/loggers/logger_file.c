/*
 * Streamlined logger for optic-flow/Kalman-filter data collection.
 *
 * Retains:
 *   - aircraft kinematics/attitude/body rates
 *   - external-pose receive timing
 *   - moving-target/setup ground truth
 *   - vision timing/validity and centroid
 *   - raw + KF3-filtered OF and image acceleration
 *   - complete KF3 state/covariance/tuning diagnostics
 *   - minimal VS activation + 2 s KF3 pre-roll diagnostics
 *   - lateral OF/yaw controller diagnostics
 *   - logger execution-time diagnostics
 *
 * Removes the older hover-tuning blocks:
 *   detailed VS settle kinematics/trim diagnostics,
 *   forward PID internals, vertical/horizontal guidance internals,
 *   Bebop RPMs, full ins_ext_pose dump, INDI internals, actuator commands.
 */

#include "logger_file.h"

#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "std.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "generated/modules.h"

#include "modules/visual_servoing/visual_servoing.h"
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/visual_servoing/vs_distance_observer.h"
#include "modules/visual_servoing/vs_log_utils.h"
#ifdef VS_SIM_NO_EXT_POSE
#include "mcu_periph/sim_clock_bridge.h"
#endif
#include "modules/mission/moving_setup_logger_optitrack.h"
#ifndef VS_SIM_NO_EXT_POSE
#include "modules/ins/ins_ext_pose.h"
#endif

#ifndef LOGGER_FILE_PATH
#define LOGGER_FILE_PATH /data/video/usb
#endif

static FILE *logger_file = NULL;
static FILE *command_file = NULL;
#if VS_COMMAND_TRACE
static char command_file_buffer[64*1024];
#endif
static double logger_last_flush_s;
static bool logger_exit_registered;
#ifndef VS_EXPERIMENT_CASE_ID
#define VS_EXPERIMENT_CASE_ID 0
#endif
#ifndef VS_EXPERIMENT_X0_M
#define VS_EXPERIMENT_X0_M 0
#endif
#ifndef NPS_GAZEBO_SYNC_TIME
#define NPS_GAZEBO_SYNC_TIME 0
#endif
#ifndef VS_SELF_OSC_FREQ_HZ
#define VS_SELF_OSC_FREQ_HZ .53
#endif
#ifndef VS_SELF_OSC_AMPLITUDE
#define VS_SELF_OSC_AMPLITUDE 1.123
#endif
#ifndef VS_SELF_OSC_H2_AMPLITUDE
#define VS_SELF_OSC_H2_AMPLITUDE -.608
#endif


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
    "cod_input_count,"
    "cod_accept_count,"
    "cod_reject_count,"
    "cod_init_count,"
    "cod_input_dt_us,"
    "cod_gate_dt_us,"
    "cod_backward_ts_count,"
  );
}

static void logger_file_write_camera_fps_row(FILE *file)
{
  fprintf(
    file,
    "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,",
    (unsigned int)cod_callback_count,
    (unsigned int)cod_exec_time_us,
    (unsigned int)cod_exec_time_max_us,
    (unsigned int)cod_input_count,
    (unsigned int)cod_accept_count,
    (unsigned int)cod_reject_count,
    (unsigned int)cod_init_count,
    (unsigned int)cod_input_dt_us,
    (unsigned int)cod_gate_dt_us,
    (unsigned int)cod_backward_ts_count
  );
}

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

static void logger_file_write_kf_visual_header(FILE *file)
{
  fprintf(
    file,

    /* Vision timing / validity. */
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
    "vs_pose_ok,"

    /* Minimal activation / pre-roll diagnostics. */
    "vs_activation_state,"
    "vs_settle_condition,"
    "vs_settle_ready,"
    "kf_warmup_start_us,"
    "kf_warmup_valid_frames,"
    "kf_warmup_elapsed,"
    "kf_warmup_ready,"

    /* Direct visual measurements. */
    "vs_color_count,"
    "vs_centroid_x,"
    "vs_centroid_y,"

    /*
     * Raw finite-difference diagnostics and KF3 output.
     *
     * vs_of_y   = KF3 estimated image velocity / optic flow [px/s]
     * vs_of_y_d = KF3 estimated image acceleration [px/s^2]
     */
    "vs_raw_of_y,"
    "vs_of_y,"
    "vs_raw_of_y_d,"
    "vs_of_y_d,"

    /*
     * KF3 state / prediction diagnostics.
     *
     * State ordering:
     *   [ centroid_y, OF_y, image_acceleration_y ]
     *
     * The true scalar innovation is centroid measurement minus
     * predicted centroid.
     */
    "kf_centroid,"
    "kf_centroid_pred,"
    "kf_x1_pred,"
    "kf_x2_pred,"
    "kf_innov_centroid,"

    /* Complete KF3 posterior covariance P. */
    "kf_p00,"
    "kf_p01,"
    "kf_p02,"
    "kf_p10,"
    "kf_p11,"
    "kf_p12,"
    "kf_p20,"
    "kf_p21,"
    "kf_p22,"

    /*
     * KF3 tuning diagnostics.
     *
     * q11 = Q(OF,OF)
     * q22 = Q(acceleration,acceleration)
     * r11 = centroid measurement variance [px^2]
     * r22 = unused compatibility field (0)
     */
    "kf_q11,"
    "kf_q22,"
    "kf_r11,"
    "kf_r22,"
    "kf_initialized,"

    /* Lateral-controller context. */
    "vs_yaw_vel,"
    "vs_of_scale,"
    "vs_of_scaled,"
    "vs_of_gain,"
    "vs_yaw_gain,"
    "vs_mu_y_of,"
    "vs_mu_y_yaw,"
    "vs_mu_y_fallback,"
    "vs_mu_y_active,"
    "vs_mu_y_control,"
    "vs_using_of,"
    "vs_using_fallback,"
  );
}

static void logger_file_write_kf_visual_row(FILE *file)
{
  fprintf(
    file,

    /* Vision timing / validity. */
    "%u,%u,%u,"
    "%u,%u,%u,%u,"
    "%f,%f,%f,"
    "%u,"

    /* Activation / KF3 pre-roll. */
    "%u,%u,%u,"
    "%u,%u,"
    "%f,%u,"

    /* Direct visual measurements. */
    "%f,%f,%f,"

    /* Raw diagnostics + KF3 output. */
    "%f,%f,%f,%f,"

    /* KF3 state/prediction + true centroid innovation. */
    "%f,%f,%f,%f,%f,"

    /* Complete 3x3 posterior covariance. */
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"

    /* Tuning + initialized flag. */
    "%f,%f,%f,%f,"
    "%u,"

    /* Lateral-controller context. */
    "%f,%f,%f,%f,%f,"
    "%f,%f,%f,%f,%f,"
    "%u,%u,",

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

    visual_servoing.pose_ok ? 1U : 0U,

    (unsigned int)visual_servoing.activation_state,
    visual_servoing.settle_condition ? 1U : 0U,
    visual_servoing.settle_ready ? 1U : 0U,

    (unsigned int)visual_servoing.kf_warmup_start_us,
    (unsigned int)visual_servoing.kf_warmup_valid_frames,
    visual_servoing.kf_warmup_elapsed,
    visual_servoing.kf_warmup_ready ? 1U : 0U,

    visual_servoing.color_count,
    visual_servoing.box_centroid_x,
    visual_servoing.box_centroid_y,

    visual_servoing.raw_of_y,
    visual_servoing.of_y,
    visual_servoing.raw_of_y_d,
    visual_servoing.of_y_d,

    visual_servoing.kf_centroid,
    visual_servoing.kf_centroid_pred,
    visual_servoing.kf_x1_pred,
    visual_servoing.kf_x2_pred,
    visual_servoing.kf_innov_centroid,

    visual_servoing.kf_p00,
    visual_servoing.kf_p01,
    visual_servoing.kf_p02,

    visual_servoing.kf_p10,
    visual_servoing.kf_p11,
    visual_servoing.kf_p12,

    visual_servoing.kf_p20,
    visual_servoing.kf_p21,
    visual_servoing.kf_p22,

    visual_servoing.kf_q11,
    visual_servoing.kf_q22,
    visual_servoing.kf_r11,
    visual_servoing.kf_r22,

    visual_servoing.kf_initialized ? 1U : 0U,

    visual_servoing.yaw_vel,
    visual_servoing.of_scale,
    visual_servoing.of_scaled,
    visual_servoing.ol_y_OF_gain,
    visual_servoing.ol_y_YAW_gain,

    visual_servoing.mu_y_of,
    visual_servoing.mu_y_yaw,
    visual_servoing.mu_y_fallback,
    visual_servoing.mu_y_active,
    visual_servoing.mu_y_control,

    visual_servoing.using_of_control ? 1U : 0U,
    visual_servoing.using_lateral_fallback ? 1U : 0U
  );
}

static void logger_observer_header(FILE *f)
{
  fputs("run_id,case_id,clock_sync,gazebo_pair_s,system_pair_s,clock_offset_s,clock_drift_s,clock_pairs,horizontal_mode,",f);
  fputs("b54_capture_raw_us,b54_receive_us,b54_process_us,b54_accepted_sequence,b54_duplicate_count,b54_backward_count,b54_skipped_count,b54_source_age_s,b54_receive_age_s,b54_yaw_at_capture,b54_shadow_mu_y_of,b54_shadow_mu_y_yaw,b54_command_us,b54_command_sequence,b54_command_valid,b54_mu_y_unbounded,b54_mu_y_trim,b54_mu_y_target,b54_mu_y_final,b54_roll_sp,b54_total_limited,b54_slew_limited,b54_roll_limited,b54_oe_count,b54_oe_raw,b54_oe_derivative,b54_oe_output,b54_oe_innovation,b54_oe_variance,b54_oe_p00,b54_oe_p01,b54_oe_p02,b54_oe_p10,b54_oe_p11,b54_oe_p12,b54_oe_p20,b54_oe_p21,b54_oe_p22,b54_observer_h,b54_observer_rho,b54_observer_range_m,b54_observer_initial_h,b54_observer_residual,b54_observer_correction_raw,b54_observer_correction,b54_observer_valid,b54_observer_started,b54_observer_reason,b54_observer_resets,b54_observer_frames,b54_observer_steps,b54_observer_state_epoch_s,b54_observer_available_s,b54_observer_base_epoch_s,b54_observer_A_query_s,b54_held_F,b54_held_E,b54_held_A,b54_held_K,b54_held_g,b54_held_U,b54_model_t,b54_model_z,b54_model_y,b54_model_v,b54_model_u,b54_model_ready,b54_model_seeded,b54_model_warm_elapsed_s,b54_model_sequence,b54_model_resets,b54_vs_module_entry_s,b54_vs_oscillation_onset_s,b54_vs_have_oscillation_onset,b54_vs_entry_n,b54_vs_entry_e,b54_vs_entry_height,b54_vs_entry_vn,b54_vs_entry_ve,b54_vs_entry_vd,",f);
}
static void logger_observer_row(FILE *f)
{
  const struct VsDistanceObserver *o=&vs_distance_observer;
  fprintf(f,"%s,%u,%u,",vs_run_id(),(unsigned)VS_EXPERIMENT_CASE_ID,(unsigned)NPS_GAZEBO_SYNC_TIME);
#ifdef VS_SIM_NO_EXT_POSE
  fprintf(f,"%.9f,%.9f,%.9g,%.9g,%u,",sim_clock_bridge.gazebo_s,sim_clock_bridge.system_s,
      sim_clock_bridge.initial_offset_s,sim_clock_bridge.drift_s,(unsigned)sim_clock_bridge.pairs);
#else
  fputs("nan,nan,nan,nan,0,",f);
#endif
  fprintf(f,"%u,",(unsigned)guidance_h.mode);
  fprintf(f,"%u,",(unsigned)(visual_servoing.vision_capture_raw_us));
  fprintf(f,"%u,",(unsigned)(visual_servoing.vision_receive_us));
  fprintf(f,"%u,",(unsigned)(visual_servoing.vision_process_us));
  fprintf(f,"%u,",(unsigned)(visual_servoing.vision_accepted_sequence));
  fprintf(f,"%u,",(unsigned)(visual_servoing.vision_duplicate_count));
  fprintf(f,"%u,",(unsigned)(visual_servoing.vision_backward_count));
  fprintf(f,"%u,",(unsigned)(visual_servoing.vision_skipped_count));
  fprintf(f,"%.12g,",(double)(visual_servoing.vision_source_age));
  fprintf(f,"%.12g,",(double)(visual_servoing.vision_receive_age));
  fprintf(f,"%.12g,",(double)(visual_servoing.observer_yaw_at_capture));
  fprintf(f,"%.12g,",(double)(visual_servoing.shadow_mu_y_of));
  fprintf(f,"%.12g,",(double)(visual_servoing.shadow_mu_y_yaw));
  fprintf(f,"%u,",(unsigned)(visual_servoing.command_stamp_us));
  fprintf(f,"%u,",(unsigned)(visual_servoing.command_sequence));
  fprintf(f,"%u,",(unsigned)(visual_servoing.applied_command_valid));
  fprintf(f,"%.12g,",(double)(visual_servoing.mu_y_unbounded));
  fprintf(f,"%.12g,",(double)(visual_servoing.mu_y_trim));
  fprintf(f,"%.12g,",(double)(visual_servoing.mu_y_target));
  fprintf(f,"%.12g,",(double)(visual_servoing.mu_y));
  fprintf(f,"%.12g,",(double)(visual_servoing.roll_sp_cmd));
  fprintf(f,"%u,",(unsigned)(visual_servoing.lat_total_limited));
  fprintf(f,"%u,",(unsigned)(visual_servoing.lat_slew_limited));
  fprintf(f,"%u,",(unsigned)(visual_servoing.roll_limited));
  fprintf(f,"%.12g,",(double)(o->oe.count));
  fprintf(f,"%.12g,",(double)(o->oe.OE));
  fprintf(f,"%.12g,",(double)(o->oe.dot_OE));
  fprintf(f,"%.12g,",(double)(o->oe_output));
  fprintf(f,"%.12g,",(double)(o->oe_innovation));
  fprintf(f,"%.12g,",(double)(o->oe_variance));
  fprintf(f,"%.12g,",(double)(o->oe.P[0]));
  fprintf(f,"%.12g,",(double)(o->oe.P[1]));
  fprintf(f,"%.12g,",(double)(o->oe.P[2]));
  fprintf(f,"%.12g,",(double)(o->oe.P[3]));
  fprintf(f,"%.12g,",(double)(o->oe.P[4]));
  fprintf(f,"%.12g,",(double)(o->oe.P[5]));
  fprintf(f,"%.12g,",(double)(o->oe.P[6]));
  fprintf(f,"%.12g,",(double)(o->oe.P[7]));
  fprintf(f,"%.12g,",(double)(o->oe.P[8]));
  fprintf(f,"%.12g,",(double)(o->h));
  fprintf(f,"%.12g,",(double)(o->rho));
  fprintf(f,"%.12g,",(double)(o->range_m));
  fprintf(f,"%.12g,",(double)(o->initial_h));
  fprintf(f,"%.12g,",(double)(o->residual));
  fprintf(f,"%.12g,",(double)(o->correction_raw));
  fprintf(f,"%.12g,",(double)(o->correction));
  fprintf(f,"%u,",(unsigned)(o->valid));
  fprintf(f,"%u,",(unsigned)(o->started));
  fprintf(f,"%u,",(unsigned)(o->reason));
  fprintf(f,"%u,",(unsigned)(o->resets));
  fprintf(f,"%u,",(unsigned)(o->frames));
  fprintf(f,"%u,",(unsigned)(o->integration_steps));
  fprintf(f,"%.12g,",(double)(o->estimate_epoch_s));
  fprintf(f,"%.12g,",(double)(o->available_s));
  fprintf(f,"%.12g,",(double)(o->base_t));
  fprintf(f,"%.12g,",(double)(o->acceleration_query_s));
  fprintf(f,"%.12g,",(double)(o->held.F));
  fprintf(f,"%.12g,",(double)(o->held.E));
  fprintf(f,"%.12g,",(double)(o->held.A));
  fprintf(f,"%.12g,",(double)(o->held.K));
  fprintf(f,"%.12g,",(double)(o->held.g));
  fprintf(f,"%.12g,",(double)(o->held.U));
  fprintf(f,"%.12g,",(double)(o->model.t));
  fprintf(f,"%.12g,",(double)(o->model.z));
  fprintf(f,"%.12g,",(double)(o->model.y));
  fprintf(f,"%.12g,",(double)(o->model.v));
  fprintf(f,"%.12g,",(double)(o->model.u));
  fprintf(f,"%u,",(unsigned)(o->model.ready));
  fprintf(f,"%u,",(unsigned)(o->model.seeded));
  fprintf(f,"%.12g,",(double)(o->model.warm_elapsed_s));
  fprintf(f,"%u,",(unsigned)(o->model.sequence));
  fprintf(f,"%u,",(unsigned)(o->model.resets));
  fprintf(f,"%.12g,",(double)(visual_servoing.module_entry_s));
  fprintf(f,"%.12g,",(double)(visual_servoing.oscillation_onset_s));
  fprintf(f,"%u,",(unsigned)(visual_servoing.have_oscillation_onset));
  fprintf(f,"%.12g,",(double)(visual_servoing.entry_n));
  fprintf(f,"%.12g,",(double)(visual_servoing.entry_e));
  fprintf(f,"%.12g,",(double)(visual_servoing.entry_height));
  fprintf(f,"%.12g,",(double)(visual_servoing.entry_vn));
  fprintf(f,"%.12g,",(double)(visual_servoing.entry_ve));
  fprintf(f,"%.12g,",(double)(visual_servoing.entry_vd));
}

void logger_file_command_trace(void)
{
  if (!command_file) { return; }
  const struct VsDistanceObserver *o=&vs_distance_observer;
  const struct NedCoor_f *v=stateGetSpeedNed_f(), *a=stateGetAccelNed_f();
  fprintf(command_file,"%s,%u,%.9f,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u,%u,%u,%.12g,%.12g,%.12g,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
    vs_run_id(),(unsigned)VS_EXPERIMENT_CASE_ID,
    visual_servoing_time_s(visual_servoing.command_stamp_us),
    (unsigned)visual_servoing.command_stamp_us,(unsigned)visual_servoing.command_sequence,
    visual_servoing.mu_y_unbounded,visual_servoing.mu_y_control,visual_servoing.mu_y_of,
    visual_servoing.mu_y_yaw,visual_servoing.mu_y_active,visual_servoing.mu_y_trim,
    visual_servoing.mu_y,visual_servoing.roll_sp_cmd,
    (unsigned)visual_servoing.applied_command_valid,(unsigned)visual_servoing.lat_total_limited,
    (unsigned)visual_servoing.lat_slew_limited,(unsigned)visual_servoing.roll_limited,
    o->model.z,o->model.y,o->model.v,(unsigned)o->model.ready,
    v->x,v->y,v->z,a->x,a->y,a->z,stateGetBodyRates_f()->r);
}

static void logger_metadata(FILE *f)
{
  fprintf(f,"key,value\nrun_id,%s\ncase_id,%u\nclock_sync,%u\nx0_m,%.9g\nf_hz,%.9g\nA1_m_s2,%.9g\nA2_m_s2,%.9g\n",
    vs_run_id(),(unsigned)VS_EXPERIMENT_CASE_ID,(unsigned)NPS_GAZEBO_SYNC_TIME,
    (double)VS_EXPERIMENT_X0_M,(double)VS_SELF_OSC_FREQ_HZ,(double)VS_SELF_OSC_AMPLITUDE,(double)VS_SELF_OSC_H2_AMPLITUDE);
  fprintf(f,"L,%.17g\ngamma,%.17g\nc_rel,%.17g\nOF_observer_gain,%.17g\nOE_q,%.17g\nOE_output_gain,%.17g\nR0_m,%.17g\nextra_alignment_s,%.17g\nmodel_delay_s,%.17g\nmodel_gain,%.17g\nmodel_T_s,%.17g\nmodel_slew,%.17g\nmodel_drag,%.17g\nmax_step_s,%.17g\nvision_timeout_s,%.17g\n",
    (double)VS_OBS_L,(double)VS_OBS_GAMMA,(double)VS_OBS_C_REL,(double)VS_OBS_OF_GAIN,
    (double)VS_OE_Q,(double)VS_OE_GAIN,(double)VS_OBS_INITIAL_R_M,(double)VS_OBS_ACCEL_ALIGNMENT_S,
    (double)VS_MODEL_DELAY_S,(double)VS_MODEL_GAIN,(double)VS_MODEL_T_S,(double)VS_MODEL_SLEW,
    (double)VS_MODEL_DRAG,(double)VS_OBS_MAX_STEP_S,(double)VS_OBS_TIMEOUT_S);
  fprintf(f,"normalizer_ETA,%.17g\nnormalizer_AO,%.17g\nnormalizer_AA,%.17g\nnormalizer_delay_s,%.17g\nh0_window_s,%.17g\nmodel_quiet_warmup_s,%.17g\n",
    (double)VS_OBS_ETA,(double)VS_OBS_AO,(double)VS_OBS_AA,(double)VS_OBS_NORMALIZATION_DELAY_S,
    (double)VS_OBS_H0_WINDOW_S,(double)VS_MODEL_QUIET_WARMUP_S);
  fprintf(f,"OF_control_gain,%.12g\nyaw_control_gain,%.12g\nOF_pixel_scale,%.12g\nlat_accel_max,%.12g\nlat_accel_slew,%.12g\nroll_max_deg,%.12g\n",
    (double)visual_servoing.ol_y_OF_gain,(double)visual_servoing.ol_y_YAW_gain,(double)visual_servoing.of_scale,
    (double)visual_servoing.lat_max_accel,(double)visual_servoing.lat_accel_slew,(double)visual_servoing.max_roll_deg);
  fputs("build_profile," VS_OBSERVER_PROFILE "\nestimator_time_base,physical_seconds_in_NPS\n",f);
  fprintf(f,"OF_buffer_s,%.17g\nOE_buffer_s,%.17g\n",
    (double)VS_OBS_OF_BUFFER_S,(double)VS_OBS_OE_BUFFER_S);
  fputs("OE_initial_sd,0.5\nOE_derivative_initial_sd,2\ncount_absolute_sd,1\ncount_relative_sd,0.015\ncamera_configured_hz,25\nscan_stride_x,2\nscan_stride_y,2\nstate_epoch_assumption,availability_minus_configured_visual_delay_phase_approximation\n",f);
#ifdef SYS_TIME_FREQUENCY
  fprintf(f,"sys_time_frequency,%u\n",(unsigned)SYS_TIME_FREQUENCY);
#endif
#ifdef PERIODIC_FREQUENCY
  fprintf(f,"periodic_frequency,%u\n",(unsigned)PERIODIC_FREQUENCY);
#endif
#ifdef VS_KF3_SIGMA_JERK_PX_S3
  fprintf(f,"centroid_sigma_jerk,%.12g\ncentroid_initial_OF_sd,%.12g\ncentroid_initial_OFD_sd,%.12g\n",
    (double)VS_KF3_SIGMA_JERK_PX_S3,(double)VS_KF3_INITIAL_OF_STD_PX_S,(double)VS_KF3_INITIAL_OFD_STD_PX_S2);
#endif
}

static void logger_file_write_header(FILE *file)
{
  fprintf(file, "time,");

  /* State estimate used to relate visual motion to vehicle motion. */
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "acc_x,acc_y,acc_z,");
  fprintf(file, "att_phi,att_theta,att_psi,");
  fprintf(file, "body_p,body_q,body_r,");

#ifdef INS_EXT_POSE_H
  /*
   * Keep only receive timing/age rather than the full external-pose
   * diagnostic dump. This is enough to reject stale-pose intervals.
   */
  fprintf(
    file,
    "ext_rx_count,"
    "ext_last_rx_usec,"
    "ext_dt_usec,"
    "ext_age_usec,"
    "ext_new_sample,"
  );
#endif

  logger_file_write_camera_fps_header(file);
  logger_file_write_setup_header(file);
  logger_file_write_kf_visual_header(file);
  logger_observer_header(file);

  fprintf(
    file,
    "logger_write_usec,"
    "logger_max_write_usec\n"
  );
}

static void logger_file_write_row(FILE *file)
{
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
  fprintf(file, "%f,%f,%f,", body_rates->p, body_rates->q, body_rates->r);

#ifdef INS_EXT_POSE_H
  const uint32_t logger_now_usec = get_sys_time_usec();

  const uint32_t ext_age_usec =
    (ins_ext_pose_rx_count > 0U)
      ? (logger_now_usec - ins_ext_pose_last_rx_usec)
      : 0U;

  static uint32_t previous_ext_rx_count = 0U;

  const uint8_t ext_new_sample =
    (ins_ext_pose_rx_count != previous_ext_rx_count)
      ? 1U
      : 0U;

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

  logger_file_write_camera_fps_row(file);
  logger_file_write_setup_row(file);
  logger_file_write_kf_visual_row(file);
  logger_observer_row(file);

  /*
   * These describe the previous completed logger call; the current
   * call duration is only known after this row is emitted.
   */
  fprintf(
    file,
    "%u,%u\n",
    (unsigned int)logger_last_write_usec,
    (unsigned int)logger_max_write_usec
  );
}

void logger_file_start(void)
{
  if (logger_file) { return; }
  const char *directory=getenv("B54_LOG_DIR");
  if (!directory || !*directory) { directory=STRINGIFY(LOGGER_FILE_PATH); }
  if (vs_mkdirs(directory)) { perror("[logger_file] mkdir"); return; }
  char filename[1400], command_name[1400], metadata_name[1400];
  /* Human-readable local wall time for filenames; keep CSV run_id unchanged. */
  const time_t started=time(NULL);
  struct tm local_started;
  char date_time[32];
  if (started==(time_t)-1 || !localtime_r(&started,&local_started) ||
      !strftime(date_time,sizeof(date_time),"%Y-%m-%d_%H-%M-%S",&local_started)) {
    fprintf(stderr,"[logger_file] Could not format the local logging date/time.\n");
    return;
  }
  unsigned suffix=0;
  for (;;) {
    snprintf(filename,sizeof(filename),"%s/%s_pprz_%03u.csv",directory,date_time,suffix);
    logger_file=fopen(filename,"wx");
    if (logger_file || errno!=EEXIST || ++suffix>999) { break; }
  }
  if (!logger_file) { perror("[logger_file] open"); return; }
  snprintf(command_name,sizeof(command_name),"%s/%s_command_%03u.csv",directory,date_time,suffix);
  snprintf(metadata_name,sizeof(metadata_name),"%s/%s_metadata_%03u.csv",directory,date_time,suffix);
#if VS_COMMAND_TRACE
  command_file=fopen(command_name,"wx");
  if (!command_file) { perror("[logger_file] command trace open"); logger_file_stop(); return; }
  setvbuf(command_file,command_file_buffer,_IOFBF,sizeof(command_file_buffer));
  fputs("run_id,case_id,time_s,command_us,command_sequence,mu_y_unbounded,mu_y_bounded,mu_y_of,mu_y_yaw,mu_y_active,mu_y_trim,mu_y_final,roll_sp,valid,total_limited,slew_limited,roll_limited,model_z,model_y,model_v,model_ready,vn,ve,vd,an,ae,ad,yaw_rate\n",command_file);
#endif
  FILE *metadata=fopen(metadata_name,"wx");
  if (!metadata) { perror("[logger_file] metadata open"); logger_file_stop(); return; }
  logger_metadata(metadata); fclose(metadata);
  logger_file_logger_file_periodic_status=MODULES_RUN;
  logger_last_flush_s=visual_servoing_time_s(get_sys_time_usec());
  if (!logger_exit_registered) { atexit(logger_file_stop); logger_exit_registered=true; }

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

  logger_last_write_usec = 0U;
  logger_max_write_usec = 0U;

  printf(
    "[logger_file] Start logging to %s...\n",
    filename
  );

  logger_file_write_header(logger_file);

  /* One startup flush only; periodic logging remains buffered. */
  fflush(logger_file);
}

void logger_file_stop(void)
{
  if (command_file) { fclose(command_file); command_file=NULL; }
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

  const uint64_t write_start_usec = vs_profile_us();

  logger_file_write_row(logger_file);
  const double now=visual_servoing_time_s(get_sys_time_usec());
  if (now-logger_last_flush_s>=1.) {
    logger_last_flush_s=now;
    if (fflush(logger_file) || (command_file && fflush(command_file))) {
      perror("[logger_file] flush"); logger_file_stop(); return;
    }
  }
  if (ferror(logger_file) || (command_file && ferror(command_file))) {
    perror("[logger_file] write"); logger_file_stop(); return;
  }

  logger_last_write_usec =
    vs_profile_us() - write_start_usec;

  if (logger_last_write_usec >
      logger_max_write_usec) {

    logger_max_write_usec =
      logger_last_write_usec;
  }
}
