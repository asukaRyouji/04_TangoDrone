/*
 * Streamlined logger for optic-flow/Kalman-filter data collection.
 *
 * Retains:
 *   - aircraft kinematics/attitude/body rates
 *   - external-pose receive timing
 *   - moving-target/setup ground truth
 *   - vision timing/validity and centroid
 *   - raw + KF-filtered OF and OF derivative
 *   - KF state/covariance/tuning parameters
 *   - lateral OF/yaw controller diagnostics
 *   - logger execution-time diagnostics
 *
 * Removes the older hover-tuning blocks:
 *   camera callback profiling, VS settle/activation diagnostics,
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
#include "generated/airframe.h"
#include "generated/modules.h"

#include "modules/visual_servoing/visual_servoing.h"
#include "modules/mission/moving_setup_logger_optitrack.h"
#include "modules/ins/ins_ext_pose.h"

#ifndef LOGGER_FILE_PATH
#define LOGGER_FILE_PATH /data/video/usb
#endif

static FILE *logger_file = NULL;

static uint32_t logger_last_write_usec = 0;
static uint32_t logger_max_write_usec = 0;

static char logger_file_buffer[64 * 1024];

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

    /* Direct visual measurements. */
    "vs_color_count,"
    "vs_centroid_x,"
    "vs_centroid_y,"

    /* Raw and filtered OF signals. */
    "vs_raw_of_y,"
    "vs_of_y,"
    "vs_raw_of_y_d,"
    "vs_of_y_d,"

    /* KF state before measurement update. */
    "kf_x1_pred,"
    "kf_x2_pred,"

    /* KF posterior covariance. */
    "kf_p11,"
    "kf_p12,"
    "kf_p21,"
    "kf_p22,"

    /* KF tuning values, repeated intentionally as flight metadata. */
    "kf_q11,"
    "kf_q22,"
    "kf_r11,"
    "kf_r22,"
    "kf_initialized,"

    /* Innovation can be reconstructed from raw measurement - prediction. */
    "kf_innov_of,"
    "kf_innov_ofd,"

    /* Lateral-controller context. */
    "vs_yaw_vel,"
    "vs_of_scale,"
    "vs_of_scaled,"
    "vs_of_gain,"
    "vs_yaw_gain,"
    "vs_mu_y_of,"
    "vs_mu_y_yaw,"
    "vs_mu_y_fallback,"
    "vs_mu_y_control,"
    "vs_using_of,"
    "vs_using_fallback,"
  );
}

static void logger_file_write_kf_visual_row(FILE *file)
{
  const float kf_innov_of =
    visual_servoing.raw_of_y - visual_servoing.kf_x1_pred;

  const float kf_innov_ofd =
    visual_servoing.raw_of_y_d - visual_servoing.kf_x2_pred;

  fprintf(
    file,

    "%u,%u,%u,"
    "%u,%u,%u,%u,"
    "%f,%f,%f,"
    "%u,"

    "%f,%f,%f,"

    "%f,%f,%f,%f,"

    "%f,%f,"

    "%f,%f,%f,%f,"

    "%f,%f,%f,%f,"
    "%u,"

    "%f,%f,"

    "%f,%f,%f,%f,%f,"
    "%f,%f,%f,%f,"
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

    visual_servoing.color_count,
    visual_servoing.box_centroid_x,
    visual_servoing.box_centroid_y,

    visual_servoing.raw_of_y,
    visual_servoing.of_y,
    visual_servoing.raw_of_y_d,
    visual_servoing.of_y_d,

    visual_servoing.kf_x1_pred,
    visual_servoing.kf_x2_pred,

    visual_servoing.kf_p11,
    visual_servoing.kf_p12,
    visual_servoing.kf_p21,
    visual_servoing.kf_p22,

    visual_servoing.kf_q11,
    visual_servoing.kf_q22,
    visual_servoing.kf_r11,
    visual_servoing.kf_r22,

    visual_servoing.kf_initialized ? 1U : 0U,

    kf_innov_of,
    kf_innov_ofd,

    visual_servoing.yaw_vel,
    visual_servoing.of_scale,
    visual_servoing.of_scaled,
    visual_servoing.ol_y_OF_gain,
    visual_servoing.ol_y_YAW_gain,

    visual_servoing.mu_y_of,
    visual_servoing.mu_y_yaw,
    visual_servoing.mu_y_fallback,
    visual_servoing.mu_y_control,

    visual_servoing.using_of_control ? 1U : 0U,
    visual_servoing.using_lateral_fallback ? 1U : 0U
  );
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

  logger_file_write_setup_header(file);
  logger_file_write_kf_visual_header(file);

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

  logger_file_write_setup_row(file);
  logger_file_write_kf_visual_row(file);

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
  logger_file_logger_file_periodic_status = MODULES_RUN;

  if (access(STRINGIFY(LOGGER_FILE_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(LOGGER_FILE_PATH));

    if (system(save_dir_cmd) != 0) {
      printf(
        "[logger_file] Could not create log file directory %s.\n",
        STRINGIFY(LOGGER_FILE_PATH)
      );
      return;
    }
  }

  char date_time[80];
  time_t now = time(0);
  struct tm tstruct = *localtime(&now);

  strftime(
    date_time,
    sizeof(date_time),
    "%Y%m%d-%H%M%S",
    &tstruct
  );

  uint32_t counter = 0U;
  char filename[512];

  sprintf(
    filename,
    "%s/%s.csv",
    STRINGIFY(LOGGER_FILE_PATH),
    date_time
  );

  while ((logger_file = fopen(filename, "r"))) {
    fclose(logger_file);

    sprintf(
      filename,
      "%s/%s_%05d.csv",
      STRINGIFY(LOGGER_FILE_PATH),
      date_time,
      counter
    );

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

  logger_last_write_usec =
    get_sys_time_usec() - write_start_usec;

  if (logger_last_write_usec >
      logger_max_write_usec) {

    logger_max_write_usec =
      logger_last_write_usec;
  }
}
