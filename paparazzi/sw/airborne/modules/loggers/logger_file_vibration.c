/*
 * Compact high-rate vibration logger for the Bebop / Paparazzi hover tests.
 *
 * Replace:
 *   sw/airborne/modules/loggers/logger_file.c
 * with this file only for the short vibration-identification tests.
 *
 * Recommended logger frequency:
 *   500 Hz minimum; use 1000 Hz only after checking CPU/write timing.
 *
 * This logger intentionally omits the large guidance/EKF diagnostic row.
 * It records only the signals needed to distinguish:
 *   gyro/IMU vibration, real attitude motion, INDI derivative response,
 *   and motor/RPM excitation.
 */

#include "logger_file.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "std.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include "generated/airframe.h"
#include "generated/modules.h"

#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#endif

#include "modules/ins/ins_ext_pose.h"

#ifndef COMMAND_THRUST
#error "logger_file_vibration.c is intended for rotorcraft builds with COMMAND_THRUST."
#endif

#ifndef BOARD_BEBOP
#error "logger_file_vibration.c is intended for the Bebop airframe."
#endif

/*
 * ekf_U and ekf_Z are global in ins_ext_pose.c but are not currently exposed
 * by ins_ext_pose.h. Local extern declarations avoid changing the header.
 *
 * ekf_U[0:2] = raw accelerometer inputs [m/s^2]
 * ekf_U[3:5] = raw gyro inputs [rad/s]
 * ekf_Z[6:8] = most recent external-attitude measurement [rad]
 */
extern float ekf_U[EKF_NUM_INPUTS];
extern float ekf_Z[EKF_NUM_OUTPUTS];

#ifndef LOGGER_FILE_PATH
#define LOGGER_FILE_PATH /data/video/usb
#endif

/*
 * Four MiB prevents routine storage writes during a short 10-15 s test.
 * fclose() flushes the buffer when logging is stopped normally.
 */
#ifndef VIB_LOG_BUFFER_SIZE
#define VIB_LOG_BUFFER_SIZE (4U * 1024U * 1024U)
#endif

static FILE *logger_file = NULL;
static char logger_file_buffer[VIB_LOG_BUFFER_SIZE];

static uint32_t logger_sample_count = 0;
static uint32_t logger_previous_sample_usec = 0;
static uint32_t logger_last_write_usec = 0;
static uint32_t logger_max_write_usec = 0;

static void logger_file_write_header(FILE *file)
{
  fprintf(
    file,
    "sample,time_usec,logger_dt_usec,"

    /* Raw IMU supplied to the EKF. */
    "raw_acc_x,raw_acc_y,raw_acc_z,"
    "raw_gyro_p,raw_gyro_q,raw_gyro_r,"

    /* Estimated IMU biases. */
    "bias_acc_x,bias_acc_y,bias_acc_z,"
    "bias_gyro_p,bias_gyro_q,bias_gyro_r,"

    /* Bias-corrected rates published to state/INDI. */
    "body_p,body_q,body_r,"

    /* INDI gyro filter and angular-acceleration estimate. */
    "indi_rate_filt_p,indi_rate_filt_q,indi_rate_filt_r,"
    "indi_rate_d_p,indi_rate_d_q,indi_rate_d_r,"

    /* Rate demand and feedback actually used by INDI. */
    "indi_rate_sp_p,indi_rate_sp_q,indi_rate_sp_r,"
    "indi_rate_fb_p,indi_rate_fb_q,indi_rate_fb_r,"

    /* State attitude, external attitude, and attitude setpoint. */
    "att_phi,att_theta,att_psi,"
    "ext_phi,ext_theta,ext_psi,"
    "att_sp_phi,att_sp_theta,att_sp_psi,"

    /* Controller outputs. */
    "indi_du_p,indi_du_q,indi_du_r,"
    "cmd_roll,cmd_pitch,cmd_yaw,cmd_thrust,"

#ifdef BOARD_BEBOP
    /* Motor excitation and tracking. */
    "rpm_ref_1,rpm_ref_2,rpm_ref_3,rpm_ref_4,"
    "rpm_obs_1,rpm_obs_2,rpm_obs_3,rpm_obs_4,"
#endif

    /* Synchronisation and logger-health diagnostics. */
    "indi_rate_run_count,ext_rx_count,ext_new_sample,"
    "logger_write_usec,logger_max_write_usec\n"
  );
}

static void logger_file_write_row(FILE *file)
{
  const uint32_t now_usec = get_sys_time_usec();
  const uint32_t logger_dt_usec =
    (logger_previous_sample_usec > 0U)
      ? (now_usec - logger_previous_sample_usec)
      : 0U;
  logger_previous_sample_usec = now_usec;

  const struct FloatRates *body_rates = stateGetBodyRates_f();
  const struct FloatEulers *att = stateGetNedToBodyEulers_f();

  static uint32_t previous_ext_rx_count = 0U;
  const uint8_t ext_new_sample =
    (ins_ext_pose_rx_count != previous_ext_rx_count) ? 1U : 0U;
  previous_ext_rx_count = ins_ext_pose_rx_count;

  fprintf(
    file,
    "%u,%u,%u,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%f,%f,%f,"
    "%d,%d,%d,%d,",

    (unsigned int)logger_sample_count,
    (unsigned int)now_usec,
    (unsigned int)logger_dt_usec,

    ekf_U[0], ekf_U[1], ekf_U[2],
    ekf_U[3], ekf_U[4], ekf_U[5],

    ekf_X[9], ekf_X[10], ekf_X[11],
    ekf_X[12], ekf_X[13], ekf_X[14],

    body_rates->p, body_rates->q, body_rates->r,

    indi.rate[0].o[0], indi.rate[1].o[0], indi.rate[2].o[0],
    indi.rate_d[0], indi.rate_d[1], indi.rate_d[2],

    indi.rate_sp.p, indi.rate_sp.q, indi.rate_sp.r,
    indi.rate_feedback.p, indi.rate_feedback.q, indi.rate_feedback.r,

    att->phi, att->theta, att->psi,
    ekf_Z[6], ekf_Z[7], ekf_Z[8],

    ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.phi),
    ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.theta),
    ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.psi),

    indi.du.p, indi.du.q, indi.du.r,

    stabilization_cmd[COMMAND_ROLL],
    stabilization_cmd[COMMAND_PITCH],
    stabilization_cmd[COMMAND_YAW],
    stabilization_cmd[COMMAND_THRUST]
  );

#ifdef BOARD_BEBOP
  fprintf(
    file,
    "%d,%d,%d,%d,"
    "%d,%d,%d,%d,",
    actuators_bebop.rpm_ref[0],
    actuators_bebop.rpm_ref[1],
    actuators_bebop.rpm_ref[2],
    actuators_bebop.rpm_ref[3],
    actuators_bebop.rpm_obs[0],
    actuators_bebop.rpm_obs[1],
    actuators_bebop.rpm_obs[2],
    actuators_bebop.rpm_obs[3]
  );
#endif

  fprintf(
    file,
    "%u,%u,%u,%u,%u\n",
    (unsigned int)indi.rate_run_count,
    (unsigned int)ins_ext_pose_rx_count,
    (unsigned int)ext_new_sample,
    (unsigned int)logger_last_write_usec,
    (unsigned int)logger_max_write_usec
  );

  logger_sample_count++;
}

void logger_file_start(void)
{
  logger_file_logger_file_periodic_status = MODULES_RUN;

  if (access(STRINGIFY(LOGGER_FILE_PATH), F_OK)) {
    char save_dir_cmd[256];
    snprintf(
      save_dir_cmd,
      sizeof(save_dir_cmd),
      "mkdir -p %s",
      STRINGIFY(LOGGER_FILE_PATH)
    );

    if (system(save_dir_cmd) != 0) {
      printf(
        "[logger_file] Could not create log directory %s.\n",
        STRINGIFY(LOGGER_FILE_PATH)
      );
      return;
    }
  }

  char date_time[80];
  const time_t now = time(NULL);
  const struct tm *tstruct = localtime(&now);

  if (tstruct == NULL) {
    printf("[logger_file] localtime() failed.\n");
    return;
  }

  strftime(
    date_time,
    sizeof(date_time),
    "%Y%m%d-%H%M%S-vibration",
    tstruct
  );

  char filename[512];
  snprintf(
    filename,
    sizeof(filename),
    "%s/%s.csv",
    STRINGIFY(LOGGER_FILE_PATH),
    date_time
  );

  logger_file = fopen(filename, "w");
  if (logger_file == NULL) {
    printf("[logger_file] ERROR opening %s.\n", filename);
    return;
  }

  if (setvbuf(
        logger_file,
        logger_file_buffer,
        _IOFBF,
        sizeof(logger_file_buffer)
      ) != 0) {
    printf("[logger_file] WARNING: setvbuf() failed.\n");
  }

  logger_sample_count = 0U;
  logger_previous_sample_usec = 0U;
  logger_last_write_usec = 0U;
  logger_max_write_usec = 0U;

  logger_file_write_header(logger_file);
  fflush(logger_file);  /* Header only; no periodic flush during the test. */

  printf("[logger_file] High-rate vibration log: %s\n", filename);
}

void logger_file_stop(void)
{
  if (logger_file != NULL) {
    fclose(logger_file);  /* Flushes the buffered data. */
    logger_file = NULL;
  }
}

void logger_file_periodic(void)
{
  if (logger_file == NULL) {
    return;
  }

  const uint32_t write_start_usec = get_sys_time_usec();
  logger_file_write_row(logger_file);

  logger_last_write_usec = get_sys_time_usec() - write_start_usec;
  if (logger_last_write_usec > logger_max_write_usec) {
    logger_max_write_usec = logger_last_write_usec;
  }
}
