#ifndef VS_DISTANCE_OBSERVER_H
#define VS_DISTANCE_OBSERVER_H
#include <stdbool.h>
#include <stdint.h>
#include "vs_count_oe_kf.h"
#include "vs_observer_config.h"

enum VsObserverReason {
  VS_OBS_STARTUP=0, VS_OBS_OK=1, VS_OBS_WAIT_MODEL=2, VS_OBS_WAIT_HISTORY=3,
  VS_OBS_VISION_GAP=4, VS_OBS_MODE_EXIT=5, VS_OBS_BAD_INPUT=6,
  VS_OBS_MODEL_GAP=7, VS_OBS_WAIT_ONSET=8
};
struct VsTimedValue { double t, value; };
struct VsModelPoint { double t, z, y, v, u; };
struct VsModel {
  double t, z, y, v, u, warm_start_s, warm_elapsed_s;
  bool seeded, ready;
  uint32_t sequence, resets;
  unsigned head, count;
  struct VsModelPoint ring[VS_MODEL_RING_SIZE];
};
struct VsObserverInput { double F, E, A, K, g, U; };
struct VsDistanceObserver {
  CountOEKF oe;
  struct VsModel model;
  struct VsObserverInput held;
  struct VsTimedValue of_history[VS_OF_RING_SIZE];
  struct VsTimedValue oe_history[VS_OF_RING_SIZE];
  unsigned of_head, of_count;
  double oe_time, oe_innovation, oe_variance, oe_output;
  double base_h, base_rho, base_t, h, rho, range_m, initial_h;
  double residual, correction_raw, correction, estimate_epoch_s, available_s;
  double last_frame_s, acceleration_query_s, onset_s;
  bool have_oe_time, armed, started, valid, timed_out;
  uint32_t frames, resets, integration_steps, cap_evaluations, bound_steps;
  enum VsObserverReason reason;
};
extern struct VsDistanceObserver vs_distance_observer;
void vs_dobs_init(struct VsDistanceObserver *s);
void vs_dobs_reset_range(struct VsDistanceObserver *s, enum VsObserverReason why);
/* During quiet NAV warmup use u=0 and quiet=true. In MODULE use actual bounded
 * pre-slew command and quiet=false. Invalid/non-VS mode must invalidate history. */
void vs_model_command(struct VsModel *m, double t, double u, bool valid, bool quiet);
bool vs_model_acceleration(const struct VsModel *m, double query_s, double *value);
void vs_dobs_arm(struct VsDistanceObserver *s, double onset_s);
/* Once per accepted visual sample; source epoch and availability are distinct. */
void vs_dobs_frame(struct VsDistanceObserver *s, double capture_s, double available_s,
                   double count, double F, double delayed_K, double delayed_command);
/* Predictions are derived from the last capture-epoch base state. They never
 * consume the same measurement twice or move that base beyond a future frame. */
void vs_dobs_propagate(struct VsDistanceObserver *s, double now_s);
void vs_dobs_integrate(double *h, double *rho, const struct VsObserverInput *u,
                       double dt, double max_step_s, uint32_t *steps);
#endif
