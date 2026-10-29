#ifndef COUNT_OE_KF_H
#define COUNT_OE_KF_H
/* Standalone count EKF; no flight-controller or ground-truth dependency.
 * State OE is image expansion (0.5*d(log count)/dt); apply the trained output
 * calibration separately to obtain the evaluated OEproxy approximation.
 * Use one consistent time unit for dt, process noise, and derivative states.
 */
typedef struct { double count, OE, dot_OE, P[9]; int initialized; } CountOEKF;
typedef struct {
  double q_dot_OE_driver; /* continuous white-noise spectral density, s^-5 */
  double relative_count_sd, absolute_count_sd;
  double initial_OE_sd, initial_dot_OE_sd;
} CountOEKFParameters;
void count_oe_kf_init(CountOEKF *, double count, const CountOEKFParameters *);
int count_oe_kf_step(CountOEKF *, double count, double dt, const CountOEKFParameters *);
#endif
