/* NPS-only paired clock diagnostics. No dependency on simulated truth states. */
#ifndef SIM_CLOCK_BRIDGE_H
#define SIM_CLOCK_BRIDGE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
struct SimClockBridge {
  double gazebo_s, system_s, initial_offset_s, drift_s, system_per_gazebo;
  uint32_t pairs;
  int ready, synchronized;
};
extern struct SimClockBridge sim_clock_bridge;
void sim_clock_pair_update(double gazebo_s, double physics_step_s);
uint32_t sim_clock_capture_to_sys_us(uint32_t capture_us);
#ifdef __cplusplus
}
#endif
#endif
