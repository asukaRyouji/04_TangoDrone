/*
 *
 * Copyright (C) 2009-2011 The Paparazzi Team
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
 */

/**
 * @file arch/sim/mcu_periph/sys_time_arch.c
 * Handling of sys_time in sim.
 */

#include "mcu_periph/sys_time.h"
#include "mcu_periph/sim_clock_bridge.h"
#include <math.h>

/* Opt in only for this experiment's NPS target; all other builds keep legacy timing. */
#ifndef NPS_GAZEBO_SYNC_TIME
#define NPS_GAZEBO_SYNC_TIME 0
#endif
static uint64_t sim_cpu_fraction;
struct SimClockBridge sim_clock_bridge;

void sim_clock_pair_update(double gazebo_s, double physics_step_s)
{
  const double system_s = sys_time.nb_sec + 1.e-6 * sys_time.nb_sec_rem;
  if (!sim_clock_bridge.ready) {
    sim_clock_bridge.initial_offset_s = gazebo_s - system_s;
    sim_clock_bridge.ready = 1;
  }
  sim_clock_bridge.gazebo_s = gazebo_s;
  sim_clock_bridge.system_s = system_s;
  sim_clock_bridge.drift_s = gazebo_s - system_s - sim_clock_bridge.initial_offset_s;
  sim_clock_bridge.synchronized = NPS_GAZEBO_SYNC_TIME;
  sim_clock_bridge.system_per_gazebo =
    (NPS_GAZEBO_SYNC_TIME ? 1. / sys_time.ticks_per_sec :
     1.e-6 * sys_time.resolution_cpu_ticks) / physics_step_s;
  sim_clock_bridge.pairs++;
}

uint32_t sim_clock_capture_to_sys_us(uint32_t capture_us)
{
  if (!sim_clock_bridge.ready) { return capture_us; }
  const uint32_t world_us = (uint32_t)(uint64_t)llround(sim_clock_bridge.gazebo_s * 1.e6);
  const int32_t age_delta = (int32_t)(capture_us - world_us);
  const int64_t mapped = (int64_t)llround(sim_clock_bridge.system_s * 1.e6) +
    (int64_t)llround(age_delta * sim_clock_bridge.system_per_gazebo);
  return (uint32_t)mapped;
}


void sys_time_arch_init(void)
{
  // simulate 1us cpu ticks
  sys_time.cpu_ticks_per_sec = 1e6;
  sys_time.resolution_cpu_ticks = (uint32_t)(sys_time.resolution * sys_time.cpu_ticks_per_sec + 0.5);
  sim_cpu_fraction = 0;
  sim_clock_bridge = (struct SimClockBridge){0};
}

void sys_tick_handler(void)
{
  sys_time.nb_tick++;
#if NPS_GAZEBO_SYNC_TIME
  /* Rational accumulator: retain the sub-microsecond part of each tick. */
  sim_cpu_fraction += sys_time.cpu_ticks_per_sec;
  sys_time.nb_sec_rem += sim_cpu_fraction / sys_time.ticks_per_sec;
  sim_cpu_fraction %= sys_time.ticks_per_sec;
#else
  sys_time.nb_sec_rem += sys_time.resolution_cpu_ticks;
#endif
  if (sys_time.nb_sec_rem >= sys_time.cpu_ticks_per_sec) {
    sys_time.nb_sec_rem -= sys_time.cpu_ticks_per_sec;
    sys_time.nb_sec++;
  }
  for (unsigned int i = 0; i < SYS_TIME_NB_TIMER; i++) {
    if (sys_time.timer[i].in_use &&
        sys_time.nb_tick >= sys_time.timer[i].end_time) {
      sys_time.timer[i].end_time += sys_time.timer[i].duration;
      sys_time.timer[i].elapsed = true;
      if (sys_time.timer[i].cb) {
        sys_time.timer[i].cb(i);
      }
    }
  }
}
