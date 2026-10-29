#ifndef VS_EXPERIMENT_CHECK_H
#define VS_EXPERIMENT_CHECK_H
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "vs_observer_config.h"
/* Target/module defines are compiler flags in this Paparazzi revision. Check
 * the running binary itself rather than expecting them in generated airframe.h. */
static inline int vs_experiment_value_matches(const char *name,double compiled)
{
  const char *text=getenv(name);
  if (!text) { return 1; }
  char *end;
  const double requested=strtod(text,&end);
  if (!*text || *end || !isfinite(requested) || fabs(requested-compiled)>1.e-9) {
    fprintf(stderr,"[B54] %s requests %s; running NPS was compiled with %.12g. Rebuild the selected case.\n",
            name,text,compiled);
    return 0;
  }
  return 1;
}
static inline int vs_experiment_matches(void)
{
#ifdef VS_EXPERIMENT_CASE_ID
  return vs_experiment_value_matches("B54_EXPECT_CASE",VS_EXPERIMENT_CASE_ID) &&
    vs_experiment_value_matches("B54_EXPECT_CLOCK",NPS_GAZEBO_SYNC_TIME) &&
    vs_experiment_value_matches("B54_EXPECT_FREQ",VS_SELF_OSC_FREQ_HZ) &&
    vs_experiment_value_matches("B54_EXPECT_A1",VS_SELF_OSC_AMPLITUDE) &&
    vs_experiment_value_matches("B54_EXPECT_A2",VS_SELF_OSC_H2_AMPLITUDE) &&
    vs_experiment_value_matches("B54_EXPECT_X0",VS_EXPERIMENT_X0_M) &&
    vs_experiment_value_matches("B54_EXPECT_ALIGNMENT",VS_OBS_ACCEL_ALIGNMENT_S);
#else
  return 1;
#endif
}
#endif
