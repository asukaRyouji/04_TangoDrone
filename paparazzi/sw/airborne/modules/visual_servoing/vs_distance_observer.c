/* B54 shadow range observer, frozen KF50 and command-only response model. */
#ifndef VS_OBSERVER_STANDALONE
#include "generated/airframe.h"
#endif
#include "vs_distance_observer.h"
#include <math.h>
#include <string.h>

struct VsDistanceObserver vs_distance_observer;
static const CountOEKFParameters oe_parameters = {VS_OE_Q, .015, 1., .5, 2.};
static double clip(double x, double lo, double hi) { return fmax(lo, fmin(x, hi)); }

void vs_dobs_init(struct VsDistanceObserver *s)
{
  memset(s, 0, sizeof(*s));
  s->rho = s->base_rho = 1. / VS_OBS_INITIAL_R_M;
  s->range_m = VS_OBS_INITIAL_R_M;
  s->reason = VS_OBS_STARTUP;
}

void vs_dobs_reset_range(struct VsDistanceObserver *s, enum VsObserverReason why)
{
  s->started = s->valid = false;
  s->timed_out = false;
  s->reason = why;
  s->resets++;
}

struct VsDynamics { double z,y,v; };

static void model_part(struct VsDynamics *m, double speed, double h)
{
  const double T=VS_MODEL_T_S, c=VS_MODEL_DRAG, gain=VS_MODEL_GAIN;
  const double ec=exp(-c*h), et=exp(-h/T), J=-expm1(-c*h)/c;
  const double I=fabs(c*h)<1.e-4 ? h*h*(.5-c*h/6+c*c*h*h/24) : (h-J)/c;
  const double K=(et-ec)/(c-1/T);
  m->v=ec*m->v+gain*((m->z-speed*T)*J+speed*I+(m->y-m->z+speed*T)*K);
  const double k=-expm1(-h/T);
  m->y+=k*(m->z-m->y)+speed*(h-T*k);
  m->z+=speed*h;
}

static void model_advance(struct VsDynamics *m,double old_u,double dt,double u)
{
  if (dt>0) {
    const double slope=(u-old_u)/dt, delta=old_u-m->z, rate=VS_MODEL_SLEW;
    const double speed=fabs(delta)>1.e-12 ? copysign(rate,delta) : clip(slope,-rate,rate);
    const double hit=fabs(speed-slope)>1.e-20 ? delta/(speed-slope) : INFINITY;
    if (fabs(delta)>1.e-12 && hit>1.e-13 && hit<dt) {
      model_part(m,speed,hit); model_part(m,clip(slope,-rate,rate),dt-hit);
    } else { model_part(m,speed,dt); }
  }
}

void vs_model_command(struct VsModel *m, double t, double u, bool valid, bool quiet)
{
  if (!valid || !isfinite(t) || !isfinite(u)) {
    if (m->seeded) { m->resets++; }
    m->seeded=m->ready=false; m->count=m->head=0; m->warm_elapsed_s=0;
    return;
  }
  if (!m->seeded) {
    /* A genuinely quiet start is required for the unmeasured v_m(0)=0 prior. */
    if (!quiet) { return; }
    m->t=m->warm_start_s=t; m->z=m->y=m->u=u; m->v=0;
    m->seeded=true;
  } else {
    const double dt=t-m->t;
    if (dt<0 || dt>VS_MODEL_COMMAND_MAX_GAP_S) {
      m->seeded=m->ready=false; m->count=m->head=0; m->resets++; return;
    }
    struct VsDynamics dynamics={m->z,m->y,m->v};
    model_advance(&dynamics,m->u,dt,u);
    m->z=dynamics.z; m->y=dynamics.y; m->v=dynamics.v;
    m->t=t; m->u=u;
  }
  if (quiet) {
    m->warm_elapsed_s=t-m->warm_start_s;
    if (m->warm_elapsed_s>=VS_MODEL_QUIET_WARMUP_S) { m->ready=true; }
  }
  m->ring[m->head]=(struct VsModelPoint){t,m->z,m->y,m->v,m->u};
  m->head=(m->head+1)%VS_MODEL_RING_SIZE;
  if (m->count<VS_MODEL_RING_SIZE) { m->count++; }
  m->sequence++;
}

bool vs_model_acceleration(const struct VsModel *m, double query_s, double *value)
{
  if (!m->ready || m->count<2 || !isfinite(query_s)) { return false; }
  const unsigned first=(m->head+VS_MODEL_RING_SIZE-m->count)%VS_MODEL_RING_SIZE;
  struct VsModelPoint a=m->ring[first];
  if (query_s<a.t) { return false; }
  for (unsigned i=1;i<m->count;i++) {
    const struct VsModelPoint b=m->ring[(first+i)%VS_MODEL_RING_SIZE];
    if (query_s<=b.t && b.t>a.t) {
      const double dt=query_s-a.t;
      struct VsDynamics predicted_state={a.z,a.y,a.v};
      const double input=a.u+(b.u-a.u)*dt/(b.t-a.t);
      model_advance(&predicted_state,a.u,dt,input);
      *value=VS_MODEL_GAIN*predicted_state.y-VS_MODEL_DRAG*predicted_state.v;
      return true;
    }
    a=b;
  }
  return false; /* No extrapolation and no uninitialized ring entries. */
}

static void derivative(double h,double rho,const struct VsObserverInput *u,double *dh,double *dr)
{
  const double e=h-u->F, D=1+VS_OBS_AO*u->g*u->g+VS_OBS_AA*u->U*u->U;
  const double raw=VS_OBS_GAMMA/(1+VS_OBS_ETA*u->K*u->K)*u->A*e/D;
  const double lim=VS_OBS_C_REL*fmax(rho,1.e-12);
  *dh=u->F*u->E-rho*u->A-VS_OBS_L*e;
  *dr=rho*u->E+clip(raw,-lim,lim);
  if ((rho<=.05 && *dr<0)||(rho>=20 && *dr>0)) { *dr=0; }
  if ((h<=-20 && *dh<0)||(h>=20 && *dh>0)) { *dh=0; }
}

void vs_dobs_integrate(double *h,double *rho,const struct VsObserverInput *u,
                       double dt,double max_step_s,uint32_t *steps)
{
  if (!(dt>0) || !isfinite(dt) || !(max_step_s>0)) { return; }
  const unsigned n=(unsigned)ceil(dt/max_step_s);
  const double d=dt/n;
  for (unsigned i=0;i<n;i++) {
    double ah,ar,bh,br,ch,cr,dh,dr;
    derivative(*h,*rho,u,&ah,&ar);
    derivative(*h+d*ah/2,*rho+d*ar/2,u,&bh,&br);
    derivative(*h+d*bh/2,*rho+d*br/2,u,&ch,&cr);
    derivative(*h+d*ch,*rho+d*cr,u,&dh,&dr);
    *h=clip(*h+d*(ah+2*bh+2*ch+dh)/6,-20,20);
    *rho=clip(*rho+d*(ar+2*br+2*cr+dr)/6,.05,20);
  }
  if (steps) { *steps+=n; }
}

static bool past_mean(const struct VsDistanceObserver *s,double end,double *mean)
{
  if (s->of_count<2) { return false; }
  const unsigned first=(s->of_head+VS_OF_RING_SIZE-s->of_count)%VS_OF_RING_SIZE;
  struct VsTimedValue a=s->of_history[first];
  const double start=end-VS_OBS_H0_WINDOW_S;
  if (a.t>start) { return false; }
  double integral=0, width=0;
  for (unsigned i=1;i<s->of_count;i++) {
    const struct VsTimedValue b=s->of_history[(first+i)%VS_OF_RING_SIZE];
    const double lo=fmax(start,a.t), hi=fmin(end,b.t);
    if (hi>lo && b.t>a.t) {
      const double vl=a.value+(b.value-a.value)*(lo-a.t)/(b.t-a.t);
      const double vh=a.value+(b.value-a.value)*(hi-a.t)/(b.t-a.t);
      integral+=(hi-lo)*(vl+vh)/2; width+=hi-lo;
    }
    a=b;
  }
  if (width<VS_OBS_H0_WINDOW_S-1.e-8) { return false; }
  *mean=integral/width; return true;
}

void vs_dobs_arm(struct VsDistanceObserver *s,double onset_s)
{
  if (!s->armed) { s->armed=true; s->onset_s=onset_s; s->reason=VS_OBS_WAIT_MODEL; }
}

static bool of_at(const struct VsDistanceObserver *s,double t,double *F)
{
  if (s->of_count<2) { return false; }
  const unsigned first=(s->of_head+VS_OF_RING_SIZE-s->of_count)%VS_OF_RING_SIZE;
  struct VsTimedValue a=s->of_history[first];
  if (t<a.t) { return false; }
  for (unsigned i=1;i<s->of_count;i++) {
    const struct VsTimedValue b=s->of_history[(first+i)%VS_OF_RING_SIZE];
    if (t<=b.t && b.t>a.t) {
      *F=a.value+(b.value-a.value)*(t-a.t)/(b.t-a.t); return true;
    }
    a=b;
  }
  return false;
}

void vs_dobs_frame(struct VsDistanceObserver *s,double t,double available,double count,
                   double F,double K,double command)
{
  const double dt=s->have_oe_time ? t-s->oe_time : 0;
  if (!(count>0) || !isfinite(count) || !isfinite(F) || !isfinite(K) || !isfinite(command) ||
      (s->have_oe_time && !(dt>0)) || t>available+1.e-6) {
    vs_dobs_reset_range(s,VS_OBS_BAD_INPUT); return;
  }
  if (!s->have_oe_time || dt>.2) {
    count_oe_kf_init(&s->oe,count,&oe_parameters); s->oe_innovation=0;
    s->oe_variance=1+.015*.015*count*count;
  } else {
    const double predicted=s->oe.count*exp(2*s->oe.OE*dt+s->oe.dot_OE*dt*dt);
    s->oe_innovation=count-predicted;
    s->oe_variance=1+.015*.015*predicted*predicted;
    if (count_oe_kf_step(&s->oe,count,dt,&oe_parameters)) {
      s->have_oe_time=false; vs_dobs_reset_range(s,VS_OBS_BAD_INPUT); return;
    }
  }
  s->oe_output=VS_OE_GAIN*s->oe.OE;
  s->have_oe_time=true; s->oe_time=t;
  if (dt>VS_OBS_TIMEOUT_S) {
    s->of_count=s->of_head=0; vs_dobs_reset_range(s,VS_OBS_VISION_GAP);
  }
  s->of_history[s->of_head]=(struct VsTimedValue){t,F};
  s->of_head=(s->of_head+1)%VS_OF_RING_SIZE;
  if (s->of_count<VS_OF_RING_SIZE) { s->of_count++; }
  s->frames++; s->last_frame_s=t;
  if (!s->armed || t<s->onset_s) { s->reason=VS_OBS_WAIT_ONSET; return; }
  double old_F;
  if (!of_at(s,t-VS_OBS_NORMALIZATION_DELAY_S,&old_F)) {
    vs_dobs_reset_range(s,VS_OBS_WAIT_HISTORY); return;
  }
  /* Preserve the uncalibrated original frontend g at its fixed 45 ms epoch.
   * The observer-only OF gain must not also rescale these normalizers. */
  const double g=old_F/VS_OBS_OF_GAIN, U=command-K*g;
  if (available-t>VS_OBS_TIMEOUT_S) {
    vs_dobs_reset_range(s,VS_OBS_VISION_GAP); return;
  }
  s->acceleration_query_s=t-VS_OBS_ACCEL_ALIGNMENT_S-VS_MODEL_DELAY_S;
  double A;
  if (!vs_model_acceleration(&s->model,s->acceleration_query_s,&A)) {
    vs_dobs_reset_range(s,VS_OBS_WAIT_MODEL); return;
  }
  if (!s->started) {
    if (!past_mean(s,t,&s->initial_h)) { s->reason=VS_OBS_WAIT_HISTORY; return; }
    s->base_h=s->initial_h; s->base_rho=1./VS_OBS_INITIAL_R_M;
    s->base_t=t; s->started=true; s->timed_out=false;
  } else {
    vs_dobs_integrate(&s->base_h,&s->base_rho,&s->held,t-s->base_t,
                      VS_OBS_MAX_STEP_S,&s->integration_steps);
    s->base_t=t;
  }
  s->held=(struct VsObserverInput){F,s->oe_output,A,K,g,U};
  vs_dobs_propagate(s,available);
}

void vs_dobs_propagate(struct VsDistanceObserver *s,double now)
{
  s->available_s=now;
  if (!s->started) { s->valid=false; return; }
  const double age=now-s->last_frame_s;
  const double horizon=fmin(now,s->last_frame_s+VS_OBS_TIMEOUT_S);
  s->h=s->base_h; s->rho=s->base_rho;
  vs_dobs_integrate(&s->h,&s->rho,&s->held,horizon-s->base_t,
                    VS_OBS_MAX_STEP_S,&s->integration_steps);
  s->range_m=1/s->rho;
  s->estimate_epoch_s=horizon-VS_OBS_ACCEL_ALIGNMENT_S;
  s->residual=s->h-s->held.F;
  const double D=1+VS_OBS_AO*s->held.g*s->held.g+VS_OBS_AA*s->held.U*s->held.U;
  s->correction_raw=VS_OBS_GAMMA/(1+VS_OBS_ETA*s->held.K*s->held.K)*
                    s->held.A*s->residual/D;
  s->correction=clip(s->correction_raw,-VS_OBS_C_REL*s->rho,VS_OBS_C_REL*s->rho);
  s->timed_out=age>VS_OBS_TIMEOUT_S;
  s->valid=age>=0 && !s->timed_out && s->model.ready;
  if (age<0) { s->reason=VS_OBS_BAD_INPUT; }
  else if (s->timed_out) { s->reason=VS_OBS_VISION_GAP; }
  else if (!s->model.ready) { s->reason=VS_OBS_MODEL_GAP; }
  else { s->reason=VS_OBS_OK; }
}
