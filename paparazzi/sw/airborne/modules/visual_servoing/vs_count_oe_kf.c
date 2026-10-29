#include "vs_count_oe_kf.h"
#include <math.h>
#include <string.h>

static void predict_cov(const double *A,const double *P,const double *Q,double *out){
  double covariance_product[9]={0};
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)covariance_product[i*3+j]+=A[i*3+k]*P[k*3+j];
  for(int i=0;i<3;i++)for(int j=0;j<3;j++){
    out[i*3+j]=Q[i*3+j];for(int k=0;k<3;k++)out[i*3+j]+=covariance_product[i*3+k]*A[j*3+k];
  }
}
static int correct(double *x,double *P,double z,double R){
  double S=P[0]+R;if(!(S>0)||!isfinite(S))return -1;
  double K[3]={P[0]/S,P[3]/S,P[6]/S},J[9]={1,0,0,0,1,0,0,0,1},Pnew[9],noise[9];
  double innovation=z-x[0];
  for(int i=0;i<3;i++){x[i]+=K[i]*innovation;J[i*3]-=K[i];}
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)noise[i*3+j]=R*K[i]*K[j];
  predict_cov(J,P,noise,Pnew);
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)P[i*3+j]=.5*(Pnew[i*3+j]+Pnew[j*3+i]);
  for(int i=0;i<9;i++)if(!isfinite(P[i]))return -2;
  for(int i=0;i<3;i++)if(!isfinite(x[i])||P[i*3+i]<=0)return -3;
  return 0;
}
static double count_variance(double count,const CountOEKFParameters *p){
  return p->absolute_count_sd*p->absolute_count_sd+p->relative_count_sd*p->relative_count_sd*count*count;
}
void count_oe_kf_init(CountOEKF *s,double count,const CountOEKFParameters *p){
  s->count=count;s->OE=0;s->dot_OE=0;memset(s->P,0,9*sizeof(double));
  s->P[0]=count_variance(count,p);s->P[4]=p->initial_OE_sd*p->initial_OE_sd;
  s->P[8]=p->initial_dot_OE_sd*p->initial_dot_OE_sd;s->initialized=1;
}
int count_oe_kf_step(CountOEKF *s,double measured,double dt,const CountOEKFParameters *p){
  if(!(measured>0)||!isfinite(measured))return -10;
  if(!s->initialized){count_oe_kf_init(s,measured,p);return 0;}
  if(!(dt>0)||!isfinite(dt))return -11;
  double dt2=dt*dt,dt3=dt2*dt,dt4=dt3*dt,dt5=dt4*dt;
  double exponent=2*s->OE*dt+s->dot_OE*dt2;
  if(fabs(exponent)>20)return -12;
  double factor=exp(exponent),predicted=s->count*factor;
  double x[3]={predicted,s->OE+s->dot_OE*dt,s->dot_OE};
  double A[9]={factor,2*dt*predicted,dt2*predicted,0,1,dt,0,0,1};
  /* Exact integrated white driver in [log(sqrt count),OE,dotOE], mapped
   * to count coordinates at the predicted state with the EKF Jacobian. */
  double Q[9]={dt5/20,dt4/8,dt3/6,dt4/8,dt3/3,dt2/2,dt3/6,dt2/2,dt};
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)Q[i*3+j]*=p->q_dot_OE_driver*(i==0?2*predicted:1)*(j==0?2*predicted:1);
  double P[9];predict_cov(A,s->P,Q,P);
  int code=correct(x,P,measured,count_variance(predicted,p));if(code)return code;
  /* Candidate rejection bounds, not clipping or projection. */
  if(x[0]<1e-6||x[0]>1e7||fabs(x[1])>20||fabs(x[2])>200)return -13;
  s->count=x[0];s->OE=x[1];s->dot_OE=x[2];memcpy(s->P,P,sizeof(P));return 0;
}
