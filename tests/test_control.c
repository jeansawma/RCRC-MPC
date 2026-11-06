#include "control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static Controller controller,other;
static unsigned checks;
#define CHECK(x) do { if(!(x)){fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);assert(x);}checks++; }while(0)
static float clip(float x,float lo,float hi){return fmaxf(lo,fminf(hi,x));}
static uint32_t rng=1789;
static float random_fraction(void){rng=rng*1664525u+1013904223u;return (float)(rng>>8)/16777216.f;}
static void validation(void){
 ControlConfig c;char why[80];ctrl_defaults(&c);CHECK(ctrl_validate(&c,why,sizeof(why)));
 for(size_t i=0;i<ctrl_param_count;i++){
  float value;CHECK(ctrl_config_get(&c,ctrl_params[i].name,&value));
  CHECK(!ctrl_config_set(&c,ctrl_params[i].name,NAN));
  CHECK(!ctrl_config_set(&c,ctrl_params[i].name,INFINITY));
  CHECK(!ctrl_config_set(&c,ctrl_params[i].name,ctrl_params[i].minimum-1));
  CHECK(ctrl_config_set(&c,ctrl_params[i].name,value));
  for(size_t j=i+1;j<ctrl_param_count;j++)CHECK(strcmp(ctrl_params[i].name,ctrl_params[j].name)!=0);
 }
 CHECK(!ctrl_config_set(&c,"unknown",1));c.reference=31;CHECK(!ctrl_validate(&c,why,sizeof(why)));
 ctrl_defaults(&c);c.sampling_us=1000.5f;CHECK(!ctrl_validate(&c,why,sizeof(why)));
 ctrl_defaults(&c);c.mode=CTRL_FCS;c.slew=.5f;CHECK(!ctrl_validate(&c,why,sizeof(why)));
 ctrl_defaults(&c);c.duty_min=.5f;c.slew=.1f;CHECK(!ctrl_validate(&c,why,sizeof(why)));
 c.slew=.5f;CHECK(ctrl_validate(&c,why,sizeof(why)));
 for(int mode=0;mode<=2;mode+=2){
  c.mode=(float)mode;CHECK(ctrl_init(&controller,&c));CHECK(controller.previous_duty==0);
  ControlResult first;CHECK(ctrl_step(&controller,0,0,30,&first)==CTRL_OK);
  CHECK(first.duty>=c.duty_min&&first.duty<=c.slew);
 }
 ctrl_defaults(&c);CHECK(ctrl_init(&controller,&c));
 ControlResult out;CHECK(ctrl_step(&controller,NAN,0,30,&out)==CTRL_BAD_MEASUREMENT);CHECK(out.duty==0);
 CHECK(ctrl_step(&controller,0,0,31,&out)==CTRL_BAD_MEASUREMENT);
 CHECK(ctrl_step(&controller,0,0,0,&out)==CTRL_BAD_MEASUREMENT);
 CHECK(ctrl_step(&controller,31,0,30,&out)==CTRL_BAD_MEASUREMENT);
 CHECK(ctrl_init(&controller,&controller.cfg)); /* aliased reinitialization */
}
static void model(void){
 ControlConfig c;ctrl_defaults(&c);c.sampling_us=1000;CHECK(ctrl_init(&controller,&c));
 /* A resistor current balance establishes this equilibrium independently. */
 float y=5,x=y*(1+c.r2/c.load2);
 float u=x+c.r1*(x/c.load1+y/c.load2),nx,ny;
 ctrl_predict(&controller,x,y,u,&nx,&ny);
 CHECK(fabsf(nx-x)<2e-6f);CHECK(fabsf(ny-y)<2e-6f);
 /* exp(A t) must equal exp(A t/2)^2, including the held-input integral. */
 c.sampling_us=500;CHECK(ctrl_init(&other,&c));
 for(int i=0;i<100;i++){
  x=random_fraction()*30;y=random_fraction()*30;u=random_fraction()*30;
  float hx,hy,fx,fy;
  ctrl_predict(&controller,x,y,u,&nx,&ny);
  ctrl_predict(&other,x,y,u,&hx,&hy);ctrl_predict(&other,hx,hy,u,&fx,&fy);
  CHECK(fabsf(nx-fx)<1e-5f);CHECK(fabsf(ny-fy)<1e-5f);
 }
}
static void scalar_qp_oracle(void){
 /* Horizon-one problem has a closed-form minimizer; no solver reused. */
 for(int i=0;i<200;i++){
  ControlConfig c;ctrl_defaults(&c);c.mode=CTRL_CSMPC;c.mpc_horizon=1;
  c.reference=random_fraction()*25;c.duty_min=.1f;c.duty_max=.9f;
  c.slew=.1f+random_fraction()*.42f;c.mpc_q=.1f+random_fraction()*10;
  c.mpc_r=.01f+random_fraction();CHECK(ctrl_init(&controller,&c));
  controller.previous_duty=.1f+.8f*random_fraction();float previous=controller.previous_duty;
  float x=random_fraction()*25,y=random_fraction()*25,bus=10+random_fraction()*20;
  float unused,free_y,high_y;ctrl_predict(&controller,x,y,c.low_drop,&unused,&free_y);
  ctrl_predict(&controller,x,y,bus-c.high_drop,&unused,&high_y);
  float response=high_y-free_y;
  float expected=(c.mpc_r*previous+c.mpc_q*response*(c.reference-free_y))/
                 (c.mpc_r+c.mpc_q*response*response);
  expected=clip(expected,fmaxf(c.duty_min,previous-c.slew),fminf(c.duty_max,previous+c.slew));
  ControlResult out;CHECK(ctrl_step(&controller,x,y,bus,&out)==CTRL_OK);
  CHECK(fabsf(out.duty-expected)<5e-5f);CHECK(out.qp_residual<.002f);
 }
}
static void random_qp(void){
 for(int i=0;i<256;i++){
  ControlConfig c;ctrl_defaults(&c);c.mode=CTRL_CSMPC;c.mpc_horizon=(float)(1+(i%CTRL_MPC_MAX));
  c.reference=random_fraction()*25;c.slew=.01f+random_fraction()*.99f;
  c.mpc_q=.1f+random_fraction()*3;c.mpc_r=.02f+random_fraction();
  CHECK(ctrl_init(&controller,&c));controller.previous_duty=random_fraction();
  float previous=controller.previous_duty,x=random_fraction()*25,y=random_fraction()*25;
  ControlResult out;ControlStatus status=ctrl_step(&controller,x,y,30,&out);
  if(status!=CTRL_OK)fprintf(stderr,"random QP i%d horizon%g failed status%d iter%u\n",i,c.mpc_horizon,status,out.iterations);
  CHECK(status==CTRL_OK);CHECK(out.duty>=c.duty_min&&out.duty<=c.duty_max);
  CHECK(fabsf(out.duty-previous)<=c.slew+2e-6f);CHECK(out.iterations<=96);CHECK(out.qp_residual<.002f);
 }
}
static void antiwindup(void){
 ControlConfig c;ctrl_defaults(&c);c.reference=30;CHECK(ctrl_init(&controller,&c));
 ControlResult out;
 for(int i=0;i<5000;i++){CHECK(ctrl_step(&controller,0,0,5,&out)==CTRL_OK);CHECK(out.duty==1);}
 CHECK(fabsf(controller.integral)<1e-6f);
 controller.cfg.reference=0;CHECK(ctrl_step(&controller,0,0,5,&out)==CTRL_OK);CHECK(out.duty==0);
 CHECK(controller.derivative==0); /* no derivative kick on reference change */
}
static void closed_loop(void){
 for(int mode=0;mode<3;mode++){
  ControlConfig c;ctrl_defaults(&c);c.mode=(float)mode;CHECK(ctrl_init(&controller,&c));
  float x=0,y=0,total=0,min=100,max=-100;ControlResult out;
  int steps=(int)(5000000/c.sampling_us),tail=(int)(1000000/c.sampling_us);
  for(int i=0;i<steps;i++){
   CHECK(ctrl_step(&controller,x,y,30,&out)==CTRL_OK);
   CHECK(isfinite(out.duty)&&out.duty>=0&&out.duty<=1);
   if(mode==CTRL_FCS)CHECK(out.duty==0||out.duty==1);
   ctrl_predict(&controller,x,y,30*out.duty,&x,&y);
   if(i>=steps-tail){total+=y;min=fminf(min,y);max=fmaxf(max,y);}
  }
  float average=total/tail;
  printf("mode%d 5s: mean %.6f V, ripple %.6f V\n",mode,average,max-min);
  CHECK(fabsf(average-c.reference)<(mode==CTRL_FCS?.2f:.02f));
  CHECK(max-min<(mode==CTRL_FCS?1.f:.02f));
 }
}
int main(void){validation();model();scalar_qp_oracle();random_qp();antiwindup();closed_loop();printf("PASS %u checks\n",checks);return 0;}
