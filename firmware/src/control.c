#include "control.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define PARAM(n,u,lo,hi,i) {#n,u,offsetof(ControlConfig,n),lo,hi,i}
const ControlParam ctrl_params[]={
 PARAM(sampling_us,"us",500,10000,1),PARAM(mode,"enum",0,2,1),
 PARAM(reference,"V",0,30,0),PARAM(max_voltage,"V",1,30,0),PARAM(max_bus,"V",1,30,0),
 PARAM(r1,"ohm",1,100000,0),PARAM(r2,"ohm",1,100000,0),
 PARAM(c1,"F",1e-7f,0.1f,0),PARAM(c2,"F",1e-7f,0.1f,0),
 PARAM(load1,"ohm",100,1e9f,0),PARAM(load2,"ohm",100,1e9f,0),
 PARAM(kp,"duty/V",0,10,0),PARAM(ki,"duty/(V*s)",0,100,0),
 PARAM(kd,"duty*s/V",0,1,0),PARAM(derivative_tau,"s",0.0001f,1,0),
 PARAM(mpc_q,"1/V^2",0.0001f,100,0),PARAM(mpc_r,"1/duty^2",0.0001f,100,0),
 PARAM(mpc_horizon,"samples",1,CTRL_MPC_MAX,1),
 PARAM(duty_min,"fraction",0,0.99f,0),PARAM(duty_max,"fraction",0.01f,1,0),
 PARAM(slew,"duty/sample",0.0001f,1,0),PARAM(high_drop,"V",0,10,0),PARAM(low_drop,"V",0,5,0)
};
const size_t ctrl_param_count=sizeof(ctrl_params)/sizeof(ctrl_params[0]);
static float clamp(float x,float a,float b){return fminf(b,fmaxf(a,x));}
void ctrl_defaults(ControlConfig *c){
 *c=(ControlConfig){.sampling_us=5000,.mode=CTRL_PID,.reference=5,
 .max_voltage=30,.max_bus=30,.r1=470,.r2=470,.c1=100e-6f,.c2=100e-6f,
 .load1=55700,.load2=55700,.kp=0.05f,.ki=1,.kd=0,.derivative_tau=.01f,
 .mpc_q=1,.mpc_r=.05f,.mpc_horizon=8,.duty_min=0,.duty_max=1,.slew=1,
 .high_drop=0,.low_drop=0};
}
int ctrl_config_get(const ControlConfig*c,const char*n,float*v){
 for(size_t i=0;i<ctrl_param_count;i++)if(!strcmp(n,ctrl_params[i].name)){
  *v=*(const float*)((const char*)c+ctrl_params[i].offset);return 1;}
 return 0;
}
int ctrl_config_set(ControlConfig*c,const char*n,float v){
 for(size_t i=0;i<ctrl_param_count;i++)if(!strcmp(n,ctrl_params[i].name)){
  const ControlParam*p=&ctrl_params[i];
  if(!isfinite(v)||v<p->minimum||v>p->maximum||(p->integer&&v!=floorf(v)))return 0;
  *(float*)((char*)c+p->offset)=v;return 1;}
 return 0;
}
int ctrl_validate(const ControlConfig*c,char*reason,size_t capacity){
 const char *error=NULL;
 for(size_t i=0;i<ctrl_param_count;i++){
  const ControlParam*p=&ctrl_params[i];float v=*(const float*)((const char*)c+p->offset);
  if(!isfinite(v)||v<p->minimum||v>p->maximum||(p->integer&&v!=floorf(v))){error=p->name;break;}
 }
 if(!error&&c->reference>c->max_voltage)error="reference exceeds max_voltage";
 if(!error&&c->max_voltage>c->max_bus)error="max_voltage exceeds max_bus";
 if(!error&&c->duty_min>=c->duty_max)error="duty bounds";
 if(!error&&c->duty_min>c->slew)error="duty_min exceeds initial slew from zero";
 if(!error&&c->high_drop+c->low_drop>=c->max_bus)error="bridge drops exceed max_bus";
 if(!error&&(int)c->mode==CTRL_FCS&&(c->duty_min!=0||c->duty_max!=1||c->slew!=1))
  error="FCS needs duty_min=0 duty_max=1 slew=1";
 if(reason&&capacity)snprintf(reason,capacity,"%s",error?error:"ok");
 return error==NULL;
}
int ctrl_init(Controller*s,const ControlConfig*c){
 if(!ctrl_validate(c,NULL,0))return 0;
 ControlConfig copy=*c;
 memset(s,0,sizeof(*s));s->cfg=copy;c=&s->cfg;
 ctrl_reset(s);
 /* 2x2 exponential of continuous ladder matrix. Double precision here avoids
    cancellation in A^-1 (exp(A Ts)-I) for slow plants; ISR uses floats. */
 double a=-(1.0/c->r1+1.0/c->r2+1.0/c->load1)/c->c1;
 double b=1.0/(c->r2*c->c1),cc=1.0/(c->r2*c->c2);
 double d=-(1.0/c->r2+1.0/c->load2)/c->c2,t=c->sampling_us*1e-6;
 double h=(a+d)/2,delta=sqrt((a-d)*(a-d)/4+b*cc);
 /* Eigenvalue form avoids overflow from exp(h*t)*cosh(delta*t). */
 double ep=exp((h+delta)*t),em=exp((h-delta)*t);
 double co=(ep+em)/2,si=(ep-em)/(2*delta);
 double e00=co+si*(a-h),e01=si*b,e10=si*cc,e11=co+si*(d-h);
 s->a[0][0]=(float)e00;s->a[0][1]=(float)e01;
 s->a[1][0]=(float)e10;s->a[1][1]=(float)e11;
 double det=a*d-b*cc,bc=1.0/(c->r1*c->c1);
 s->b[0]=(float)((d*(e00-1)-b*e10)*bc/det);
 s->b[1]=(float)((-cc*(e00-1)+a*e10)*bc/det);
 for(int i=0;i<2;i++)if(!isfinite(s->b[i]))return 0;
 return 1;
}
void ctrl_reset(Controller*s){
 s->integral=0;s->derivative=0;s->previous_measurement=0;
 s->previous_duty=0;s->initialized=0;
}
void ctrl_predict(const Controller*s,float v1,float v2,float u,float*n1,float*n2){
 *n1=s->a[0][0]*v1+s->a[0][1]*v2+s->b[0]*u;
 *n2=s->a[1][0]*v1+s->a[1][1]*v2+s->b[1]*u;
}
static float pid(Controller*s,float v2,float bus){
 const ControlConfig*c=&s->cfg;float dt=c->sampling_us*1e-6f;
 float e=c->reference-v2;
 float derivative=s->initialized?(v2-s->previous_measurement)/dt:0;
 s->derivative+=(dt/(c->derivative_tau+dt))*(derivative-s->derivative);
 float equilibrium_v1=c->reference*(1+c->r2/c->load2);
 float equilibrium_u=equilibrium_v1+c->r1*(equilibrium_v1/c->load1+c->reference/c->load2);
 float base=(equilibrium_u-c->low_drop)/bus+c->kp*e-c->kd*s->derivative;
 float next=clamp(s->integral+c->ki*dt*e,-1,1);
 float lo=fmaxf(c->duty_min,s->previous_duty-c->slew),hi=fminf(c->duty_max,s->previous_duty+c->slew);
 if(!((base+next>hi&&e>0)||(base+next<lo&&e<0)))s->integral=next;
 return clamp(base+s->integral,lo,hi);
}
static float fcs(Controller*s,float v1,float v2,float bus){
 float best=INFINITY,action=0;
 /* Four future physical switch decisions (16 trajectories); no duty PWM. */
 for(unsigned seq=0;seq<16;seq++){
  float a=v1,b=v2,previous=s->previous_duty,cost=0;
  for(unsigned j=0;j<4;j++){
   float u=(float)((seq>>j)&1u),na,nb;
   ctrl_predict(s,a,b,s->cfg.low_drop+bus*u,&na,&nb);a=na;b=nb;
   float e=b-s->cfg.reference,du=u-previous;
   cost+=s->cfg.mpc_q*e*e+s->cfg.mpc_r*du*du;previous=u;
  }
  if(cost<best){best=cost;action=(float)(seq&1u);}
 }
 return action;
}
/* Cholesky solve, matrix overwritten with lower triangle. */
static int factor(float a[CTRL_MPC_MAX][CTRL_MPC_MAX],int n){
 for(int i=0;i<n;i++)for(int j=0;j<=i;j++){
  float x=a[i][j];for(int k=0;k<j;k++)x-=a[i][k]*a[j][k];
  if(i==j){if(!(x>1e-12f)||!isfinite(x))return 0;a[i][j]=sqrtf(x);}
  else a[i][j]=x/a[j][j];
 }return 1;
}
static void solve(float a[CTRL_MPC_MAX][CTRL_MPC_MAX],const float*b,float*x,int n){
 for(int i=0;i<n;i++){float v=b[i];for(int j=0;j<i;j++)v-=a[i][j]*x[j];x[i]=v/a[i][i];}
 for(int i=n-1;i>=0;i--){float v=x[i];for(int j=i+1;j<n;j++)v-=a[j][i]*x[j];x[i]=v/a[i][i];}
}
static float dot(const float*a,const float*b,int n){float x=0;for(int i=0;i<n;i++)x+=a[i]*b[i];return x;}
static int contains(const int*w,int count,int x){for(int i=0;i<count;i++)if(w[i]==x)return 1;return 0;}
static int representative(int *parent,int i){while(parent[i]!=i)i=parent[i];return i;}
/* Every inequality is a coordinate bound or adjacent difference. Its normal
   is an edge in a graph with an extra ground vertex. A cycle is exactly a
   linearly dependent working-set row, including redundant slew/box bounds. */
static void endpoints(const float*row,int n,int*a,int*b){
 *a=n;*b=n;
 for(int i=0;i<n;i++)if(row[i]!=0){if(*a==n)*a=i;else *b=i;}
}
static void working_graph(Controller*s,const int*active,int nw,int n,int*parent){
 for(int i=0;i<=n;i++)parent[i]=i;
 for(int j=0;j<nw;j++){int a,b;endpoints(s->constraints[active[j]],n,&a,&b);
  parent[representative(parent,a)]=representative(parent,b);}
}
static int qp(Controller*s,float v1,float v2,float bus,ControlResult*out){
 int n=(int)s->cfg.mpc_horizon,m=4*n;
 float free_y[CTRL_MPC_MAX],g[CTRL_MPC_MAX]={0},bound[4*CTRL_MPC_MAX];
 float u[CTRL_MPC_MAX],grad[CTRL_MPC_MAX],p0[CTRL_MPC_MAX],p[CTRL_MPC_MAX];
 float lambda[CTRL_MPC_MAX]={0},rhs[CTRL_MPC_MAX];int active[CTRL_MPC_MAX],nw=0;
 float q=s->cfg.mpc_q,r=s->cfg.mpc_r,a=v1,b=v2;
 memset(s->response,0,sizeof(s->response));
 for(int k=0;k<n;k++){
  float na,nb;ctrl_predict(s,a,b,s->cfg.low_drop,&na,&nb);a=na;b=nb;free_y[k]=b;
  float ga=s->b[0]*bus,gb=s->b[1]*bus;
  for(int j=k;j<n;j++){
   s->response[j][k]=gb;
   na=s->a[0][0]*ga+s->a[0][1]*gb;nb=s->a[1][0]*ga+s->a[1][1]*gb;ga=na;gb=nb;
  }
 }
 for(int i=0;i<n;i++){
  for(int j=0;j<n;j++){
   float h=0;for(int k=0;k<n;k++)h+=q*s->response[k][i]*s->response[k][j];
   /* D'D: delta u0=u0-u_previous, delta ui=ui-u(i-1). */
   if(i==j)h+=r*(i==n-1?1:2);
   else if(i==j+1||j==i+1)h-=r;
   s->h[i][j]=2*h;s->work[i][j]=2*h;
  }
  for(int k=0;k<n;k++)g[i]+=2*q*s->response[k][i]*(free_y[k]-s->cfg.reference);
 }
 g[0]-=2*r*s->previous_duty;
 if(!factor(s->work,n))return 0;
 for(int j=0;j<n;j++){
  float basis[CTRL_MPC_MAX]={0},x[CTRL_MPC_MAX];basis[j]=1;solve(s->work,basis,x,n);
  for(int i=0;i<n;i++)s->hinv[i][j]=x[i];
 }
 memset(s->constraints,0,sizeof(s->constraints));
 for(int i=0;i<n;i++){
  int row=4*i;
  s->constraints[row][i]=1;bound[row]=s->cfg.duty_max;
  s->constraints[row+1][i]=-1;bound[row+1]=-s->cfg.duty_min;
  s->constraints[row+2][i]=1;s->constraints[row+3][i]=-1;
  bound[row+2]=s->cfg.slew;bound[row+3]=s->cfg.slew;
  if(i){s->constraints[row+2][i-1]=-1;s->constraints[row+3][i-1]=1;}
  else {bound[row+2]+=s->previous_duty;bound[row+3]-=s->previous_duty;}
  u[i]=clamp(s->previous_duty,s->cfg.duty_min,s->cfg.duty_max);
 }
 for(int row=0;row<m;row++)for(int i=0;i<n;i++)
  s->inv_constraints[row][i]=dot(s->hinv[i],s->constraints[row],n);
 /* Feasible-start primal active-set QP. Add one blocking inequality per step;
    negative multipliers remove constraints. Positive mpc_r guarantees SPD. */
 int done=0;
 for(unsigned iteration=0;iteration<96;iteration++){
  out->iterations=iteration+1;
  for(int i=0;i<n;i++)grad[i]=g[i]+dot(s->h[i],u,n);
  for(int i=0;i<n;i++)p0[i]=-dot(s->hinv[i],grad,n);
  memcpy(p,p0,(size_t)n*sizeof(float));
  if(nw){
   for(int i=0;i<nw;i++){
    rhs[i]=dot(s->constraints[active[i]],p0,n);
    for(int j=0;j<nw;j++)s->work[i][j]=dot(s->constraints[active[i]],s->inv_constraints[active[j]],n);
   }
   if(!factor(s->work,nw))return 0;
   solve(s->work,rhs,lambda,nw);
   for(int i=0;i<n;i++)for(int j=0;j<nw;j++)p[i]-=s->inv_constraints[active[j]][i]*lambda[j];
  }
  int parent[CTRL_MPC_MAX+1];working_graph(s,active,nw,n,parent);
  /* Remove roundoff from the equality-constrained search direction. */
  for(int group=0;group<n;group++)if(representative(parent,group)==group){
   float mean=0;int count=0;
   for(int i=0;i<n;i++)if(representative(parent,i)==group){mean+=p[i];count++;}
   if(count)for(int i=0;i<n;i++)if(representative(parent,i)==group)p[i]=mean/count;
  }
  for(int i=0;i<n;i++)if(representative(parent,i)==representative(parent,n))p[i]=0;
  float norm=0;for(int i=0;i<n;i++)norm=fmaxf(norm,fabsf(p[i]));
  if(norm<2e-5f){
   int remove=-1;float lowest=-2e-5f;
   for(int j=0;j<nw;j++)if(lambda[j]<lowest){lowest=lambda[j];remove=j;}
   if(remove<0){done=1;break;}
   active[remove]=active[--nw];continue;
  }
  float alpha=1;int block=-1;
  for(int j=0;j<m;j++)if(!contains(active,nw,j)){
   int edge_a,edge_b;endpoints(s->constraints[j],n,&edge_a,&edge_b);
   if(representative(parent,edge_a)==representative(parent,edge_b))continue;
   float cp=dot(s->constraints[j],p,n);
   if(cp>1e-7f){float candidate=(bound[j]-dot(s->constraints[j],u,n))/cp;
    if(candidate<alpha){alpha=fmaxf(0,candidate);block=j;}}
  }
  for(int i=0;i<n;i++)u[i]+=alpha*p[i];
  if(block>=0&&alpha<1){if(nw>=n)return 0;active[nw++]=block;}
 }
 if(!done)return 0;
 float residual=0,scale=1;
 for(int i=0;i<n;i++){
  float stationarity=g[i]+dot(s->h[i],u,n);
  for(int j=0;j<nw;j++)stationarity+=s->constraints[active[j]][i]*lambda[j];
  scale=fmaxf(scale,fabsf(g[i]));residual=fmaxf(residual,fabsf(stationarity));
 }
 residual/=scale;
 for(int j=0;j<m;j++)residual=fmaxf(residual,dot(s->constraints[j],u,n)-bound[j]);
 for(int j=0;j<nw;j++){
  residual=fmaxf(residual,-lambda[j]/scale);
  residual=fmaxf(residual,fabsf(lambda[j]*(dot(s->constraints[active[j]],u,n)-bound[active[j]]))/scale);
 }
 out->qp_residual=residual;
 if(!isfinite(residual)||residual>0.002f)return 0;
 out->duty=clamp(u[0],fmaxf(s->cfg.duty_min,s->previous_duty-s->cfg.slew),
                       fminf(s->cfg.duty_max,s->previous_duty+s->cfg.slew));return 1;
}
ControlStatus ctrl_step(Controller*s,float v1,float v2,float vbus,ControlResult*out){
 memset(out,0,sizeof(*out));out->status=CTRL_OK;
 if(!isfinite(v1)||!isfinite(v2)||!isfinite(vbus)||v1<-.25f||v2<-.25f||
    v1>s->cfg.max_voltage||v2>s->cfg.max_voltage||vbus>s->cfg.max_bus||
    vbus<s->cfg.high_drop+s->cfg.low_drop+1){out->status=CTRL_BAD_MEASUREMENT;return out->status;}
 float bus=vbus-s->cfg.high_drop-s->cfg.low_drop;
 if((int)s->cfg.mode==CTRL_PID)out->duty=pid(s,v2,bus);
 else if((int)s->cfg.mode==CTRL_FCS)out->duty=fcs(s,v1,v2,bus);
 else if(!qp(s,v1,v2,bus,out)){out->duty=0;out->status=CTRL_QP_FAILED;return out->status;}
 float unused;ctrl_predict(s,v1,v2,s->cfg.low_drop+bus*out->duty,&unused,&out->predicted_v2);
 s->previous_measurement=v2;s->previous_duty=out->duty;s->initialized=1;
 return out->status;
}
