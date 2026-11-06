#ifndef CONTROL_H
#define CONTROL_H
#include <stdint.h>
#include <stddef.h>
#define CTRL_MPC_MAX 16
typedef enum { CTRL_PID=0, CTRL_FCS=1, CTRL_CSMPC=2 } ControlMode;
typedef enum { CTRL_OK=0, CTRL_BAD_CONFIG=1, CTRL_BAD_MEASUREMENT=2,
               CTRL_QP_FAILED=3 } ControlStatus;
typedef struct {
    float sampling_us, mode, reference, max_voltage, max_bus;
    float r1,r2,c1,c2,load1,load2;
    float kp,ki,kd,derivative_tau;
    float mpc_q,mpc_r,mpc_horizon;
    float duty_min,duty_max,slew,high_drop,low_drop;
} ControlConfig;
typedef struct {
    const char *name, *unit;
    size_t offset;
    float minimum, maximum;
    int integer;
} ControlParam;
extern const ControlParam ctrl_params[];
extern const size_t ctrl_param_count;
typedef struct {
    float duty,predicted_v2,qp_residual;
    uint32_t iterations;
    ControlStatus status;
} ControlResult;
typedef struct {
    ControlConfig cfg;
    float a[2][2],b[2];
    float integral,derivative,previous_measurement,previous_duty;
    uint32_t initialized;
    /* QP workspace is persistent, never allocated on interrupt stack. */
    float h[CTRL_MPC_MAX][CTRL_MPC_MAX], hinv[CTRL_MPC_MAX][CTRL_MPC_MAX];
    float response[CTRL_MPC_MAX][CTRL_MPC_MAX];
    float constraints[4*CTRL_MPC_MAX][CTRL_MPC_MAX];
    float inv_constraints[4*CTRL_MPC_MAX][CTRL_MPC_MAX];
    float work[CTRL_MPC_MAX][CTRL_MPC_MAX];
} Controller;
void ctrl_defaults(ControlConfig *cfg);
int ctrl_validate(const ControlConfig *cfg, char *reason, size_t capacity);
int ctrl_config_set(ControlConfig *cfg,const char *name,float value);
int ctrl_config_get(const ControlConfig *cfg,const char *name,float *value);
int ctrl_init(Controller *ctrl,const ControlConfig *cfg);
void ctrl_reset(Controller *ctrl);
ControlStatus ctrl_step(Controller *ctrl,float v1,float v2,float vbus,ControlResult *result);
/* Exact zero-order-held physical input model; useful for host simulations. */
void ctrl_predict(const Controller *ctrl,float v1,float v2,float input_voltage,float *next_v1,float *next_v2);
#endif
