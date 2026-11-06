/* Test-only ABI: independent Python/SciPy oracle controls all QP inputs. */
#include "control.h"
#include <string.h>
int oracle_run(const float *settings, float *output) {
    ControlConfig cfg; Controller ctrl; ControlResult result;
    ctrl_defaults(&cfg);
    cfg.mode=CTRL_CSMPC; cfg.mpc_horizon=settings[0];
    cfg.sampling_us=settings[1]; cfg.r1=settings[2]; cfg.r2=settings[3];
    cfg.c1=settings[4]; cfg.c2=settings[5]; cfg.reference=settings[6];
    cfg.slew=settings[7]; cfg.mpc_q=settings[8]; cfg.mpc_r=settings[9];
    if (!ctrl_init(&ctrl,&cfg)) return -1;
    ctrl.previous_duty=settings[13];
    memcpy(output,ctrl.a,sizeof ctrl.a);
    memcpy(output+4,ctrl.b,sizeof ctrl.b);
    int status=ctrl_step(&ctrl,settings[10],settings[11],settings[12],&result);
    output[6]=result.duty; output[7]=result.qp_residual;
    output[8]=(float)result.iterations;
    return status;
}
