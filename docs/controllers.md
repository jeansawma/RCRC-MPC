# RCRC controllers

The controlled voltage is **ToC4**, across C19. ToC3/C18 is the first measured
state. These are two cascaded passive RC sections, rather than a buck converter.
The circuit shown in `Power 2.png` has R1=R2=470 ohm and C18=C19=100 microfarad.
Each voltage sensing divider adds a 55.7 kohm shunt load. Additional external
loads should be included as the parallel combination in `load1` and `load2`.

## Plant model

With x=[vC18,vC19], input voltage u at OUT3, and output y=vC19:

```
dx1/dt = [u/R1 - (1/R1+1/R2+1/load1)*x1 + x2/R2] / C1
dx2/dt = [x1/R2 - (1/R2+1/load2)*x2] / C2
```

`ctrl_init()` computes the exact zero-order-hold model `x+ = Ad*x + Bd*u`
from a closed form 2x2 matrix exponential. Double precision is used during
configuration to avoid cancellation for slow plants; control steps use float.
No forward Euler approximation or fixed sample-time assumption is hidden in
the controller. Both state voltages are measured, so an observer is unnecessary.

For duty d, the averaged input is
`u = low_drop + d*(Vbus - high_drop - low_drop)`.
The L298 has load-dependent transistor drops. Defaults of zero describe an ideal
bridge; measure/calibrate `high_drop` (Vbus minus high-state OUT3) and `low_drop`
(low-state OUT3) before comparing predictions with hardware. A fixed-drop model
cannot capture every current/temperature condition.

## PI / PID (mode 0)

The default derivative gain is zero, giving a PI controller. Set `kd` above zero
for PID. The implementation uses:

- reference feedforward from the model's resistor-current DC equilibrium;
- proportional error in volts;
- integral gain in duty/(V*s), with conditional integration antiwindup;
- a first order filtered derivative **of measured output**, avoiding reference
  derivative kick;
- the configured duty interval and per-sample duty slew limit.

The gains are starting values for the attached nominal RC circuit. Arbitrary
runtime gain changes are not guaranteed stable. An output limit can constrain
the actuator without making an unreachable reference reachable.

## Finite control set MPC (mode 1)

At every control sample, all 16 four-step binary switching sequences are
predicted using the exact held-input model. The cost is

```
sum(k=1..4) [mpc_q*(y[k]-reference)^2 + mpc_r*(s[k]-s[k-1])^2]
s[k] in {0,1}
```

Only the first switch action is applied, directly and held until the next sample.
This is finite control set operation: its output is always exactly zero or one.
The 20 kHz PWM carrier is bypassed for this mode. The switching rate depends on
the optimization and control sampling rate, and the resulting output ripple is
larger than averaged PWM control. `mpc_horizon` applies to CSMPC only; the FCS
horizon is fixed at four to bound exhaustive enumeration cost. FCS configuration
requires `duty_min=0`, `duty_max=1`, and `slew=1`; continuous constraints that
would prohibit binary actions are rejected.

## Continuous control set MPC (mode 2)

The measured state is propagated over `mpc_horizon` samples (1..16). Condensing
the prediction gives `Y = Yfree + G*U`. The controller solves the convex QP

```
minimize  mpc_q*||Y-reference||^2 + mpc_r*||D*U-[previous_duty,0,...]||^2
subject to duty_min <= U[i] <= duty_max
           abs(U[0]-previous_duty) <= slew
           abs(U[i]-U[i-1]) <= slew
```

Both weights must be strictly positive. `D` is the first-difference matrix, so
the Hessian is positive definite. The first optimum duty is applied using PWM;
the problem is rebuilt at the next sample with the new measured voltages/bus.
There is no external QP runtime dependency.

The solver is a feasible-start primal active-set method. It adds blocking
inequalities, removes constraints with negative multipliers, and uses Cholesky
linear solves. Coordinate bounds and adjacent-difference constraints form graph
incidence rows; graph connectivity excludes exactly dependent working-set rows
such as a redundant full-range slew bound. This avoids singular KKT systems
at simultaneous duty and slew limits.

The implementation reports iterations and a residual combining normalized
stationarity, primal feasibility, dual multiplier feasibility, and
complementarity. It fails closed if factorization fails, the residual is above
0.002, or 96 iterations do not establish optimality. A failure returns zero duty
and `CTRL_QP_FAILED`; the application must also disable the bridge. The caller
must never silently use a prior optimum after a failed solve.

The dense setup is O(N^3); a working-set step is at most O(N^3). Persistent
workspace avoids large allocations on the control interrupt stack. Default N=8
with 5 ms sampling gives a 40 ms prediction window and a conservative execution
budget. This is a starting configuration, not a measured MCU execution-time
guarantee. Measure execution time and deadline faults on the actual board before
reducing the sample period or increasing the horizon. USB/OLED work must not
delay the control deadline.

These MPC formulations penalize voltage error but do not impose hard predicted
output-voltage constraints or an augmented disturbance integrator. Actual
measured voltage/bus limits are checked independently. MPC can retain steady
error under model mismatch; calibration, identified model parameters, or a future
offset-free MPC extension are needed when that error is material. Output limit
checks are software protection, not an independent hardware shutdown path.

## Configuration and reset

`ctrl_validate()` checks finite scalar bounds, integer parameters, ordered duty
bounds, reference versus voltage limit, and consistency of the FCS action set.
`ctrl_config_set()` checks the individual field; validate a complete candidate
before applying it. `ctrl_init()` rebuilds the model and resets controller
history. `ctrl_reset()` initializes the previous duty to zero, matching the
disabled power stage. Configuration requires `duty_min <= slew`, so the initial
step can satisfy both the minimum duty and the slew limit from zero.

Changing a model, sampling period, gains, or controller requires a stopped power
stage and reinitialization. Live reference updates are an application policy.
All state, output, and bus measurements must be in physical volts; conversion
from ADC codes and board-specific calibration belong to the platform/application.

## Host validation

`tests/test_control.c` verifies metadata validation and nonfinite rejection,
zero-order-hold semigroup behavior and loaded DC equilibrium, antiwindup,
closed-form horizon-one QP solutions with active box/slew limits, randomized
multi-step QP feasibility, binary FCS actions, and nominal closed-loop regulation
for all three modes. The host plant uses the same exact model for these nominal
regulation checks; it does not establish robustness to every physical mismatch.

`tests/test_oracle.py` provides an independent SciPy matrix exponential and
optimization comparison. Host tests establish numerical/software behavior;
on-board timing, ADC accuracy, USB enumeration, display behavior, and power-stage
operation still require physical commissioning.
